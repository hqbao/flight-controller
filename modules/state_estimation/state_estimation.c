/**
 * STATE_ESTIMATION — Unified ESKF estimator (fusion6).
 *
 * See state_estimation.h for subscribe/publish topic list.
 *
 * Threading / ISR notes:
 *   - All sensor PubSub callbacks may fire from scheduler ISRs (SCHEDULER_*).
 *     fusion6 update functions are not reentrant; they are only called from
 *     these ISR-context callbacks which are serialized by the scheduler.
 *   - The 1 kHz gyro callback drives fusion6_predict() with the most recent
 *     500 Hz accel sample held in cache. This is the standard "high-rate
 *     gyro / lower-rate accel" pattern; the accel update step (gravity
 *     correction) runs at 500 Hz separately.
 *   - SCHEDULER_25HZ runs from the 25 Hz tick; we snapshot the filter state
 *     and publish on STATE_UPDATE.
 *
 * Initialization sequence:
 *   1. After IMU calibration loads (gyro_ready + accel_ready in imu module),
 *      we start receiving SENSOR_IMU1_*_UPDATE.
 *   2. Vehicle MUST be still for ~0.5 s. We accumulate STATIC_INIT_SAMPLES
 *      accel+gyro pairs into fusion6_static_init_sample().
 *   3. Once threshold reached we call fusion6_static_init_finalize();
 *      on success init_done=1 and predict/update steps go live.
 *   4. On failure (motion gate triggered) we reset the counter and retry.
 */
#include "state_estimation.h"
#include "sensor_unit.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <fusion6.h>
#include <vector3d.h>
#include <quat.h>
#include <string.h>
#include <math.h>

/* ============================================================
 * Tunables (unit-test mirrored against tuning_params_t at TUNING_READY)
 * ============================================================ */

/* Number of static-init samples (accel @ 500 Hz → 0.5 s = 250 samples). */
#define STATIC_INIT_SAMPLES   250

/* Optical-flow quality gate (sensor_unit linear interpolation). */
#define OPTFLOW_Q_MIN         30.0
#define OPTFLOW_Q_GOOD        80.0

/* Lidar plausibility window (mm). */
#define LIDAR_MIN_MM          50
#define LIDAR_MAX_MM          4000

/* IMU LSB → m/s² scaling (matches macro.h MAX_IMU_ACCEL = 1g LSB count). */
#define ACCEL_LSB_TO_MPS2     (GRAVITY_MSS_LOCAL / (double)MAX_IMU_ACCEL)
#define GRAVITY_MSS_LOCAL     9.80665

/* ============================================================
 * State
 * ============================================================ */

static fusion6_t        g_f;
static fusion6_config_t g_cfg;

static int      g_init_started = 0;     /* 1 once we begin accumulating samples */
static int      g_static_count = 0;
static uint32_t g_now_us = 0;           /* mirrors g_f.now_us */

/* Cached latest IMU samples (body NED, fully converted) */
static double   g_last_gyro_body[3]  = {0, 0, 0};
static double   g_last_accel_body[3] = {0, 0, 0};
static int      g_have_gyro  = 0;
static int      g_have_accel = 0;

/* GPS lazy origin */
static int      g_gps_origin_set = 0;
static int32_t  g_gps_lat0_e7    = 0;
static int32_t  g_gps_lon0_e7    = 0;
static int32_t  g_gps_alt0_mm    = 0;

/* GPS quality gate — only feed fusion6 when receiver reports a reliable
 * 3D fix (gnssFixOK + fix_type==3 + num_sv>=6 + h_acc<5m). Set by gps.c
 * in EXTERNAL_SENSOR_GPS_QUALITY. Default 0 — until quality message
 * arrives (no receiver attached, or no fix yet), all GPS updates are
 * dropped. Mirrors gps_navigation's gating pattern. */
static uint8_t  g_gps_reliable   = 0;

/* Forward decl — defined below; called from on_accel at 500 Hz. */
static void publish_state(void);

/* ============================================================
 * Helpers
 * ============================================================ */

/* Convert calibrated IMU sample (deg/s gyro + LSB accel) to fusion6 units. */
static inline void map_gyro(const float raw_dps[3], double out_body_rads[3]) {
	double gx = (double)raw_dps[0] * (double)DEG2RAD;
	double gy = (double)raw_dps[1] * (double)DEG2RAD;
	double gz = (double)raw_dps[2] * (double)DEG2RAD;
	imu_axis_map_gyro(gx, gy, gz, out_body_rads);
}

static inline void map_accel(const float raw_lsb[3], double out_body_mps2[3]) {
	double ax = (double)raw_lsb[0] * ACCEL_LSB_TO_MPS2;
	double ay = (double)raw_lsb[1] * ACCEL_LSB_TO_MPS2;
	double az = (double)raw_lsb[2] * ACCEL_LSB_TO_MPS2;
	imu_axis_map_accel(ax, ay, az, out_body_mps2);
}

/* Compute meas_us in fusion6's internal clock for a sensor sample we're about
 * to feed in. Fusion6 advances now_us internally during predict; we snapshot
 * it after each predict into g_now_us. */
static inline uint32_t meas_time_us(double latency_s) {
	uint32_t lat_us = (uint32_t)(latency_s * 1e6);
	return (lat_us > g_now_us) ? 0 : (g_now_us - lat_us);
}

/* ============================================================
 * Static init driver
 * ============================================================ */

static void static_init_step(void) {
	fusion6_static_init_sample(&g_f, g_last_accel_body, g_last_gyro_body);
	g_static_count++;
	if (g_static_count >= STATIC_INIT_SAMPLES) {
		if (fusion6_static_init_finalize(&g_f)) {
			/* Success — predict/update loop is now live. */
			g_init_started = 1;
		} else {
			/* Motion gate failed — drop accumulator, retry. */
			g_static_count = 0;
		}
	}
}

/* ============================================================
 * Sensor callbacks
 * ============================================================ */

static void on_gyro(uint8_t *data, size_t size) {
	if (size < 12) return;
	float raw[3];
	memcpy(raw, data, 12);
	map_gyro(raw, g_last_gyro_body);
	g_have_gyro = 1;

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	/* Predict at gyro rate, holding latest accel sample. */
	if (!g_have_accel) return;
	double dt = 1.0 / (double)GYRO_FREQ;
	fusion6_predict(&g_f, g_last_gyro_body, g_last_accel_body, dt);
	g_now_us = g_f.now_us;
}

static void on_accel(uint8_t *data, size_t size) {
	if (size < 12) return;
	float raw[3];
	memcpy(raw, data, 12);
	map_accel(raw, g_last_accel_body);
	g_have_accel = 1;

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) {
		/* Drive static init only once both gyro + accel have at least one
		 * sample — guarantees the gyro accumulator gets non-zero data. */
		if (g_have_gyro && !g_init_started) static_init_step();
		return;
	}

	/* Gravity update at accel rate. */
	fusion6_update_accel(&g_f, g_last_accel_body);

	/* Publish full state at accel rate (500 Hz) so the inner attitude
	 * controller reading STATE_UPDATE has fresh attitude. */
	publish_state();
}

static void on_compass(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	vector3d_t mag_body;
	memcpy(&mag_body, data, sizeof(vector3d_t));

	/* Tilt-compensate using the current attitude estimate.
	 * 1) Build tilt-only quaternion (zero yaw).
	 * 2) Rotate body-frame mag → level frame.
	 * 3) heading = atan2(-mag_y, mag_x)  (NED: 0 = North, +π/2 = East).
	 * Same convention as legacy attitude_estimation::mag_update(). */
	quaternion_t q = {(float)g_f.q[0], (float)g_f.q[1],
	                  (float)g_f.q[2], (float)g_f.q[3]};
	vector3d_t euler;
	quat_to_euler(&euler, &q);

	quaternion_t q_tilt;
	quat_from_euler(&q_tilt, euler.x, euler.y, 0);

	vector3d_t mag_earth;
	quat_rotate_vector(&mag_earth, &q_tilt, &mag_body);

	double yaw_meas = atan2(-mag_earth.y, mag_earth.x);
	fusion6_update_mag_yaw(&g_f, yaw_meas);
}

static void on_baro(uint8_t *data, size_t size) {
	if (size < sizeof(double)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	/* air_pressure publishes altitude in MILLIMETERS as a double
	 * (ALTITUDE_SCALE_FACTOR = 1000.0 in air_pressure.c). */
	double alt_mm;
	memcpy(&alt_mm, data, sizeof(double));
	double alt_m = alt_mm * 1e-3;
	fusion6_update_baro(&g_f, alt_m);
}

static void on_optflow(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	optflow_data_t msg;
	memcpy(&msg, data, sizeof(optflow_data_t));

	/* Range-finder altitude piggy-backs on the optflow message (downward only). */
	int upward = (msg.direction == OPTFLOW_UPWARD);

	/* Lidar: only valid for the downward-facing camera. */
	double h_agl = -1.0;
	if (!upward) {
		int32_t z_mm = (int32_t)msg.z;
		if (lidar_valid_mm(z_mm, LIDAR_MIN_MM, LIDAR_MAX_MM)) {
			h_agl = lidar_mm_to_m(z_mm);
			fusion6_update_lidar(&g_f, h_agl);
		}
	}

	/* Optical flow update — needs height above ground. If we don't have it
	 * from this message, skip (fusion6 also rejects h_agl < 0.1). */
	if (h_agl < 0.1) return;
	if (msg.dt_us == 0) return;

	double flow_x = optflow_to_rad_per_s(msg.dx, msg.dt_us, upward);
	double flow_y = optflow_to_rad_per_s(msg.dy, msg.dt_us, upward);

	/* Adaptive R via quality. INFINITY → caller skips update. */
	double R_scale = optflow_quality_to_R_scale(msg.clarity,
	                                            OPTFLOW_Q_MIN, OPTFLOW_Q_GOOD);
	if (!isfinite(R_scale)) return;

	/* fusion6_update_optflow takes raw quality 0..255 (rejects <30). Pass through. */
	fusion6_update_optflow(&g_f, flow_x, flow_y, h_agl, msg.clarity);
}

static void on_gps_quality(uint8_t *data, size_t size) {
	if (size < sizeof(gps_quality_t)) return;
	gps_quality_t q;
	memcpy(&q, data, sizeof(gps_quality_t));
	g_gps_reliable = q.reliable;
}

static void on_gps_pos(uint8_t *data, size_t size) {
	if (size < sizeof(gps_position_t)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;
	if (!g_gps_reliable) return;

	gps_position_t p;
	memcpy(&p, data, sizeof(gps_position_t));

	gps_origin_capture(&g_gps_origin_set,
	                   &g_gps_lat0_e7, &g_gps_lon0_e7, &g_gps_alt0_mm,
	                   p.lat, p.lon, p.alt);

	double pos_ned[3];
	gps_lla_to_ned(p.lat, p.lon, p.alt,
	               g_gps_lat0_e7, g_gps_lon0_e7, g_gps_alt0_mm, pos_ned);

	uint32_t meas_us = meas_time_us(g_cfg.latency_gps_s);
	fusion6_update_gps_pos_at(&g_f, pos_ned, NULL, meas_us);
}

static void on_gps_vel(uint8_t *data, size_t size) {
	if (size < sizeof(gps_velocity_t)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;
	if (!g_gps_reliable) return;

	gps_velocity_t v;
	memcpy(&v, data, sizeof(gps_velocity_t));

	double vel_ned[3];
	gps_vel_mm_per_s_to_m_per_s(v.velN, v.velE, v.velD, vel_ned);

	uint32_t meas_us = meas_time_us(g_cfg.latency_gps_s);
	fusion6_update_gps_vel_at(&g_f, vel_ned, 0.0 /*default sigma*/, meas_us);
}

/* ============================================================
 * Tuning
 * ============================================================ */

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t t;
	memcpy(&t, data, sizeof(tuning_params_t));

	g_cfg.latency_baro_s    = t.est_latency_baro_ms    * 1e-3;
	g_cfg.latency_lidar_s   = t.est_latency_lidar_ms   * 1e-3;
	g_cfg.latency_optflow_s = t.est_latency_optflow_ms * 1e-3;
	g_cfg.latency_gps_s     = t.est_latency_gps_ms     * 1e-3;

	g_cfg.timeout_gps_us     = (uint32_t)(t.est_timeout_gps_ms     * 1000.0f);
	g_cfg.timeout_optflow_us = (uint32_t)(t.est_timeout_optflow_ms * 1000.0f);
	g_cfg.timeout_lidar_us   = (uint32_t)(t.est_timeout_lidar_ms   * 1000.0f);
	g_cfg.timeout_mag_us     = (uint32_t)(t.est_timeout_mag_ms     * 1000.0f);
	g_cfg.timeout_baro_us    = (uint32_t)(t.est_timeout_baro_ms    * 1000.0f);

	g_cfg.p_runaway_pos_m2 = t.est_p_runaway_pos_m2;

	/* Process noise (Q) */
	g_cfg.sigma_accel      = t.est_q_accel;
	g_cfg.sigma_gyro       = t.est_q_gyro;
	g_cfg.sigma_bias_accel = t.est_q_ba;
	g_cfg.sigma_bias_gyro  = t.est_q_bg;
	g_cfg.sigma_bias_baro  = t.est_q_bbaro;

	/* Measurement noise (R) */
	g_cfg.R_accel       = t.est_r_accel;
	g_cfg.R_mag_yaw     = t.est_r_mag_yaw;
	g_cfg.R_baro        = t.est_r_baro;
	g_cfg.R_lidar       = t.est_r_lidar;
	g_cfg.R_optflow     = t.est_r_optflow;
	g_cfg.R_gps_pos_xy  = t.est_r_gps_pos_h;
	g_cfg.R_gps_vel     = t.est_r_gps_vel;

	/* Innovation gating */
	g_cfg.accel_mag_gate = t.est_accel_g_tol;

	/* Live-mirror into the running filter so the new values take effect on
	 * the next call. fusion6 reads these out of f->cfg every step.
	 * NOTE: est_chi2_pos/vel and est_gps_lever_x/y/z are reserved tunables;
	 * fusion6_config_t does not currently expose chi-square gates or GPS
	 * lever-arm fields, so they stay in tuning_params_t for future use. */
	memcpy(&g_f.cfg, &g_cfg, sizeof(fusion6_config_t));
}

/* ============================================================
 * Publish helpers
 *
 * publish_state() is called at accel rate (500 Hz) from on_accel so that
 * the inner attitude controller — running at ATT_CTL_SCHEDULER (500 Hz) —
 * always sees fresh attitude. Health checks still run at 25 Hz from
 * on_25hz to avoid flooding flight_state with redundant transitions.
 * ============================================================ */

static void publish_state(void) {
	if (!g_init_started) return;

	fusion6_state_t s;
	fusion6_get_state(&g_f, &s);

	nav_state_t out;
	out.position    = s.pos;
	out.velocity    = s.vel;
	out.q[0]        = s.q.w;
	out.q[1]        = s.q.x;
	out.q[2]        = s.q.y;
	out.q[3]        = s.q.z;
	out.euler.roll  = s.euler.x * (double)RAD2DEG;
	out.euler.pitch = s.euler.y * (double)RAD2DEG;
	out.euler.yaw   = s.euler.z * (double)RAD2DEG;
	out.accel_body  = s.accel_body;
	out.accel_earth = s.accel_earth;
	out.gyro_bias   = s.gyro_bias;
	out.accel_bias  = s.accel_bias;
	out.baro_bias   = s.baro_bias;
	out.trace_P_pos = s.trace_P_pos;
	out.trace_P_vel = s.trace_P_vel;
	out.trace_P_att = s.trace_P_att;
	out.health_flags = s.health_flags;
	out.init_done   = (s.health_flags & FUSION6_HF_INIT_DONE) ? 1 : 0;
	out._pad        = 0;

	publish(STATE_UPDATE, (uint8_t *)&out, sizeof(out));
}

static void on_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (!g_init_started) return;
	fusion6_check_health(&g_f, g_now_us);
}

/* ============================================================
 * Setup
 * ============================================================ */

void state_estimation_setup(void) {
	fusion6_config_default(&g_cfg);
	fusion6_init(&g_f, &g_cfg);

	subscribe(SENSOR_IMU1_GYRO_FILTERED_UPDATE, on_gyro);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, on_accel);
	subscribe(SENSOR_COMPASS, on_compass);
	subscribe(SENSOR_AIR_PRESSURE, on_baro);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, on_optflow);
	subscribe(EXTERNAL_SENSOR_GPS, on_gps_pos);
	subscribe(EXTERNAL_SENSOR_GPS_VELOC, on_gps_vel);
	subscribe(EXTERNAL_SENSOR_GPS_QUALITY, on_gps_quality);
	subscribe(TUNING_READY, on_tuning_ready);
	subscribe(SCHEDULER_25HZ, on_25hz);
}
