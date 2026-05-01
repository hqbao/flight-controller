/**
 * STATE_ESTIMATION — glue around fusion6.
 *
 * Sensor flow:
 *   IMU gyro  (1 kHz, post-notch) → on_gyro  → fusion6_predict
 *   IMU accel (500 Hz)           → on_accel → fusion6_update_accel
 *                                            → publish STATE_UPDATE @ 500 Hz
 *   Compass   (25 Hz, unit vec)  → on_compass → mag_axis_map
 *                                             → mag_heading (tilt-comp,
 *                                                decl-corrected)
 *                                             → fusion6_update_mag_heading
 *                                                (1-D yaw pseudo-meas)
 *                                             → diagnostic stream
 *   Optflow   (~25 Hz, DOWN only) → on_optflow → derotate (gyro)
 *                                             → range·flow → v_body_xy
 *                                             → fusion6_update_velocity_xy_body
 *                                             → diagnostic stream
 *
 * Magnetometer feeds yaw only (1-D, decl-corrected heading). Roll/pitch are
 * still driven by the gravity update; mag does NOT touch the horizontal
 * components of attitude. Optical flow feeds horizontal body velocity (vx,vy)
 * and indirectly tightens roll/pitch through the cross-coupling. Vertical
 * velocity / position are not yet observed by any update — `vz` and `pz`
 * still drift on accel-only integration.
 *
 * The UP-pointing camera publishes on the same EXTERNAL_SENSOR_OPTFLOW topic
 * but is intentionally ignored here: it has no range finder so there is no
 * way to convert its angular flow to body-frame m/s. UP frames are silently
 * dropped at the top of on_optflow().
 *
 * Telemetry:
 *   STATE_UPDATE          (nav_state_t, 500 Hz) — for downstream control
 *   LOG_CLASS_ATTITUDE    (50 Hz, 9×float)     — tools/attitude_view.py
 *   LOG_CLASS_MAG_FUSION  (25 Hz, 7×float)     — mag diagnostics
 *   LOG_CLASS_VEL_FUSION  (25 Hz, 6×float)     — optflow velocity diagnostics
 */

#include "state_estimation.h"
#include "sensor_unit.h"

#include <pubsub.h>
#include <macro.h>
#include <messages.h>
#include <platform.h>
#include <fusion6.h>
#include <vector3d.h>
#include <quat.h>
#include <math.h>
#include <string.h>

/* ============================================================
 * Tunables
 * ============================================================ */

#define STATIC_INIT_SAMPLES   250

#define GRAVITY_MSS_LOCAL     9.80665
#define ACCEL_LSB_TO_MPS2     (GRAVITY_MSS_LOCAL / (double)MAX_IMU_ACCEL)
#define MAG_DIAG_DECLINATION_RAD   (-0.6  * (double)DEG2RAD)

/* --- Optical-flow gates ---
 * Range: VL53L1X reports 0..4 m; reject below 50 mm where geometry breaks
 * down and above 4 m where the ToF saturates.
 * Clarity: optflow library returns ~0..100; gate at 5 — frames below this
 * are dropped entirely.
 * Velocity: a 4 m/s body velocity at 1 m range gives ~4 rad/s flow — well
 * above any sane indoor scenario; reject obvious glitches above this.
 *
 * Measurement noise lives inside fusion6 as cfg.R_vel_xy_body (per-axis
 * variance, applied as diag R). Static for now; revisit if bench tests show
 * the filter is over- or under-trusting the camera.
 */
#define OPTFLOW_RANGE_MIN_MM     50
#define OPTFLOW_RANGE_MAX_MM     4000
#define OPTFLOW_CLARITY_MIN      5.0
#define OPTFLOW_VEL_MAX_MPS      6.0

/* Zero-velocity update (ZUPT): when the rangefinder reports below the
 * optflow gate (taxi / takeoff / landing inside the optflow blind zone),
 * fuse v_body=(0,0) instead of leaving the estimator open-loop. Without
 * this, accel-bias + tilt-error integration walks the velocity by
 * ~0.15 m/s/s every second the camera is gated out. With it, the
 * estimator is pinned at zero on the ground and bias states keep getting
 * observed. ZUPT shares cfg.R_vel_xy_body with regular optflow updates. */
#define OPTFLOW_ZUPT_RANGE_MAX_MM   200    /* trigger only if 0 ≤ z_mm < this */

/* ============================================================
 * State
 * ============================================================ */

static fusion6_t        g_f;

static int      g_init_started = 0;
static int      g_static_count = 0;
static uint64_t g_last_gyro_us = 0;

static double   g_last_gyro_body[3]  = {0, 0, 0};
static double   g_last_accel_body[3] = {0, 0, 0};

/* Latest compass diagnostics for the LOG_CLASS_MAG_FUSION stream. */
static double   g_last_mag_meas[3] = {0, 0, 0};
static double   g_last_mag_heading = 0.0;          /* radians, decl-corrected, [-π, π] */

/* Latest optflow snapshot (DOWN camera only — UP has no range finder so it
 * is not fused; see on_optflow() for the gate). */
static double   g_last_optflow_v[2]    = {0, 0};   /* [vx, vy] body, m/s */
static double   g_last_optflow_clarity = 0.0;
static double   g_last_optflow_range_m = 0.0;

static uint8_t  g_log_class      = 0;

static void publish_state(void);

/* ============================================================
 * Helpers
 * ============================================================ */

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

static inline void map_mag(const vector3d_t *sensor_unit, double out_body_unit[3]) {
	mag_axis_map(sensor_unit->x, sensor_unit->y, sensor_unit->z, out_body_unit);
}

static void update_mag_diagnostics(void) {
	/* Diagnostic-only: tilt-compensated magnetic heading. The body-yaw-aligned
	 * level-frame mag vector m_lvl = Rz(-yaw) · R(q) · m_body has roll & pitch
	 * removed, so atan2(-m_lvl.y, m_lvl.x) gives the magnetic heading. We only
	 * need its two horizontal components, then drop it. Nothing here mutates
	 * the ESKF. */
	vector3d_t m_b = { g_last_mag_meas[0], g_last_mag_meas[1], g_last_mag_meas[2] };
	vector3d_t m_e;
	quat_rotate_vector(&m_e, &g_f.q, &m_b);

	vector3d_t eul;
	quat_to_euler(&eul, &g_f.q);
	double cy = cos(eul.z);
	double sy = sin(eul.z);
	double lvl_x =  cy * m_e.x + sy * m_e.y;
	double lvl_y = -sy * m_e.x + cy * m_e.y;

	double heading = atan2(-lvl_y, lvl_x) - MAG_DIAG_DECLINATION_RAD;
	g_last_mag_heading = atan2(sin(heading), cos(heading));
}

static void static_init_step(void) {
	fusion6_static_init_sample(&g_f, g_last_accel_body, g_last_gyro_body);
	g_static_count++;
	if (g_static_count >= STATIC_INIT_SAMPLES) {
		if (fusion6_static_init_finalize(&g_f)) {
			g_init_started = 1;
		} else {
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

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	uint64_t now_us = platform_time_us();
	if (g_last_gyro_us == 0) {
		g_last_gyro_us = now_us;
		return;
	}
	double dt = (double)(now_us - g_last_gyro_us) * 1e-6;
	g_last_gyro_us = now_us;
	if (dt < 0.0001 || dt > 0.02) return;

	fusion6_predict(&g_f, g_last_gyro_body, g_last_accel_body, dt);
}

static void on_accel(uint8_t *data, size_t size) {
	if (size < 12) return;
	float raw[3];
	memcpy(raw, data, 12);
	map_accel(raw, g_last_accel_body);

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) {
		if (!g_init_started) static_init_step();
		return;
	}

	fusion6_update_accel(&g_f, g_last_accel_body);
	publish_state();
}

static void on_compass(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	vector3d_t *m = (vector3d_t *)data;
	map_mag(m, g_last_mag_meas);

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;
	update_mag_diagnostics();
	fusion6_update_mag_heading(&g_f, g_last_mag_heading);
}

/* ============================================================
 * Optical-flow update (DOWN-pointing camera only).
 *
 * The UP camera shares this topic but has no range finder, so its samples
 * carry z=0 and would always fail the range gate. To keep things explicit
 * we drop UP frames at the top — also avoids burning cycles on a frame we
 * never intend to fuse.
 *
 * Pinhole geometry (downward looking at static surface at range r):
 *   flow_rate_x_image ≈ vx_body / r  −  ω_y_body
 *   flow_rate_y_image ≈ vy_body / r  +  ω_x_body
 * Inverting:
 *   vx_body = r · (flow_rate_x + ω_y)
 *   vy_body = r · (flow_rate_y − ω_x)
 *
 * Sign convention is to be confirmed on bench (cart slide test) — flip the
 * wy / wx signs here if estimator velocity moves opposite to truth.
 *
 * ZUPT branch: when the rangefinder is below the gate, fuse v_body=(0,0)
 * instead of skipping. See OPTFLOW_ZUPT_RANGE_MAX_MM.
 * ============================================================ */
static void on_optflow(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t msg;
	memcpy(&msg, data, sizeof(msg));
	if (msg.direction != OPTFLOW_DOWNWARD) return;   /* UP has no range — skip */
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	/* Always cache the raw clarity + range as reported by the camera so the
	 * VEL_FUSION viewer can tell "camera silent" from "camera publishing but
	 * gated out". This MUST stay above all gate-and-return paths below
	 * (dt_us, range, clarity, outlier) — otherwise the diagnostic strips
	 * freeze on every gated frame. v_meas, by contrast, is invalidated to
	 * NaN whenever we don't fuse, so the viewer leaves a gap instead of
	 * holding a stale value that would look like a fresh measurement. */
	g_last_optflow_clarity = msg.clarity;
	g_last_optflow_range_m = (double)msg.z * 1e-3;   /* mm → m */
	g_last_optflow_v[0] = NAN;
	g_last_optflow_v[1] = NAN;

	if (msg.dt_us == 0) return;   /* FC-side dt outside [10ms, 500ms] sanity band */

	int32_t z_mm = (int32_t)msg.z;
	if (!lidar_valid_mm(z_mm, OPTFLOW_RANGE_MIN_MM, OPTFLOW_RANGE_MAX_MM)) {
		/* Below the optflow gate — try a ZUPT. The VL53L1X reports z=0
		 * when the target is closer than its minimum range (camera
		 * resting on / very close to the ground), so we accept z in
		 * [0, 200) as "close to the ground" and pin the horizontal
		 * velocity to zero. */
		if (z_mm >= 0 && z_mm < OPTFLOW_ZUPT_RANGE_MAX_MM) {
			double v_zero[2] = { 0.0, 0.0 };
			fusion6_update_velocity_xy_body(&g_f, v_zero);
			g_last_optflow_v[0] = 0.0;
			g_last_optflow_v[1] = 0.0;
		}
		return;
	}
	double range_m = lidar_mm_to_m(z_mm);

	if (msg.clarity < OPTFLOW_CLARITY_MIN) return;   /* below clarity floor — skip */

	double flow_x = optflow_to_rad_per_s(msg.dx, msg.dt_us, /*upward=*/0);
	double flow_y = optflow_to_rad_per_s(msg.dy, msg.dt_us, /*upward=*/0);

	/* Derotate using the most recent body angular rate. */
	double wx = g_last_gyro_body[0];
	double wy = g_last_gyro_body[1];
	double flow_lin_x = flow_x + wy;
	double flow_lin_y = flow_y - wx;

	/* Translate to body horizontal velocity (m/s). All inputs above are
	 * finite by construction (dt_us bounded by FC bridge, range bounded by
	 * lidar gate, gyro bounded by IMU full-scale), so no NaN/Inf gate is
	 * needed here — only the physical-plausibility outlier reject. */
	double v_body_x = range_m * flow_lin_x;
	double v_body_y = range_m * flow_lin_y;

	if (fmax(fabs(v_body_x), fabs(v_body_y)) > OPTFLOW_VEL_MAX_MPS) return;

	g_last_optflow_v[0] = v_body_x;
	g_last_optflow_v[1] = v_body_y;

	double v_xy[2] = { v_body_x, v_body_y };
	fusion6_update_velocity_xy_body(&g_f, v_xy);
}

/* ============================================================
 * Publish helpers
 * ============================================================ */

static void publish_state(void) {
	if (!g_init_started) return;

	fusion6_state_t s;
	fusion6_get_state(&g_f, &s);

	nav_state_t out;
	memset(&out, 0, sizeof(out));
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
	out.baro_bias   = 0.0;          /* baro not wired yet */
	out.trace_P_pos = s.trace_P_pos;
	out.trace_P_vel = s.trace_P_vel;
	out.trace_P_att = s.trace_P_att;
	out.health_flags = s.health_flags;
	out.init_done   = (s.health_flags & FUSION6_HF_INIT_DONE) ? 1 : 0;
	out._pad        = 0;

	publish(STATE_UPDATE, (uint8_t *)&out, sizeof(out));
}

/* ============================================================
 * Attitude-debug log stream (LOG_CLASS_ATTITUDE = 0x03)
 *
 * 9-float / 36-byte payload @ 50 Hz, consumed by tools/attitude_view.py.
 *   float[0..2] = accel_meas_body
 *   float[3..5] = gravity_pred_body  (from fused quaternion only)
 *   float[6..8] = accel_linear_body  (= accel_meas - gravity_pred_body)
 * ============================================================ */
static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_log_class = data[0];
}

static void on_attitude_log_50hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_ATTITUDE) return;
	if (!g_init_started) return;

	fusion6_state_t s;
	fusion6_get_state(&g_f, &s);

	vector3d_t a_meas;
	vector3d_init(&a_meas,
	              g_last_accel_body[0],
	              g_last_accel_body[1],
	              g_last_accel_body[2]);
	vector3d_t grav_pred;
	quat_to_accel(&grav_pred, &s.q, (float)GRAVITY_MSS_LOCAL);
	vector3d_t linear_accel;
	vector3d_sub(&linear_accel, &a_meas, &grav_pred);

	float payload[9];
	payload[0] = (float)a_meas.x;
	payload[1] = (float)a_meas.y;
	payload[2] = (float)a_meas.z;
	payload[3] = (float)grav_pred.x;
	payload[4] = (float)grav_pred.y;
	payload[5] = (float)grav_pred.z;
	payload[6] = (float)linear_accel.x;
	payload[7] = (float)linear_accel.y;
	payload[8] = (float)linear_accel.z;

	publish(SEND_LOG, (uint8_t *)payload, sizeof(payload));
}

/* ============================================================
 * Magnetometer diagnostic log stream (LOG_CLASS_MAG_FUSION = 0x1F)
 *
 * 7-float / 28-byte payload @ 25 Hz, consumed by tools/mag_diagnostic_view.py.
 *   float[0..2]   = m_meas (body NED, unit)
 *   float[3..5]   = roll, pitch, yaw  (deg)
 *   float[6]      = mag_heading_deg (decl-corrected, [-180, 180])
 * ============================================================ */
static void on_mag_fusion_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_MAG_FUSION) return;
	if (!g_init_started) return;

	fusion6_state_t s;
	fusion6_get_state(&g_f, &s);

	float payload[7];
	payload[0] = (float)g_last_mag_meas[0];
	payload[1] = (float)g_last_mag_meas[1];
	payload[2] = (float)g_last_mag_meas[2];
	payload[3] = (float)(s.euler.x * (double)RAD2DEG);
	payload[4] = (float)(s.euler.y * (double)RAD2DEG);
	payload[5] = (float)(s.euler.z * (double)RAD2DEG);
	payload[6] = (float)(g_last_mag_heading * (double)RAD2DEG);

	publish(SEND_LOG, (uint8_t *)payload, sizeof(payload));
}

/* ============================================================
 * Optflow velocity fusion log stream (LOG_CLASS_VEL_FUSION = 0x20)
 *
 * 6-float / 24-byte payload @ 25 Hz, consumed by tools/optflow_velocity_view.py.
 *   float[0..1] = v_meas DOWN  (vx, vy, body, m/s)
 *   float[2..3] = v_pred       (vx, vy from current ESKF, body, m/s)
 *   float[4]    = clarity DOWN
 *   float[5]    = range DOWN (m)
 * Cached values are zero until a sample passes the gates. Only the
 * downward-pointing camera is fused (UP has no range finder).
 * ============================================================ */
static void on_vel_fusion_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_VEL_FUSION) return;
	if (!g_init_started) return;

	/* Predicted body velocity from current ESKF state: v_body = R^T · v_ned. */
	fusion6_state_t s;
	fusion6_get_state(&g_f, &s);
	vector3d_t v_ned = s.vel;
	vector3d_t v_body_pred;
	matrix_t R; quat_to_rot_matrix(&R, &s.q);
	matrix_mult_transpose_vec3(&v_body_pred, &R, &v_ned);

	float payload[6];
	payload[0] = (float)g_last_optflow_v[0];
	payload[1] = (float)g_last_optflow_v[1];
	payload[2] = (float)v_body_pred.x;
	payload[3] = (float)v_body_pred.y;
	payload[4] = (float)g_last_optflow_clarity;
	payload[5] = (float)g_last_optflow_range_m;

	publish(SEND_LOG, (uint8_t *)payload, sizeof(payload));
}

/* ============================================================
 * Setup
 * ============================================================ */

void state_estimation_setup(void) {
	fusion6_config_t cfg;
	fusion6_config_default(&cfg);
	fusion6_init(&g_f, &cfg);

	subscribe(SENSOR_IMU1_GYRO_FILTERED_UPDATE, on_gyro);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, on_accel);
	subscribe(SENSOR_COMPASS, on_compass);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, on_optflow);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_50HZ, on_attitude_log_50hz);
	subscribe(SCHEDULER_25HZ, on_mag_fusion_log_25hz);
	subscribe(SCHEDULER_25HZ, on_vel_fusion_log_25hz);
}
