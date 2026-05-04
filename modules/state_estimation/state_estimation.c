/**
 * STATE_ESTIMATION — glue around fusion6.
 *
 * Sensor flow:
 *   IMU gyro  (1 kHz, post-notch) → on_gyro  → fusion6_predict
 *                                            → publish STATE_UPDATE @ 1 kHz
 *   IMU accel (500 Hz)           → on_accel → fusion6_update_accel
 *   Compass   (25 Hz, unit vec)  → on_compass → mag_axis_map
 *                                             → mag_heading (tilt-comp,
 *                                                decl-corrected)
 *                                             → fusion6_update_mag_heading
 *                                                (1-D yaw pseudo-meas)
 *                                             → diagnostic stream
 *   Optflow   (~25 Hz, DOWN only) → on_optflow → range·flow → v_body_xy
 *                                             → fusion6_update_velocity_xy_body
 *                                             → diagnostic stream
 *                                             (gyro derotation TBD — see
 *                                              on_optflow comment block)
 *   Baro      (25 Hz, mm above startup) → on_baro → mm→m + sign flip (NED)
 *                                                 → fusion6_update_baro_z
 *                                                   (1-D scalar on p.z)
 *   GPS pos   (NAV-PVT, ~5 Hz)  → on_gps_position → lazy-init local NED
 *                                                    origin → equirect LLA→NED
 *                                                 → fusion6_update_pos_ned
 *                                                   (3 sequential scalars
 *                                                    on δp[0..2])
 *   GPS vel   (NAV-PVT, ~5 Hz)  → on_gps_velocity → mm/s → m/s NED
 *                                                 → fusion6_update_vel_ned
 *                                                   (3 sequential scalars
 *                                                    on δv[3..5])
 *   GPS qual  (NAV-PVT)        → on_gps_quality  → gate pos+vel updates by
 *                                                  fix_type ≥ 3 AND
 *                                                  num_sv ≥ MIN AND
 *                                                  h_acc / v_acc finite
 *
 * Magnetometer feeds yaw only (1-D, decl-corrected heading). Roll/pitch are
 * still driven by the gravity update; mag does NOT touch the horizontal
 * components of attitude. Optical flow feeds horizontal body velocity (vx,vy)
 * and indirectly tightens roll/pitch through the cross-coupling. Vertical
 * position is observed by the barometer (1-D update on p.z); vertical
 * velocity is still inferred indirectly through the position innovation +
 * accel integration.
 *
 * The UP-pointing camera publishes on the same EXTERNAL_SENSOR_OPTFLOW topic
 * but is intentionally ignored here: it has no range finder so there is no
 * way to convert its angular flow to body-frame m/s. UP frames are silently
 * dropped at the top of on_optflow().
 *
 * Telemetry:
 *   STATE_UPDATE          (nav_state_t, 1 kHz) — for downstream control
 *   LOG_CLASS_ATTITUDE    (50 Hz, 9×float)     — tools/attitude_view.py
 *   LOG_CLASS_MAG_FUSION  (25 Hz, 7×float)     — mag diagnostics
 *   LOG_CLASS_BARO_FUSION (25 Hz, 4×float)     — baro altitude diagnostics
 *   LOG_CLASS_OPTFLOW       (25 Hz, 8×float)     — raw flow / v_meas / v_pred / range / clarity
 *   LOG_CLASS_GPS_FUSION  (5 Hz, 14×float)     — gps pos/vel vs ESKF p/v + quality
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
#include <matrix.h>
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

/* --- Barometer ---
 * air_pressure module publishes a `double` payload at ~25 Hz containing
 *   1000 × (altitude_m − startup_baseline_m)  (positive = up)
 * i.e. millimetres above the power-on altitude. The 100-sample warmup in
 * air_pressure.c means we only start receiving frames after baseline is
 * established, so no extra warmup gate is needed here.
 *
 * We convert mm → m, then negate to get NED p.z (NED +Z = Down). A wide
 * sanity gate rejects DPS310 read glitches (single-frame jumps of many
 * tens of metres). R_baro is configured inside fusion6 (cfg.R_baro).
 *
/* Lidar / baro hand-off:
 * Rotor downwash compresses static pressure under the airframe near the
 * ground, biasing the baro low (apparent altitude high) by 0.5..2 m
 * depending on throttle and prop diameter. When the downward rangefinder
 * reports a valid AGL within (LIDAR_REGIME_MIN_M, LIDAR_REGIME_MAX_M] we
 * fuse the lidar directly into p.z (fusion6_update_lidar_z) and skip the
 * baro update for the duration of the lidar's freshness window. Outside
 * the regime, baro is fused normally.
 *
 * LIDAR_REGIME_MAX_M = 1.0 m matches the original ground-effect gate;
 * raise it when a longer-range lidar (e.g. TFmini-S at 12 m) is fitted.
 * LIDAR_REGIME_MIN_M = 0.05 m guards against the VL53L1X dead-zone (and
 * the producer's bogus 1 mm "in dead zone" sentinel). */
#define BARO_SANITY_GATE_M       50.0
#define LIDAR_REGIME_MIN_M       0.05
#define LIDAR_REGIME_MAX_M       1.0
#define BARO_RANGE_FRESH_US      500000ULL   /* 0.5 s */

/* --- GPS gates ---
 * Reject fixes that are obviously degraded. h_acc/v_acc come from
 * gps_quality_t in millimetres; we convert to metres before comparing.
 * Position sanity gate is wide because the equirectangular projection
 * preserves accuracy to <1% out to ~10 km, and we don't want to lock
 * GPS out after a brief glitch — the per-axis innovation gate inside
 * fusion6 (via R) does the fine-grained noise rejection. */
#define GPS_MIN_NUM_SV           6
#define GPS_MIN_FIX_TYPE         3       /* 3D fix */
#define GPS_MAX_H_ACC_M          15.0
#define GPS_MAX_V_ACC_M          25.0
#define GPS_POS_SANITY_GATE_M    1000.0  /* skip a single frame if it claims
                                            we teleported >1 km from current
                                            estimate — typical sign of a
                                            corrupted NAV-PVT */
#define GPS_QUALITY_FRESH_US     2000000ULL   /* 2 s */

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
static uint64_t g_last_optflow_range_us = 0;       /* 0 = no frame yet */
/* Latest optflow rate snapshot for the LOG_CLASS_OPTFLOW stream.
 * Raw angular flow rate in rad/s, body frame, before any derotation.
 * NaN until a frame with a valid dt_us has been seen. */
static double   g_last_optflow_flow_raw[2]   = {NAN, NAN};

/* Latest baro snapshot (NED metres on p.z, after sign flip + unit convert). */
static double   g_last_baro_z_ned_m    = 0.0;
static int      g_have_baro            = 0;    /* set on first accepted frame */

/* Lidar (downward range-finder) fusion state.
 * Lidar measures slant AGL (m) along the body z-axis. We fuse as p.z by
 * pinning a one-shot "lidar origin" (NED p.z value at first valid
 * sample): origin_z = p.z + AGL_tilt. Subsequent samples become
 * z_ned_meas = origin_z - AGL_tilt. Origin is recaptured if the lidar
 * has been gated out for longer than LIDAR_ORIGIN_GAP_US so that a
 * takeoff/land at a different ground height doesn't carry a stale
 * offset. */
static int      g_lidar_origin_set     = 0;
static double   g_lidar_origin_z_ned   = 0.0;
static uint64_t g_last_lidar_fuse_us   = 0;
#define LIDAR_ORIGIN_GAP_US      2000000ULL   /* 2 s */

/* GPS local NED origin (lazy-initialised on first usable fix). */
static int      g_gps_origin_set       = 0;
static int32_t  g_gps_origin_lat_e7    = 0;
static int32_t  g_gps_origin_lon_e7    = 0;
static int32_t  g_gps_origin_alt_mm    = 0;

/* GPS quality gate state. g_gps_quality_ok is the boolean OR of all
 * acceptance conditions — checked by both pos and vel handlers. */
static int      g_gps_quality_ok       = 0;
static uint64_t g_gps_quality_us       = 0;     /* time of last quality frame */

/* Latest GPS snapshots for telemetry / debugging.
 * Initialised to NaN so the GPS-fusion viewer can distinguish "no fix yet"
 * from "fix at NED origin" (which is a legitimate measurement of 0,0,0). */
static double   g_last_gps_pos_ned[3]  = {NAN, NAN, NAN};
static double   g_last_gps_vel_ned[3]  = {NAN, NAN, NAN};
static uint8_t  g_last_gps_num_sv      = 0;
static uint8_t  g_last_gps_fix_type    = 0;

static uint8_t  g_log_class      = 0;

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
	out.baro_bias   = 0.0;          /* baro fused without an explicit bias
	                                   state; air_pressure.c zeros the offset
	                                   at startup which suffices for now. */
	out.trace_P_pos = s.trace_P_pos;
	out.trace_P_vel = s.trace_P_vel;
	out.trace_P_att = s.trace_P_att;
	out.health_flags = s.health_flags;
	out.init_done   = (s.health_flags & FUSION6_HF_INIT_DONE) ? 1 : 0;
	out._pad        = 0;

	publish(STATE_UPDATE, (uint8_t *)&out, sizeof(out));
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
	publish_state();
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
}

static void on_compass(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	vector3d_t *m = (vector3d_t *)data;
	map_mag(m, g_last_mag_meas);

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	/* Tilt-compensated magnetic heading. The body-yaw-aligned level-frame
	 * mag vector m_lvl = Rz(-yaw) · R(q) · m_body has roll & pitch removed,
	 * so atan2(-m_lvl.y, m_lvl.x) gives the magnetic heading. */
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
 * Inverting (no derotation — pure-translation approximation):
 *   vx_body ≈ r · flow_rate_x
 *   vy_body ≈ r · flow_rate_y
 *
 * Gyro derotation is intentionally OFF for now: bench tests showed the
 * camera barely tracks rotation at the current clarity, so subtracting
 * ω just synthesised phantom velocity. Will be revisited.
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
	 * OPTFLOW_DEROT viewer can tell "camera silent" from "camera publishing
	 * but gated out". This MUST stay above all gate-and-return paths below
	 * (dt_us, range, clarity, outlier) — otherwise the diagnostic strips
	 * freeze on every gated frame. v_meas, by contrast, is invalidated to
	 * NaN whenever we don't fuse, so the viewer leaves a gap instead of
	 * holding a stale value that would look like a fresh measurement. */
	g_last_optflow_clarity = msg.clarity;
	g_last_optflow_range_us = platform_time_us();
	g_last_optflow_range_m = (double)msg.z * 1e-3;   /* mm → m */
	g_last_optflow_v[0] = NAN;
	g_last_optflow_v[1] = NAN;

	if (msg.dt_us == 0) return;   /* FC-side dt outside [10ms, 500ms] sanity band */

	int32_t z_mm = (int32_t)msg.z;
	if (z_mm <= 0 || z_mm < OPTFLOW_RANGE_MIN_MM) {
		/* Below the optflow gate. Two cases:
		 *   z_mm > 0  but < MIN  → sensor sees ground, very close: ZUPT
		 *                          (camera resting / about-to-touch).
		 *   z_mm <= 0           → sensor reported no valid range (out of
		 *                          range, signal fail, etc.). DO NOT ZUPT —
		 *                          we have no idea how high we are; pinning
		 *                          v_xy=0 would be dangerous if the craft is
		 *                          actually drifting at altitude. Skip. */
		if (z_mm > 0 && z_mm < OPTFLOW_ZUPT_RANGE_MAX_MM) {
			double v_zero[2] = { 0.0, 0.0 };
			fusion6_update_velocity_xy_body(&g_f, v_zero);
			g_last_optflow_v[0] = 0.0;
			g_last_optflow_v[1] = 0.0;
		}
		return;
	}
	/* No upper-range gate: above OPTFLOW_RANGE_MAX_MM the lidar may be
	 * less accurate but the camera flow rate is still informative; we
	 * accept whatever range the sensor reports and let the increased
	 * R_vel_xy_body absorb the extra noise. */
	double range_m = lidar_mm_to_m(z_mm);

	/* Lidar fusion (vertical p.z) — runs independently of clarity / flow,
	 * since clarity gates the optical-flow velocity update only and has
	 * nothing to do with the time-of-flight range measurement. We fuse
	 * any AGL inside (LIDAR_REGIME_MIN_M, LIDAR_REGIME_MAX_M]; outside
	 * that band the baro path owns p.z. Tilt-compensate by projecting
	 * the slant range onto the local vertical with cos(roll)*cos(pitch)
	 * — adequate for the ±15° envelope where the lidar regime is
	 * useful in the first place. */
	if (range_m > LIDAR_REGIME_MIN_M && range_m <= LIDAR_REGIME_MAX_M) {
		vector3d_t eul_l;
		quat_to_euler(&eul_l, &g_f.q);
		double tilt = cos(eul_l.x) * cos(eul_l.y);
		if (tilt < 0.5) tilt = 0.5;   /* don't divide-by-near-zero on extreme tilt */
		double agl_m = range_m * tilt;

		uint64_t now_us = platform_time_us();
		if (!g_lidar_origin_set ||
		    (g_last_lidar_fuse_us != 0 &&
		     (now_us - g_last_lidar_fuse_us) > LIDAR_ORIGIN_GAP_US)) {
			/* (Re)pin the lidar origin so the first sample after a gap
			 * doesn't yank p.z by a meter. */
			g_lidar_origin_z_ned = g_f.p.z + agl_m;
			g_lidar_origin_set = 1;
		}
		double z_ned_meas = g_lidar_origin_z_ned - agl_m;
		fusion6_update_lidar_z(&g_f, z_ned_meas);
		g_last_lidar_fuse_us = now_us;
	}

	if (msg.clarity < OPTFLOW_CLARITY_MIN) return;   /* below clarity floor — skip */

	double flow_x = optflow_to_rad_per_s(msg.dx, msg.dt_us, /*upward=*/0);
	double flow_y = optflow_to_rad_per_s(msg.dy, msg.dt_us, /*upward=*/0);

	/* Cache raw flow rate for the OPTFLOW diagnostic stream. Updated even
	 * when the frame is later dropped by the outlier gate so the viewer
	 * keeps showing the camera's live output. */
	g_last_optflow_flow_raw[0] = flow_x;
	g_last_optflow_flow_raw[1] = flow_y;

	/* Translate to body horizontal velocity (m/s). All inputs above are
	 * finite by construction (dt_us bounded by FC bridge, range bounded by
	 * lidar gate), so no NaN/Inf gate is needed here — only the
	 * physical-plausibility outlier reject. */
	double v_body_x = range_m * flow_x;
	double v_body_y = range_m * flow_y;

	if (fmax(fabs(v_body_x), fabs(v_body_y)) > OPTFLOW_VEL_MAX_MPS) return;

	g_last_optflow_v[0] = v_body_x;
	g_last_optflow_v[1] = v_body_y;

	double v_xy[2] = { v_body_x, v_body_y };
	fusion6_update_velocity_xy_body(&g_f, v_xy);
}

/* ============================================================
 * Barometric altitude update.
 *
 * Payload: double, millimetres above startup baseline, +up.
 * Convert to NED metres on p.z (negate, then mm→m), sanity-gate, then
 * fuse via the 1-D scalar update. fusion6 owns R_baro.
 * ============================================================ */
static void on_baro(uint8_t *data, size_t size) {
	if (size < sizeof(double)) return;
	double baro_mm_up;
	memcpy(&baro_mm_up, data, sizeof(double));
	if (!isfinite(baro_mm_up)) return;

	double z_ned_m = -baro_mm_up * 1e-3;   /* +up mm → +down m */

	g_last_baro_z_ned_m = z_ned_m;

	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;

	/* First baro frame after INIT_DONE: snap p.z (and zero v.z) to the
	 * measurement. Without this, accel-only integration between INIT_DONE
	 * (~0.25 s after boot) and the first baro frame (~4 s after boot, due
	 * to the air_pressure 100-sample warmup) can walk p.z past the sanity
	 * gate, locking baro out forever. */
	if (!g_have_baro) {
		g_f.p.z = z_ned_m;
		g_f.v.z = 0.0;
		g_f.health_flags |= FUSION6_HF_BARO_OK;
		g_have_baro = 1;
		return;
	}

	if (fabs(z_ned_m - g_f.p.z) > BARO_SANITY_GATE_M) return;

	/* Lidar / baro hand-off: when the lidar has fused recently, it owns
	 * p.z and we skip baro to avoid the ground-effect bias (rotor
	 * downwash compresses static pressure under the airframe). The
	 * freshness check uses BARO_RANGE_FRESH_US so a stale lidar cache
	 * cannot lock baro out forever after the lidar saturates. */
	if (g_last_lidar_fuse_us != 0 &&
	    (platform_time_us() - g_last_lidar_fuse_us) < BARO_RANGE_FRESH_US) {
		return;
	}

	fusion6_update_baro_z(&g_f, z_ned_m);
}

/* ============================================================
 * GPS — quality, position, velocity.
 *
 * gps module publishes three topics from each NAV-PVT solution:
 *   EXTERNAL_SENSOR_GPS_QUALITY (gps_quality_t)  — fix type, satellite
 *                                                  count, accuracy
 *                                                  estimates.
 *   EXTERNAL_SENSOR_GPS         (gps_position_t) — lat/lon (deg×1e7),
 *                                                  altitude (mm MSL).
 *   EXTERNAL_SENSOR_GPS_VELOC   (gps_velocity_t) — N/E/D velocity (mm/s).
 *
 * Quality is OR-gated against fix type, satellite count and accuracy
 * estimates. The flag stays valid for GPS_QUALITY_FRESH_US so a single
 * dropped quality frame doesn't lock GPS out instantly. The first
 * accepted position frame seeds the local NED origin (so subsequent
 * position fixes are reported relative to "where we got our first
 * fix"). Vertical position is intentionally fed through GPS as well as
 * baro — the per-axis R values let baro dominate vertical (R_baro=0.25
 * vs R_gps_pos_d=25.0) while GPS still corrects for long-term baro
 * drift. fusion6 owns R_gps_*.
 * ============================================================ */
static void on_gps_quality(uint8_t *data, size_t size) {
	if (size < sizeof(gps_quality_t)) return;
	gps_quality_t q;
	memcpy(&q, data, sizeof(q));

	g_gps_quality_us = platform_time_us();
	g_gps_quality_ok = (q.fix_type >= GPS_MIN_FIX_TYPE)
	                && (q.num_sv   >= GPS_MIN_NUM_SV)
	                && (q.h_acc * 1e-3 <= GPS_MAX_H_ACC_M)
	                && (q.v_acc * 1e-3 <= GPS_MAX_V_ACC_M)
	                && (q.reliable != 0);
	g_last_gps_num_sv   = q.num_sv;
	g_last_gps_fix_type = q.fix_type;
}

static int gps_quality_fresh_and_ok(void) {
	if (!g_gps_quality_ok) return 0;
	if (g_gps_quality_us == 0) return 0;
	return (platform_time_us() - g_gps_quality_us) < GPS_QUALITY_FRESH_US;
}

static void on_gps_position(uint8_t *data, size_t size) {
	if (size < sizeof(gps_position_t)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;
	if (!gps_quality_fresh_and_ok()) return;

	gps_position_t p;
	memcpy(&p, data, sizeof(p));

	gps_origin_capture(&g_gps_origin_set,
	                   &g_gps_origin_lat_e7, &g_gps_origin_lon_e7,
	                   &g_gps_origin_alt_mm,
	                   p.lat, p.lon, p.alt);

	double pos_ned[3];
	gps_lla_to_ned(p.lat, p.lon, p.alt,
	               g_gps_origin_lat_e7, g_gps_origin_lon_e7, g_gps_origin_alt_mm,
	               pos_ned);

	g_last_gps_pos_ned[0] = pos_ned[0];
	g_last_gps_pos_ned[1] = pos_ned[1];
	g_last_gps_pos_ned[2] = pos_ned[2];

	/* Sanity gate: drop a single frame if it claims a >1 km jump from the
	 * current estimate. The fusion gain takes care of small jumps; this
	 * only catches obviously corrupt NAV-PVTs. */
	double dN = pos_ned[0] - g_f.p.x;
	double dE = pos_ned[1] - g_f.p.y;
	double dD = pos_ned[2] - g_f.p.z;
	if (fmax(fmax(fabs(dN), fabs(dE)), fabs(dD)) > GPS_POS_SANITY_GATE_M) return;

	fusion6_update_pos_ned(&g_f, pos_ned);
}

static void on_gps_velocity(uint8_t *data, size_t size) {
	if (size < sizeof(gps_velocity_t)) return;
	if (!(g_f.health_flags & FUSION6_HF_INIT_DONE)) return;
	if (!gps_quality_fresh_and_ok()) return;

	gps_velocity_t v;
	memcpy(&v, data, sizeof(v));

	double vel_ned[3];
	gps_vel_mm_per_s_to_m_per_s(v.velN, v.velE, v.velD, vel_ned);

	g_last_gps_vel_ned[0] = vel_ned[0];
	g_last_gps_vel_ned[1] = vel_ned[1];
	g_last_gps_vel_ned[2] = vel_ned[2];

	fusion6_update_vel_ned(&g_f, vel_ned);
}

/* ============================================================
 * Attitude-debug log stream (LOG_CLASS_ATTITUDE = 0x03)
 *
 * 9-float / 36-byte payload @ 50 Hz, consumed by tools/attitude_view.py.
 *   float[0..2] = accel_meas_body
 *   float[3..5] = gravity_pred_body  (from fused quaternion only)
 *   float[6..8] = accel_linear_body  (= accel_meas - gravity_pred_body)
 * ============================================================ */
static void eskf_pager_reset_all(void);

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_log_class = data[0];
	/* Reset all ESKF pagers so a fresh class starts at row 0 with a new
	 * snapshot — avoids the viewer reassembling rows from a stale prior
	 * snapshot whose seq it would otherwise treat as new. */
	eskf_pager_reset_all();
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
 * Baro fusion log stream (LOG_CLASS_BARO_FUSION = 0x21)
 *
 * 4-float / 16-byte payload @ 25 Hz, consumed by tools/baro_fusion_view.py.
 *   float[0] = z_meas       (NED metres on p.z; +down, after sign flip)
 *   float[1] = p_z          (current ESKF p.z, NED, m)
 *   float[2] = v_z          (current ESKF v.z, NED, m/s)
 *   float[3] = baro_ok      (1.0 if FUSION6_HF_BARO_OK set, else 0.0)
 * Cached values are zero until the first SENSOR_AIR_PRESSURE arrives.
 * ============================================================ */
static void on_baro_fusion_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_BARO_FUSION) return;
	if (!g_init_started) return;

	float payload[4];
	payload[0] = (float)g_last_baro_z_ned_m;
	payload[1] = (float)g_f.p.z;
	payload[2] = (float)g_f.v.z;
	payload[3] = (g_f.health_flags & FUSION6_HF_BARO_OK) ? 1.0f : 0.0f;

	publish(SEND_LOG, (uint8_t *)payload, sizeof(payload));
}

/* ============================================================
 * Optflow log stream (LOG_CLASS_OPTFLOW = 0x22)
 *
 * 8-float / 32-byte payload @ 25 Hz, consumed by tools/optflow_view.py.
 *   float[0..1] = flow_raw   (rad/s, body, no derotation)
 *   float[2..3] = v_meas     (m/s,  body, = flow_raw · range, what we fuse)
 *   float[4..5] = v_pred     (m/s,  body, current ESKF prediction)
 *   float[6]    = range      (m,    downward lidar)
 *   float[7]    = clarity    (camera quality metric, 0..~100)
 * Flow / v_meas are NaN until on_optflow has accepted a frame.
 * ============================================================ */
static void on_optflow_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_OPTFLOW) return;
	if (!g_init_started) return;

	/* Predicted body velocity from current ESKF state: v_body = R^T · v_ned. */
	fusion6_state_t s;
	fusion6_get_state(&g_f, &s);
	vector3d_t v_ned = s.vel;
	vector3d_t v_body_pred;
	matrix_t R; quat_to_rot_matrix(&R, &s.q);
	matrix_mult_transpose_vec3(&v_body_pred, &R, &v_ned);

	float payload[8];
	payload[0] = (float)g_last_optflow_flow_raw[0];
	payload[1] = (float)g_last_optflow_flow_raw[1];
	payload[2] = (float)g_last_optflow_v[0];
	payload[3] = (float)g_last_optflow_v[1];
	payload[4] = (float)v_body_pred.x;
	payload[5] = (float)v_body_pred.y;
	payload[6] = (float)g_last_optflow_range_m;
	payload[7] = (float)g_last_optflow_clarity;

	publish(SEND_LOG, (uint8_t *)payload, sizeof(payload));
}

/* ============================================================
 * GPS fusion log stream (LOG_CLASS_GPS_FUSION = 0x23)
 *
 * 14-float / 56-byte payload @ 5 Hz, consumed by tools/gps_fusion_view.py.
 * 5 Hz matches the typical NAV-PVT rate so each frame represents one GPS
 * tick of evidence; the GPS snapshots are NaN until the first accepted
 * fix arrives, letting the viewer keep a blank track until then.
 *   float[0..2]  = gps_pos_ned   (m,   NED, lazy-origin-relative)
 *   float[3..5]  = gps_vel_ned   (m/s, NED)
 *   float[6..8]  = eskf_p        (m,   NED, current state)
 *   float[9..11] = eskf_v        (m/s, NED)
 *   float[12]    = gps_ok        (1.0 if FUSION6_HF_GPS_OK set, else 0.0)
 *   float[13]    = num_sv        (most recent satellite count, as float)
 * ============================================================ */
static void on_gps_fusion_log_5hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_GPS_FUSION) return;
	if (!g_init_started) return;

	float payload[14];
	payload[0]  = (float)g_last_gps_pos_ned[0];
	payload[1]  = (float)g_last_gps_pos_ned[1];
	payload[2]  = (float)g_last_gps_pos_ned[2];
	payload[3]  = (float)g_last_gps_vel_ned[0];
	payload[4]  = (float)g_last_gps_vel_ned[1];
	payload[5]  = (float)g_last_gps_vel_ned[2];
	payload[6]  = (float)g_f.p.x;
	payload[7]  = (float)g_f.p.y;
	payload[8]  = (float)g_f.p.z;
	payload[9]  = (float)g_f.v.x;
	payload[10] = (float)g_f.v.y;
	payload[11] = (float)g_f.v.z;
	payload[12] = (g_f.health_flags & FUSION6_HF_GPS_OK) ? 1.0f : 0.0f;
	payload[13] = (float)g_last_gps_num_sv;

	publish(SEND_LOG, (uint8_t *)payload, sizeof(payload));
}

/* ============================================================
 * ESKF matrix introspection log streams (LOG_CLASS_ESKF_{P,F,K,H})
 *
 * Streams a single matrix at a time (the active LOG_CLASS) **paged**
 * one row per scheduler tick at 25 Hz. A 15×15 matrix completes in
 * 15·40 ms = 600 ms (~1.7 snapshot/s); K/H complete faster (m≤3 rows
 * for K, 1 row for H). Pacing is required because the dblink TX queue
 * is only 8 slots deep — bursting all 15 rows at 1 Hz overran it and
 * the Python side never saw a full snapshot.
 *
 * Per-row payload layout (12 B header + 4·cols B data, max 12+60 = 72 B):
 *   uint8  matrix_id       0=P, 1=F, 2=K, 3=H
 *   uint8  rows            total rows in matrix
 *   uint8  cols            total cols in matrix
 *   uint8  row_idx         which row this frame contains (0..rows-1)
 *   uint8  update_type     fusion6_update_id_t (K/H only); 0xFF for P/F
 *   uint8  pad             reserved (0)
 *   uint16 seq_le          monotonic per-class snapshot id
 *   uint32 t_ms_le         platform_time_ms() at snapshot start
 *   float32 row[cols]      row data, little-endian, row-major
 *
 * Read-only: pulls fusion6_get_matrix() snapshots; does not mutate filter
 * state. A fresh matrix_copy is taken on row 0 of each snapshot so the
 * paged rows of one snapshot are mutually consistent even if the filter
 * mutates between row emissions.
 * ============================================================ */
typedef struct {
	matrix_t snap;
	uint16_t seq;
	uint32_t t_ms;
	uint8_t  rows;
	uint8_t  cols;
	uint8_t  row_idx;       /* next row to emit; 0 ⇒ take fresh snapshot */
	uint8_t  update_type;
} eskf_pager_t;

static uint16_t g_eskf_seq[4] = { 0, 0, 0, 0 };
static eskf_pager_t g_eskf_pager[4];

static void eskf_pager_reset_all(void) {
	for (int i = 0; i < 4; i++) g_eskf_pager[i].row_idx = 0;
}

static void eskf_pager_tick(fusion6_matrix_id_t which, uint8_t class_idx) {
	eskf_pager_t *p = &g_eskf_pager[class_idx];

	if (p->row_idx == 0) {
		/* Start a new snapshot. */
		uint8_t ut = FUSION6_UPDATE_NONE;
		if (!fusion6_get_matrix(&g_f, which, &p->snap, &ut)) return;
		if (p->snap.rows <= 0 || p->snap.cols <= 0) return;
		if (p->snap.rows > 15 || p->snap.cols > 15) return;
		p->rows = (uint8_t)p->snap.rows;
		p->cols = (uint8_t)p->snap.cols;
		p->update_type = ut;
		p->seq  = ++g_eskf_seq[class_idx];
		p->t_ms = platform_time_ms();
	}

	/* Emit one row. */
	uint8_t buf[12 + 4 * 15];
	buf[0] = (uint8_t)which;
	buf[1] = p->rows;
	buf[2] = p->cols;
	buf[3] = p->row_idx;
	buf[4] = p->update_type;
	buf[5] = 0;
	memcpy(&buf[6], &p->seq,  sizeof(uint16_t));
	memcpy(&buf[8], &p->t_ms, sizeof(uint32_t));

	float *row = (float *)&buf[12];
	for (int c = 0; c < p->cols; c++) {
		row[c] = (float)p->snap.data[p->row_idx][c];
	}
	publish(SEND_LOG, buf, 12 + (size_t)p->cols * sizeof(float));

	p->row_idx++;
	if (p->row_idx >= p->rows) p->row_idx = 0;
}

static void on_eskf_P_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_ESKF_P) return;
	if (!g_init_started) return;
	eskf_pager_tick(FUSION6_MATRIX_P, 0);
}

static void on_eskf_F_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_ESKF_F) return;
	if (!g_init_started) return;
	eskf_pager_tick(FUSION6_MATRIX_F, 1);
}

static void on_eskf_K_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_ESKF_K) return;
	if (!g_init_started) return;
	eskf_pager_tick(FUSION6_MATRIX_K, 2);
}

static void on_eskf_H_log_25hz(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_log_class != LOG_CLASS_ESKF_H) return;
	if (!g_init_started) return;
	eskf_pager_tick(FUSION6_MATRIX_H, 3);
}

/* ============================================================
 * Setup
 * ============================================================ */

void state_estimation_setup(void) {
	fusion6_config_t cfg;
	fusion6_config_default(&cfg);
	/* De-weight the optical-flow body-velocity measurement.
	 * Default is 0.01 = (0.10 m/s)² per axis, which is too tight given
	 * (a) no gyro derotation yet — rotation injects phantom velocity
	 *     (~ω·range, e.g. 0.2 rad/s × 1 m = 0.2 m/s of bogus signal), and
	 * (b) clarity often sits in the 10–30 band where the camera output is
	 *     noisy enough that ±0.3 m/s residuals are normal.
	 * Raising to (0.30 m/s)² = 0.09 keeps optflow as a soft anchor without
	 * letting it (or rotation-induced phantoms) dominate the prediction. */
	cfg.R_vel_xy_body = 0.09;
	fusion6_init(&g_f, &cfg);

	subscribe(SENSOR_IMU1_GYRO_FILTERED_UPDATE, on_gyro);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, on_accel);
	subscribe(SENSOR_COMPASS, on_compass);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, on_optflow);
	subscribe(SENSOR_AIR_PRESSURE, on_baro);
	subscribe(EXTERNAL_SENSOR_GPS_QUALITY, on_gps_quality);
	subscribe(EXTERNAL_SENSOR_GPS, on_gps_position);
	subscribe(EXTERNAL_SENSOR_GPS_VELOC, on_gps_velocity);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_50HZ, on_attitude_log_50hz);
	subscribe(SCHEDULER_25HZ, on_mag_fusion_log_25hz);
	subscribe(SCHEDULER_25HZ, on_baro_fusion_log_25hz);
	subscribe(SCHEDULER_25HZ, on_optflow_log_25hz);
	subscribe(SCHEDULER_5HZ,  on_gps_fusion_log_5hz);
	subscribe(SCHEDULER_25HZ, on_eskf_P_log_25hz);
	subscribe(SCHEDULER_25HZ, on_eskf_F_log_25hz);
	subscribe(SCHEDULER_25HZ, on_eskf_K_log_25hz);
	subscribe(SCHEDULER_25HZ, on_eskf_H_log_25hz);
}
