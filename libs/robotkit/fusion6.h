#ifndef FUSION6_H
#define FUSION6_H

#include <stdint.h>
#include "vector3d.h"
#include "quat.h"
#include "matrix.h"

/**
 * FUSION6 — 16-state Error-State Kalman Filter (ESKF) for UAV navigation.
 *
 * State (see FUSION6_ESKF.md §1 for the full math):
 *   Nominal (17 doubles): p, v, q, b_a, b_g, b_baro
 *   Error    (16 doubles): δp, δv, δθ, δb_a, δb_g, δb_baro
 *
 * NED frame throughout: +X=North, +Y=East, +Z=Down.
 * Quaternion convention: [w, x, y, z], identity = [1, 0, 0, 0].
 * Gravity: g_ned = (0, 0, +9.80665) m/s². At rest, accel reads ~(0, 0, −g).
 *
 * Sensors supported (each via its own update function):
 *   - Gyroscope + accelerometer (predict + gravity update)
 *   - Magnetometer (scalar yaw)
 *   - Barometer (altitude − bias)
 *   - Range finder (altitude with terrain≡0)
 *   - Optical flow (nonlinear, derotated)
 *   - GPS position + velocity (NED)
 *
 * All inputs MUST be axis-mapped to body NED before passing in (state_estimation
 * module is responsible for axis mapping). Bias subtraction is internal.
 *
 * Health monitoring is built in: per-sensor staleness, NaN/Inf guards, bias
 * sanity clamping, P-trace runaway detection. See §6 in the .md.
 *
 * Per-sensor enable mask (§9 in .md) lets observers selectively disable any
 * update for divergence isolation; innovations are still computed & exposed.
 */

/* ============================================================
 * Constants & enums
 * ============================================================ */

#define FUSION6_N           16          /* error-state dimension */
#define FUSION6_NUM_SENSORS 7           /* accel, mag, baro, lidar, optflow, gps_pos, gps_vel */

/* === State-rewind ring sizing (FUSION6 §8 — Stage 2) ===
 *  IMU ring at 1 kHz × 500 ms = 500 entries × 56 B = ~28 KB.
 *  Checkpoint ring: 10 entries × ~2.2 KB (mostly 16×16 double P) = ~22 KB.
 *  Total per fusion6_t instance: ~50 KB. STM32H7 has 1 MB SRAM. */
#define FUSION6_IMU_RING_LEN     500
#define FUSION6_CKPT_RING_LEN    10
#define FUSION6_CKPT_INTERVAL_US 50000  /* snapshot every 50 ms */

/* Sensor enable-mask bit positions (also indices into reject_count[] etc.) */
typedef enum {
    FUSION6_SENSOR_ACCEL    = 0,
    FUSION6_SENSOR_MAG      = 1,
    FUSION6_SENSOR_BARO     = 2,
    FUSION6_SENSOR_LIDAR    = 3,
    FUSION6_SENSOR_OPTFLOW  = 4,
    FUSION6_SENSOR_GPS_POS  = 5,
    FUSION6_SENSOR_GPS_VEL  = 6,
    /* Bit 7 reserved */
} fusion6_sensor_bit_e;

/* Health flags (uint16_t). These are exposed to consumers via the state output. */
#define FUSION6_HF_GPS_FRESH         (1u << 0)
#define FUSION6_HF_OPTFLOW_FRESH     (1u << 1)
#define FUSION6_HF_LIDAR_FRESH       (1u << 2)
#define FUSION6_HF_MAG_FRESH         (1u << 3)
#define FUSION6_HF_BARO_FRESH        (1u << 4)
#define FUSION6_HF_GYRO_FRESH        (1u << 5)
#define FUSION6_HF_ACCEL_FRESH       (1u << 6)
#define FUSION6_HF_POSITION_DIVERGED (1u << 7)
#define FUSION6_HF_NAN_RECOVERY      (1u << 8)
#define FUSION6_HF_BIAS_CLAMPED      (1u << 9)
#define FUSION6_HF_INIT_DONE         (1u << 10)

/* ============================================================
 * Configuration struct
 * ============================================================ */

typedef struct {
    /* Process noise PSDs (see §2.6) */
    double sigma_accel;       /* m/s² /√Hz */
    double sigma_gyro;        /* rad/s /√Hz */
    double sigma_bias_accel;  /* m/s³ /√Hz */
    double sigma_bias_gyro;   /* rad/s² /√Hz */
    double sigma_bias_baro;   /* m/s /√Hz */

    /* Measurement noise std-devs (per-sensor base R; latency inflation added on top) */
    double R_accel;           /* m/s² */
    double R_mag_yaw;         /* rad */
    double R_baro;            /* m */
    double R_lidar;           /* m */
    double R_optflow;         /* rad/s */
    double R_gps_pos_xy;      /* m (horizontal); vertical uses 2× */
    double R_gps_vel;         /* m/s */

    /* Per-sensor latency in seconds (for R inflation; Stage-2 rewind in future) */
    double latency_baro_s;
    double latency_lidar_s;
    double latency_optflow_s;
    double latency_gps_s;

    /* Per-sensor staleness timeouts in microseconds */
    uint32_t timeout_gps_us;
    uint32_t timeout_optflow_us;
    uint32_t timeout_lidar_us;
    uint32_t timeout_mag_us;
    uint32_t timeout_baro_us;
    uint32_t timeout_accel_us;
    uint32_t timeout_gyro_us;

    /* Divergence detection */
    double p_runaway_pos_m2;  /* trace(P_pos) above this → position_diverged */

    /* Bias sanity bounds (clamped after every predict) */
    double bias_clamp_gyro;   /* rad/s, default 0.5 */
    double bias_clamp_accel;  /* m/s², default 2.0 */
    double bias_clamp_baro;   /* m,    default 10.0 */

    /* Accel update gating (skip if |‖a‖−g| > this) */
    double accel_mag_gate;    /* m/s², default 1.5 */

    double gravity;           /* m/s², default 9.80665 */
} fusion6_config_t;

/** Populate cfg with project-default values. Caller may override fields after. */
void fusion6_config_default(fusion6_config_t *cfg);

/* ============================================================
 * Output state (snapshot for downstream consumers)
 * ============================================================ */

typedef struct {
    vector3d_t pos;           /* NED, m */
    vector3d_t vel;           /* NED, m/s */
    quaternion_t q;           /* body→earth, [w,x,y,z], float (down-converted at boundary) */
    vector3d_t euler;         /* roll, pitch, yaw — radians (caller can convert to deg) */
    vector3d_t accel_body;    /* linear accel (gravity removed), body frame, m/s² */
    vector3d_t accel_earth;   /* linear accel (gravity removed), earth frame, m/s² (positive-up Z by convention) */
    vector3d_t gyro_bias;     /* rad/s */
    vector3d_t accel_bias;    /* m/s² */
    double baro_bias;         /* m */
    double trace_P_pos;       /* m² */
    double trace_P_vel;       /* m²/s² */
    double trace_P_att;       /* rad² */
    uint16_t health_flags;    /* see FUSION6_HF_* */
    uint32_t reject_count[FUSION6_NUM_SENSORS];
} fusion6_state_t;

/* ============================================================
 * Last-innovation snapshot (per sensor, exposed for observers)
 * ============================================================ */

typedef struct {
    double y[3];          /* innovation (raw residual, up to 3 elements used) */
    double S_diag[3];     /* innovation covariance diagonal */
    double NIS;           /* normalized innovation squared */
    uint8_t dim;          /* 1, 2, or 3 (used elements) */
    uint8_t gate_pass;    /* 1=accepted, 0=rejected by chi² */
    uint8_t disabled;     /* 1=sensor mask bit was clear, update skipped (innovation still computed) */
    uint8_t pad;
} fusion6_innov_t;

/* ============================================================
 * Rewind buffer entries (FUSION6 §8 Stage 2)
 * ============================================================ */

typedef struct {
    double gyro[3];       /* RAW gyro (pre-bias-subtraction) — replay re-subtracts current bias */
    double accel[3];      /* RAW accel (pre-bias-subtraction) */
    double dt;            /* time slice */
    uint32_t now_us;      /* timestamp at end of this predict step */
} fusion6_imu_sample_t;

typedef struct {
    /* Full nominal state */
    vector3d_t p, v;
    double q[4];
    vector3d_t b_a, b_g;
    double b_baro;
    /* Full covariance */
    matrix_t P;
    /* Timestamp at moment of snapshot (= now_us right after the predict that
     * triggered checkpointing). */
    uint32_t now_us;
    /* IMU ring sequence number at snapshot time (samples with seq > this are
     * "after the checkpoint" and need to be replayed). */
    uint64_t imu_seq_at_ckpt;
} fusion6_checkpoint_t;

/* ============================================================
 * Filter state (one instance = one estimator)
 * ============================================================ */

typedef struct {
    /* === Nominal state === */
    vector3d_t p;             /* position NED, m */
    vector3d_t v;             /* velocity NED, m/s */
    quaternion_t q_d_w;       /* quaternion body→earth (DOUBLE quaternion stored as 4 doubles below) */
    double q[4];              /* canonical quaternion [w,x,y,z] in DOUBLE precision (q_d_w is just for boundary down-conversion) */
    vector3d_t b_a;           /* accel bias, body frame, m/s² */
    vector3d_t b_g;           /* gyro bias, body frame, rad/s */
    double b_baro;            /* baro altitude bias, m */

    /* === Covariance (16×16) === */
    matrix_t P;

    /* === IMU caching for midpoint integration === */
    double gyro_prev[3];      /* last bias-corrected gyro, for ω midpoint */
    double accel_cache[3];    /* last bias-corrected accel, used in predict + update */
    int has_prev_gyro;        /* 0 until first predict; bootstrap with current sample */

    /* === Static init accumulators === */
    int init_sample_count;
    double init_accel_sum[3];
    double init_accel_sum_sq[3];
    double init_gyro_sum[3];
    double init_gyro_sum_sq[3];

    /* === Health & timing === */
    uint32_t last_update_us[FUSION6_NUM_SENSORS];
    uint32_t reject_count[FUSION6_NUM_SENSORS];
    uint16_t health_flags;
    uint32_t bias_clamped_until_us;     /* sticky 1 s window timestamp (0 = not set) */

    /* === Sensor enable mask (debug isolation) === */
    uint16_t sensor_enable_mask;

    /* === Last-innovation snapshots (per sensor, for observers) === */
    fusion6_innov_t last_innov[FUSION6_NUM_SENSORS];

    /* === State rewind (FUSION6 §8 Stage 2) === */
    uint32_t now_us;                  /* internal monotonic clock; advances by dt*1e6 per predict */
    uint64_t imu_seq;                 /* total IMU samples processed (monotonic) */
    fusion6_imu_sample_t imu_ring[FUSION6_IMU_RING_LEN];
    fusion6_checkpoint_t ckpt_ring[FUSION6_CKPT_RING_LEN];
    int ckpt_count;                   /* valid entries in ckpt_ring (≤ FUSION6_CKPT_RING_LEN) */
    int ckpt_head;                    /* next write index in ckpt_ring */
    uint32_t last_ckpt_us;            /* now_us at last checkpoint creation */
    int in_replay;                    /* 1 = predict() suppresses ring recording (during rewind) */
    uint32_t rewind_count;            /* total successful rewinds — exposed for observability */
    uint32_t rewind_fallback_count;   /* GPS arrived too old → Stage-1 fallback */

    /* === Latched configuration === */
    fusion6_config_t cfg;
} fusion6_t;

/* ============================================================
 * API — mirrors fusion1..5 pattern
 * ============================================================ */

/** Initialize filter with given config. Sets P0 (large initial covariance), zeros
 *  state, sets sensor mask = 0xFF, init_done = 0. Must be followed by static init
 *  (sample + finalize) before the filter is trusted. */
void fusion6_init(fusion6_t *f, const fusion6_config_t *cfg);

/** Add one IMU sample to the static-init accumulator. Caller must hold the
 *  vehicle still. Inputs are body-frame, axis-mapped. */
void fusion6_static_init_sample(fusion6_t *f,
                                const double accel[3], const double gyro[3]);

/** Finalize static init. Returns 1 on success (gate passed, attitude/bias set,
 *  init_done=1), 0 on failure (motion gate triggered — caller retries). */
int fusion6_static_init_finalize(fusion6_t *f);

/** Predict step (ESKF §2). Caller passes already bias-corrected? NO — fusion6
 *  subtracts bias internally. Inputs are body-frame, axis-mapped. dt is real
 *  measured tick interval in seconds (clamp to [5e-4, 2e-3] before calling). */
void fusion6_predict(fusion6_t *f,
                     const double gyro[3], const double accel[3], double dt);

/** Accelerometer-as-gravity update (§5.1). Magnitude-gated, adaptive R, Joseph. */
void fusion6_update_accel(fusion6_t *f, const double accel[3]);

/** Magnetometer scalar yaw update (§5.2). yaw_meas in radians, NED convention
 *  (0 = North, +π/2 = East, in [-π, π]). Caller has tilt-compensated. */
void fusion6_update_mag_yaw(fusion6_t *f, double yaw_meas_rad);

/** Barometer altitude update (§5.3). baro_alt_m is positive-up altitude.
 *  Internal: h_pred = -p.z + b_baro. */
void fusion6_update_baro(fusion6_t *f, double baro_alt_m);

/** Range finder altitude update (§5.4). Caller pre-checks tilt < 30° and validity.
 *  Treats terrain as 0 (limitation, documented). */
void fusion6_update_lidar(fusion6_t *f, double range_m);

/** Optical flow update (§5.5). flow_x/y_rad_per_s already de-rotated by caller
 *  (gyro contribution removed). h_agl: above-ground level meters (>= 0.1).
 *  quality: 0..255, <30 → reject. */
void fusion6_update_optflow(fusion6_t *f,
                            double flow_x_rad_per_s,
                            double flow_y_rad_per_s,
                            double h_agl_m,
                            double quality);

/** GPS position update (§5.6). pos_ned in meters relative to lazy-init origin.
 *  sigma_xyz overrides default R when non-NULL (per-axis std-dev).
 *  Stage-1 path — no rewind, applies at current time with R inflation by latency·v. */
void fusion6_update_gps_pos(fusion6_t *f,
                            const double pos_ned[3],
                            const double sigma_xyz[3]);

/** GPS velocity update (§5.7). vel_ned in m/s, sigma scalar (set to 0 to use default).
 *  Stage-1 path — no rewind, applies at current time. */
void fusion6_update_gps_vel(fusion6_t *f, const double vel_ned[3], double sigma);

/** GPS position update WITH rewind (§8 Stage 2). meas_us is the measurement
 *  capture timestamp (microseconds, same clock as fusion6 internal now_us).
 *  Behavior:
 *   - If meas_us is within rewind window (oldest checkpoint ≤ meas_us ≤ now_us):
 *     rewinds to nearest checkpoint, replays IMU to meas_us, applies update,
 *     replays IMU forward to current time. Increments rewind_count.
 *   - Otherwise: falls back to Stage-1 (R inflation, no rewind). Increments
 *     rewind_fallback_count.
 *  Always safe to call even when no rewind history exists. */
void fusion6_update_gps_pos_at(fusion6_t *f,
                               const double pos_ned[3],
                               const double sigma_xyz[3],
                               uint32_t meas_us);

/** GPS velocity update with rewind. See _pos_at(). */
void fusion6_update_gps_vel_at(fusion6_t *f,
                               const double vel_ned[3],
                               double sigma,
                               uint32_t meas_us);

/** Snapshot the current filter state into the output struct. Computes Euler
 *  angles, linear accel, etc. Cheap (no allocations). */
void fusion6_get_state(const fusion6_t *f, fusion6_state_t *out);

/** Health check. Call from state_estimation @ 25 Hz with the current time
 *  (microseconds, monotonic). Updates health_flags based on staleness +
 *  P-trace runaway. */
void fusion6_check_health(fusion6_t *f, uint32_t now_us);

/** Set per-sensor enable mask for debug isolation (§9). Default = 0xFF.
 *  When a bit is clear, that sensor's update is skipped but innovations are
 *  still computed & stored in last_innov[]. */
void fusion6_set_sensor_enable(fusion6_t *f, uint16_t mask);

/** Get a copy of the last-innovation snapshot for a given sensor index (one of
 *  FUSION6_SENSOR_*). Returns the snapshot by value (small struct). */
fusion6_innov_t fusion6_get_last_innov(const fusion6_t *f, int sensor);

#endif /* FUSION6_H */
