#ifndef FUSION6_H
#define FUSION6_H

#include <stdint.h>
#include "vector3d.h"
#include "quat.h"
#include "matrix.h"

/**
 * FUSION6 — 15-state Error-State Kalman Filter (ESKF).
 *
 * Implements IMU-driven prediction, accelerometer (gravity) update, a 1-D
 * yaw pseudo-measurement from a tilt-compensated magnetic heading
 * (`fusion6_update_mag_heading`), a 2-D horizontal body-velocity update
 * (`fusion6_update_velocity_xy_body`) for optical-flow / camera-derived
 * velocity, a 1-D barometric altitude update (`fusion6_update_baro_z`)
 * on p.z, and 3-D GPS position/velocity updates in NED
 * (`fusion6_update_pos_ned`, `fusion6_update_vel_ned`) using per-axis
 * (NE / D) variances from the config block.
 *
 * State (NED frame, +X=N, +Y=E, +Z=Down):
 *   Nominal: p[3], v[3], q, b_a[3], b_g[3]
 *   Error    (15 doubles): δp[3], δv[3], δθ[3], δb_a[3], δb_g[3]
 *
 * Quaternion: [w, x, y, z], identity = [1,0,0,0], body→earth.
 * Gravity: g_ned = (0, 0, +9.80665) m/s². At rest accel reads ~(0, 0, −g).
 *
 * Inputs MUST be axis-mapped to body NED before passing in. Bias subtraction
 * is internal.
 */

#define FUSION6_N 15

#define FUSION6_HF_INIT_DONE     (1u << 10)
#define FUSION6_HF_BIAS_CLAMPED  (1u << 9)
/* Sensor-lamp bits 0..7 follow the flight-controller viewer convention
 * (HEALTH_BITS in tools/flight_telemetry_view.py): Gyro=0, Accel=1,
 * Compass=2, Baro=3, Range=4, OptD=5, OptU=6, GPS=7. fusion6 only owns
 * BARO_OK today; the rest are produced by the FC glue when it has the
 * sensor wired. */
#define FUSION6_HF_BARO_OK       (1u << 3)
#define FUSION6_HF_GPS_OK        (1u << 7)

typedef struct {
    /* Process noise PSDs */
    double sigma_accel;       /* m/s² /√Hz */
    double sigma_gyro;        /* rad/s /√Hz */
    double sigma_bias_accel;  /* m/s³ /√Hz */
    double sigma_bias_gyro;   /* rad/s² /√Hz */

    /* Measurement noise */
    double R_accel;           /* m/s² */
    double R_mag_heading;     /* rad² (yaw pseudo-measurement variance) */
    double R_vel_xy_body;     /* (m/s)² (horizontal body-velocity variance,
                                 applied to both axes — diag R_meas) */
    double R_baro;            /* m² (barometric altitude variance, NED p.z) */
    double R_gps_pos_ne;      /* m² per horizontal axis (GPS pos N, E)   */
    double R_gps_pos_d;       /* m² (GPS pos D — vertical, typically larger) */
    double R_gps_vel_ne;      /* (m/s)² per horizontal axis (GPS vel N, E) */
    double R_gps_vel_d;       /* (m/s)² (GPS vel D — vertical)            */
    /* Bias sanity bounds */
    double bias_clamp_gyro;   /* rad/s */
    double bias_clamp_accel;  /* m/s² */

    /* Skip accel update if |‖a‖−g| > this */
    double accel_mag_gate;    /* m/s² */

    double gravity;           /* m/s² */
} fusion6_config_t;

void fusion6_config_default(fusion6_config_t *cfg);

typedef struct {
    vector3d_t pos;           /* NED, m */
    vector3d_t vel;           /* NED, m/s */
    quaternion_t q;           /* body→earth */
    vector3d_t euler;         /* roll, pitch, yaw — radians */
    vector3d_t accel_body;    /* linear accel (gravity removed), body, m/s² */
    vector3d_t accel_earth;   /* linear accel, earth, m/s² (positive-up Z) */
    vector3d_t gyro_bias;     /* rad/s */
    vector3d_t accel_bias;    /* m/s² */
    double trace_P_pos;
    double trace_P_vel;
    double trace_P_att;
    uint16_t health_flags;
} fusion6_state_t;

typedef struct {
    /* Nominal */
    vector3d_t p;
    vector3d_t v;
    quaternion_t q;
    vector3d_t b_a;
    vector3d_t b_g;

    /* Covariance */
    matrix_t P;

    /* IMU caching */
    vector3d_t gyro_prev;
    vector3d_t accel_cache;
    int has_prev_gyro;

    /* Static init accumulators */
    int init_sample_count;
    double init_accel_sum[3];
    double init_accel_sum_sq[3];
    double init_gyro_sum[3];
    double init_gyro_sum_sq[3];

    /* Status */
    uint16_t health_flags;

    fusion6_config_t cfg;
} fusion6_t;

/* API */
void fusion6_init(fusion6_t *f, const fusion6_config_t *cfg);

void fusion6_static_init_sample(fusion6_t *f,
                                const double accel[3], const double gyro[3]);
int  fusion6_static_init_finalize(fusion6_t *f);

void fusion6_predict(fusion6_t *f,
                     const double gyro[3], const double accel[3], double dt);

void fusion6_update_accel(fusion6_t *f, const double accel[3]);

/** Yaw pseudo-measurement from a magnetic heading (decl-corrected, NED, rad).
 *  1-D update with H = [0..., R21/c², R22/c², 0...] in the δθ block,
 *  c² = cos²(pitch) = R00² + R10². Skipped if INIT_DONE not set or near
 *  gimbal lock (pitch ≳ 84°). Innovation is wrapped to [-π, π]. */
void fusion6_update_mag_heading(fusion6_t *f, double mag_heading_rad);

/** Horizontal body-velocity update (e.g. from a downward / upward optical-flow
 *  camera + range-to-surface). 2-D measurement z = (vx_body, vy_body) in m/s.
 *  Per-axis measurement variance is taken from cfg.R_vel_xy_body (diag R).
 *  Vertical velocity is NOT updated here. Skipped if INIT_DONE not set or if
 *  the innovation covariance is singular. See FUSION6_ESKF.md §3c. */
void fusion6_update_velocity_xy_body(fusion6_t *f,
                                     const double v_xy_body[2]);

/** Barometric altitude update. 1-D scalar measurement on p.z (NED, +Down).
 *  Caller must convert sensor reading to NED p.z metres before calling
 *  (a barometer that reports altitude-above-startup +up should be negated).
 *  Uses cfg.R_baro. Skipped if INIT_DONE not set or innovation-cov is
 *  non-positive. Sets FUSION6_HF_BARO_OK on success. */
void fusion6_update_baro_z(fusion6_t *f, double z_ned_m);

/** GPS 3-D position update in NED metres (local origin, +Z=Down).
 *  Decomposed as three independent scalar updates on δp[0..2] using
 *  cfg.R_gps_pos_ne for N and E, cfg.R_gps_pos_d for D. Sequential
 *  scalar updates are exact for a diagonal R when the H rows are
 *  orthogonal (here each row is a coordinate basis vector). Skipped if
 *  INIT_DONE is not set or if any innovation covariance is non-positive.
 *  Sets FUSION6_HF_GPS_OK on success. */
void fusion6_update_pos_ned(fusion6_t *f, const double pos_ned_m[3]);

/** GPS 3-D velocity update in NED m/s. Sequential scalar updates on
 *  δv[3..5] with cfg.R_gps_vel_ne (N, E) and cfg.R_gps_vel_d (D).
 *  Skipped if INIT_DONE is not set or if any innovation covariance is
 *  non-positive. Sets FUSION6_HF_GPS_OK on success. */
void fusion6_update_vel_ned(fusion6_t *f, const double vel_ned_mps[3]);

void fusion6_get_state(const fusion6_t *f, fusion6_state_t *out);

#endif /* FUSION6_H */
