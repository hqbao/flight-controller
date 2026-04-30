#ifndef FUSION6_H
#define FUSION6_H

#include <stdint.h>
#include "vector3d.h"
#include "quat.h"
#include "matrix.h"

/**
 * FUSION6 — 15-state Error-State Kalman Filter (ESKF), simplified core.
 *
 * Only IMU-driven prediction and accelerometer (gravity) update are
 * implemented. Other sensors (mag/baro/lidar/optflow/GPS) are intentionally
 * removed so the core attitude+inertial path is small and easy to verify.
 * They will be re-added incrementally on top of this proven core.
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

typedef struct {
    /* Process noise PSDs */
    double sigma_accel;       /* m/s² /√Hz */
    double sigma_gyro;        /* rad/s /√Hz */
    double sigma_bias_accel;  /* m/s³ /√Hz */
    double sigma_bias_gyro;   /* rad/s² /√Hz */

    /* Measurement noise (accel only for now) */
    double R_accel;           /* m/s² */

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

void fusion6_get_state(const fusion6_t *f, fusion6_state_t *out);

#endif /* FUSION6_H */
