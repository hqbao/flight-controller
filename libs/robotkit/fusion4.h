#ifndef FUSION4_H
#define FUSION4_H

#include "vector3d.h"
#include "matrix.h"
#include "quat.h"

/**
 * FUSION4: 7-State Extended Kalman Filter (Attitude + Gyro Bias)
 * 
 * State Vector (7x1):
 * [q0, q1, q2, q3, bx, by, bz]
 * 
 * q: Quaternion (Body to Earth)
 * b: Gyroscope Bias (rad/s)
 */
typedef struct {
    // State
    quaternion_t q;
    vector3d_t gyro_bias;

    // Covariance Matrix P (7x7)
    matrix_t P;

    // Process Noise Q (7x7)
    matrix_t Q;

    // Measurement Noise R (3x3) - Accelerometer
    matrix_t R;

    // Jacobians
    matrix_t F; // 7x7 State Transition
    matrix_t H; // 3x7 Measurement

    // Kalman Gain
    matrix_t K; // 7x3

    // Intermediate Matrices (pre-allocated to avoid runtime malloc)
    matrix_t F_T;       // 7x7
    matrix_t H_T;       // 7x3
    matrix_t P_new;     // 7x7
    matrix_t K_temp;    // 7x3
    matrix_t S;         // 3x3
    matrix_t S_inv;     // 3x3
    matrix_t KH;        // 7x7
    matrix_t I7x7;      // 7x7
    
    // Vectors / Outputs
    vector3d_t v_pred;           // Predicted gravity in body frame
    vector3d_t v_true;           // Measured gravity (normalized accel)
    vector3d_t v_linear_acc;     // Linear acceleration (dynamic motion)
    vector3d_t v_linear_acc_earth_frame;

    vector3d_t accel;
    vector3d_t accel_lpf;
    double accel_scale;
    double lpf_gain;
    char no_correction;

    // Internal scratchpad
    matrix_t y; // Innovation (3x1)
    
} fusion4_t;

/**
 * Initialize Fusion4 Algorithm
 * @param f Pointer to fusion4 object
 * @param gyro_noise Process noise for gyroscope (variance)
 * @param bias_noise Process noise for bias random walk (variance)
 * @param accel_noise Measurement noise for accelerometer (variance)
 * @param accel_scale Scaling factor for accel (usually 1G value in raw units, e.g. 1.0 or 4096.0)
 * @param lpf_gain Low pass filter gain for accelerometer preprocessing
 */
void fusion4_init(fusion4_t *f, double gyro_noise, double bias_noise, double accel_noise, double accel_scale, double lpf_gain);

/**
 * Predict Step (Gyro Integration)
 * @param f Pointer to fusion4 object
 * @param gx Gyro X (deg/s)
 * @param gy Gyro Y (deg/s)
 * @param gz Gyro Z (deg/s)
 * @param dt Time step (seconds)
 */
void fusion4_predict(fusion4_t *f, double gx, double gy, double gz, double dt);

/**
 * Update Step (Accelerometer Correction)
 * @param f Pointer to fusion4 object
 * @param ax Accel X (raw)
 * @param ay Accel Y (raw)
 * @param az Accel Z (raw)
 */
void fusion4_update(fusion4_t *f, double ax, double ay, double az);

#endif
