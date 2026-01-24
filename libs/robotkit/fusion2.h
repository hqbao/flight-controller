#ifndef FUSION2_H
#define FUSION2_H

#include "vector3d.h"
#include "matrix.h"
#include "quat.h"

/**
 * FUSION2: Extended Kalman Filter for Attitude Estimation
 * 
 * Vector naming convention (unified across fusion1, fusion2, fusion3):
 * - v_pred: Predicted gravity vector in body frame (from quaternion)
 * - v_true: Measured/true gravity vector (normalized accelerometer)
 * - v_linear_acc: Linear acceleration with gravity removed (body frame)
 * - v_linear_acc_earth_frame: Linear acceleration in earth frame
 */
typedef struct {
    // State vector
    quaternion_t q;

    // State covariance matrix
    matrix_t P; // 4x4

    // Process noise covariance
    matrix_t Q; // 4x4

    // Measurement noise covariance for accelerometer
    matrix_t R; // 3x3

    // Rotation matrix transposed
    matrix_t Rq_T; // 3x3

    // Jacobian matrices
    matrix_t F;  // State transition Jacobian 4x4
    matrix_t H;  // State measurement Jacobian 3x4

    // Kalman gain
    matrix_t K; // 4x3

    char no_correction;

    vector3d_t v_true;
    vector3d_t v_pred;
    vector3d_t v_linear_acc;
    vector3d_t v_linear_acc_earth_frame;
    vector3d_t accel;
    double accel_scale;    
    
    // Low pass filter for accelerometer
    vector3d_t accel_lpf;
    double lpf_gain;

    matrix_t I4x4;
    matrix_t omega;
    matrix_t v001_3x1;
    matrix_t q4x1;
    matrix_t q4x1_temp;
    matrix_t q4x1_temp2;
    quaternion_t q_temp;
    quaternion_t q_temp2;
    matrix_t P4x4_temp;
    matrix_t P4x4_temp2;
    matrix_t F_T;
    matrix_t H_T;
    matrix_t S;
    matrix_t S_inv;
    matrix_t S_temp;
    matrix_t S_temp2;
    matrix_t K_temp; // 4x3
    matrix_t v; // 3x1
    matrix_t KH; // 4x3
} fusion2_t;

void fusion2_init(fusion2_t *f, double gyro_noise, double accel_noise, double accel_scale, double lpf_gain);
void fusion2_predict(fusion2_t *f, double gx, double gy, double gz, double dt);
void fusion2_update(fusion2_t *f, double ax, double ay, double az);

#endif