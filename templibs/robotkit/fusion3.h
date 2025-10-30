#ifndef FUSION3_H
#define FUSION3_H

#include "vector3d.h"
#include "matrix.h"
#include "quat.h"

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

    vector3d_t true_norm_accel;
    vector3d_t pred_norm_accel;
    vector3d_t true_euler_angle;
    vector3d_t pred_euler_angle;

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
} fusion3_t;

void fusion3_init(fusion3_t *f, double gyro_noise, double accel_noise);
void fusion3_predict(fusion3_t *f, double gx, double gy, double gz, double dt);
void fusion3_update(fusion3_t *f, double ax, double ay, double az);

#endif