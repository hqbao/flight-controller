#ifndef FUSION1_H
#define FUSION1_H

#include "vector3d.h"
#include "quat.h"

typedef struct {
    // === Orientation State ===
    quaternion_t q;                    // Current orientation quaternion (unit quaternion)
    vector3d_t v_pred;                 // Predicted gravity vector in body frame (from q)
    vector3d_t v_true;                 // Measured gravity vector (normalized accelerometer)
    
    // === Mahony Correction Terms ===
    vector3d_t gyro_bias;              // Proportional correction (Kp * error), deg/s
    vector3d_t gyro_bias_integral;     // Integral correction (Ki * ∫error), deg/s
    
    // === Accelerometer Processing ===
    vector3d_t a;                      // Raw accelerometer reading
    vector3d_t a_smooth;               // Low-pass filtered accelerometer
    vector3d_t v_linear_acc;           // Linear acceleration (gravity removed)
    vector3d_t v_linear_acc_smooth;    // Smoothed linear acceleration
    vector3d_t v_linear_acc_earth_frame; // Linear acceleration in earth frame

    
    // === Filter Parameters ===
    double freq;                       // Update frequency (Hz)
    double gain_acc_smooth;            // Accelerometer smoothing gain (formerly k0)
    double gain_prop;                  // Mahony Kp - proportional gain (formerly k1)
    double gain_int;                   // Mahony Ki - integral gain (formerly ki)
    double gain_lin_acc_decay;         // Linear acceleration decay rate (formerly k2)
    
    // === Adaptive Correction ===
    double accel_scale;                // Accelerometer scale (1g in raw units)
    double accel_uncertainty;          // Reduces correction during high linear accel
    double min_linear_accel;           // Min threshold for accel uncertainty
    double max_linear_accel;           // Max threshold for accel uncertainty
    double linear_accel;               // Current linear acceleration magnitude
    
    // === Debug/Testing ===
    char no_correction;                // If 1, disable Mahony correction (gyro-only)
} fusion1_t;

void fusion1_init(fusion1_t *f, double gain_acc_smooth, double gain_prop, double gain_int, double gain_lin_acc_decay, double freq);
void fusion1_remove_linear_accel(fusion1_t *f, double min_linear_accel, double max_linear_accel, double accel_scale);
void fusion1_predict(fusion1_t *f, double gx, double gy, double gz, double dt);
void fusion1_update(fusion1_t *f, double ax, double ay, double az);

#endif