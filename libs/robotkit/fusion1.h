#ifndef FUSION1_H
#define FUSION1_H

#include "vector3d.h"
#include "quat.h"

/**
 * FUSION1: Mahony Complementary Filter for Attitude Estimation
 * 
 * Vector naming convention (unified across fusion1, fusion2, fusion3):
 * - v_pred: Predicted gravity vector in body frame (from quaternion)
 * - v_true: Measured/true gravity vector (normalized accelerometer)
 * - v_linear_acc: Linear acceleration with gravity removed (body frame)
 * - v_linear_acc_earth_frame: Linear acceleration in earth frame
 */
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
    vector3d_t v_linear_acc_earth_frame; // Linear acceleration in earth frame

    
    // === Filter Parameters ===
    double gain_acc_smooth;            // Accelerometer smoothing gain (formerly k0)
    double gain_prop;                  // Mahony Kp - proportional gain (formerly k1)
    double gain_int;                   // Mahony Ki - integral gain (formerly ki)
    
    // === Configuration ===
    double accel_scale;                // Accelerometer scale (1g in raw units)
    
    // === Debug/Testing ===
    char no_correction;                // If 1, disable Mahony correction (gyro-only)
} fusion1_t;

void fusion1_init(fusion1_t *f, double gain_acc_smooth, double gain_prop, double gain_int, double accel_scale);

/**
 * Update Mahony gains at runtime.
 * Allows caller to dynamically adjust correction strength
 * (e.g. reduce Kp/Ki during high linear acceleration).
 */
void fusion1_set_gains(fusion1_t *f, double gain_prop, double gain_int);

void fusion1_predict(fusion1_t *f, double gx, double gy, double gz, double dt);
void fusion1_update(fusion1_t *f, double ax, double ay, double az, double dt);

#endif