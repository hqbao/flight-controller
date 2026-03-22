#ifndef FUSION3_H
#define FUSION3_H

#include "vector3d.h"
#include "quat.h"

/**
 * FUSION3: Madgwick Filter with Gyroscope Bias Estimation
 * 
 * Madgwick gradient descent filter with online gyroscope bias tracking.
 * Combines the computational efficiency of Madgwick's gradient descent
 * with bias estimation similar to Mahony's integral term.
 * Set zeta=0 to disable bias estimation (standard Madgwick behavior).
 * 
 * State:
 * - Quaternion (orientation)
 * - Gyro Bias (drift)
 * 
 * Parameters:
 * - Beta: Gain for orientation correction (divergence rate)
 * - Zeta: Gain for bias estimation (drift rate)
 */
typedef struct {
    // === State ===
    quaternion_t q;                    // Orientation
    vector3d_t bias;                   // Gyroscope bias (rad/s)
    
    // === Vectors ===
    vector3d_t v_pred;                 // Predicted gravity
    vector3d_t v_true;                 // Measured gravity
    
    // === Accelerometer Processing ===
    vector3d_t a;                      // Raw accel
    vector3d_t a_smooth;               // Filtered accel
    vector3d_t v_linear_acc;
    vector3d_t v_linear_acc_earth_frame;
    
    // === Parameters ===
    double beta;                       // Algorithm gain
    double zeta;                       // Bias gain
    
    // === Configuration ===
    double k0;                         // Low pass filter accel
    double accel_scale;
    int no_correction;

} fusion3_t;

void fusion3_init(fusion3_t *f, double k0, double beta, double zeta, double accel_scale);

/**
 * Update Madgwick beta gain at runtime.
 * Allows caller to dynamically adjust correction strength
 * (e.g. reduce beta during high linear acceleration).
 */
void fusion3_set_beta(fusion3_t *f, double beta);
void fusion3_predict(fusion3_t *f, double gx, double gy, double gz, double dt);
void fusion3_update(fusion3_t *f, double ax, double ay, double az, double dt);

#endif
