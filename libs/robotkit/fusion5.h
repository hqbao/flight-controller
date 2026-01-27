#ifndef FUSION5_H
#define FUSION5_H

#include "vector3d.h"
#include "quat.h"

/**
 * FUSION5: Madgwick Filter with Gyroscope Bias Estimation
 * 
 * Extends Fusion3 (standard Madgwick) by adding gyroscope bias estimation.
 * This effectively combines the benefits of Madgwick's gradient descent approach
 * with the bias tracking usually found in Mahony or EKF filters.
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
    vector3d_t v_linear_acc_smooth;
    vector3d_t v_linear_acc_earth_frame;
    
    // === Parameters ===
    double freq;
    double beta;                       // Algorithm gain
    double zeta;                       // Bias gain
    
    // === Dynamic Tuning ===
    double k0;                         // Low pass filter accel
    double k2;                         // Low pass filter lin accel
    double linear_accel;
    double min_linear_accel;
    double max_linear_accel;
    double accel_scale;
    double accel_uncertainty;
    int no_correction;

} fusion5_t;

void fusion5_init(fusion5_t *f, double k0, double beta, double zeta, double accel_scale, double freq);
void fusion5_remove_linear_accel(fusion5_t *f, double k2, double min_linear_accel, double max_linear_accel);
void fusion5_predict(fusion5_t *f, double gx, double gy, double gz, double dt);
void fusion5_update(fusion5_t *f, double ax, double ay, double az);

#endif
