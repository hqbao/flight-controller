#ifndef FUSION3_H
#define FUSION3_H

#include "vector3d.h"
#include "quat.h"

/**
 * FUSION3: Madgwick Filter for Attitude Estimation
 * 
 * The Madgwick filter uses gradient descent optimization to find the quaternion
 * that minimizes the error between predicted and measured gravity direction.
 * 
 * Key Features:
 * 1. Gradient descent for orientation error correction
 * 2. Single beta parameter (gain) controls correction speed
 * 3. Computationally efficient (no matrix inversions)
 * 4. Good performance under dynamic conditions
 * 
 * Algorithm:
 * 1. PREDICT: Integrate gyroscope to update quaternion
 * 2. UPDATE: Compute gradient of error function, apply correction
 * 
 * Mathematical Foundation:
 * - Objective function: f(q) = ||v_pred(q) - v_meas||²
 * - Gradient: ∇f gives direction of steepest increase
 * - Correction: q̇_correction = -β * normalize(∇f)
 * - Total derivative: q̇ = 0.5*q⊗ω - β*normalize(∇f)
 */
typedef struct {
    // === Orientation State ===
    quaternion_t q;                    // Current orientation quaternion (unit quaternion)
    vector3d_t v_pred;                 // Predicted gravity vector in body frame (from q)
    vector3d_t v_true;                 // Measured gravity vector (normalized accelerometer)
    
    // === Accelerometer Processing ===
    vector3d_t a;                      // Raw accelerometer reading
    vector3d_t a_smooth;               // Low-pass filtered accelerometer
    vector3d_t v_linear_acc;           // Linear acceleration (gravity removed)
    vector3d_t v_linear_acc_smooth;    // Smoothed linear acceleration
    vector3d_t v_linear_acc_earth_frame; // Linear acceleration in earth frame

    
    // === Filter Parameters ===
    double freq;                       // Update frequency (Hz)
    double k0;                         // Accelerometer smoothing gain (2.0-4.0)
    double beta;                       // Madgwick beta gain (0.01-0.5)
    double k2;                         // Linear acceleration decay rate
    
    // === Adaptive Correction ===
    double accel_scale;                // Accelerometer scale (1g in raw units)
    double accel_uncertainty;          // Reduces correction during high linear accel
    double min_linear_accel;           // Min threshold for accel uncertainty
    double max_linear_accel;           // Max threshold for accel uncertainty
    double linear_accel;               // Current linear acceleration magnitude
    
    // === Debug/Testing ===
    char no_correction;                // If 1, disable gradient correction (gyro-only)
} fusion3_t;

/**
 * Initialize Madgwick filter with identity quaternion
 * 
 * @param f    Pointer to fusion3_t struct
 * @param k0   Accelerometer smoothing gain (2.0-4.0, higher = smoother)
 * @param beta Madgwick beta gain (0.01-0.5, higher = faster correction but more noise)
 * @param freq Update frequency in Hz (typically 500Hz for drones)
 */
void fusion3_init(fusion3_t *f, double k0, double beta, double freq);

/**
 * PREDICT STEP: Integrate gyroscope to update quaternion
 * 
 * @param f  Pointer to fusion3_t struct
 * @param gx Gyroscope X rate in deg/s
 * @param gy Gyroscope Y rate in deg/s  
 * @param gz Gyroscope Z rate in deg/s
 * @param dt Time step in seconds (typically 0.001s for 1kHz gyro)
 */
void fusion3_predict(fusion3_t *f, double gx, double gy, double gz, double dt);

/**
 * UPDATE STEP: Compute gradient correction from accelerometer
 * 
 * @param f  Pointer to fusion3_t struct
 * @param ax Accelerometer X in raw units
 * @param ay Accelerometer Y in raw units
 * @param az Accelerometer Z in raw units
 */
void fusion3_update(fusion3_t *f, double ax, double ay, double az);

/**
 * Enable linear acceleration estimation (optional)
 * 
 * @param f                 Pointer to fusion3_t struct
 * @param k2                Linear acceleration decay gain
 * @param min_linear_accel  Minimum threshold
 * @param max_linear_accel  Maximum threshold  
 * @param accel_scale       Scale factor (1g in raw units)
 */
void fusion3_remove_linear_accel(fusion3_t *f, double k2, double min_linear_accel, double max_linear_accel, double accel_scale);

#endif
