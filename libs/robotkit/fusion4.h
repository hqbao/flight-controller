#ifndef FUSION4_H
#define FUSION4_H

/**
 * Fusion 4: Linear Kalman Filter for 1D Position/Velocity (Scalar Optimized)
 * with Accelerometer Bias Estimation (3-state)
 * 
 * State Vector: x = [position, velocity, accel_bias]^T
 * 
 * Efficient implementation without matrix library dependencies.
 * Uses < 100 bytes of memory per instance.
 */
typedef struct {
    // State Vector
    // x[0] = Position
    // x[1] = Velocity
    // x[2] = Accel Bias
    double x[3];
    
    // Covariance Matrix P (Symmetric)
    // [ P00  P01  P02 ]
    // [ P10  P11  P12 ]
    // [ P20  P21  P22 ]
    double P[3][3];

    // Stored parameters
    double sigma_accel; // Process noise std dev (m/s^2)
    double sigma_vel;   // Measurement noise std dev (m/s)
    double sigma_bias;  // Bias random walk std dev

} fusion4_t;

/**
 * Initialize the Kalman Filter
 * @param f Pointer to filter
 * @param sigma_accel Standard deviation of acceleration process noise
 * @param sigma_vel Standard deviation of velocity measurement noise
 * @param sigma_bias Standard deviation of bias random walk
 */
void fusion4_init(fusion4_t *f, double sigma_accel, double sigma_vel, double sigma_bias);

/**
 * Prediction Step (High Frequency - IMU)
 * Updates state estimate using system model and control input (acceleration)
 * 
 * @param f Pointer to filter
 * @param accel Linear acceleration in m/s^2 (Control Input u)
 * @param dt Time step in seconds
 */
void fusion4_predict(fusion4_t *f, double accel, double dt);

/**
 * Update Step (Low Frequency - Optical Flow)
 * Fuses velocity measurement
 * 
 * @param f Pointer to filter
 * @param vel_measured Measured velocity (m/s)
 */
void fusion4_update(fusion4_t *f, double vel_measured);



/**
 * Update Step (Position)
 * Fuses position measurement (e.g. from GPS, Barometer, or Range Finder)
 * 
 * @param f Pointer to filter
 * @param pos_measured Measured position (m)
 * @param sigma_pos Measurement noise std dev (m)
 */
void fusion4_update_position(fusion4_t *f, double pos_measured, double sigma_pos);

/** Get current position estimate */
double fusion4_get_position(fusion4_t *f);

/** Get current velocity estimate */
double fusion4_get_velocity(fusion4_t *f);

#endif
