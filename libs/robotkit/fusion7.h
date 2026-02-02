#ifndef FUSION7_H
#define FUSION7_H

/**
 * Fusion 7: Linear Kalman Filter for 1D Position/Velocity (Scalar Optimized)
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

} fusion7_t;

/**
 * Initialize the Kalman Filter
 * @param f Pointer to filter
 * @param sigma_accel Standard deviation of acceleration process noise
 * @param sigma_vel Standard deviation of velocity measurement noise
 * @param sigma_bias Standard deviation of bias random walk
 */
void fusion7_init(fusion7_t *f, double sigma_accel, double sigma_vel, double sigma_bias);

/**
 * Prediction Step (High Frequency - IMU)
 * Updates state estimate using system model and control input (acceleration)
 * 
 * @param f Pointer to filter
 * @param accel Linear acceleration in m/s^2 (Control Input u)
 * @param dt Time step in seconds
 */
void fusion7_predict(fusion7_t *f, double accel, double dt);

/**
 * Update Step (Low Frequency - Optical Flow)
 * Fuses velocity measurement
 * 
 * @param f Pointer to filter
 * @param vel_measured Measured velocity (m/s)
 */
void fusion7_update(fusion7_t *f, double vel_measured);



/**
 * Get current position estimate
 */
/**
 * Update Step (Position)
 * Fuses position measurement (e.g. from GPS, Barometer, or Range Finder)
 * 
 * @param f Pointer to filter
 * @param pos_measured Measured position (m)
 * @param sigma_pos Measurement noise std dev (m)
 */
void fusion7_update_position(fusion7_t *f, double pos_measured, double sigma_pos);

double fusion7_get_position(fusion7_t *f);

/**
 * Get current velocity estimate
 */
double fusion7_get_velocity(fusion7_t *f);

#endif
