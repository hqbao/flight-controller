#ifndef FUSION6_H
#define FUSION6_H

/**
 * Fusion 6: Scalar Cascaded Complementary Filter
 * 
 * A cascaded complementary filter for 1D position estimation.
 * Instantiate this struct for each axis (X, Y, Z).
 * 
 * Algorithm:
 * - Stage 0: Interim position/velocity estimation
 * - Stage 1: Final position/velocity estimation with stronger correction
 * - Feedback path from Stage 1 velocity to Stage 0
 */
typedef struct {
    // Stage 0 (Interim)
    double veloc0;
    double pos_est0;
    double pos_est0_prev; // For derivative calculation

    // Outputs (Final Stage)
    double pos_final;
    double veloc_final;

    // Internal State
    double pos_measured_maintained;

    // Parameters
    struct {
        double stage1_integ;
        double stage1_corr;
        
        double stage2_integ;
        double stage2_corr;

        double veloc_feedback;
    } params;
} fusion6_t;

/**
 * Initialize the filter with specific tuning parameters.
 * 
 * @param f Pointer to fusion6 object
 * @param s1_integ Stage 1 integration gain (e.g. 0.05 for XY, 1.0 for Z)
 * @param s1_corr  Stage 1 correction gain (e.g. 0.5)
 * @param s2_integ Stage 2 integration gain (e.g. 0.05 for XY, 1.0 for Z)
 * @param s2_corr  Stage 2 correction gain (e.g. 20.0)
 * @param v_fb     Velocity feedback gain (e.g. 0.005)
 */
void fusion6_init(fusion6_t *f, double s1_integ, double s1_corr, double s2_integ, double s2_corr, double v_fb);

/**
 * Predict internal state based on acceleration.
 * 
 * @param f Pointer to fusion6 object
 * @param accel Input acceleration (m/s^2)
 * @param dt Time delta in seconds
 */
void fusion6_predict(fusion6_t *f, double accel, double dt);

/**
 * Update the filter with new velocity measurements.
 * 
 * @param f Pointer to fusion6 object
 * @param veloc_measured Input velocity measurement (m/s) (e.g. from optical flow)
 * @param dt Time delta in seconds
 */
void fusion6_update(fusion6_t *f, double veloc_measured, double dt);

#endif
