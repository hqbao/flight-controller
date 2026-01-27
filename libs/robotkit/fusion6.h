#ifndef FUSION6_H
#define FUSION6_H

/**
 * B1 Complementary Filter (Scalar Version)
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

    // Stage 1 (Final)
    double veloc1;
    double pos_est1;

    // Outputs
    double pos_final;
    double veloc_final;

    // Parameters
    struct {
        double stage1_integ;
        double stage1_corr;
        
        double stage2_integ;
        double stage2_corr;

        double veloc_feedback;
    } params;

    // Timestep
    double dt;
} fusion6_t;

/**
 * Initialize the filter with specific tuning parameters.
 * 
 * @param f Pointer to fusion6 object
 * @param frequency Update frequency in Hz (e.g. 500.0)
 * @param s1_integ Stage 1 integration gain (e.g. 0.05 for XY, 1.0 for Z)
 * @param s1_corr  Stage 1 correction gain (e.g. 0.5)
 * @param s2_integ Stage 2 integration gain (e.g. 0.05 for XY, 1.0 for Z)
 * @param s2_corr  Stage 2 correction gain (e.g. 20.0)
 * @param v_fb     Velocity feedback gain (e.g. 0.005)
 */
void fusion6_init(fusion6_t *f, double frequency, double s1_integ, double s1_corr, double s2_integ, double s2_corr, double v_fb);

/**
 * Update the filter with new measurements.
 * 
 * @param f Pointer to fusion6 object
 * @param accel Input acceleration (m/s^2)
 * @param pos_measured Input position measurement (e.g. Optical Flow or Baro)
 */
void fusion6_update(fusion6_t *f, double accel, double pos_measured);

#endif
