#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

/**
 * Second-order IIR biquad notch (band-reject) filter.
 *
 * Attenuates a narrow frequency band centered at `center_freq_hz`.
 * The filter bandwidth is controlled by the Q factor:
 *   - Higher Q = narrower notch, sharper rejection
 *   - Lower Q  = wider notch, more surrounding frequencies attenuated
 *
 * Usage:
 *   notch_filter_t nf;
 *   notch_filter_init(&nf, 100.0f, 1000.0f, 5.0f);  // 100Hz notch, 1kHz sample rate, Q=5
 *   float out = notch_filter_apply(&nf, sample);
 *
 * For dynamic RPM-based notch:
 *   notch_filter_update(&nf, new_freq_hz, sample_rate_hz, q_factor);
 *
 * Transfer function (Direct Form II Transposed):
 *   H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 */

typedef struct {
    /* Biquad coefficients */
    float b0, b1, b2;
    float a1, a2;

    /* Filter state (Direct Form II Transposed) */
    float d1, d2;

    /* Configuration (stored for reconfiguration) */
    float center_freq_hz;
    float sample_rate_hz;
    float q_factor;
} notch_filter_t;

/**
 * Initialize the notch filter and compute coefficients.
 * @param nf              Pointer to notch filter instance
 * @param center_freq_hz  Center frequency to reject (Hz)
 * @param sample_rate_hz  Sample rate of input signal (Hz)
 * @param q_factor        Quality factor (typical: 3-8 for motor vibration)
 */
void notch_filter_init(notch_filter_t *nf, float center_freq_hz,
                       float sample_rate_hz, float q_factor);

/**
 * Update the notch filter center frequency (e.g., from RPM telemetry).
 * Recomputes coefficients without resetting filter state.
 * @param nf              Pointer to notch filter instance
 * @param center_freq_hz  New center frequency (Hz)
 * @param sample_rate_hz  Sample rate (Hz)
 * @param q_factor        Quality factor
 */
void notch_filter_update(notch_filter_t *nf, float center_freq_hz,
                         float sample_rate_hz, float q_factor);

/**
 * Apply the notch filter to one sample.
 * @param nf     Pointer to notch filter instance
 * @param input  Input sample
 * @return       Filtered output sample
 */
float notch_filter_apply(notch_filter_t *nf, float input);

/**
 * Reset filter state (delay elements) to zero.
 * Use after a large discontinuity or when re-enabling the filter.
 * @param nf  Pointer to notch filter instance
 */
void notch_filter_reset(notch_filter_t *nf);

#endif
