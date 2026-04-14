#ifndef FFT_ANALYSIS_H
#define FFT_ANALYSIS_H

/**
 * FFT Analysis Library — Pure DSP functions for vibration analysis.
 *
 * Provides:
 * - In-place radix-2 Cooley-Tukey FFT
 * - Hanning window precomputation
 * - Power spectrum conversion
 * - Peak detection with SNR + absolute power + separation constraints
 *
 * No PubSub or platform dependencies — usable on any target.
 */

/**
 * Precompute Hanning window coefficients.
 *
 * @param window  Output buffer (must be fft_size elements)
 * @param fft_size  FFT size (must be power of 2)
 */
void fft_hanning_window(float *window, int fft_size);

/**
 * In-place radix-2 Cooley-Tukey DIT FFT.
 *
 * @param re  Real part (fft_size elements, modified in-place)
 * @param im  Imaginary part (fft_size elements, modified in-place)
 * @param n   FFT size (must be power of 2)
 */
void fft_radix2(float *re, float *im, int n);

/**
 * FFT peak detection configuration.
 */
typedef struct {
	int fft_size;           /* FFT size (e.g. 256) */
	float sample_hz;        /* Sample rate in Hz */
	float min_hz;           /* Minimum search frequency */
	float max_hz;           /* Maximum search frequency */
	float peak_snr;         /* SNR threshold (peak / mean, e.g. 8.0) */
	float peak_min_pwr;     /* Absolute minimum power threshold */
	float min_sep_hz;       /* Minimum separation between peaks in Hz */
	int num_peaks;          /* Number of peaks to find */
	int spectrum_bins;      /* Number of spectrum bins for power conversion */
} fft_peak_config_t;

/**
 * Convert FFT output to power spectrum in-place and find peaks.
 *
 * Modifies re[] in-place: after return, re[0..spectrum_bins-1] contains
 * the power spectrum (usable for logging).
 *
 * @param re        Real part from fft_radix2 (modified: becomes power spectrum)
 * @param im        Imaginary part from fft_radix2 (consumed)
 * @param cfg       Peak detection configuration
 * @param out_freq  Output peak frequencies (cfg->num_peaks elements, 0 = no peak)
 */
void fft_find_peaks(float *re, float *im, const fft_peak_config_t *cfg,
                    float *out_freq);

#endif /* FFT_ANALYSIS_H */
