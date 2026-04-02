#include "fft.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <string.h>
#include <math.h>

/*
 * FFT Module — On-board vibration analysis, peak detection, and spectrum logging.
 *
 * Two responsibilities:
 *
 * 1. PEAK DETECTION (always active):
 *    Collects gyro samples in a 256-point ring buffer, runs 256-point
 *    Hanning-windowed FFT at 10 Hz (cycling X→Y→Z), detects top 2 vibration
 *    peaks above 50 Hz (SNR threshold 8×, min separation 20 Hz), applies
 *    EMA smoothing (alpha 0.3), and publishes fft_peaks_t on
 *    FFT_PEAKS_UPDATE. Out-of-range peaks reset to 0; in-range re-initializes.
 *    The notch_filter module subscribes and adapts its center frequencies.
 *
 * 2. LOG STREAMING (when Python tool requests via LOG_CLASS):
 *    - FFT_PEAKS (0x17): 6 floats (3 axes × 2 peaks) at 10 Hz, 24 bytes.
 *    - FFT_SPECTRUM_X/Y/Z (0x18-0x1A): Combined frame at ~3.3 Hz/axis:
 *      [axis(1)] [52 uint8 dB-scaled bins, 0-200 Hz] [peak1(float)] [peak2(float)]
 *      = 61 bytes. Absolute dB scaling: floor -30 dB, range 60 dB.
 *      Peaks are appended in the same frame to avoid UART buffer corruption
 *      (HAL_UART_Transmit_IT is non-blocking with a single shared buffer).
 *
 * ISR safety: The 256-point FFT is too expensive to run inside a SCHEDULER
 * ISR (blocks the 1 kHz control loop). SCHEDULER_10HZ only sets a flag;
 * the actual FFT runs in LOOP (main-thread context), where the control ISR
 * can preempt it freely. Sample collection (ring buffer write) remains in
 * the gyro callback at 1 kHz — a single memcpy, negligible cost.
 */

/* ================================================================
 *  Log Streaming State
 * ================================================================ */

static uint8_t g_log_peaks = 0;       /* 1 = streaming smoothed peaks */
static uint8_t g_log_spectrum = 0;    /* 1-3 = streaming spectrum for axis X/Y/Z */

/* ================================================================
 *  FFT Peak Detection Configuration
 * ================================================================ */

#define FFT_SIZE            256      /* Must be power of 2 */
#define FFT_SAMPLE_HZ      ((float)GYRO_FREQ)
#define FFT_MIN_HZ          50.0f   /* Ignore below (body motion, not vibration) */
#define FFT_MAX_HZ         200.0f   /* Ignore above (motor vibration range) */
#define FFT_SPECTRUM_HZ    200.0f   /* Spectrum log: send bins up to this freq */
#define FFT_SPECTRUM_BINS  ((int)(FFT_SPECTRUM_HZ / FREQ_BIN_HZ) + 1)
#define FFT_PEAK_SNR         8.0f   /* Peak must be 8× above mean power */
#define FREQ_EMA_ALPHA       0.3f   /* Frequency smoothing (0=frozen, 1=instant) */
#define MIN_PEAK_SEP_HZ     20.0f   /* Minimum Hz between two peaks */

#define FFT_SPECTRUM_FLOOR_DB (-30.0f) /* Absolute dB floor (normalized power) */
#define FFT_SPECTRUM_RANGE_DB  60.0f   /* dB range (floor+60 → white) */

#define FREQ_BIN_HZ         (FFT_SAMPLE_HZ / (float)FFT_SIZE)
#define MIN_PEAK_SEP_BINS   ((int)(MIN_PEAK_SEP_HZ / FREQ_BIN_HZ))

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ================================================================
 *  FFT State
 * ================================================================ */

static float g_ring[3][FFT_SIZE];         /* Per-axis gyro ring buffer */
static uint16_t g_ring_idx;               /* Write position */
static uint32_t g_sample_count;           /* Saturates at FFT_SIZE */
static float g_window[FFT_SIZE];          /* Pre-computed Hanning window */
static float g_fft_re[FFT_SIZE];          /* FFT workspace: real */
static float g_fft_im[FFT_SIZE];          /* FFT workspace: imaginary */
static uint8_t g_fft_axis;               /* Current axis (0=X, 1=Y, 2=Z) */
static float g_smooth_freq[3][FFT_NUM_PEAKS]; /* EMA-smoothed peak frequencies */
static volatile uint8_t g_fft_pending;   /* Flag: ISR requests FFT run in LOOP */

/* ================================================================
 *  FFT Implementation — In-place Radix-2 Cooley-Tukey DIT
 * ================================================================ */

static void init_window(void) {
	for (int i = 0; i < FFT_SIZE; i++) {
		g_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI
			* (float)i / (float)(FFT_SIZE - 1)));
	}
}

static void fft_radix2(float *re, float *im, int n) {
	/* Bit-reversal permutation */
	int j = 0;
	for (int i = 0; i < n - 1; i++) {
		if (i < j) {
			float t;
			t = re[i]; re[i] = re[j]; re[j] = t;
			t = im[i]; im[i] = im[j]; im[j] = t;
		}
		int m = n >> 1;
		while (m >= 1 && j >= m) {
			j -= m;
			m >>= 1;
		}
		j += m;
	}

	/* Butterfly stages */
	for (int step = 1; step < n; step <<= 1) {
		float angle = -(float)M_PI / (float)step;
		float wr = cosf(angle);
		float wi = sinf(angle);
		for (int group = 0; group < n; group += step << 1) {
			float cr = 1.0f, ci = 0.0f;
			for (int pair = 0; pair < step; pair++) {
				int a = group + pair;
				int b = a + step;
				float tr = cr * re[b] - ci * im[b];
				float ti = cr * im[b] + ci * re[b];
				re[b] = re[a] - tr;
				im[b] = im[a] - ti;
				re[a] += tr;
				im[a] += ti;
				float nr = cr * wr - ci * wi;
				ci = cr * wi + ci * wr;
				cr = nr;
			}
		}
	}
}

/*
 * Find top FFT_NUM_PEAKS frequency peaks in the power spectrum.
 * Operates on g_fft_re/g_fft_im (destroyed — converted to power spectrum).
 */
static void find_peaks(float *out_freq) {
	int min_bin = (int)(FFT_MIN_HZ / FREQ_BIN_HZ) + 1;
	int max_bin = (int)(FFT_MAX_HZ / FREQ_BIN_HZ);
	if (max_bin > FFT_SIZE / 2) max_bin = FFT_SIZE / 2;

	/* Convert to power spectrum in-place (full range for spectrum log).
	 * Normalize by N/2 so bins represent true amplitude before squaring. */
	int pwr_max = max_bin;
	if (FFT_SPECTRUM_BINS > pwr_max) pwr_max = FFT_SPECTRUM_BINS;
	float inv_half_n = 2.0f / (float)FFT_SIZE;
	for (int i = 0; i < pwr_max; i++) {
		float re = g_fft_re[i] * inv_half_n;
		float im = g_fft_im[i] * inv_half_n;
		g_fft_re[i] = re * re + im * im;
	}

	/* SNR threshold from peak-detection range only */
	float sum = 0.0f;
	for (int i = min_bin; i < max_bin; i++) {
		sum += g_fft_re[i];
	}
	float mean = sum / (float)(max_bin - min_bin);
	float threshold = mean * FFT_PEAK_SNR;

	int found_bins[FFT_NUM_PEAKS];
	for (int p = 0; p < FFT_NUM_PEAKS; p++) {
		out_freq[p] = 0.0f;
		found_bins[p] = -1;
	}

	/* Find peaks: strongest first, then next with min separation */
	for (int p = 0; p < FFT_NUM_PEAKS; p++) {
		float best_mag = 0.0f;
		int best_bin = -1;

		for (int i = min_bin; i < max_bin; i++) {
			if (g_fft_re[i] < threshold) continue;

			/* Local maximum check */
			if (i > min_bin && g_fft_re[i] <= g_fft_re[i - 1]) continue;
			if (i < max_bin - 1 && g_fft_re[i] <= g_fft_re[i + 1]) continue;

			/* Enforce minimum separation from already-found peaks */
			int too_close = 0;
			for (int k = 0; k < p; k++) {
				if (found_bins[k] < 0) continue;
				int diff = i - found_bins[k];
				if (diff < 0) diff = -diff;
				if (diff < MIN_PEAK_SEP_BINS) {
					too_close = 1;
					break;
				}
			}
			if (too_close) continue;

			if (g_fft_re[i] > best_mag) {
				best_mag = g_fft_re[i];
				best_bin = i;
			}
		}

		if (best_bin >= 0) {
			out_freq[p] = (float)best_bin * FREQ_BIN_HZ;
			found_bins[p] = best_bin;
		}
	}
}

/* ================================================================
 *  Callbacks
 * ================================================================ */

/* Log streaming: select mode from Python tool */
static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	uint8_t cls = data[0];

	if (cls == LOG_CLASS_FFT_PEAKS) {
		g_log_peaks = 1;
		g_log_spectrum = 0;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_X) {
		g_log_peaks = 0;
		g_log_spectrum = 1;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_Y) {
		g_log_peaks = 0;
		g_log_spectrum = 2;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_Z) {
		g_log_peaks = 0;
		g_log_spectrum = 3;
	} else {
		g_log_peaks = 0;
		g_log_spectrum = 0;
	}
}

/* Gyro raw callback (1 kHz): collect ring buffer for FFT analysis */
static void on_gyro_update(uint8_t *data, size_t size) {
	if (size < 12) return;
	float gyro[3];
	memcpy(gyro, data, sizeof(gyro));

	for (int i = 0; i < 3; i++) {
		g_ring[i][g_ring_idx] = gyro[i];
	}
	g_ring_idx = (g_ring_idx + 1) & (FFT_SIZE - 1);
	if (g_sample_count < FFT_SIZE) g_sample_count++;
}

/* 10 Hz trigger — runs in ISR, just sets flag for LOOP */
static void on_fft_trigger(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_sample_count >= FFT_SIZE) g_fft_pending = 1;
}

/* FFT analysis: runs in main-thread LOOP context (~3.3 Hz per axis) */
static void on_loop(uint8_t *data, size_t size) {
	(void)data; (void)size;

	if (!g_fft_pending) return;
	g_fft_pending = 0;

	uint8_t axis = g_fft_axis;
	g_fft_axis = (g_fft_axis + 1) % 3;

	/* Copy ring buffer → FFT workspace with Hanning window */
	uint16_t start = g_ring_idx;
	for (int i = 0; i < FFT_SIZE; i++) {
		int idx = (start + i) & (FFT_SIZE - 1);
		g_fft_re[i] = g_ring[axis][idx] * g_window[i];
		g_fft_im[i] = 0.0f;
	}

	fft_radix2(g_fft_re, g_fft_im, FFT_SIZE);

	float peak_freq[FFT_NUM_PEAKS];
	find_peaks(peak_freq);

	/* Peak smoothing: only track peaks within valid range [MIN_HZ, MAX_HZ].
	 * Out-of-range or missing → reset to 0. Back in range → snap (re-init).
	 * Consecutive in-range → EMA smooth. */
	for (int i = 0; i < FFT_NUM_PEAKS; i++) {
		if (peak_freq[i] < FFT_MIN_HZ || peak_freq[i] > FFT_MAX_HZ) {
			/* Out of range or not found — reset */
			g_smooth_freq[axis][i] = 0.0f;
			continue;
		}

		if (g_smooth_freq[axis][i] <= 0.0f) {
			/* First valid peak after reset — snap to it */
			g_smooth_freq[axis][i] = peak_freq[i];
		} else {
			/* Consecutive valid peaks — EMA smooth */
			g_smooth_freq[axis][i] =
				FREQ_EMA_ALPHA * peak_freq[i] +
				(1.0f - FREQ_EMA_ALPHA) * g_smooth_freq[axis][i];
		}
	}

	/* Publish smoothed peaks for this axis */
	fft_peaks_t peaks;
	peaks.axis = axis;
	for (int i = 0; i < FFT_NUM_PEAKS; i++) {
		peaks.freq[i] = g_smooth_freq[axis][i];
	}
	publish(FFT_PEAKS_UPDATE, (uint8_t *)&peaks, sizeof(peaks));

	/* Stream all smoothed peaks over SEND_LOG (24 bytes = 6 floats) */
	if (g_log_peaks) {
		float buf[3 * FFT_NUM_PEAKS];
		for (int a = 0; a < 3; a++) {
			for (int p = 0; p < FFT_NUM_PEAKS; p++) {
				buf[a * FFT_NUM_PEAKS + p] = g_smooth_freq[a][p];
			}
		}
		publish(SEND_LOG, (uint8_t *)buf, sizeof(buf));
	}

	/* Stream spectrum + peaks as single frame to avoid UART buffer conflict.
	 * Layout: [axis(1)] [bins(52 uint8)] [peak1(float)] [peak2(float)]
	 * Total: 1 + 52 + 8 = 61 bytes */
	if (g_log_spectrum && axis == (g_log_spectrum - 1)) {
		/* g_fft_re[0..FFT_SIZE/2] already holds power spectrum from find_peaks().
		 * Convert to dB scale: 10*log10(power), map to 0-255. */
		uint8_t spec_buf[1 + FFT_SPECTRUM_BINS + FFT_NUM_PEAKS * sizeof(float)];
		spec_buf[0] = axis;

		for (int i = 0; i < FFT_SPECTRUM_BINS; i++) {
			float pwr = g_fft_re[i];
			if (pwr <= 0.0f) {
				spec_buf[1 + i] = 0;
				continue;
			}
			float db = 10.0f * log10f(pwr);
			float norm = (db - FFT_SPECTRUM_FLOOR_DB) / FFT_SPECTRUM_RANGE_DB;
			if (norm < 0.0f) norm = 0.0f;
			if (norm > 1.0f) norm = 1.0f;
			spec_buf[1 + i] = (uint8_t)(norm * 255.0f);
		}

		/* Append peak frequencies for the selected axis */
		float peaks_f[FFT_NUM_PEAKS];
		for (int p = 0; p < FFT_NUM_PEAKS; p++) {
			peaks_f[p] = g_smooth_freq[axis][p];
		}
		memcpy(&spec_buf[1 + FFT_SPECTRUM_BINS], peaks_f, sizeof(peaks_f));

		publish(SEND_LOG, spec_buf, sizeof(spec_buf));
	}
}

/* --- Setup --- */

void fft_setup(void) {
	init_window();

	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
	subscribe(SCHEDULER_10HZ, on_fft_trigger);
	subscribe(LOOP, on_loop);
}
