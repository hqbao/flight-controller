#include "fft.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <fft_analysis.h>
#include <string.h>
#include <math.h>

/*
 * FFT Module — On-board vibration analysis, peak detection, and spectrum logging.
 *
 * Uses robotkit fft_analysis library for DSP (FFT, peak detection).
 * This module handles PubSub wiring, ring buffer collection, EMA smoothing,
 * and log streaming.
 *
 * Two responsibilities:
 *
 * 1. PEAK DETECTION (always active):
 *    Collects gyro samples in a 256-point ring buffer, runs 256-point
 *    Hanning-windowed FFT at 10 Hz (cycling X→Y→Z). Each peak slot scans
 *    a fixed, NON-OVERLAPPING frequency band (see g_band_min/max_hz) so
 *    slot identity is stable: slot[0] always tracks the low-band peak,
 *    slot[1] mid, slot[2] high. EMA-smoothed frequencies are published
 *    on FFT_PEAKS_UPDATE. Lock-and-hold: when no peak is found in a
 *    band, the smoother keeps the last valid frequency. Axis-focused:
 *    when streaming spectrum, only the selected axis runs at 10 Hz;
 *    otherwise round-robin ~3.3 Hz per axis.
 *
 * 2. LOG STREAMING (when Python tool requests via LOG_CLASS):
 *    - FFT_PEAKS (0x17): 6 floats (3 axes × 2 peaks) at 10 Hz, 24 bytes.
 *    - FFT_SPECTRUM_X/Y/Z (0x18-0x1A): Combined frame at 10 Hz (axis-focused):
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
static uint8_t g_log_spectrum = 0;    /* 1-3 = streaming spectrum for axis X/Y/Z (raw only) */
static uint8_t g_log_spectrum_dual = 0; /* 1-3 = streaming raw + filtered side-by-side */

/* ================================================================
 *  Configuration
 * ================================================================ */

#define FFT_SIZE            256      /* Must be power of 2 */
#define FFT_SAMPLE_HZ      ((float)GYRO_FREQ)
#define FFT_SPECTRUM_HZ    400.0f   /* Spectrum log: send bins up to this freq */
#define FREQ_BIN_HZ        (FFT_SAMPLE_HZ / (float)FFT_SIZE)
#define FFT_SPECTRUM_BINS  ((int)(FFT_SPECTRUM_HZ / FREQ_BIN_HZ) + 1)
#define FFT_PEAK_MIN_PWR     1.0f   /* Absolute minimum power threshold */
static float g_freq_ema_alpha  = 0.15f;

/* Banded peak detection — each peak slot scans a fixed, NON-OVERLAPPING
 * frequency band. Slot[0] = low band, slot[1] = mid, slot[2] = high.
 * Boundaries fall in valleys between motor harmonics (1st/2nd/3rd) so
 * a peak never straddles a boundary. With FFT_NUM_PEAKS=3 these cover
 * fundamentals from ~80 Hz (bigger frame) to ~200 Hz (small bicopter). */
static const float g_band_min_hz[FFT_NUM_PEAKS] = {  50.0f, 220.0f, 390.0f };
static const float g_band_max_hz[FFT_NUM_PEAKS] = { 220.0f, 390.0f, 500.0f };

#define FFT_SPECTRUM_FLOOR_DB (-30.0f) /* Absolute dB floor (normalized power) */
#define FFT_SPECTRUM_RANGE_DB  60.0f   /* dB range (floor+60 → white) */

/* ================================================================
 *  State
 * ================================================================ */

static float g_ring[3][FFT_SIZE];         /* Per-axis raw gyro ring buffer */
static float g_ring_filt[3][FFT_SIZE];    /* Per-axis notch-filtered gyro ring buffer */
static uint16_t g_ring_idx;               /* Write position (raw) */
static uint16_t g_ring_filt_idx;          /* Write position (filtered) */
static uint32_t g_sample_count;           /* Saturates at FFT_SIZE */
static float g_window[FFT_SIZE];          /* Pre-computed Hanning window */
static float g_fft_re[FFT_SIZE];          /* FFT workspace: real */
static float g_fft_im[FFT_SIZE];          /* FFT workspace: imaginary */
static uint8_t g_fft_axis;               /* Current axis (0=X, 1=Y, 2=Z) */
static float g_smooth_freq[3][FFT_NUM_PEAKS]; /* EMA-smoothed peak frequencies */
static volatile uint8_t g_fft_pending;   /* Flag: ISR requests FFT run in LOOP */

/* ================================================================
 *  Helpers
 * ================================================================ */

/* Convert FFT output bins [0, max_bin) to power spectrum in-place
 * (matches the normalization fft_find_peaks used to apply, so dB
 * scaling for the spectrum log frame is unchanged). */
static void compute_power_spectrum(int max_bin) {
	float inv_half_n = 2.0f / (float)FFT_SIZE;
	for (int i = 0; i < max_bin; i++) {
		float r = g_fft_re[i] * inv_half_n;
		float m = g_fft_im[i] * inv_half_n;
		g_fft_re[i] = r * r + m * m;
	}
}

/* Find the strongest local-maximum bin within band b on g_fft_re[]
 * (already in power-spectrum form). Returns frequency in Hz, or 0
 * if no bin in the band exceeds FFT_PEAK_MIN_PWR. */
static float find_peak_in_band(int b) {
	int lo = (int)(g_band_min_hz[b] / FREQ_BIN_HZ);
	int hi = (int)(g_band_max_hz[b] / FREQ_BIN_HZ);
	if (lo < 1) lo = 1;
	if (hi > FFT_SIZE / 2) hi = FFT_SIZE / 2;
	int best_bin = -1;
	float best_pwr = FFT_PEAK_MIN_PWR;
	for (int i = lo; i < hi; i++) {
		float p = g_fft_re[i];
		if (p <= best_pwr) continue;
		if (i > lo     && p <= g_fft_re[i - 1]) continue;
		if (i < hi - 1 && p <= g_fft_re[i + 1]) continue;
		best_pwr = p;
		best_bin = i;
	}
	return (best_bin >= 0) ? (float)best_bin * FREQ_BIN_HZ : 0.0f;
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
		g_log_spectrum_dual = 0;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_X) {
		g_log_peaks = 0;
		g_log_spectrum = 1;
		g_log_spectrum_dual = 0;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_Y) {
		g_log_peaks = 0;
		g_log_spectrum = 2;
		g_log_spectrum_dual = 0;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_Z) {
		g_log_peaks = 0;
		g_log_spectrum = 3;
		g_log_spectrum_dual = 0;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_DUAL_X) {
		g_log_peaks = 0;
		g_log_spectrum = 0;
		g_log_spectrum_dual = 1;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_DUAL_Y) {
		g_log_peaks = 0;
		g_log_spectrum = 0;
		g_log_spectrum_dual = 2;
	} else if (cls == LOG_CLASS_FFT_SPECTRUM_DUAL_Z) {
		g_log_peaks = 0;
		g_log_spectrum = 0;
		g_log_spectrum_dual = 3;
	} else {
		g_log_peaks = 0;
		g_log_spectrum = 0;
		g_log_spectrum_dual = 0;
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

/* Filtered (post-notch) gyro callback (1 kHz): collect parallel ring buffer */
static void on_gyro_filtered_update(uint8_t *data, size_t size) {
	if (size < 12) return;
	float gyro[3];
	memcpy(gyro, data, sizeof(gyro));

	for (int i = 0; i < 3; i++) {
		g_ring_filt[i][g_ring_filt_idx] = gyro[i];
	}
	g_ring_filt_idx = (g_ring_filt_idx + 1) & (FFT_SIZE - 1);
}

/* 10 Hz trigger — runs in ISR, just sets flag for LOOP */
static void on_fft_trigger(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_sample_count >= FFT_SIZE) g_fft_pending = 1;
}

/* FFT analysis: runs in main-thread LOOP context (10 Hz, axis-focused when logging) */
static void on_loop(uint8_t *data, size_t size) {
	(void)data; (void)size;

	if (!g_fft_pending) return;
	g_fft_pending = 0;

	uint8_t axis;
	if (g_log_spectrum_dual) {
		axis = g_log_spectrum_dual - 1;
	} else if (g_log_spectrum) {
		axis = g_log_spectrum - 1;
	} else {
		axis = g_fft_axis;
		g_fft_axis = (g_fft_axis + 1) % 3;
	}

	/* Copy ring buffer → FFT workspace with Hanning window */
	uint16_t start = g_ring_idx;
	for (int i = 0; i < FFT_SIZE; i++) {
		int idx = (start + i) & (FFT_SIZE - 1);
		g_fft_re[i] = g_ring[axis][idx] * g_window[i];
		g_fft_im[i] = 0.0f;
	}

	fft_radix2(g_fft_re, g_fft_im, FFT_SIZE);

	/* Convert to power spectrum (covers both peak detection range and
	 * spectrum log range — take the larger). */
	int pwr_max = FFT_SIZE / 2;
	if (FFT_SPECTRUM_BINS > pwr_max) pwr_max = FFT_SPECTRUM_BINS;
	compute_power_spectrum(pwr_max);

	/* Banded peak detection: one peak per fixed non-overlapping band,
	 * giving stable slot identity for EMA + notch tracking. */
	float peak_freq[FFT_NUM_PEAKS];
	for (int b = 0; b < FFT_NUM_PEAKS; b++) {
		peak_freq[b] = find_peak_in_band(b);
	}

	/* EMA smoothing — lock-and-hold: 0 means "no peak this frame",
	 * keep last valid frequency. */
	for (int i = 0; i < FFT_NUM_PEAKS; i++) {
		if (peak_freq[i] <= 0.0f) continue;

		if (g_smooth_freq[axis][i] <= 0.0f) {
			/* First valid peak after reset — snap to it */
			g_smooth_freq[axis][i] = peak_freq[i];
		} else {
			/* Consecutive valid peaks — EMA smooth */
			g_smooth_freq[axis][i] =
					g_freq_ema_alpha * peak_freq[i] +
				(1.0f - g_freq_ema_alpha) * g_smooth_freq[axis][i];
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
	 * Layout: [axis(1)] [bins(103 uint8)] [peak1(float)] [peak2(float)]
	 * Total: 1 + 103 + 8 = 112 bytes */
	if (g_log_spectrum && axis == (g_log_spectrum - 1)) {
		/* g_fft_re[0..spectrum_bins-1] holds power spectrum from fft_find_peaks(). */
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

	/* DUAL spectrum: raw + filtered side-by-side for the selected axis.
	 * Layout: [axis(1)] [raw_bins(103)] [filt_bins(103)] [raw_peak1(4)] [raw_peak2(4)] [filt_peak1(4)] [filt_peak2(4)]
	 * Total: 1 + 103 + 103 + 16 = 223 bytes per frame at 10 Hz (~58% of 38400 baud bus). */
	if (g_log_spectrum_dual && axis == (g_log_spectrum_dual - 1)) {
		uint8_t dual_buf[1 + 2 * FFT_SPECTRUM_BINS + 2 * FFT_NUM_PEAKS * sizeof(float)];
		dual_buf[0] = axis;

		/* Raw spectrum bytes (from the FFT we just ran on g_ring) */
		for (int i = 0; i < FFT_SPECTRUM_BINS; i++) {
			float pwr = g_fft_re[i];
			if (pwr <= 0.0f) { dual_buf[1 + i] = 0; continue; }
			float db = 10.0f * log10f(pwr);
			float norm = (db - FFT_SPECTRUM_FLOOR_DB) / FFT_SPECTRUM_RANGE_DB;
			if (norm < 0.0f) norm = 0.0f;
			if (norm > 1.0f) norm = 1.0f;
			dual_buf[1 + i] = (uint8_t)(norm * 255.0f);
		}

		/* Save raw peaks before overwriting g_fft_re/g_fft_im */
		float raw_peaks_f[FFT_NUM_PEAKS];
		for (int p = 0; p < FFT_NUM_PEAKS; p++) {
			raw_peaks_f[p] = g_smooth_freq[axis][p];
		}

		/* Now run FFT on filtered ring for same axis */
		uint16_t fstart = g_ring_filt_idx;
		for (int i = 0; i < FFT_SIZE; i++) {
			int idx = (fstart + i) & (FFT_SIZE - 1);
			g_fft_re[i] = g_ring_filt[axis][idx] * g_window[i];
			g_fft_im[i] = 0.0f;
		}
		fft_radix2(g_fft_re, g_fft_im, FFT_SIZE);

		int fpwr_max = FFT_SIZE / 2;
		if (FFT_SPECTRUM_BINS > fpwr_max) fpwr_max = FFT_SPECTRUM_BINS;
		compute_power_spectrum(fpwr_max);

		/* Same banded peak detection on the filtered spectrum. */
		float filt_peak_freq[FFT_NUM_PEAKS];
		for (int b = 0; b < FFT_NUM_PEAKS; b++) {
			filt_peak_freq[b] = find_peak_in_band(b);
		}

		/* Filtered spectrum bytes */
		for (int i = 0; i < FFT_SPECTRUM_BINS; i++) {
			float pwr = g_fft_re[i];
			if (pwr <= 0.0f) { dual_buf[1 + FFT_SPECTRUM_BINS + i] = 0; continue; }
			float db = 10.0f * log10f(pwr);
			float norm = (db - FFT_SPECTRUM_FLOOR_DB) / FFT_SPECTRUM_RANGE_DB;
			if (norm < 0.0f) norm = 0.0f;
			if (norm > 1.0f) norm = 1.0f;
			dual_buf[1 + FFT_SPECTRUM_BINS + i] = (uint8_t)(norm * 255.0f);
		}

		/* Append raw peaks then filtered peaks (un-smoothed for filtered side) */
		memcpy(&dual_buf[1 + 2 * FFT_SPECTRUM_BINS],
		       raw_peaks_f, sizeof(raw_peaks_f));
		memcpy(&dual_buf[1 + 2 * FFT_SPECTRUM_BINS + FFT_NUM_PEAKS * sizeof(float)],
		       filt_peak_freq, sizeof(filt_peak_freq));

		publish(SEND_LOG, dual_buf, sizeof(dual_buf));
	}
}

/* --- Tuning --- */

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t tp;
	memcpy(&tp, data, sizeof(tp));

	g_freq_ema_alpha = tp.fft_freq_alpha;
}

/* --- Setup --- */

void fft_setup(void) {
	fft_hanning_window(g_window, FFT_SIZE);

	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
	subscribe(SENSOR_IMU1_GYRO_FILTERED_UPDATE, on_gyro_filtered_update);
	subscribe(SCHEDULER_10HZ, on_fft_trigger);
	subscribe(LOOP, on_loop);
	subscribe(TUNING_READY, on_tuning_ready);
}
