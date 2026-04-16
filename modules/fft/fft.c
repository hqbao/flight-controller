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
 *    Hanning-windowed FFT at 10 Hz (cycling X→Y→Z), detects top 2 vibration
 *    peaks above 50 Hz (SNR threshold 3×, min separation 40 Hz), applies
 *    EMA smoothing (alpha 0.3), and publishes fft_peaks_t on
 *    FFT_PEAKS_UPDATE. Lock-and-hold: out-of-range peaks keep last valid
 *    frequency. Axis-focused: when streaming spectrum, only the selected
 *    axis runs at 10 Hz; otherwise round-robin ~3.3 Hz per axis.
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
static uint8_t g_log_spectrum = 0;    /* 1-3 = streaming spectrum for axis X/Y/Z */

/* ================================================================
 *  Configuration
 * ================================================================ */

#define FFT_SIZE            256      /* Must be power of 2 */
#define FFT_SAMPLE_HZ      ((float)GYRO_FREQ)
#define FFT_MIN_HZ          50.0f   /* Ignore below (body motion, not vibration) */
#define FFT_MAX_HZ         200.0f   /* Ignore above (motor vibration range) */
#define FFT_SPECTRUM_HZ    200.0f   /* Spectrum log: send bins up to this freq */
#define FREQ_BIN_HZ        (FFT_SAMPLE_HZ / (float)FFT_SIZE)
#define FFT_SPECTRUM_BINS  ((int)(FFT_SPECTRUM_HZ / FREQ_BIN_HZ) + 1)
#define FFT_PEAK_SNR         3.0f   /* Peak must be 3× above mean power */
#define FFT_PEAK_MIN_PWR     1.0f   /* Absolute minimum power threshold */
#define FREQ_EMA_ALPHA       0.3f   /* Frequency smoothing (0=frozen, 1=instant) */
#define MIN_PEAK_SEP_HZ     40.0f   /* Minimum Hz between two peaks */

#define FFT_SPECTRUM_FLOOR_DB (-30.0f) /* Absolute dB floor (normalized power) */
#define FFT_SPECTRUM_RANGE_DB  60.0f   /* dB range (floor+60 → white) */

/* ================================================================
 *  State
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

static fft_peak_config_t g_peak_cfg;

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

/* FFT analysis: runs in main-thread LOOP context (10 Hz, axis-focused when logging) */
static void on_loop(uint8_t *data, size_t size) {
	(void)data; (void)size;

	if (!g_fft_pending) return;
	g_fft_pending = 0;

	uint8_t axis;
	if (g_log_spectrum) {
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

	float peak_freq[FFT_NUM_PEAKS];
	fft_find_peaks(g_fft_re, g_fft_im, &g_peak_cfg, peak_freq);

	/* EMA smoothing — lock-and-hold: keep last valid frequency
	 * when peak is not detected this frame */
	for (int i = 0; i < FFT_NUM_PEAKS; i++) {
		if (peak_freq[i] < FFT_MIN_HZ || peak_freq[i] > FFT_MAX_HZ) {
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
}

/* --- Setup --- */

void fft_setup(void) {
	fft_hanning_window(g_window, FFT_SIZE);

	g_peak_cfg.fft_size = FFT_SIZE;
	g_peak_cfg.sample_hz = FFT_SAMPLE_HZ;
	g_peak_cfg.min_hz = FFT_MIN_HZ;
	g_peak_cfg.max_hz = FFT_MAX_HZ;
	g_peak_cfg.peak_snr = FFT_PEAK_SNR;
	g_peak_cfg.peak_min_pwr = FFT_PEAK_MIN_PWR;
	g_peak_cfg.min_sep_hz = MIN_PEAK_SEP_HZ;
	g_peak_cfg.num_peaks = FFT_NUM_PEAKS;
	g_peak_cfg.spectrum_bins = FFT_SPECTRUM_BINS;

	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
	subscribe(SCHEDULER_10HZ, on_fft_trigger);
	subscribe(LOOP, on_loop);
}
