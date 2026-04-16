#include "notch_filter.h"
#include <pubsub.h>
#include <macro.h>
#include <messages.h>
#include <notch_filter.h>
#include <string.h>

/*
 * Dynamic Notch Filter Module — Adaptive vibration rejection.
 *
 * Subscribes to SENSOR_IMU1_GYRO_UPDATE (calibrated gyro, deg/s),
 * applies cascaded second-order IIR notch filters per axis, and publishes
 * the filtered result on SENSOR_IMU1_GYRO_FILTERED_UPDATE.
 *
 * Subscribes to FFT_PEAKS_UPDATE from the FFT module to dynamically
 * adjust notch center frequencies based on detected vibration peaks.
 * Starts in passthrough (freq=0) — notch only activates once FFT detects
 * peaks above NOTCH_MIN_HZ. Once active, if peaks drop below the
 * threshold the notch holds its last valid frequency (lock-and-hold)
 * to maintain vibration rejection during brief FFT dropouts.
 *
 * Signal routing:
 *   FFT module ──(FFT_PEAKS_UPDATE)──→ notch center freq update
 *   IMU (calibrated deg/s) ──→ Notch cascade A→B ──→ GYRO_FILTERED_UPDATE
 */

/* --- Configuration --- */
#define NOTCH_SAMPLE_HZ     ((float)GYRO_FREQ)
#define NOTCH_Q_FACTOR       3.0f
#define NOTCH_MIN_HZ         50.0f   /* Below this → lock-and-hold last valid frequency */

/* --- Filter State --- */
static notch_filter_t g_notch_a[3];  /* Per-axis notch A */
static notch_filter_t g_notch_b[3];  /* Per-axis notch B */
static float g_filtered[3];          /* Output buffer for publish */

/* --- Gyro callback (1 kHz) --- */

static void on_gyro_update(uint8_t *data, size_t size) {
	if (size < 12) return;  /* 3 floats */

	float gyro[3];
	memcpy(gyro, data, 12);

	/* Cascade: notch_a → notch_b per axis */
	for (int i = 0; i < 3; i++) {
		float v = notch_filter_apply(&g_notch_a[i], gyro[i]);
		g_filtered[i] = notch_filter_apply(&g_notch_b[i], v);
	}

	publish(SENSOR_IMU1_GYRO_FILTERED_UPDATE,
		(uint8_t *)g_filtered, sizeof(g_filtered));
}

/* --- FFT peaks callback (from fft module, 10 Hz axis-focused) --- */

static void on_fft_peaks(uint8_t *data, size_t size) {
	if (size < sizeof(fft_peaks_t)) return;
	fft_peaks_t peaks;
	memcpy(&peaks, data, sizeof(peaks));

	if (peaks.axis >= 3) return;

	if (peaks.freq[0] > NOTCH_MIN_HZ) {
		notch_filter_update(&g_notch_a[peaks.axis],
			peaks.freq[0], NOTCH_SAMPLE_HZ, NOTCH_Q_FACTOR);
	}
	/* Below MIN_HZ → keep last valid frequency (lock-and-hold) */

	if (FFT_NUM_PEAKS >= 2 && peaks.freq[1] > NOTCH_MIN_HZ) {
		notch_filter_update(&g_notch_b[peaks.axis],
			peaks.freq[1], NOTCH_SAMPLE_HZ, NOTCH_Q_FACTOR);
	}
	/* Below MIN_HZ → keep last valid frequency (lock-and-hold) */
}

/* --- Setup --- */

void notch_filter_setup(void) {
	/* Start in passthrough — notch only activates once FFT detects real peaks */
	for (int i = 0; i < 3; i++) {
		notch_filter_init(&g_notch_a[i], 0.0f,
			NOTCH_SAMPLE_HZ, NOTCH_Q_FACTOR);
		notch_filter_init(&g_notch_b[i], 0.0f,
			NOTCH_SAMPLE_HZ, NOTCH_Q_FACTOR);
	}

	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
	subscribe(FFT_PEAKS_UPDATE, on_fft_peaks);
}
