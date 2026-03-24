#include "notch_filter.h"
#include <pubsub.h>
#include <macro.h>
#include <messages.h>
#include <notch_filter.h>
#include <string.h>

/*
 * Notch Filter Module — Per-axis biquad notch for gyro vibration rejection.
 *
 * Subscribes to SENSOR_IMU1_GYRO_UPDATE (calibrated gyro, deg/s),
 * applies cascaded second-order IIR notch filters per axis, and publishes
 * the filtered result on SENSOR_IMU1_GYRO_FILTERED_UPDATE.
 *
 * Current mode: Fixed center frequencies targeting motor vibration band.
 * Future mode:  Dynamic center frequency from bidirectional DShot RPM
 *               telemetry — call notch_filter_update() per motor.
 *
 * Signal routing:
 *   IMU (calibrated deg/s) --> Notch Filter --> Attitude Estimation
 *   IMU (calibrated deg/s) --> FFT raw view (unfiltered, for analysis)
 *   Notch output           --> FFT filtered view (for comparison)
 */

/* --- Configuration --- */

/*
 * Two cascaded notch filters per axis covering 110–130 Hz vibration band:
 *   Notch A: 117 Hz, Q=3.0  → targets Z-axis peak (~119 Hz)
 *   Notch B: 123 Hz, Q=3.0  → targets X/Y-axis peak (~123 Hz)
 * Combined: >15 dB rejection across 110–130 Hz.
 */
#define NOTCH_A_CENTER_HZ  117.0f
#define NOTCH_B_CENTER_HZ  123.0f
#define NOTCH_SAMPLE_HZ    ((float)GYRO_FREQ)  /* Gyro sample rate (Hz) */
#define NOTCH_Q_FACTOR      3.0f

/* --- Filter state --- */

static notch_filter_t g_notch_a[3];  /* Per-axis: X, Y, Z */
static notch_filter_t g_notch_b[3];
static float g_filtered[3];          /* Output buffer for publish */

/* --- Gyro callback (1 kHz) --- */

static void on_gyro_update(uint8_t *data, size_t size) {
	if (size < 12) return;  /* 3 floats */
	float gyro[3];
	memcpy(gyro, data, sizeof(gyro));

	/* Cascade: notch A → notch B per axis */
	for (int i = 0; i < 3; i++) {
		float tmp = notch_filter_apply(&g_notch_a[i], gyro[i]);
		g_filtered[i] = notch_filter_apply(&g_notch_b[i], tmp);
	}

	publish(SENSOR_IMU1_GYRO_FILTERED_UPDATE, (uint8_t *)g_filtered, sizeof(g_filtered));
}

/* --- Setup --- */

void notch_filter_setup(void) {
	for (int i = 0; i < 3; i++) {
		notch_filter_init(&g_notch_a[i], NOTCH_A_CENTER_HZ,
			NOTCH_SAMPLE_HZ, NOTCH_Q_FACTOR);
		notch_filter_init(&g_notch_b[i], NOTCH_B_CENTER_HZ,
			NOTCH_SAMPLE_HZ, NOTCH_Q_FACTOR);
	}

	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
}
