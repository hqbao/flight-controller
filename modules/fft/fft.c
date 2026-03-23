#include "fft.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <string.h>

/*
 * FFT Module — Gyro vibration analysis streaming.
 *
 * Streams batched gyro samples over SEND_LOG for host-side FFT / spectrogram.
 * Supports raw and notch-filtered gyro for before/after comparison:
 *   Raw:      LOG_CLASS_FFT_GYRO_X/Y/Z           (from SENSOR_IMU1_GYRO_UPDATE)
 *   Filtered: LOG_CLASS_FFT_GYRO_FILTERED_X/Y/Z   (from SENSOR_IMU1_GYRO_FILTERED_UPDATE)
 *
 * Pipeline: 1 kHz gyro -> decimate to 250 Hz -> scale to int16 (×10)
 *           -> batch 50 samples -> publish(SEND_LOG) -> logger -> UART
 *
 * Bandwidth: 5 frames/sec × 108 bytes = 540 B/s (56% of 9600 baud)
 */

#define FFT_BATCH_SIZE   50
#define FFT_TARGET_HZ    250
#define FFT_DECIMATION   (GYRO_FREQ / FFT_TARGET_HZ)
#define FFT_SCALE        10.0f

static uint8_t g_active_axis = 0;     /* 0=off, 1=X, 2=Y, 3=Z */
static uint8_t g_use_filtered = 0;    /* 0=raw, 1=filtered */
static int16_t g_batch_buf[FFT_BATCH_SIZE];
static uint16_t g_batch_idx = 0;
static uint8_t g_skip_count = 0;

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	uint8_t cls = data[0];

	/* Raw (unfiltered) axes */
	if (cls == LOG_CLASS_FFT_GYRO_X) {
		g_active_axis = 1; g_use_filtered = 0;
	} else if (cls == LOG_CLASS_FFT_GYRO_Y) {
		g_active_axis = 2; g_use_filtered = 0;
	} else if (cls == LOG_CLASS_FFT_GYRO_Z) {
		g_active_axis = 3; g_use_filtered = 0;
	/* Filtered axes */
	} else if (cls == LOG_CLASS_FFT_GYRO_FILTERED_X) {
		g_active_axis = 1; g_use_filtered = 1;
	} else if (cls == LOG_CLASS_FFT_GYRO_FILTERED_Y) {
		g_active_axis = 2; g_use_filtered = 1;
	} else if (cls == LOG_CLASS_FFT_GYRO_FILTERED_Z) {
		g_active_axis = 3; g_use_filtered = 1;
	} else {
		g_active_axis = 0;
	}
	if (g_active_axis) {
		g_batch_idx = 0;
		g_skip_count = 0;
	}
}

static void process_gyro(uint8_t *data, size_t size, uint8_t is_filtered) {
	if (!g_active_axis) return;
	if (g_use_filtered != is_filtered) return;
	if (size < 12) return;

	g_skip_count++;
	if (g_skip_count < FFT_DECIMATION) return;
	g_skip_count = 0;

	/* Safe extraction — no uint8_t* to float* cast (alignment) */
	float gyro[3];
	memcpy(gyro, data, sizeof(gyro));
	float val = gyro[g_active_axis - 1];

	int32_t scaled = (int32_t)(val * FFT_SCALE);
	if (scaled > 32767) scaled = 32767;
	if (scaled < -32768) scaled = -32768;
	g_batch_buf[g_batch_idx] = (int16_t)scaled;
	g_batch_idx++;

	if (g_batch_idx >= FFT_BATCH_SIZE) {
		publish(SEND_LOG, (uint8_t *)g_batch_buf,
				FFT_BATCH_SIZE * sizeof(int16_t));
		g_batch_idx = 0;
	}
}

static void on_gyro_update(uint8_t *data, size_t size) {
	process_gyro(data, size, 0);
}

static void on_gyro_filtered_update(uint8_t *data, size_t size) {
	process_gyro(data, size, 1);
}

void fft_setup(void) {
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
	subscribe(SENSOR_IMU1_GYRO_FILTERED_UPDATE, on_gyro_filtered_update);
}
