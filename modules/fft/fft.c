#include "fft.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <string.h>

/*
 * FFT Module — Gyro Vibration Analysis
 *
 * Streams batched gyro samples over SEND_LOG for host-side FFT visualization.
 * Activated at runtime via LOG_CLASS_IMU_GYRO (0x05) — no recompilation needed.
 *
 * Data flow:
 *   SENSOR_IMU1_GYRO_UPDATE (1 kHz, calibrated deg/s)
 *     -> decimate to 250 Hz
 *     -> scale to int16 (x10, 0.1 dps resolution)
 *     -> batch 50 samples
 *     -> publish(SEND_LOG) -> logger -> UART
 *
 * Bandwidth: 5 frames/sec x 108 bytes = 540 B/s (56% of 9600 baud)
 *
 * Future: perform on-board FFT and send frequency bins instead of raw samples.
 */

#define FFT_BATCH_SIZE   50
#define FFT_DECIMATION   4      /* 1 kHz / 4 = 250 Hz */
#define FFT_SCALE        10.0f  /* deg/s -> int16: 0.1 dps resolution */

static uint8_t g_active = 0;
static int16_t g_batch_buf[FFT_BATCH_SIZE];
static uint16_t g_batch_idx = 0;
static uint8_t g_skip_count = 0;

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_active = (data[0] == LOG_CLASS_IMU_GYRO);
	if (g_active) {
		g_batch_idx = 0;
		g_skip_count = 0;
	}
}

static void on_gyro_update(uint8_t *data, size_t size) {
	if (!g_active) return;
	if (size < 12) return;

	/* Decimate 1 kHz -> 250 Hz */
	g_skip_count++;
	if (g_skip_count < FFT_DECIMATION) return;
	g_skip_count = 0;

	/* data = float[3] {gx, gy, gz} in deg/s */
	float *gyro = (float *)data;
	float gz = gyro[2]; /* Z axis */

	/* Scale to int16: 0.1 deg/s resolution, +-3276 deg/s range */
	int32_t scaled = (int32_t)(gz * FFT_SCALE);
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

void fft_setup(void) {
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
}
