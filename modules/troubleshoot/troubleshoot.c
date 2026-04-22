#include "troubleshoot.h"
#include <pubsub.h>
#include <messages.h>
#include <stdint.h>
#include <stddef.h>

/*
 * --- TROUBLESHOOT MODULE ---
 *
 * Diagnostic-only module for ad-hoc data collection.
 *
 * Current diagnostic: ACCEL CLIP DETECTOR
 *   Subscribes to SENSOR_IMU1_ACCEL_RAW_UPDATE (int16 LSB at 500 Hz).
 *   Tracks per-axis min/max LSB and clip-count over a 1 s window.
 *   A "clip" is any sample within ~1% of the INT16 rail (>= 32440).
 *   At AFS_2G, INT16 max = ±2 g rail. Frequent clips => sensor cannot
 *   represent the true acceleration => raises Z-bias rectification risk.
 *
 *   Publishes a 16-byte payload at 1 Hz on SEND_LOG when the active log
 *   class is LOG_CLASS_TROUBLESHOOT_ACCEL:
 *     int16 min_x, min_y, min_z
 *     int16 max_x, max_y, max_z
 *     uint16 clip_x, clip_y, clip_z   (sample count in window)
 *     uint16 sample_count             (samples seen in window, sanity)
 *
 *   Window resets after each publish.
 */

#define CLIP_THRESHOLD 32440  /* ~99% of INT16_MAX (32767) */

static uint8_t g_active_log_class = 0;

static int16_t g_min[3] = { 32767,  32767,  32767};
static int16_t g_max[3] = {-32768, -32768, -32768};
static uint16_t g_clip[3] = {0, 0, 0};
static uint16_t g_samples = 0;

static int16_t g_msg_min[3];
static int16_t g_msg_max[3];
static uint16_t g_msg_clip[3];
static uint16_t g_msg_samples;

static void window_reset(void) {
	g_min[0] = g_min[1] = g_min[2] =  32767;
	g_max[0] = g_max[1] = g_max[2] = -32768;
	g_clip[0] = g_clip[1] = g_clip[2] = 0;
	g_samples = 0;
}

static void on_accel_raw(uint8_t *data, size_t size) {
	if (size < 6) return;
	int16_t v[3];
	v[0] = (int16_t)(data[0] | (data[1] << 8));
	v[1] = (int16_t)(data[2] | (data[3] << 8));
	v[2] = (int16_t)(data[4] | (data[5] << 8));

	for (int i = 0; i < 3; i++) {
		if (v[i] < g_min[i]) g_min[i] = v[i];
		if (v[i] > g_max[i]) g_max[i] = v[i];
		int16_t a = v[i] < 0 ? -v[i] : v[i];
		if (a >= CLIP_THRESHOLD) g_clip[i]++;
	}
	g_samples++;
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	uint8_t cls = data[0];
	if (cls == LOG_CLASS_TROUBLESHOOT_ACCEL) {
		g_active_log_class = cls;
		window_reset();
	} else {
		g_active_log_class = 0;
	}
}

static void on_scheduler_1hz(uint8_t *data, size_t size) {
	if (g_active_log_class != LOG_CLASS_TROUBLESHOOT_ACCEL) return;

	g_msg_min[0] = g_min[0]; g_msg_min[1] = g_min[1]; g_msg_min[2] = g_min[2];
	g_msg_max[0] = g_max[0]; g_msg_max[1] = g_max[1]; g_msg_max[2] = g_max[2];
	g_msg_clip[0] = g_clip[0]; g_msg_clip[1] = g_clip[1]; g_msg_clip[2] = g_clip[2];
	g_msg_samples = g_samples;

	uint8_t buf[20];
	/* min[3] (6) + max[3] (6) + clip[3] (6) + samples (2) = 20 bytes */
	for (int i = 0; i < 3; i++) {
		buf[2*i  ] = (uint8_t)(g_msg_min[i] & 0xFF);
		buf[2*i+1] = (uint8_t)((g_msg_min[i] >> 8) & 0xFF);
	}
	for (int i = 0; i < 3; i++) {
		buf[6 + 2*i  ] = (uint8_t)(g_msg_max[i] & 0xFF);
		buf[6 + 2*i+1] = (uint8_t)((g_msg_max[i] >> 8) & 0xFF);
	}
	for (int i = 0; i < 3; i++) {
		buf[12 + 2*i  ] = (uint8_t)(g_msg_clip[i] & 0xFF);
		buf[12 + 2*i+1] = (uint8_t)((g_msg_clip[i] >> 8) & 0xFF);
	}
	buf[18] = (uint8_t)(g_msg_samples & 0xFF);
	buf[19] = (uint8_t)((g_msg_samples >> 8) & 0xFF);

	publish(SEND_LOG, buf, sizeof(buf));
	window_reset();
}

void troubleshoot_setup(void) {
	subscribe(SENSOR_IMU1_ACCEL_RAW_UPDATE, on_accel_raw);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_1HZ, on_scheduler_1hz);
}
