#include "oscillation_detection.h"
#include <pubsub.h>
#include <vector3d.h>
#include <string.h>
#include <math.h>

#define SHAKE_DETECTION_FREQ 100.0f

static double g_gyro_sum_x = 0;
static double g_gyro_sum_y = 0;

static float g_shake_freq_x = 0;
static float g_shake_freq_y = 0;

static float g_gyro_prev_x = 0;
static int g_freq_count_x = 0;
static float g_gyro_prev_y = 0;
static int g_freq_count_y = 0;

static void gyro_update(uint8_t *data, size_t size) {
	float *g = (float*)data;
	g_gyro_sum_x += g[0];
	g_gyro_sum_y += g[1];
}

static void loop_100hz(uint8_t *data, size_t size) {
	float gyro_x = (float)g_gyro_sum_x;
	float gyro_y = (float)g_gyro_sum_y;
	
	g_gyro_sum_x = 0;
	g_gyro_sum_y = 0;

	// Shake detection X
	if ((gyro_x >= 0 && g_gyro_prev_x >= 0) || (gyro_x < 0 && g_gyro_prev_x < 0)) {
		g_freq_count_x++;
	} else {
		if (g_freq_count_x > 0) {
			g_shake_freq_x = (SHAKE_DETECTION_FREQ * 0.5f) / (float)g_freq_count_x;
		}
		g_freq_count_x = 1;
	}
	g_gyro_prev_x = gyro_x;

	// Shake detection Y
	if ((gyro_y >= 0 && g_gyro_prev_y >= 0) || (gyro_y < 0 && g_gyro_prev_y < 0)) {
		g_freq_count_y++;
	} else {
		if (g_freq_count_y > 0) {
			g_shake_freq_y = (SHAKE_DETECTION_FREQ * 0.5f) / (float)g_freq_count_y;
		}
		g_freq_count_y = 1;
	}
	g_gyro_prev_y = gyro_y;
}

static void loop_5hz(uint8_t *data, size_t size) {
	vector3d_t freq_msg;
	freq_msg.x = g_shake_freq_x;
	freq_msg.y = g_shake_freq_y;
	freq_msg.z = 0;
	publish(OSCILLATION_FREQ_DETECTED, (uint8_t*)&freq_msg, sizeof(vector3d_t));
}

void oscillation_detection_setup(void) {
	subscribe(SENSOR_IMU1_GYRO_UPDATE, gyro_update);
	subscribe(SCHEDULER_100HZ, loop_100hz);
	subscribe(SCHEDULER_5HZ, loop_5hz);
}
