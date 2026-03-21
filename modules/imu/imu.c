#include "imu.h"
#include <pubsub.h>
#include <platform.h>
#include <stdlib.h>
#include <math.h>
#include "icm42688p.h"
#include "icm42688p_i2c.h"
#include "icm42688p_spi.h"
#include <string.h>
#include <macro.h>
#include <messages.h>

/*
 * --- IMU SENSOR MODULE ---
 *
 * Pure sensor driver for ICM-42688P. Reads gyro and accel data,
 * applies calibration values received from the calibration module,
 * and publishes calibrated sensor data.
 *
 * Calibration values are delivered via PubSub from the calibration module:
 *   - CALIBRATION_GYRO_READY  → temp polynomial coefficients (3x3 floats)
 *   - CALIBRATION_ACCEL_READY → accel bias (3 floats) + scale matrix (3x3)
 */

#define SSF_GYRO (16.4)

/* --- Sensor state --- */

static icm42688p_t g_imu_sensor = {{0}, I2C_PORT1, SPI_PORT1, 0};
static float g_imu_data[7] = {0};     /* [0-2]=accel, [3-5]=gyro, [6]=temp (°C) */

static float g_accel_raw[3] = {0};    /* Raw accel before calibration (LSB) */
static float g_accel_cal[3] = {0};    /* Calibrated accel (LSB, bias+scale corrected) */
static float g_gyro_raw[3] = {0};     /* Raw gyro before calibration (LSB) */
static float g_gyro_cal[3] = {0};     /* Calibrated gyro (deg/s) */

/* --- Calibration state --- */

static float g_temp_coeff[3][3] = {0};  /* [axis][0]*T² + [axis][1]*T + [axis][2] */
static float g_accel_bias[3] = {0};
static float g_accel_scale[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f}
};
static uint8_t g_gyro_ready = 0;
static uint8_t g_accel_ready = 0;

/* --- Logging state --- */

static float g_log_msg[4] = {0};      /* 3 data values + temperature */
static uint8_t g_active_log_class = 0;

/* --- Sensor read & processing (1 kHz gyro, 500 Hz accel) --- */

static void on_sensor_read_tick(uint8_t *data, size_t size) {
	icm42688p_read(&g_imu_sensor);
}

static void on_imu_data_ready(void) {
	/* Save raw gyro before calibration (LSB) */
	g_gyro_raw[0] = g_imu_data[3];
	g_gyro_raw[1] = g_imu_data[4];
	g_gyro_raw[2] = g_imu_data[5];

	if (g_gyro_ready) {
		/* Compute gyro bias from temperature polynomial */
		float T = g_imu_data[6];
		float T2 = T * T;
		float bias_x = g_temp_coeff[0][0] * T2 + g_temp_coeff[0][1] * T + g_temp_coeff[0][2];
		float bias_y = g_temp_coeff[1][0] * T2 + g_temp_coeff[1][1] * T + g_temp_coeff[1][2];
		float bias_z = g_temp_coeff[2][0] * T2 + g_temp_coeff[2][1] * T + g_temp_coeff[2][2];

		/* Apply bias and convert to dps */
		g_imu_data[3] -= bias_x;
		g_imu_data[4] -= bias_y;
		g_imu_data[5] -= bias_z;

		g_imu_data[3] = g_imu_data[3] / SSF_GYRO;
		g_imu_data[4] = g_imu_data[4] / SSF_GYRO;
		g_imu_data[5] = g_imu_data[5] / SSF_GYRO;

		/* Save calibrated gyro for logger */
		g_gyro_cal[0] = g_imu_data[3];
		g_gyro_cal[1] = g_imu_data[4];
		g_gyro_cal[2] = g_imu_data[5];

		publish(SENSOR_IMU1_GYRO_UPDATE, (uint8_t*)&g_imu_data[3], 12);
	}

	/* Raw gyro for calibration module — lower priority, published after */
	publish(SENSOR_IMU1_GYRO_RAW, (uint8_t*)g_gyro_raw, 12);
}

static void on_i2c_callback(uint8_t *data, size_t size) {
	if (data[0] == I2C_PORT1) {
		icm42688p_get_i2c(&g_imu_sensor, g_imu_data);
		on_imu_data_ready();
	}
}

static void on_spi_callback(uint8_t *data, size_t size) {
	if (data[0] == SPI_PORT1) {
		icm42688p_get_spi(&g_imu_sensor, g_imu_data);
		on_imu_data_ready();
	}
}

static void on_scheduler_500hz(uint8_t *data, size_t size) {
	if (g_gyro_ready) {
		/* Save raw accel before calibration (for LOG_CLASS_IMU_ACCEL_RAW) */
		g_accel_raw[0] = g_imu_data[0];
		g_accel_raw[1] = g_imu_data[1];
		g_accel_raw[2] = g_imu_data[2];

		/* Apply accel calibration: V_cal = S * (V_raw - B) */
		float ax = g_imu_data[0] - g_accel_bias[0];
		float ay = g_imu_data[1] - g_accel_bias[1];
		float az = g_imu_data[2] - g_accel_bias[2];

		g_imu_data[0] = g_accel_scale[0][0] * ax + g_accel_scale[0][1] * ay + g_accel_scale[0][2] * az;
		g_imu_data[1] = g_accel_scale[1][0] * ax + g_accel_scale[1][1] * ay + g_accel_scale[1][2] * az;
		g_imu_data[2] = g_accel_scale[2][0] * ax + g_accel_scale[2][1] * ay + g_accel_scale[2][2] * az;

		/* Save calibrated accel for logger */
		g_accel_cal[0] = g_imu_data[0];
		g_accel_cal[1] = g_imu_data[1];
		g_accel_cal[2] = g_imu_data[2];

		publish(SENSOR_IMU1_ACCEL_UPDATE, (uint8_t*)g_imu_data, 12);
	}
}

/* --- Calibration receivers --- */

static void on_gyro_calibration_ready(uint8_t *data, size_t size) {
	if (size < sizeof(calibration_gyro_t)) return;
	calibration_gyro_t *cal = (calibration_gyro_t *)data;
	memcpy(g_temp_coeff, cal->temp_coeff, sizeof(g_temp_coeff));
	g_gyro_ready = 1;
}

static void on_accel_calibration_ready(uint8_t *data, size_t size) {
	if (size < sizeof(calibration_accel_t)) return;
	calibration_accel_t *cal = (calibration_accel_t *)data;
	memcpy(g_accel_bias, cal->bias, sizeof(g_accel_bias));
	memcpy(g_accel_scale, cal->scale, sizeof(g_accel_scale));
	g_accel_ready = 1;
}

static void on_scheduler_1hz(uint8_t *data, size_t size) {
	if (!g_gyro_ready) {
		publish(CALIBRATION_GYRO_REQUEST, NULL, 0);
	}
	if (!g_accel_ready) {
		publish(CALIBRATION_ACCEL_REQUEST, NULL, 0);
	}
}

/* --- Logging (25 Hz) --- */

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	uint8_t cls = data[0];
	if (cls == LOG_CLASS_IMU_ACCEL_RAW || cls == LOG_CLASS_IMU_ACCEL_CALIB ||
		cls == LOG_CLASS_IMU_GYRO_RAW || cls == LOG_CLASS_IMU_GYRO_CALIB) {
		g_active_log_class = cls;
	} else {
		g_active_log_class = 0;
	}
}

static void on_scheduler_25hz(uint8_t *data, size_t size) {
	if (g_active_log_class == 0) return;

	float *src;
	switch (g_active_log_class) {
		case LOG_CLASS_IMU_ACCEL_RAW:   src = g_accel_raw; break;
		case LOG_CLASS_IMU_ACCEL_CALIB: src = g_accel_cal; break;
		case LOG_CLASS_IMU_GYRO_RAW:    src = g_gyro_raw; break;
		case LOG_CLASS_IMU_GYRO_CALIB:  src = g_gyro_cal; break;
		default: return;
	}

	g_log_msg[0] = src[0];
	g_log_msg[1] = src[1];
	g_log_msg[2] = src[2];
	g_log_msg[3] = g_imu_data[6]; /* temperature */

	publish(SEND_LOG, (uint8_t*)g_log_msg, sizeof(g_log_msg));
}

/* --- Setup --- */

void imu_setup(void) {
	icm42688p_init(&g_imu_sensor,
		AFS_2G, GFS_2000DPS, 
		AODR_500Hz, GODR_4kHz,
		accel_mode_LN, gyro_mode_LN);
	subscribe(SCHEDULER_1KHZ, on_sensor_read_tick);
	subscribe(SCHEDULER_500HZ, on_scheduler_500hz);
	subscribe(I2C_CALLBACK_UPDATE, on_i2c_callback);
	subscribe(SPI_CALLBACK_UPDATE, on_spi_callback);
	subscribe(CALIBRATION_GYRO_READY, on_gyro_calibration_ready);
	subscribe(CALIBRATION_ACCEL_READY, on_accel_calibration_ready);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
	subscribe(SCHEDULER_1HZ, on_scheduler_1hz);
}
