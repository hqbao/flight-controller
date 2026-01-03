#include "imu.h"
#include <pubsub.h>
#include <platform.h>
#include <stdlib.h>
#include <math.h>
#include "icm42688p.h"
#include "icm42688p_i2c.h"
#include "icm42688p_spi.h"
#include <string.h>

/* Macro to enable/disable sending MONITOR_DATA via logger */
#define ENABLE_IMU_MONITOR_LOG 0

#define GYRO_FREQ 1000
#define CALIBRATION_FREQ (GYRO_FREQ * 2) // 2 seconds
#define IMU_MOTION 32
#define SSF_GYRO (16.4)

typedef enum {
	init = 0,
	calibrating,
	ready,
} imu_mode_t;

typedef icm42688p_t imu_sensor_t;

typedef struct {
	imu_sensor_t imu_sensor;
	float gyro_accel[6];
	float gyro_offset[6];
	int64_t gyro_calibration[3];
	float gyro_min[3];
	float gyro_max[3];
	int16_t gyro_calibration_counter;
	imu_mode_t mode;
	topic_t topic_gyro_calibration_update;
	topic_t topic_gyro_update;
	topic_t topic_accel_update;
} imu_t;

static imu_t g_imu1 = {
	{{0}, I2C_PORT1, SPI_PORT1, 0},
	{0, 0, 0, 0, 0, 0},
	{0},
	{0},
	{0},
	{0},
	0,
	init,
	SENSOR_IMU1_GYRO_CALIBRATION_UPDATE,
	SENSOR_IMU1_GYRO_UPDATE,
	SENSOR_IMU1_ACCEL_UPDATE
};

static void calibrate(imu_t *imu) {
	float gx = imu->gyro_accel[3];
	float gy = imu->gyro_accel[4];
	float gz = imu->gyro_accel[5];

	imu->gyro_calibration[0] += gx;
	imu->gyro_calibration[1] += gy;
	imu->gyro_calibration[2] += gz;

	// Update Min/Max for stability check
	if (gx < imu->gyro_min[0]) imu->gyro_min[0] = gx;
	if (gx > imu->gyro_max[0]) imu->gyro_max[0] = gx;
	
	if (gy < imu->gyro_min[1]) imu->gyro_min[1] = gy;
	if (gy > imu->gyro_max[1]) imu->gyro_max[1] = gy;
	
	if (gz < imu->gyro_min[2]) imu->gyro_min[2] = gz;
	if (gz > imu->gyro_max[2]) imu->gyro_max[2] = gz;

	imu->gyro_calibration_counter++;

	if (imu->gyro_calibration_counter >= CALIBRATION_FREQ) {
		// Check stability (Max - Min < Threshold)
		// Threshold: 32 LSB (~2 dps) spread allowed over 2 seconds
		if ((imu->gyro_max[0] - imu->gyro_min[0] > IMU_MOTION) ||
			(imu->gyro_max[1] - imu->gyro_min[1] > IMU_MOTION) ||
			(imu->gyro_max[2] - imu->gyro_min[2] > IMU_MOTION)) {
			
			// Failed: Reset and retry
			imu->mode = init;
			uint8_t result = 0;
			publish(imu->topic_gyro_calibration_update, (uint8_t*)&result, 1);
			return;
		}

		imu->gyro_offset[3] = (double)(1.0 / CALIBRATION_FREQ) * imu->gyro_calibration[0];
		imu->gyro_offset[4] = (double)(1.0 / CALIBRATION_FREQ) * imu->gyro_calibration[1];
		imu->gyro_offset[5] = (double)(1.0 / CALIBRATION_FREQ) * imu->gyro_calibration[2];
		imu->mode = ready;
		uint8_t result = 1;
		publish(imu->topic_gyro_calibration_update, (uint8_t*)&result, 1);
	}
}

static void imu1_calibrate(uint8_t *data, size_t size) {
	g_imu1.gyro_calibration_counter = 0;
	g_imu1.gyro_calibration[0] = 0;
	g_imu1.gyro_calibration[1] = 0;
	g_imu1.gyro_calibration[2] = 0;
	
	// Initialize Min/Max with current values
	g_imu1.gyro_min[0] = g_imu1.gyro_max[0] = g_imu1.gyro_accel[3];
	g_imu1.gyro_min[1] = g_imu1.gyro_max[1] = g_imu1.gyro_accel[4];
	g_imu1.gyro_min[2] = g_imu1.gyro_max[2] = g_imu1.gyro_accel[5];
	
	g_imu1.mode = calibrating;
}

static void imu1_loop(uint8_t *data, size_t size) {
	icm42688p_read(&g_imu1.imu_sensor);
}

static void imu1_data_udpate(void) {	
	if (g_imu1.mode == ready) {
		g_imu1.gyro_accel[3] -= g_imu1.gyro_offset[3];
		g_imu1.gyro_accel[4] -= g_imu1.gyro_offset[4];
		g_imu1.gyro_accel[5] -= g_imu1.gyro_offset[5];

		g_imu1.gyro_accel[3] = g_imu1.gyro_accel[3] / SSF_GYRO;
		g_imu1.gyro_accel[4] = g_imu1.gyro_accel[4] / SSF_GYRO;
		g_imu1.gyro_accel[5] = g_imu1.gyro_accel[5] / SSF_GYRO;
		publish(g_imu1.topic_gyro_update, (uint8_t*)&g_imu1.gyro_accel[3], 12);
	}
	else if (g_imu1.mode == calibrating) {
		calibrate(&g_imu1);
	}
}

static void imu1_i2c_data_udpate(uint8_t *data, size_t size) {
	if (data[0] == I2C_PORT1) {
		icm42688p_get_i2c(&g_imu1.imu_sensor, g_imu1.gyro_accel);
		imu1_data_udpate();
	}
}

static void imu1_spi_data_udpate(uint8_t *data, size_t size) {
	if (data[0] == SPI_PORT1) {
		icm42688p_get_spi(&g_imu1.imu_sensor, g_imu1.gyro_accel);
		imu1_data_udpate();
	}
}

static void publish_accel_loop(uint8_t *data, size_t size) {
	if (g_imu1.mode == ready) {
		g_imu1.gyro_accel[0] -= g_imu1.gyro_offset[0];
		g_imu1.gyro_accel[1] -= g_imu1.gyro_offset[1];
		g_imu1.gyro_accel[2] -= g_imu1.gyro_offset[2];
		publish(g_imu1.topic_accel_update, (uint8_t*)g_imu1.gyro_accel, 12);
	}
}

#if ENABLE_IMU_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	/* Pack raw accel into MONITOR_DATA message
	   Format: 3 float32 values (ax, ay, az) */
	static uint8_t out_msg[12]; /* 3 * 4 bytes (float32) */
	
	float ax = g_imu1.gyro_accel[0];
	float ay = g_imu1.gyro_accel[1];
	float az = g_imu1.gyro_accel[2];
	
	memcpy(&out_msg[0], &ax, sizeof(float));
	memcpy(&out_msg[4], &ay, sizeof(float));
	memcpy(&out_msg[8], &az, sizeof(float));
	
	publish(MONITOR_DATA, out_msg, sizeof(out_msg));
}
#endif

void imu_setup(void) {
	icm42688p_init(&g_imu1.imu_sensor,
		AFS_2G, GFS_2000DPS, 
		AODR_500Hz, GODR_4kHz,
		accel_mode_LN, gyro_mode_LN);
	subscribe(SCHEDULER_1KHZ, imu1_loop);
	subscribe(SCHEDULER_500HZ, publish_accel_loop);
	subscribe(I2C_CALLBACK_UPDATE, imu1_i2c_data_udpate);
	subscribe(SPI_CALLBACK_UPDATE, imu1_spi_data_udpate);
	subscribe(SENSOR_IMU1_CALIBRATE_GYRO, imu1_calibrate);
#if ENABLE_IMU_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
