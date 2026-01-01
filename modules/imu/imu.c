#include "imu.h"
#include <pubsub.h>
#include <platform.h>
#include <stdlib.h>
#include <math.h>
#include "icm42688p.h"
#include "icm42688p_i2c.h"
#include "icm42688p_spi.h"

#define GYRO_FREQ 250
#define CALIBRATION_FREQ (GYRO_FREQ * 2) // 2 seconds
#define IMU_MOTION 100
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
	int16_t gyro_calibration_check[3];
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
	0,
	init,
	SENSOR_IMU1_GYRO_CALIBRATION_UPDATE,
	SENSOR_IMU1_GYRO_UPDATE,
	SENSOR_IMU1_ACCEL_UPDATE
};

static void calibrate(imu_t *imu) {
	imu->gyro_calibration[0] += imu->gyro_accel[3];
	imu->gyro_calibration[1] += imu->gyro_accel[4];
	imu->gyro_calibration[2] += imu->gyro_accel[5];
	imu->gyro_calibration_counter++;

	// Check motionless
	char failed = 0;
	if (fabs(imu->gyro_accel[3] - imu->gyro_calibration_check[0]) > IMU_MOTION) failed = 1;
	if (fabs(imu->gyro_accel[4] - imu->gyro_calibration_check[1]) > IMU_MOTION) failed = 1;
	if (fabs(imu->gyro_accel[5] - imu->gyro_calibration_check[2]) > IMU_MOTION) failed = 1;
	imu->gyro_calibration_check[0] = imu->gyro_accel[3];
	imu->gyro_calibration_check[1] = imu->gyro_accel[4];
	imu->gyro_calibration_check[2] = imu->gyro_accel[5];
	if (failed == 1) {
		imu->mode = init;
		uint8_t result = 0;
		publish(imu->topic_gyro_calibration_update, (uint8_t*)&result, 1);
		return;
	}

	if (imu->gyro_calibration_counter >= CALIBRATION_FREQ) {
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
	g_imu1.gyro_calibration_check[0] = g_imu1.gyro_accel[3];
	g_imu1.gyro_calibration_check[1] = g_imu1.gyro_accel[4];
	g_imu1.gyro_calibration_check[2] = g_imu1.gyro_accel[5];
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

static void imu1_loop_10hz(uint8_t *data, size_t size) {
	if (g_imu1.mode == init) {
		print("Ignored data: %d\t%d\t%d\n", 
			(int)g_imu1.gyro_accel[3], 
			(int)g_imu1.gyro_accel[4], 
			(int)g_imu1.gyro_accel[5]);
	} else if (g_imu1.mode == calibrating) {
		print("Calibrating: %d\t%d\t%d\n", 
			(int)g_imu1.gyro_accel[3], 
			(int)g_imu1.gyro_accel[4], 
			(int)g_imu1.gyro_accel[5]);
	}
}

void imu_setup(void) {
	icm42688p_init(&g_imu1.imu_sensor,
		AFS_2G, GFS_2000DPS, 
		AODR_500Hz, GODR_4kHz,
		accel_mode_LN, gyro_mode_LN);
	subscribe(SCHEDULER_250HZ, imu1_loop);
	subscribe(SCHEDULER_250HZ, publish_accel_loop);
	subscribe(SCHEDULER_10HZ, imu1_loop_10hz);
	subscribe(I2C_CALLBACK_UPDATE, imu1_i2c_data_udpate);
	subscribe(SPI_CALLBACK_UPDATE, imu1_spi_data_udpate);
	subscribe(SENSOR_IMU1_CALIBRATE_GYRO, imu1_calibrate);
}
