#include "imu.h"
#include <pubsub.h>
#include <platform.h>
#include <stdlib.h>
#include <math.h>
#include "icm42688p.h"
#include "icm42688p_i2c.h"
#include "icm42688p_spi.h"
#include <string.h>

/* 
 * --- IMU CALIBRATION GUIDE ---
 * 
 * 1. GYROSCOPE CALIBRATION (Automatic)
 *    - Performed automatically on startup or when triggered.
 *    - Requires the device to be completely stationary for 2 seconds.
 *    - Logic: Collects samples for 2s. If the spread (Max - Min) of any axis 
 *      exceeds IMU_MOTION (32 LSB ~ 2 dps), calibration fails and retries.
 *    - Result: Updates gyro_offset[3], [4], [5].
 * 
 * 2. ACCELEROMETER CALIBRATION (Manual)
 *    - Requires external Python tool: pytest/calibrate_accel.py
 *    - Steps:
 *      a. Set ENABLE_ACCEL_MONITOR_LOG to 1 in this file.
 *      b. Flash firmware and connect via USB.
 *      c. Run 'python3 pytest/calibrate_accel.py'.
 *      d. Place drone in 6+ static positions (flat, sides, nose up/down, etc).
 *         Click "Capture Position" for each.
 *      e. Click "Compute Calib".
 *      f. Copy the resulting Bias (B) and Scale Matrix (S) into the 
 *         g_imu1 struct initialization below.
 *         - Bias (B) goes to gyro_offset[0], [1], [2].
 *         - Scale (S) goes to accel_scale[3][3].
 *      g. Set ENABLE_ACCEL_MONITOR_LOG back to 0.
 * 
 * 3. MATHEMATICAL MODEL
 *    The calibration applies the following linear correction:
 *    
 *    V_cal = S * (V_raw - B)
 *    
 *    Where:
 *    - V_raw: Raw sensor reading vector [x, y, z]
 *    - B (Bias): Offset vector [bx, by, bz]
 *      Corrects zero-g offset errors so the sphere is centered at (0,0,0).
 *    - S (Scale Matrix): 3x3 Matrix
 *      [ Sxx Sxy Sxz ]
 *      [ Syx Syy Syz ]
 *      [ Szx Szy Szz ]
 *      Corrects for scale factor errors (sensitivity) and axis misalignment 
 *      (non-orthogonality). It transforms the ellipsoid of raw data into 
 *      a perfect sphere of radius 1g.
 */

/* Macro to enable/disable sending MONITOR_DATA via logger */
#define ENABLE_ACCEL_MONITOR_LOG 1

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
	float accel_scale[3][3];
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
	.imu_sensor = {{0}, I2C_PORT1, SPI_PORT1, 0},
	.gyro_accel = {0},

	/* --- ACCELEROMETER CALIBRATION DATA --- */
	/* 1. Bias Vector (B) */
	/* Copy Bx, By, Bz into the first 3 slots. Leave the last 3 as 0 (Gyro runtime offsets). */
	.gyro_offset = {
		0.0f, 0.0f, 0.0f,  /* Bx, By, Bz */
		0.0f, 0.0f, 0.0f   /* Gyro Offsets (Do not edit) */
	},

	/* 2. Scale Matrix (S) */
	/* Copy the 3x3 Matrix here */
	.accel_scale = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	},
	/* -------------------------------------- */

	.gyro_calibration = {0},
	.gyro_min = {0},
	.gyro_max = {0},
	.gyro_calibration_counter = 0,
	.mode = init,
	.topic_gyro_calibration_update = SENSOR_IMU1_GYRO_CALIBRATION_UPDATE,
	.topic_gyro_update = SENSOR_IMU1_GYRO_UPDATE,
	.topic_accel_update = SENSOR_IMU1_ACCEL_UPDATE
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
		// Apply Offset
		float ax = g_imu1.gyro_accel[0] - g_imu1.gyro_offset[0];
		float ay = g_imu1.gyro_accel[1] - g_imu1.gyro_offset[1];
		float az = g_imu1.gyro_accel[2] - g_imu1.gyro_offset[2];

		// Apply Scale Matrix
		g_imu1.gyro_accel[0] = g_imu1.accel_scale[0][0] * ax + g_imu1.accel_scale[0][1] * ay + g_imu1.accel_scale[0][2] * az;
		g_imu1.gyro_accel[1] = g_imu1.accel_scale[1][0] * ax + g_imu1.accel_scale[1][1] * ay + g_imu1.accel_scale[1][2] * az;
		g_imu1.gyro_accel[2] = g_imu1.accel_scale[2][0] * ax + g_imu1.accel_scale[2][1] * ay + g_imu1.accel_scale[2][2] * az;

		publish(g_imu1.topic_accel_update, (uint8_t*)g_imu1.gyro_accel, 12);
	}
}

#if ENABLE_ACCEL_MONITOR_LOG
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
#if ENABLE_ACCEL_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
