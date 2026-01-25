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
 *    - Requires external Python tool: pytest/imu_calibrate_accel.py
 *    - Steps:
 *      a. Set ENABLE_ACCEL_MONITOR_LOG to 1 in this file.
 *      b. Flash firmware and connect via USB.
 *      c. Run 'python3 pytest/imu_calibrate_accel.py'.
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
#define ENABLE_ACCEL_MONITOR_LOG 0

#define GYRO_FREQ 1000
#define CALIBRATION_FREQ (GYRO_FREQ * 2) // 2 seconds
#define IMU_MOTION 32
#define SSF_GYRO (16.4)

typedef enum {
	init = 0,
	calibrating,
	ready,
} imu_state_t;

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
	imu_state_t state;
	topic_t topic_gyro_calibration_update;
	topic_t topic_gyro_update;
	topic_t topic_accel_update;
} imu_t;

#if ENABLE_ACCEL_MONITOR_LOG
static uint8_t g_monitor_msg[12] = {0};
#endif
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
	.state = init,
	.topic_gyro_calibration_update = SENSOR_IMU1_GYRO_CALIBRATION_UPDATE,
	.topic_gyro_update = SENSOR_IMU1_GYRO_UPDATE,
	.topic_accel_update = SENSOR_IMU1_ACCEL_UPDATE
};

static void save_calibration_data(imu_t *imu) {
	param_storage_t param;
	
	// Save calibration status flag
	param.id = PARAM_ID_IMU1_GYRO_CALIBRATED;
	param.value = 1.0f;
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
	
	// Save gyro bias X
	param.id = PARAM_ID_IMU1_GYRO_BIAS_X;
	param.value = imu->gyro_offset[3];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
	
	// Save gyro bias Y
	param.id = PARAM_ID_IMU1_GYRO_BIAS_Y;
	param.value = imu->gyro_offset[4];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
	
	// Save gyro bias Z
	param.id = PARAM_ID_IMU1_GYRO_BIAS_Z;
	param.value = imu->gyro_offset[5];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
}

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
			imu->state = init;
			uint8_t result = 0;
			publish(imu->topic_gyro_calibration_update, (uint8_t*)&result, 1);
			return;
		}

		imu->gyro_offset[3] = (double)(1.0 / CALIBRATION_FREQ) * imu->gyro_calibration[0];
		imu->gyro_offset[4] = (double)(1.0 / CALIBRATION_FREQ) * imu->gyro_calibration[1];
		imu->gyro_offset[5] = (double)(1.0 / CALIBRATION_FREQ) * imu->gyro_calibration[2];
		imu->state = ready;
		uint8_t result = 1;
		publish(imu->topic_gyro_calibration_update, (uint8_t*)&result, 1);
		
		// Save calibration status and gyro biases to flash
		save_calibration_data(imu);
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
	
	g_imu1.state = calibrating;
}

static void imu1_loop(uint8_t *data, size_t size) {
	icm42688p_read(&g_imu1.imu_sensor);
}

static void imu1_data_update(void) {	
	if (g_imu1.state == ready) {
		g_imu1.gyro_accel[3] -= g_imu1.gyro_offset[3];
		g_imu1.gyro_accel[4] -= g_imu1.gyro_offset[4];
		g_imu1.gyro_accel[5] -= g_imu1.gyro_offset[5];

		g_imu1.gyro_accel[3] = g_imu1.gyro_accel[3] / SSF_GYRO;
		g_imu1.gyro_accel[4] = g_imu1.gyro_accel[4] / SSF_GYRO;
		g_imu1.gyro_accel[5] = g_imu1.gyro_accel[5] / SSF_GYRO;
		publish(g_imu1.topic_gyro_update, (uint8_t*)&g_imu1.gyro_accel[3], 12);
	}
	else if (g_imu1.state == calibrating) {
		calibrate(&g_imu1);
	}
}

static void imu1_i2c_data_update(uint8_t *data, size_t size) {
	if (data[0] == I2C_PORT1) {
		icm42688p_get_i2c(&g_imu1.imu_sensor, g_imu1.gyro_accel);
		imu1_data_update();
	}
}

static void imu1_spi_data_update(uint8_t *data, size_t size) {
	if (data[0] == SPI_PORT1) {
		icm42688p_get_spi(&g_imu1.imu_sensor, g_imu1.gyro_accel);
		imu1_data_update();
	}
}

static void publish_accel_loop(uint8_t *data, size_t size) {
	if (g_imu1.state == ready) {
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
	
	float ax = g_imu1.gyro_accel[0];
	float ay = g_imu1.gyro_accel[1];
	float az = g_imu1.gyro_accel[2];
	
	memcpy(&g_monitor_msg[0], &ax, sizeof(float));
	memcpy(&g_monitor_msg[4], &ay, sizeof(float));
	memcpy(&g_monitor_msg[8], &az, sizeof(float));
	
	publish(MONITOR_DATA, g_monitor_msg, sizeof(g_monitor_msg));
}
#endif

static void check_gyro_calibration(uint8_t *data, size_t size) {
	// Request calibration status from local storage
	param_id_e param_id = PARAM_ID_IMU1_GYRO_CALIBRATED;
	publish(LOCAL_STORAGE_LOAD, (uint8_t*)&param_id, sizeof(param_id_e));
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 0) {
		// Calibration failed, retry
		imu1_calibrate(NULL, 0);
	}
}

static void handle_storage_result(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) {
		return;
	}
	
	param_storage_t *param = (param_storage_t *)data;
	
	switch (param->id) {
		case PARAM_ID_IMU1_GYRO_CALIBRATED:
			if (param->value > 0.0f) {
				// IMU is calibrated, load all 3 bias values from storage
				param_id_e bias_x = PARAM_ID_IMU1_GYRO_BIAS_X;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&bias_x, sizeof(param_id_e));
				
				param_id_e bias_y = PARAM_ID_IMU1_GYRO_BIAS_Y;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&bias_y, sizeof(param_id_e));
				
				param_id_e bias_z = PARAM_ID_IMU1_GYRO_BIAS_Z;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&bias_z, sizeof(param_id_e));
			} else {
				// IMU not calibrated, start calibration
				imu1_calibrate(NULL, 0);
			}
			break;
			
		case PARAM_ID_IMU1_GYRO_BIAS_X:
			g_imu1.gyro_offset[3] = param->value;
			break;
			
		case PARAM_ID_IMU1_GYRO_BIAS_Y:
			g_imu1.gyro_offset[4] = param->value;
			break;
			
		case PARAM_ID_IMU1_GYRO_BIAS_Z:
			g_imu1.gyro_offset[5] = param->value;
			// All biases loaded, set mode to ready
			g_imu1.state = ready;

			uint8_t result = 1;
			publish(g_imu1.topic_gyro_calibration_update, (uint8_t*)&result, 1);
			break;
			
		default:
			break;
	}
}

void imu_setup(void) {
	icm42688p_init(&g_imu1.imu_sensor,
		AFS_2G, GFS_2000DPS, 
		AODR_500Hz, GODR_4kHz,
		accel_mode_LN, gyro_mode_LN);
	subscribe(SCHEDULER_1KHZ, imu1_loop);
	subscribe(SCHEDULER_500HZ, publish_accel_loop);
	subscribe(I2C_CALLBACK_UPDATE, imu1_i2c_data_update);
	subscribe(SPI_CALLBACK_UPDATE, imu1_spi_data_update);
	subscribe(SENSOR_IMU1_CALIBRATE_GYRO, imu1_calibrate);
	subscribe(SENSOR_IMU1_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	subscribe(SENSOR_CHECK_GYRO_CALIBRATION, check_gyro_calibration);
	subscribe(LOCAL_STORAGE_RESULT, handle_storage_result);
#if ENABLE_ACCEL_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
	
	// Publish module initialized status
	module_initialized_t module_initialized = {.id = MODULE_ID_IMU, .initialized = 1};
	publish(MODULE_INITIALIZED_UPDATE, (uint8_t*)&module_initialized, sizeof(module_initialized_t));
}
