/**
 * CONFIG — Central configuration distributor.
 *
 * Loads all calibration parameters from local_storage and publishes them
 * to consuming modules via PubSub. This decouples sensor modules (IMU,
 * compass) from local_storage — they only subscribe to config topics.
 *
 * Boot sequence (runs 5 times at 1 Hz):
 *   1. Request calibrated flags from local_storage
 *   2. If a calibrated flag > 0, cascade-load all coefficients for that group
 *   3. Publish assembled structs on CALIBRATION_*_READY topics
 *   4. Publish CALIBRATION_*_STATUS (0 or 1) for fault detection
 *
 * Live updates:
 *   When a calibration module saves new data via LOCAL_STORAGE_SAVE, this
 *   module accumulates the coefficients and re-publishes the assembled struct
 *   once the last coefficient in each group arrives.
 *
 * Request handling:
 *   Subscribes to CALIBRATION_*_REQUEST topics so any module can request
 *   re-delivery of calibration data at any time.
 */
#include "config.h"
#include <pubsub.h>
#include <messages.h>
#include <string.h>

#define PUBLISH_COUNT  5

/* --- Gyro temp compensation --- */
static calibration_gyro_t g_gyro_cal = { .temp_coeff = {{0}} };
static uint8_t g_gyro_loaded = 0;

/* --- Accel calibration --- */
static calibration_accel_t g_accel_cal = {
	.bias = {0, 0, 0},
	.scale = {{1,0,0},{0,1,0},{0,0,1}}
};
static uint8_t g_accel_loaded = 0;

/* --- Mag calibration (double precision at runtime, float in flash) --- */
static calibration_mag_t g_mag_cal = {
	.offset = {0, 0, 0},
	.scale = {{1,0,0},{0,1,0},{0,0,1}}
};
static uint8_t g_mag_loaded = 0;

static int g_publish_remaining = PUBLISH_COUNT;

/* --- Publish all loaded calibrations to consuming modules -------------- */

static void publish_all(void) {
	if (g_gyro_loaded) {
		publish(CALIBRATION_GYRO_READY,
			(uint8_t *)&g_gyro_cal, sizeof(calibration_gyro_t));
		uint8_t status = 1;
		publish(CALIBRATION_GYRO_STATUS, &status, 1);
	}
	if (g_accel_loaded) {
		publish(CALIBRATION_ACCEL_READY,
			(uint8_t *)&g_accel_cal, sizeof(calibration_accel_t));
		uint8_t status = 1;
		publish(CALIBRATION_ACCEL_STATUS, &status, 1);
	}
	if (g_mag_loaded) {
		publish(CALIBRATION_MAG_READY,
			(uint8_t *)&g_mag_cal, sizeof(calibration_mag_t));
		uint8_t status = 1;
		publish(CALIBRATION_MAG_STATUS, &status, 1);
	}
}

/* --- Helper: request a range of param IDs from local_storage ----------- */

static void request_params(param_id_e first, param_id_e last) {
	for (param_id_e id = first; id <= last; id++)
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
}

/* --- Handle LOCAL_STORAGE_RESULT: populate state from flash ------------ */

static void on_storage_result(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;
	param_storage_t *p = (param_storage_t *)data;

	switch (p->id) {

	/* --- Gyro temp compensation: calibrated flag cascades load --- */
	case PARAM_ID_GYRO_TEMP_CALIBRATED:
		if (p->value > 0.0f)
			request_params(PARAM_ID_GYRO_TEMP_X_A, PARAM_ID_GYRO_TEMP_Z_C);
		else {
			/* Not calibrated — deliver zeroed coefficients + status=0 */
			publish(CALIBRATION_GYRO_READY,
				(uint8_t *)&g_gyro_cal, sizeof(calibration_gyro_t));
			uint8_t status = 0;
			publish(CALIBRATION_GYRO_STATUS, &status, 1);
		}
		break;
	case PARAM_ID_GYRO_TEMP_X_A: g_gyro_cal.temp_coeff[0][0] = p->value; break;
	case PARAM_ID_GYRO_TEMP_X_B: g_gyro_cal.temp_coeff[0][1] = p->value; break;
	case PARAM_ID_GYRO_TEMP_X_C: g_gyro_cal.temp_coeff[0][2] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Y_A: g_gyro_cal.temp_coeff[1][0] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Y_B: g_gyro_cal.temp_coeff[1][1] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Y_C: g_gyro_cal.temp_coeff[1][2] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Z_A: g_gyro_cal.temp_coeff[2][0] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Z_B: g_gyro_cal.temp_coeff[2][1] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Z_C:
		g_gyro_cal.temp_coeff[2][2] = p->value;
		g_gyro_loaded = 1;
		break;

	/* --- Accel calibration: calibrated flag cascades load --- */
	case PARAM_ID_ACCEL_CALIBRATED:
		if (p->value > 0.0f)
			request_params(PARAM_ID_ACCEL_BIAS_X, PARAM_ID_ACCEL_SCALE_22);
		else {
			uint8_t status = 0;
			publish(CALIBRATION_ACCEL_STATUS, &status, 1);
		}
		break;
	case PARAM_ID_ACCEL_BIAS_X:   g_accel_cal.bias[0] = p->value; break;
	case PARAM_ID_ACCEL_BIAS_Y:   g_accel_cal.bias[1] = p->value; break;
	case PARAM_ID_ACCEL_BIAS_Z:   g_accel_cal.bias[2] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_00: g_accel_cal.scale[0][0] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_01: g_accel_cal.scale[0][1] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_02: g_accel_cal.scale[0][2] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_10: g_accel_cal.scale[1][0] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_11: g_accel_cal.scale[1][1] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_12: g_accel_cal.scale[1][2] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_20: g_accel_cal.scale[2][0] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_21: g_accel_cal.scale[2][1] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_22:
		g_accel_cal.scale[2][2] = p->value;
		g_accel_loaded = 1;
		break;

	/* --- Mag calibration: calibrated flag cascades load (float→double) --- */
	case PARAM_ID_MAG_CALIBRATED:
		if (p->value > 0.0f)
			request_params(PARAM_ID_MAG_OFFSET_X, PARAM_ID_MAG_SCALE_22);
		else {
			uint8_t status = 0;
			publish(CALIBRATION_MAG_STATUS, &status, 1);
		}
		break;
	case PARAM_ID_MAG_OFFSET_X:  g_mag_cal.offset[0] = (double)p->value; break;
	case PARAM_ID_MAG_OFFSET_Y:  g_mag_cal.offset[1] = (double)p->value; break;
	case PARAM_ID_MAG_OFFSET_Z:  g_mag_cal.offset[2] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_00:  g_mag_cal.scale[0][0] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_01:  g_mag_cal.scale[0][1] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_02:  g_mag_cal.scale[0][2] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_10:  g_mag_cal.scale[1][0] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_11:  g_mag_cal.scale[1][1] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_12:  g_mag_cal.scale[1][2] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_20:  g_mag_cal.scale[2][0] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_21:  g_mag_cal.scale[2][1] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_22:
		g_mag_cal.scale[2][2] = (double)p->value;
		g_mag_loaded = 1;
		break;

	default: break;
	}
}

/* --- Handle LOCAL_STORAGE_SAVE: re-publish on live calibration writes --
 *
 * Calibration modules save coefficients sequentially via LOCAL_STORAGE_SAVE.
 * We accumulate each value and publish the assembled struct when the last
 * coefficient in each group arrives.
 */
static void on_storage_save(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;
	param_storage_t *p = (param_storage_t *)data;

	switch (p->id) {

	/* --- Gyro temp (save order: CALIBRATED, X_A..Z_C) --- */
	case PARAM_ID_GYRO_TEMP_X_A: g_gyro_cal.temp_coeff[0][0] = p->value; break;
	case PARAM_ID_GYRO_TEMP_X_B: g_gyro_cal.temp_coeff[0][1] = p->value; break;
	case PARAM_ID_GYRO_TEMP_X_C: g_gyro_cal.temp_coeff[0][2] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Y_A: g_gyro_cal.temp_coeff[1][0] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Y_B: g_gyro_cal.temp_coeff[1][1] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Y_C: g_gyro_cal.temp_coeff[1][2] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Z_A: g_gyro_cal.temp_coeff[2][0] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Z_B: g_gyro_cal.temp_coeff[2][1] = p->value; break;
	case PARAM_ID_GYRO_TEMP_Z_C:
		g_gyro_cal.temp_coeff[2][2] = p->value;
		g_gyro_loaded = 1;
		publish(CALIBRATION_GYRO_READY,
			(uint8_t *)&g_gyro_cal, sizeof(calibration_gyro_t));
		{
			uint8_t status = 1;
			publish(CALIBRATION_GYRO_STATUS, &status, 1);
		}
		break;

	/* --- Accel (save order: CALIBRATED, BIAS_X..SCALE_22) --- */
	case PARAM_ID_ACCEL_BIAS_X:   g_accel_cal.bias[0] = p->value; break;
	case PARAM_ID_ACCEL_BIAS_Y:   g_accel_cal.bias[1] = p->value; break;
	case PARAM_ID_ACCEL_BIAS_Z:   g_accel_cal.bias[2] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_00: g_accel_cal.scale[0][0] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_01: g_accel_cal.scale[0][1] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_02: g_accel_cal.scale[0][2] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_10: g_accel_cal.scale[1][0] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_11: g_accel_cal.scale[1][1] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_12: g_accel_cal.scale[1][2] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_20: g_accel_cal.scale[2][0] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_21: g_accel_cal.scale[2][1] = p->value; break;
	case PARAM_ID_ACCEL_SCALE_22:
		g_accel_cal.scale[2][2] = p->value;
		g_accel_loaded = 1;
		publish(CALIBRATION_ACCEL_READY,
			(uint8_t *)&g_accel_cal, sizeof(calibration_accel_t));
		{
			uint8_t status = 1;
			publish(CALIBRATION_ACCEL_STATUS, &status, 1);
		}
		break;

	/* --- Mag (save order: CALIBRATED, OFFSET_X..SCALE_22, float→double) --- */
	case PARAM_ID_MAG_OFFSET_X:  g_mag_cal.offset[0] = (double)p->value; break;
	case PARAM_ID_MAG_OFFSET_Y:  g_mag_cal.offset[1] = (double)p->value; break;
	case PARAM_ID_MAG_OFFSET_Z:  g_mag_cal.offset[2] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_00:  g_mag_cal.scale[0][0] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_01:  g_mag_cal.scale[0][1] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_02:  g_mag_cal.scale[0][2] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_10:  g_mag_cal.scale[1][0] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_11:  g_mag_cal.scale[1][1] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_12:  g_mag_cal.scale[1][2] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_20:  g_mag_cal.scale[2][0] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_21:  g_mag_cal.scale[2][1] = (double)p->value; break;
	case PARAM_ID_MAG_SCALE_22:
		g_mag_cal.scale[2][2] = (double)p->value;
		g_mag_loaded = 1;
		publish(CALIBRATION_MAG_READY,
			(uint8_t *)&g_mag_cal, sizeof(calibration_mag_t));
		{
			uint8_t status = 1;
			publish(CALIBRATION_MAG_STATUS, &status, 1);
		}
		break;

	default: break;
	}
}

/* --- Request handlers (re-deliver on demand from any module) ----------- */

static void on_gyro_request(uint8_t *data, size_t size) {
	publish(CALIBRATION_GYRO_READY,
		(uint8_t *)&g_gyro_cal, sizeof(calibration_gyro_t));
}

static void on_accel_request(uint8_t *data, size_t size) {
	publish(CALIBRATION_ACCEL_READY,
		(uint8_t *)&g_accel_cal, sizeof(calibration_accel_t));
}

static void on_mag_request(uint8_t *data, size_t size) {
	publish(CALIBRATION_MAG_READY,
		(uint8_t *)&g_mag_cal, sizeof(calibration_mag_t));
}

/* --- Boot: request from flash + publish at 1 Hz for 5 seconds --------- */

static void on_scheduler_1hz(uint8_t *data, size_t size) {
	if (g_publish_remaining <= 0) return;

	/* Request unloaded values from local_storage */
	param_id_e id;
	if (!g_gyro_loaded) {
		id = PARAM_ID_GYRO_TEMP_CALIBRATED;
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
	}
	if (!g_accel_loaded) {
		id = PARAM_ID_ACCEL_CALIBRATED;
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
	}
	if (!g_mag_loaded) {
		id = PARAM_ID_MAG_CALIBRATED;
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
	}

	publish_all();
	g_publish_remaining--;
}

/* --- Setup ------------------------------------------------------------- */

void config_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, on_storage_result);
	subscribe(LOCAL_STORAGE_SAVE, on_storage_save);
	subscribe(SCHEDULER_1HZ, on_scheduler_1hz);
	subscribe(CALIBRATION_GYRO_REQUEST, on_gyro_request);
	subscribe(CALIBRATION_ACCEL_REQUEST, on_accel_request);
	subscribe(CALIBRATION_MAG_REQUEST, on_mag_request);
}
