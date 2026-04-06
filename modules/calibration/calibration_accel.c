#include "calibration_accel.h"
#include <pubsub.h>
#include <string.h>
#include <messages.h>

/*
 * --- ACCELEROMETER CALIBRATION (OTA Upload + Flash Save) ---
 *
 * Mathematical model (ellipsoid fit → unit sphere):
 *   V_cal = S * (V_raw - B)
 *
 * Units:
 *   - bias[3]:      raw LSB values (ICM-42688P at ±2g ≈ 16384 LSB/g)
 *   - scale[3][3]:  dimensionless (ellipsoid → unit sphere transform)
 *
 * Flash loading and delivery to consumers is handled by the config module.
 * This module only handles OTA upload and saving to flash.
 */

static calibration_accel_t g_accel_cal = {
	.bias = {0.0f, 0.0f, 0.0f},
	.scale = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}
	}
};

/* --- Flash persistence --- */

static void save_accel_calibration(void) {
	param_storage_t param;

	param.id = PARAM_ID_ACCEL_CALIBRATED;
	param.value = 1.0f;
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* Bias */
	param.id = PARAM_ID_ACCEL_BIAS_X;
	param.value = g_accel_cal.bias[0];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_ACCEL_BIAS_Y;
	param.value = g_accel_cal.bias[1];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_ACCEL_BIAS_Z;
	param.value = g_accel_cal.bias[2];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* Scale matrix (row-major) */
	const param_id_e scale_ids[9] = {
		PARAM_ID_ACCEL_SCALE_00, PARAM_ID_ACCEL_SCALE_01, PARAM_ID_ACCEL_SCALE_02,
		PARAM_ID_ACCEL_SCALE_10, PARAM_ID_ACCEL_SCALE_11, PARAM_ID_ACCEL_SCALE_12,
		PARAM_ID_ACCEL_SCALE_20, PARAM_ID_ACCEL_SCALE_21, PARAM_ID_ACCEL_SCALE_22,
	};
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			param.id = scale_ids[r * 3 + c];
			param.value = g_accel_cal.scale[r][c];
			publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
		}
	}
}

/* --- OTA upload from Python tool --- */

static void on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;
	if (data[0] != DB_CMD_CALIBRATE_ACCEL) return;

	/* Payload: 12 floats = 48 bytes (bias[3] + scale[3][3]) at data[4] */
	if (size < 4 + 48) return;

	/* Extract floats via memcpy (safe regardless of buffer alignment) */
	float values[12];
	memcpy(values, &data[4], 12 * sizeof(float));

	g_accel_cal.bias[0] = values[0];
	g_accel_cal.bias[1] = values[1];
	g_accel_cal.bias[2] = values[2];
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			g_accel_cal.scale[r][c] = values[3 + r * 3 + c];
		}
	}

	save_accel_calibration();
}

/* --- Setup --- */

void calibration_accel_setup(void) {
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
}
