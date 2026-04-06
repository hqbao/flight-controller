#include "calibration_gyro.h"
#include <pubsub.h>
#include <string.h>
#include <messages.h>

/*
 * --- GYROSCOPE TEMPERATURE COMPENSATION (OTA Upload + Flash Save) ---
 *
 * Calibration model:
 *    V_cal = V_raw - bias(T)
 *    bias(T) = a·T² + b·T + c  per axis (degree-2 polynomial)
 *    Uploaded via DB_CMD_CALIBRATE_GYRO_TEMP (9 floats).
 *
 * Flash loading and delivery to consumers is handled by the config module.
 * This module only handles OTA upload and saving to flash.
 */

static float g_temp_coeff[3][3] = {0};  /* [axis][0]=a, [1]=b, [2]=c */

/* --- Flash persistence --- */

static void save_temp_calibration(void) {
	param_storage_t param;

	param.id = PARAM_ID_GYRO_TEMP_CALIBRATED;
	param.value = 1.0f;
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* X axis: a, b, c */
	param.id = PARAM_ID_GYRO_TEMP_X_A;
	param.value = g_temp_coeff[0][0];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_GYRO_TEMP_X_B;
	param.value = g_temp_coeff[0][1];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_GYRO_TEMP_X_C;
	param.value = g_temp_coeff[0][2];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* Y axis: a, b, c */
	param.id = PARAM_ID_GYRO_TEMP_Y_A;
	param.value = g_temp_coeff[1][0];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_GYRO_TEMP_Y_B;
	param.value = g_temp_coeff[1][1];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_GYRO_TEMP_Y_C;
	param.value = g_temp_coeff[1][2];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* Z axis: a, b, c */
	param.id = PARAM_ID_GYRO_TEMP_Z_A;
	param.value = g_temp_coeff[2][0];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_GYRO_TEMP_Z_B;
	param.value = g_temp_coeff[2][1];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_GYRO_TEMP_Z_C;
	param.value = g_temp_coeff[2][2];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
}

/* --- OTA upload from Python tool --- */

static void on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;
	if (data[0] != DB_CMD_CALIBRATE_GYRO_TEMP) return;

	/* Temp polynomial: 9 floats = 36 bytes at data[4] */
	/* Layout: [Xa, Xb, Xc, Ya, Yb, Yc, Za, Zb, Zc] */
	if (size < 4 + 36) return;

	float values[9];
	memcpy(values, &data[4], 9 * sizeof(float));
	g_temp_coeff[0][0] = values[0];  /* X: a */
	g_temp_coeff[0][1] = values[1];  /* X: b */
	g_temp_coeff[0][2] = values[2];  /* X: c */
	g_temp_coeff[1][0] = values[3];  /* Y: a */
	g_temp_coeff[1][1] = values[4];  /* Y: b */
	g_temp_coeff[1][2] = values[5];  /* Y: c */
	g_temp_coeff[2][0] = values[6];  /* Z: a */
	g_temp_coeff[2][1] = values[7];  /* Z: b */
	g_temp_coeff[2][2] = values[8];  /* Z: c */

	save_temp_calibration();
}

/* --- Setup --- */

void calibration_gyro_setup(void) {
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
}
