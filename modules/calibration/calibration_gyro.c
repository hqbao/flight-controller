#include "calibration_gyro.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <messages.h>

/*
 * --- GYROSCOPE TEMPERATURE COMPENSATION (OTA Upload + Flash Persistence) ---
 *
 * Calibration model:
 *    V_cal = V_raw - bias(T)
 *    bias(T) = a·T² + b·T + c  per axis (degree-2 polynomial)
 *    Uploaded via DB_CMD_CALIBRATE_GYRO_TEMP (9 floats).
 *
 * Responsibilities:
 *   - Load temp compensation from flash at boot
 *   - Save coefficients to flash when received from host
 *   - Publish CALIBRATION_GYRO_READY when calibration is available
 *   - Respond to CALIBRATION_GYRO_REQUEST by re-publishing calibration
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

/* --- Delivery --- */

void calibration_gyro_deliver(void) {
	calibration_gyro_t msg;
	memcpy(msg.temp_coeff, g_temp_coeff, sizeof(g_temp_coeff));
	publish(CALIBRATION_GYRO_READY, (uint8_t*)&msg, sizeof(calibration_gyro_t));
}

/* --- Re-delivery request from any module --- */

static void on_calibration_request(uint8_t *data, size_t size) {
	calibration_gyro_deliver();
}

/* --- OTA upload from Python tool --- */

static void on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;

	if (data[0] == DB_CMD_CALIBRATE_GYRO_TEMP) {
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
		calibration_gyro_deliver();

		uint8_t status = 1;
		publish(CALIBRATION_GYRO_STATUS, &status, 1);
	}
}

/* --- Flash load on boot --- */

static void on_storage_result(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;

	param_storage_t *param = (param_storage_t *)data;

	switch (param->id) {
		case PARAM_ID_GYRO_TEMP_CALIBRATED:
			if (param->value > 0.0f) {
				param_id_e id;
				id = PARAM_ID_GYRO_TEMP_X_A;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_X_B;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_X_C;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_Y_A;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_Y_B;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_Y_C;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_Z_A;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_Z_B;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				id = PARAM_ID_GYRO_TEMP_Z_C;
				publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
			} else {
				/* No temp comp in flash — deliver zeroed coefficients */
				calibration_gyro_deliver();
				uint8_t status = 0;
				publish(CALIBRATION_GYRO_STATUS, &status, 1);
			}
			break;

		case PARAM_ID_GYRO_TEMP_X_A: g_temp_coeff[0][0] = param->value; break;
		case PARAM_ID_GYRO_TEMP_X_B: g_temp_coeff[0][1] = param->value; break;
		case PARAM_ID_GYRO_TEMP_X_C: g_temp_coeff[0][2] = param->value; break;
		case PARAM_ID_GYRO_TEMP_Y_A: g_temp_coeff[1][0] = param->value; break;
		case PARAM_ID_GYRO_TEMP_Y_B: g_temp_coeff[1][1] = param->value; break;
		case PARAM_ID_GYRO_TEMP_Y_C: g_temp_coeff[1][2] = param->value; break;
		case PARAM_ID_GYRO_TEMP_Z_A: g_temp_coeff[2][0] = param->value; break;
		case PARAM_ID_GYRO_TEMP_Z_B: g_temp_coeff[2][1] = param->value; break;

		case PARAM_ID_GYRO_TEMP_Z_C:
			g_temp_coeff[2][2] = param->value;
			/* All temp coefficients loaded — deliver full calibration */
			calibration_gyro_deliver();
			{
				uint8_t status = 1;
				publish(CALIBRATION_GYRO_STATUS, &status, 1);
			}
			break;

		default:
			break;
	}
}

/* --- Setup --- */

void calibration_gyro_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, on_storage_result);
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
	subscribe(CALIBRATION_GYRO_REQUEST, on_calibration_request);

	/* Load temp compensation from flash */
	param_id_e param_id = PARAM_ID_GYRO_TEMP_CALIBRATED;
	publish(LOCAL_STORAGE_LOAD, (uint8_t*)&param_id, sizeof(param_id_e));
}
