#include "config_internal.h"

static calibration_gyro_t g_gyro_cal = { .temp_coeff = {{0}} };
static uint8_t g_gyro_loaded = 0;

void config_gyro_on_result(param_storage_t *p) {
	switch (p->id) {

	case PARAM_ID_GYRO_TEMP_CALIBRATED:
		if (p->value > 0.0f)
			config_request_params(PARAM_ID_GYRO_TEMP_X_A, PARAM_ID_GYRO_TEMP_Z_C);
		else {
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

	default: break;
	}
}

void config_gyro_on_save(param_storage_t *p) {
	switch (p->id) {

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

	default: break;
	}
}

void config_gyro_publish(void) {
	if (g_gyro_loaded) {
		publish(CALIBRATION_GYRO_READY,
			(uint8_t *)&g_gyro_cal, sizeof(calibration_gyro_t));
		uint8_t status = 1;
		publish(CALIBRATION_GYRO_STATUS, &status, 1);
	}
}

void config_gyro_request_load(void) {
	if (!g_gyro_loaded) {
		param_id_e id = PARAM_ID_GYRO_TEMP_CALIBRATED;
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
	}
}

void config_gyro_on_request(uint8_t *data, size_t size) {
	publish(CALIBRATION_GYRO_READY,
		(uint8_t *)&g_gyro_cal, sizeof(calibration_gyro_t));
}
