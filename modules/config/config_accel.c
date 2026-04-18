#include "config_internal.h"

static calibration_accel_t g_accel_cal = {
	.bias = {0, 0, 0},
	.scale = {{1,0,0},{0,1,0},{0,0,1}}
};
static uint8_t g_accel_loaded = 0;

void config_accel_on_result(param_storage_t *p) {
	switch (p->id) {

	case PARAM_ID_ACCEL_CALIBRATED:
		if (p->value > 0.0f)
			config_request_params(PARAM_ID_ACCEL_BIAS_X, PARAM_ID_ACCEL_SCALE_22);
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

	default: break;
	}
}

void config_accel_on_save(param_storage_t *p) {
	switch (p->id) {

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

	default: break;
	}
}

void config_accel_publish(void) {
	if (g_accel_loaded) {
		publish(CALIBRATION_ACCEL_READY,
			(uint8_t *)&g_accel_cal, sizeof(calibration_accel_t));
		uint8_t status = 1;
		publish(CALIBRATION_ACCEL_STATUS, &status, 1);
	}
}

void config_accel_request_load(void) {
	if (!g_accel_loaded) {
		param_id_e id = PARAM_ID_ACCEL_CALIBRATED;
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
	}
}

void config_accel_on_request(uint8_t *data, size_t size) {
	publish(CALIBRATION_ACCEL_READY,
		(uint8_t *)&g_accel_cal, sizeof(calibration_accel_t));
}
