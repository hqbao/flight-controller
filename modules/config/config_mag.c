#include "config_internal.h"

static calibration_mag_t g_mag_cal = {
	.offset = {0, 0, 0},
	.scale = {{1,0,0},{0,1,0},{0,0,1}}
};
static uint8_t g_mag_loaded = 0;

void config_mag_on_result(param_storage_t *p) {
	switch (p->id) {

	case PARAM_ID_MAG_CALIBRATED:
		if (p->value > 0.0f)
			config_request_params(PARAM_ID_MAG_OFFSET_X, PARAM_ID_MAG_SCALE_22);
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

void config_mag_on_save(param_storage_t *p) {
	switch (p->id) {

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

void config_mag_publish(void) {
	if (g_mag_loaded) {
		publish(CALIBRATION_MAG_READY,
			(uint8_t *)&g_mag_cal, sizeof(calibration_mag_t));
		uint8_t status = 1;
		publish(CALIBRATION_MAG_STATUS, &status, 1);
	}
}

void config_mag_request_load(void) {
	if (!g_mag_loaded) {
		param_id_e id = PARAM_ID_MAG_CALIBRATED;
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
	}
}

void config_mag_on_request(uint8_t *data, size_t size) {
	publish(CALIBRATION_MAG_READY,
		(uint8_t *)&g_mag_cal, sizeof(calibration_mag_t));
}
