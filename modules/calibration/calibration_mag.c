#include "calibration_mag.h"
#include <pubsub.h>
#include <string.h>
#include <messages.h>

/*
 * --- COMPASS CALIBRATION (OTA Upload + Flash Save) ---
 *
 * Mathematical model (same ellipsoid fit as accelerometer):
 *   V_cal = S × (V_raw − B)
 *   B = hard iron bias vector   (3-element, double)
 *   S = soft iron scale matrix  (3×3, double)
 *
 * Units:
 *   - offset[3]:    microtesla (µT), BMM350 compensated output
 *   - scale[3][3]:  dimensionless (ellipsoid → unit sphere transform)
 *
 * Storage: mag calibration uses doubles at runtime but is stored as floats
 * in flash (4 bytes per param). Float32 precision (~7 digits) is sufficient
 * for µT-range magnetometer values.
 *
 * Flash loading and delivery to consumers is handled by the config module.
 * This module only handles OTA upload and saving to flash.
 */

static calibration_mag_t g_mag_cal = {
	.offset = {0.0, 0.0, 0.0},
	.scale = {
		{1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0},
		{0.0, 0.0, 1.0}
	}
};

/* --- Flash persistence (double → float for 4-byte param storage) --- */

static void save_mag_calibration(void) {
	param_storage_t param;

	param.id = PARAM_ID_MAG_CALIBRATED;
	param.value = 1.0f;
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* Offset (double → float) */
	param.id = PARAM_ID_MAG_OFFSET_X;
	param.value = (float)g_mag_cal.offset[0];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_MAG_OFFSET_Y;
	param.value = (float)g_mag_cal.offset[1];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	param.id = PARAM_ID_MAG_OFFSET_Z;
	param.value = (float)g_mag_cal.offset[2];
	publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));

	/* Scale matrix (double → float, row-major) */
	const param_id_e scale_ids[9] = {
		PARAM_ID_MAG_SCALE_00, PARAM_ID_MAG_SCALE_01, PARAM_ID_MAG_SCALE_02,
		PARAM_ID_MAG_SCALE_10, PARAM_ID_MAG_SCALE_11, PARAM_ID_MAG_SCALE_12,
		PARAM_ID_MAG_SCALE_20, PARAM_ID_MAG_SCALE_21, PARAM_ID_MAG_SCALE_22,
	};
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			param.id = scale_ids[r * 3 + c];
			param.value = (float)g_mag_cal.scale[r][c];
			publish(LOCAL_STORAGE_SAVE, (uint8_t*)&param, sizeof(param_storage_t));
		}
	}
}

/* --- OTA upload from Python tool --- */

static void on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;
	if (data[0] != DB_CMD_CALIBRATE_MAG) return;

	/* Payload: 12 floats = 48 bytes (offset[3] + scale[3][3]) at data[4] */
	if (size < 4 + 48) return;

	/* Extract floats via memcpy (safe regardless of buffer alignment) */
	float values[12];
	memcpy(values, &data[4], 12 * sizeof(float));

	g_mag_cal.offset[0] = (double)values[0];
	g_mag_cal.offset[1] = (double)values[1];
	g_mag_cal.offset[2] = (double)values[2];
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			g_mag_cal.scale[r][c] = (double)values[3 + r * 3 + c];
		}
	}

	save_mag_calibration();
}

/* --- Setup --- */

void calibration_mag_setup(void) {
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
}
