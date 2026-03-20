#include "calibration_mag.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

/*
 * --- COMPASS CALIBRATION (OTA Upload + Flash Persistence) ---
 *
 * Calibration procedure:
 *   1. Connect board via USB, run: python3 tools/calibration_compass.py
 *   2. Click "Start Log" — yellow dot shows current raw reading
 *   3. Click "Start Stream", then rotate drone in all directions
 *      (figure-8 motion, covering as many orientations as possible)
 *   4. Tool collects continuous samples (≥20 points needed)
 *   5. Ellipsoid fit re-runs automatically every 1 second while streaming
 *   6. Click "Upload to FC" — tool transmits hard/soft iron via UART
 *      Flight controller saves to flash automatically.
 *
 * Mathematical model (same ellipsoid fit as accelerometer):
 *   - Bias B (Hard Iron) = center of the magnetic ellipsoid (in µT)
 *     Corrects permanent magnetic field offsets from PCB traces, motors, etc.
 *   - Scale S (Soft Iron) = 3x3 matrix mapping ellipsoid to unit sphere
 *     Corrects field distortion from nearby ferrous materials
 *
 * Units:
 *   - offset[3]:    microtesla (µT), BMM350 compensated output
 *   - scale[3][3]:  dimensionless (ellipsoid → unit sphere transform)
 *
 * Applied in compass.c at 25 Hz:  V_cal = S * (V_raw - B)
 * Post-calibration: result is normalized to unit vector, Z negated for NED.
 *
 * Storage: mag calibration uses doubles at runtime but is stored as floats
 * in flash (4 bytes per param). Float32 precision (~7 digits) is sufficient
 * for µT-range magnetometer values.
 *
 * On boot: loads from flash if previously calibrated, otherwise uses defaults.
 *
 * Responsibilities:
 *   - Load calibration from flash at boot
 *   - Save calibration to flash when received from host
 *   - Publish CALIBRATION_MAG_READY when calibration is available
 *   - Respond to CALIBRATION_MAG_REQUEST by re-publishing calibration
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

/* --- DB message handler (OTA upload from Python tool) --- */

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
	calibration_mag_deliver();

	uint8_t status = 1;
	publish(CALIBRATION_MAG_STATUS, &status, 1);
}

/* --- Flash load on boot (float → double) --- */

static void on_storage_result(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;

	param_storage_t *param = (param_storage_t *)data;

	switch (param->id) {
		case PARAM_ID_MAG_CALIBRATED:
			if (param->value > 0.0f) {
				/* Calibrated — load all values from flash */
				const param_id_e load_ids[] = {
					PARAM_ID_MAG_OFFSET_X, PARAM_ID_MAG_OFFSET_Y, PARAM_ID_MAG_OFFSET_Z,
					PARAM_ID_MAG_SCALE_00, PARAM_ID_MAG_SCALE_01, PARAM_ID_MAG_SCALE_02,
					PARAM_ID_MAG_SCALE_10, PARAM_ID_MAG_SCALE_11, PARAM_ID_MAG_SCALE_12,
					PARAM_ID_MAG_SCALE_20, PARAM_ID_MAG_SCALE_21, PARAM_ID_MAG_SCALE_22,
				};
				for (int i = 0; i < 12; i++) {
					param_id_e id = load_ids[i];
					publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				}
			} else {
				/* Not calibrated — notify state detector */
				uint8_t status = 0;
				publish(CALIBRATION_MAG_STATUS, &status, 1);
			}
			/* else: not calibrated, keep identity defaults */
			break;

		case PARAM_ID_MAG_OFFSET_X:  g_mag_cal.offset[0] = (double)param->value; break;
		case PARAM_ID_MAG_OFFSET_Y:  g_mag_cal.offset[1] = (double)param->value; break;
		case PARAM_ID_MAG_OFFSET_Z:  g_mag_cal.offset[2] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_00:  g_mag_cal.scale[0][0] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_01:  g_mag_cal.scale[0][1] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_02:  g_mag_cal.scale[0][2] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_10:  g_mag_cal.scale[1][0] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_11:  g_mag_cal.scale[1][1] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_12:  g_mag_cal.scale[1][2] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_20:  g_mag_cal.scale[2][0] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_21:  g_mag_cal.scale[2][1] = (double)param->value; break;
		case PARAM_ID_MAG_SCALE_22:  g_mag_cal.scale[2][2] = (double)param->value;
			/* All values loaded — notify state detector */
			{
				uint8_t status = 1;
				publish(CALIBRATION_MAG_STATUS, &status, 1);
			}
			break;
		default: break;
	}
}

void calibration_mag_deliver(void) {
	publish(CALIBRATION_MAG_READY, (uint8_t*)&g_mag_cal, sizeof(calibration_mag_t));
}

/* --- Re-delivery request from any module --- */

static void on_calibration_request(uint8_t *data, size_t size) {
	calibration_mag_deliver();
}

void calibration_mag_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, on_storage_result);
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
	subscribe(CALIBRATION_MAG_REQUEST, on_calibration_request);

	/* Load calibration status from flash (local_storage already initialized) */
	param_id_e param_id = PARAM_ID_MAG_CALIBRATED;
	publish(LOCAL_STORAGE_LOAD, (uint8_t*)&param_id, sizeof(param_id_e));
}
