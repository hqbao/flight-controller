#include "calibration_accel.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

/*
 * --- ACCELEROMETER CALIBRATION (OTA Upload + Flash Persistence) ---
 *
 * Calibration procedure:
 *   1. Connect board via USB, run: python3 tools/calibration_accel.py
 *   2. Click "Start Log" — yellow dot shows current raw reading
 *   3. Place drone in 6 static orientations (flat, left, right,
 *      nose-up, nose-down, inverted), click "Capture Position" for each
 *   4. Each position averages 100 samples for noise reduction
 *   5. Click "Compute Calib" — tool fits an ellipsoid to the 6 data points
 *   6. Click "Upload to FC" — tool transmits bias & scale via UART
 *      Flight controller saves to flash automatically.
 *
 * Mathematical model (ellipsoid fit → unit sphere):
 *   - Solves: Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
 *   - Bias B = center of the fitted ellipsoid (in raw LSB units)
 *   - Scale S = symmetric 3x3 matrix that maps the ellipsoid to a unit sphere
 *     (corrects scale errors, cross-axis coupling, and misalignment)
 *
 * Units:
 *   - bias[3]:      raw LSB values (ICM-42688P at ±2g ≈ 16384 LSB/g)
 *   - scale[3][3]:  dimensionless (ellipsoid → unit sphere transform)
 *
 * Applied in imu.c at 500 Hz:  V_cal = S * (V_raw - B)
 *
 * Identity matrix + zero bias = uncalibrated (raw passthrough).
 * On boot: loads from flash if previously calibrated, otherwise uses defaults.
 *
 * Responsibilities:
 *   - Load calibration from flash at boot
 *   - Save calibration to flash when received from host
 *   - Publish CALIBRATION_ACCEL_READY when calibration is available
 *   - Respond to CALIBRATION_ACCEL_REQUEST by re-publishing calibration
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

/* --- DB message handler (OTA upload from Python tool) --- */

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
	calibration_accel_deliver();

	uint8_t status = 1;
	publish(CALIBRATION_ACCEL_STATUS, &status, 1);
}

/* --- Flash load on boot --- */

static void on_storage_result(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;

	param_storage_t *param = (param_storage_t *)data;

	switch (param->id) {
		case PARAM_ID_ACCEL_CALIBRATED:
			if (param->value > 0.0f) {
				/* Calibrated — load all values from flash */
				const param_id_e load_ids[] = {
					PARAM_ID_ACCEL_BIAS_X, PARAM_ID_ACCEL_BIAS_Y, PARAM_ID_ACCEL_BIAS_Z,
					PARAM_ID_ACCEL_SCALE_00, PARAM_ID_ACCEL_SCALE_01, PARAM_ID_ACCEL_SCALE_02,
					PARAM_ID_ACCEL_SCALE_10, PARAM_ID_ACCEL_SCALE_11, PARAM_ID_ACCEL_SCALE_12,
					PARAM_ID_ACCEL_SCALE_20, PARAM_ID_ACCEL_SCALE_21, PARAM_ID_ACCEL_SCALE_22,
				};
				for (int i = 0; i < 12; i++) {
					param_id_e id = load_ids[i];
					publish(LOCAL_STORAGE_LOAD, (uint8_t*)&id, sizeof(param_id_e));
				}
			} else {
				/* Not calibrated — notify state detector */
				uint8_t status = 0;
				publish(CALIBRATION_ACCEL_STATUS, &status, 1);
			}
			/* else: not calibrated, keep identity defaults */
			break;

		case PARAM_ID_ACCEL_BIAS_X:  g_accel_cal.bias[0] = param->value; break;
		case PARAM_ID_ACCEL_BIAS_Y:  g_accel_cal.bias[1] = param->value; break;
		case PARAM_ID_ACCEL_BIAS_Z:  g_accel_cal.bias[2] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_00: g_accel_cal.scale[0][0] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_01: g_accel_cal.scale[0][1] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_02: g_accel_cal.scale[0][2] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_10: g_accel_cal.scale[1][0] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_11: g_accel_cal.scale[1][1] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_12: g_accel_cal.scale[1][2] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_20: g_accel_cal.scale[2][0] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_21: g_accel_cal.scale[2][1] = param->value; break;
		case PARAM_ID_ACCEL_SCALE_22: g_accel_cal.scale[2][2] = param->value;
			/* All values loaded — notify state detector */
			{
				uint8_t status = 1;
				publish(CALIBRATION_ACCEL_STATUS, &status, 1);
			}
			break;
		default: break;
	}
}

void calibration_accel_deliver(void) {
	publish(CALIBRATION_ACCEL_READY, (uint8_t*)&g_accel_cal, sizeof(calibration_accel_t));
}

/* --- Re-delivery request from any module --- */

static void on_calibration_request(uint8_t *data, size_t size) {
	calibration_accel_deliver();
}

void calibration_accel_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, on_storage_result);
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
	subscribe(CALIBRATION_ACCEL_REQUEST, on_calibration_request);

	/* Load calibration status from flash (local_storage already initialized) */
	param_id_e param_id = PARAM_ID_ACCEL_CALIBRATED;
	publish(LOCAL_STORAGE_LOAD, (uint8_t*)&param_id, sizeof(param_id_e));
}
