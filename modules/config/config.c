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
 *
 * Domain-specific logic is split into:
 *   config_gyro.c, config_accel.c, config_mag.c, config_tuning.c
 */
#include "config.h"
#include "config_internal.h"

#define PUBLISH_COUNT  5

static int g_publish_remaining = PUBLISH_COUNT;

/* --- Shared helper: request a range of param IDs from local_storage ---- */

void config_request_params(param_id_e first, param_id_e last) {
	for (param_id_e id = first; id <= last; id++)
		publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(param_id_e));
}

/* --- Handle LOCAL_STORAGE_RESULT: dispatch to domain handlers ---------- */

static void on_storage_result(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;
	param_storage_t *p = (param_storage_t *)data;

	config_tuning_on_result(p);
	config_gyro_on_result(p);
	config_accel_on_result(p);
	config_mag_on_result(p);
}

/* --- Handle LOCAL_STORAGE_SAVE: dispatch to domain handlers ------------ */

static void on_storage_save(uint8_t *data, size_t size) {
	if (size < sizeof(param_storage_t)) return;
	param_storage_t *p = (param_storage_t *)data;

	config_tuning_on_save(p);
	config_gyro_on_save(p);
	config_accel_on_save(p);
	config_mag_on_save(p);
}

/* --- Boot: request from flash + publish at 1 Hz for 5 seconds --------- */

static void on_scheduler_1hz(uint8_t *data, size_t size) {
	if (g_publish_remaining <= 0) return;

	config_gyro_request_load();
	config_accel_request_load();
	config_mag_request_load();
	config_tuning_request_load();

	config_gyro_publish();
	config_accel_publish();
	config_mag_publish();
	config_tuning_publish();

	g_publish_remaining--;
}

/* --- Setup ------------------------------------------------------------- */

void config_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, on_storage_result);
	subscribe(LOCAL_STORAGE_SAVE, on_storage_save);
	subscribe(SCHEDULER_1HZ, on_scheduler_1hz);
	subscribe(CALIBRATION_GYRO_REQUEST, config_gyro_on_request);
	subscribe(CALIBRATION_ACCEL_REQUEST, config_accel_on_request);
	subscribe(CALIBRATION_MAG_REQUEST, config_mag_on_request);
	subscribe(DB_MESSAGE_UPDATE, config_tuning_on_db_message);
}
