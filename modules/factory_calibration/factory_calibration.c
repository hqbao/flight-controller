#include "factory_calibration.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>

#define CALIB_FLAG_OFFSET_1 0
#define CALIB_DATA_OFFSET_1 1
#define CALIB_FLAG_OFFSET_2 25
#define CALIB_DATA_OFFSET_2 26
#define CALIB_DATA_SIZE 24
#define TOTAL_CALIB_LEN 50

static float g_imu_offset[6] = {0, 0, 0, 0, 0, 0};
static float g_imu2_offset[6] = {0, 0, 0, 0, 0, 0};

static uint8_t g_calibration_data[64] = {0};

static void local_storage_loaded(uint8_t *data, size_t size) {
    // WARNING: This function currently overwrites storage with zeros on every load!
    // Ensure g_imu_offset is populated before this runs, or remove this logic if not intended.
    
	g_calibration_data[CALIB_FLAG_OFFSET_1] = 1; // IMU calibrated
	memcpy(&g_calibration_data[CALIB_DATA_OFFSET_1], (uint8_t*)g_imu_offset, CALIB_DATA_SIZE);

	g_calibration_data[CALIB_FLAG_OFFSET_2] = 1; // IMU2 calibrated
	memcpy(&g_calibration_data[CALIB_DATA_OFFSET_2], (uint8_t*)g_imu2_offset, CALIB_DATA_SIZE);

	publish(LOCAL_STORAGE_SAVE, g_calibration_data, TOTAL_CALIB_LEN);
}

void factory_calibration_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, local_storage_loaded);
}
