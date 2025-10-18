#include "factory_calibration.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>

static float g_imu_offset[6] = {0, 0, 0, 0, 0, 0};
static float g_imu2_offset[6] = {0, 0, 0, 0, 0, 0};

static uint8_t g_calibration_data[64] = {0};

static void local_storage_loaded(uint8_t *data, size_t size) {
	g_calibration_data[0] = 1; // IMU calibrated
	memcpy(&g_calibration_data[1], (uint8_t*)g_imu_offset, 24);

	g_calibration_data[25] = 1; // IMU2 calibrated
	memcpy(&g_calibration_data[26], (uint8_t*)g_imu2_offset, 24);

	publish(LOCAL_STORAGE_SAVE, g_calibration_data, 0);
}

void factory_calibration_setup(void) {
	subscribe(LOCAL_STORAGE_RESULT, local_storage_loaded);
}
