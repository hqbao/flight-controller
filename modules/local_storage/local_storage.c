#include "local_storage.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

#define SHOULD_CLEAR_STORAGE 0

#define DATA_STORAGE_SIZE 	128
#define CHECKSUM_SIZE 		4
#define LOCAL_STORAGE_SIZE 	(DATA_STORAGE_SIZE + CHECKSUM_SIZE)
#define PARAM_SIZE 			4

static uint8_t g_local_data[LOCAL_STORAGE_SIZE] = {0}; // data + 4-byte checksum

static param_storage_t default_storage[] = {
	{.id=PARAM_ID_IMU1_GYRO_CALIBRATED, .value=0.0f},
	// IMU1 Gyro Bias
	{.id=PARAM_ID_IMU1_GYRO_BIAS_X, .value=0.0f},
	{.id=PARAM_ID_IMU1_GYRO_BIAS_Y, .value=0.0f},
	{.id=PARAM_ID_IMU1_GYRO_BIAS_Z, .value=0.0f},
};

// CRC32 implementation
static uint32_t crc32(const uint8_t *data, size_t length) {
	uint32_t crc = 0xFFFFFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xEDB88320;
			else
				crc >>= 1;
		}
	}
	return crc ^ 0xFFFFFFFF;
}

static void local_storage_save(uint8_t *data, size_t size) {
	// Data should be: param_id (4 bytes) + value (4 bytes)
	if (size < sizeof(param_storage_t)) {
		return;
	}
	
	param_storage_t *param = (param_storage_t *)data;
	uint32_t pos = param->id * PARAM_SIZE;
	
	if (pos >= DATA_STORAGE_SIZE) {
		return;
	}
	
	// Update the value in g_local_data
	memcpy(&g_local_data[pos], &param->value, PARAM_SIZE);
	
	// Recalculate checksum
	uint32_t checksum = crc32(g_local_data, DATA_STORAGE_SIZE);
	memcpy(&g_local_data[DATA_STORAGE_SIZE], &checksum, CHECKSUM_SIZE);
	
	// Write to flash immediately
	platform_storage_write(0, LOCAL_STORAGE_SIZE, g_local_data);
}

static void local_storage_load(uint8_t *data, size_t size) {
	// Data should contain param_id (4 bytes)
	if (size < sizeof(param_id_e)) {
		return;
	}
	
	param_id_e param_id = *(param_id_e *)data;
	uint32_t pos = param_id * PARAM_SIZE;
	
	if (pos >= DATA_STORAGE_SIZE) {
		return;
	}
	
	// Read the value from g_local_data
	param_storage_t param;
	param.id = param_id;
	memcpy(&param.value, &g_local_data[pos], PARAM_SIZE);
	
	// Publish the loaded parameter
	publish(LOCAL_STORAGE_RESULT, (uint8_t *)&param, sizeof(param_storage_t));
}

static void local_storage_save_default(void) {
	// Initialize all parameters with defaults
	memset(g_local_data, 0, LOCAL_STORAGE_SIZE);
	
	for (uint8_t i = 0; i < sizeof(default_storage) / sizeof(default_storage[0]); i++) {
		uint32_t pos = default_storage[i].id * PARAM_SIZE;
		if (pos < DATA_STORAGE_SIZE) {
			memcpy(&g_local_data[pos], &default_storage[i].value, PARAM_SIZE);
		}
	}
	
	// Calculate checksum and write to flash
	uint32_t checksum = crc32(g_local_data, DATA_STORAGE_SIZE);
	memcpy(&g_local_data[DATA_STORAGE_SIZE], &checksum, CHECKSUM_SIZE);
	platform_storage_write(0, LOCAL_STORAGE_SIZE, g_local_data);
}

static void load_local_data(void) {
	uint8_t temp_data[LOCAL_STORAGE_SIZE] = {0};
	
	// Read data from flash memory
	platform_storage_read(0, LOCAL_STORAGE_SIZE, temp_data);

	// Checksum validation - if failed, save defaults
	uint32_t checksum = *(uint32_t *)&temp_data[DATA_STORAGE_SIZE];
	uint32_t calculated_checksum = crc32(temp_data, DATA_STORAGE_SIZE);
	if (checksum != calculated_checksum) {
		local_storage_save_default();
	} else {
		// Copy valid data to g_local_data
		memcpy(g_local_data, temp_data, LOCAL_STORAGE_SIZE);
	}
}

void local_storage_setup(void) {
#if SHOULD_CLEAR_STORAGE
	// Clear storage and save defaults
	local_storage_save_default();
#endif

	// Load flash memory data once at startup
	load_local_data();

	// Subscribe to save/load requests
	subscribe(LOCAL_STORAGE_SAVE, local_storage_save);
	subscribe(LOCAL_STORAGE_LOAD, local_storage_load);
	
	// Publish module initialized status
	module_initialized_t module_initialized;
	module_initialized.id = MODULE_ID_LOCAL_STORAGE;
	module_initialized.initialized = 1;
	publish(MODULE_INITIALIZED_UPDATE, (uint8_t*)&module_initialized, sizeof(module_initialized_t));
}
