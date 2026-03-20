#include "local_storage.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

#define SHOULD_CLEAR_STORAGE 0

#define DATA_STORAGE_SIZE 	192
#define CHECKSUM_SIZE 		4
#define LOCAL_STORAGE_SIZE 	(DATA_STORAGE_SIZE + CHECKSUM_SIZE)
#define PARAM_SIZE 			4

static uint8_t g_local_data[LOCAL_STORAGE_SIZE] = {0}; // data + 4-byte checksum
static uint8_t g_active_log_class = 0;
static volatile uint8_t g_dirty = 0;  // deferred flash write flag
static uint8_t g_storage_page = 0;    // two-page readback: 0=first 30, 1=remaining 18

static param_storage_t default_storage[] = {
	// Magnetometer calibration
	{.id=PARAM_ID_MAG_CALIBRATED, .value=0.0f},
	{.id=PARAM_ID_MAG_OFFSET_X,  .value=0.0f},
	{.id=PARAM_ID_MAG_OFFSET_Y,  .value=0.0f},
	{.id=PARAM_ID_MAG_OFFSET_Z,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_00,  .value=1.0f},
	{.id=PARAM_ID_MAG_SCALE_01,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_02,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_10,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_11,  .value=1.0f},
	{.id=PARAM_ID_MAG_SCALE_12,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_20,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_21,  .value=0.0f},
	{.id=PARAM_ID_MAG_SCALE_22,  .value=1.0f},
	// Accelerometer calibration
	{.id=PARAM_ID_ACCEL_CALIBRATED, .value=0.0f},
	{.id=PARAM_ID_ACCEL_BIAS_X,  .value=0.0f},
	{.id=PARAM_ID_ACCEL_BIAS_Y,  .value=0.0f},
	{.id=PARAM_ID_ACCEL_BIAS_Z,  .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_00, .value=1.0f},
	{.id=PARAM_ID_ACCEL_SCALE_01, .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_02, .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_10, .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_11, .value=1.0f},
	{.id=PARAM_ID_ACCEL_SCALE_12, .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_20, .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_21, .value=0.0f},
	{.id=PARAM_ID_ACCEL_SCALE_22, .value=1.0f},
	// Gyro temperature compensation (degree-2 polynomial)
	{.id=PARAM_ID_GYRO_TEMP_CALIBRATED, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_X_A, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_X_B, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_X_C, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_Y_A, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_Y_B, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_Y_C, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_Z_A, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_Z_B, .value=0.0f},
	{.id=PARAM_ID_GYRO_TEMP_Z_C, .value=0.0f},
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
	
	// Update the value in g_local_data (RAM only — fast)
	memcpy(&g_local_data[pos], &param->value, PARAM_SIZE);
	
	// Mark dirty — actual flash write deferred to 1 Hz loop
	// This avoids blocking UART DMA ISR with 13 consecutive flash
	// erase+write cycles (~1.3s) during calibration uploads.
	g_dirty = 1;
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

/* --- Log class: send stored calibration data for verification --- */

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	if (data[0] == LOG_CLASS_STORAGE) {
		g_active_log_class = LOG_CLASS_STORAGE;
		g_storage_page = 0;
	} else {
		g_active_log_class = 0;
	}
}

/*
 * Flash write runs from LOOP (main thread context), NOT from scheduler ISR.
 *
 * STM32H743 flash sector erase takes ~1.3 seconds (128KB sector).
 * Running this from TIM8 ISR (SCHEDULER_1HZ) blocks ALL interrupts for
 * that duration — freezing PID loops, UART DMA, and sensor readings.
 *
 * From thread mode, ISRs preempt normally: scheduler ticks, UART DMA,
 * and PID control all continue running during the flash erase.
 */
static void loop_flush(uint8_t *data, size_t size) {
	if (!g_dirty) return;
	g_dirty = 0;
	uint32_t checksum = crc32(g_local_data, DATA_STORAGE_SIZE);
	memcpy(&g_local_data[DATA_STORAGE_SIZE], &checksum, CHECKSUM_SIZE);
	platform_storage_write(0, LOCAL_STORAGE_SIZE, g_local_data);
}

static void loop_1hz(uint8_t *data, size_t size) {
	/* Send stored calibration data for verification (runs from ISR — fast).
	 * Storage is 48 params (192 bytes) but max DB payload is 120 bytes.
	 * Send in two pages across two 1Hz ticks, then auto-stop:
	 *   Page 0: params 0-29  (120 bytes)
	 *   Page 1: params 30-47 ( 72 bytes) */
	if (g_active_log_class == 0) return;

	if (g_storage_page == 0) {
		publish(SEND_LOG, g_local_data, 30 * PARAM_SIZE);
		g_storage_page = 1;
	} else {
		publish(SEND_LOG, &g_local_data[30 * PARAM_SIZE], 18 * PARAM_SIZE);
		g_storage_page = 0;
		g_active_log_class = 0;  /* stop after both pages sent */
	}
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

	// Subscribe to log class for flash readback verification
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_1HZ, loop_1hz);

	// Flash write runs from main loop (thread mode) — NOT from scheduler ISR.
	// STM32H743 sector erase takes ~1.3s; running from ISR blocks everything.
	subscribe(LOOP, loop_flush);
}
