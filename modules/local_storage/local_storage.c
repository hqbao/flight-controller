#include "local_storage.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

#define SHOULD_CLEAR_STORAGE 0

#define DATA_STORAGE_SIZE 	1024  /* 256 params × 4 bytes — over-allocated for future use */
#define CHECKSUM_SIZE 		4
#define LOCAL_STORAGE_SIZE 	(DATA_STORAGE_SIZE + CHECKSUM_SIZE)
#define PARAM_SIZE 			4

static uint8_t g_local_data[LOCAL_STORAGE_SIZE] = {0}; // data + 4-byte checksum
static uint8_t g_active_log_class = 0;
static volatile uint8_t g_dirty = 0;  // deferred flash write flag
static volatile uint32_t g_dirty_ms = 0;  // timestamp of last save (for coalescing)
static uint8_t g_storage_page = 0;    // multi-page readback: 0..3

/* Coalesce rapid-fire saves: wait this long after the last save before
 * actually erasing+writing flash. Upload Defaults sends 71 params at 50 ms
 * apart (~3.5 s total) — without coalescing each one triggers a ~1.3 s
 * sector erase, starving the LOOP for tens of seconds and stalling the
 * FFT spectrum stream which also runs from LOOP. */
#define FLASH_COALESCE_MS  500

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
	// Default = identity scale + zero bias.
	// Calibration outputs S*(raw-B) in raw-LSB units (NOT unit-g); fusion uses
	// MAX_IMU_ACCEL to convert to g. Identity is therefore the correct default —
	// it leaves raw LSB values unchanged, which matches MAX_IMU_ACCEL = 2048
	// (AFS_16G LSB/g) so gravity comes out as ~1g in fusion.
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
	// === Tuning parameters ===
	// Attitude PID
	{.id=PARAM_ID_ATT_ROLL_P,         .value=4.0f},
	{.id=PARAM_ID_ATT_ROLL_I,         .value=1.0f},
	{.id=PARAM_ID_ATT_ROLL_D,         .value=2.0f},
	{.id=PARAM_ID_ATT_ROLL_I_LIMIT,   .value=5.0f},
	{.id=PARAM_ID_ATT_ROLL_P_LIMIT,   .value=1000000.0f},
	{.id=PARAM_ID_ATT_ROLL_O_LIMIT,   .value=1000000.0f},
	{.id=PARAM_ID_ATT_PITCH_P,        .value=4.0f},
	{.id=PARAM_ID_ATT_PITCH_I,        .value=1.0f},
	{.id=PARAM_ID_ATT_PITCH_D,        .value=2.0f},
	{.id=PARAM_ID_ATT_PITCH_I_LIMIT,  .value=5.0f},
	{.id=PARAM_ID_ATT_PITCH_P_LIMIT,  .value=1000000.0f},
	{.id=PARAM_ID_ATT_PITCH_O_LIMIT,  .value=1000000.0f},
	{.id=PARAM_ID_ATT_YAW_P,          .value=10.0f},
	{.id=PARAM_ID_ATT_YAW_I,          .value=1.0f},
	{.id=PARAM_ID_ATT_YAW_D,          .value=5.0f},
	{.id=PARAM_ID_ATT_YAW_I_LIMIT,    .value=5.0f},
	{.id=PARAM_ID_ATT_YAW_P_LIMIT,    .value=1000000.0f},
	{.id=PARAM_ID_ATT_YAW_O_LIMIT,    .value=1000000.0f},
	{.id=PARAM_ID_ATT_SMOOTH_INPUT,    .value=1.0f},
	{.id=PARAM_ID_ATT_SMOOTH_P_TERM,   .value=1.0f},
	{.id=PARAM_ID_ATT_SMOOTH_OUTPUT,   .value=1.0f},
	{.id=PARAM_ID_ATT_GAIN_TIME,       .value=1.0f},
	// Position Control
	{.id=PARAM_ID_POS_XY_P,            .value=100.0f},
	{.id=PARAM_ID_POS_Z_P,             .value=2000.0f},
	{.id=PARAM_ID_POS_VELOC_XY_SCALE,  .value=50.0f},
	{.id=PARAM_ID_POS_VELOC_Z_SCALE,   .value=2000.0f},
	{.id=PARAM_ID_POS_LPF_XY,          .value=1.0f},
	{.id=PARAM_ID_POS_LPF_Z,           .value=5.0f},
	{.id=PARAM_ID_POS_ANGLE_LIMIT,     .value=30.0f},
	{.id=PARAM_ID_POS_RC_DEADBAND,     .value=0.1f},
	// Motor/Servo
	{.id=PARAM_ID_MOTOR_MIN,     .value=150.0f},
	{.id=PARAM_ID_MOTOR_MAX,     .value=1800.0f},
	{.id=PARAM_ID_SERVO_MIN,     .value=1000.0f},
	{.id=PARAM_ID_SERVO_MAX,     .value=2000.0f},
	{.id=PARAM_ID_SERVO_CENTER,  .value=1500.0f},
	// Attitude Estimation
	{.id=PARAM_ID_ATT_MAHONY_KP,     .value=0.1f},
	{.id=PARAM_ID_ATT_MAHONY_KI,     .value=0.001f},
	{.id=PARAM_ID_ATT_F3_BETA,       .value=0.001f},
	{.id=PARAM_ID_ATT_F3_ZETA,       .value=0.0001f},
	{.id=PARAM_ID_ATT_ACCEL_SMOOTH,   .value=4.0f},
	{.id=PARAM_ID_ATT_LIN_ACC_DECAY,  .value=0.5f},
	{.id=PARAM_ID_ATT_LIN_ACCEL_MIN,  .value=0.5f},
	{.id=PARAM_ID_ATT_LIN_ACCEL_MAX,  .value=2.0f},
	// Position Estimation
	{.id=PARAM_ID_PE_XY_S1_INTEG,  .value=1.0f},
	{.id=PARAM_ID_PE_XY_S1_CORR,   .value=1.25f},
	{.id=PARAM_ID_PE_XY_S2_INTEG,  .value=1.0f},
	{.id=PARAM_ID_PE_XY_S2_CORR,   .value=10.0f},
	{.id=PARAM_ID_PE_XY_V_FB,      .value=0.1f},
	{.id=PARAM_ID_PE_Z_S1_INTEG,   .value=1.0f},
	{.id=PARAM_ID_PE_Z_S1_CORR,    .value=0.5f},
	{.id=PARAM_ID_PE_Z_S2_INTEG,   .value=1.0f},
	{.id=PARAM_ID_PE_Z_S2_CORR,    .value=10.0f},
	{.id=PARAM_ID_PE_Z_V_FB,       .value=0.1f},
	{.id=PARAM_ID_PE_OPTFLOW_GAIN,  .value=5.0f},
	// FFT/Notch
	{.id=PARAM_ID_NOTCH_Q,         .value=3.0f},
	{.id=PARAM_ID_NOTCH_MIN_HZ,    .value=50.0f},
	{.id=PARAM_ID_FFT_FREQ_ALPHA,  .value=0.15f},
	// Flight State
	{.id=PARAM_ID_DISARM_ANGLE,          .value=60.0f},
	{.id=PARAM_ID_DISARM_RANGE,          .value=10.0f},
	{.id=PARAM_ID_ALLOWED_LANDING_RANGE, .value=500.0f},
	{.id=PARAM_ID_TOOK_OFF_RANGE,        .value=100.0f},
	// Servo Bias
	{.id=PARAM_ID_SERVO_BIAS_1, .value=0.0f},
	{.id=PARAM_ID_SERVO_BIAS_2, .value=0.0f},
	{.id=PARAM_ID_SERVO_BIAS_3, .value=0.0f},
	{.id=PARAM_ID_SERVO_BIAS_4, .value=0.0f},
	// Thrust Linearization
	{.id=PARAM_ID_THRUST_P1, .value=1.0f},
	{.id=PARAM_ID_THRUST_P2, .value=0.0f},
	// RC Scale
	{.id=PARAM_ID_RC_XY_SCALE,  .value=0.01f},
	{.id=PARAM_ID_RC_Z_SCALE,   .value=0.04f},
	{.id=PARAM_ID_RC_YAW_SCALE, .value=-0.5f},
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
	
	// Mark dirty — actual flash write deferred to LOOP and coalesced.
	// Bursts of saves (e.g. Upload Defaults: 71 params) collapse into a
	// single sector erase+write FLASH_COALESCE_MS after the last save.
	g_dirty = 1;
	g_dirty_ms = platform_time_ms();
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
	/* Coalesce: only flush once writes have stopped for FLASH_COALESCE_MS.
	 * Avoids back-to-back ~1.3 s sector erases starving LOOP. */
	if ((platform_time_ms() - g_dirty_ms) < FLASH_COALESCE_MS) return;
	g_dirty = 0;
	uint32_t checksum = crc32(g_local_data, DATA_STORAGE_SIZE);
	memcpy(&g_local_data[DATA_STORAGE_SIZE], &checksum, CHECKSUM_SIZE);
	platform_storage_write(0, LOCAL_STORAGE_SIZE, g_local_data);
}

/* ---- multi-page readback at 1 Hz (streams all 104 params) ------------ */
#define LOG_PARAMS_PER_PAGE  26   /* 26 params × 4 bytes = 104 bytes per frame */
#define LOG_TOTAL_PARAMS    104   /* IDs 0..103 */
#define LOG_TOTAL_PAGES     ((LOG_TOTAL_PARAMS + LOG_PARAMS_PER_PAGE - 1) / LOG_PARAMS_PER_PAGE)
static void loop_1hz(uint8_t *data, size_t size) {
	if (g_active_log_class == 0) return;

	uint16_t start = g_storage_page * LOG_PARAMS_PER_PAGE * PARAM_SIZE;
	uint16_t remaining = LOG_TOTAL_PARAMS - g_storage_page * LOG_PARAMS_PER_PAGE;
	if (remaining > LOG_PARAMS_PER_PAGE) remaining = LOG_PARAMS_PER_PAGE;
	publish(SEND_LOG, &g_local_data[start], remaining * PARAM_SIZE);

	g_storage_page++;
	if (g_storage_page >= LOG_TOTAL_PAGES) {
		g_storage_page = 0;
		g_active_log_class = 0;   /* stop after all pages sent */
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
	
	platform_storage_read(0, LOCAL_STORAGE_SIZE, temp_data);

	uint32_t checksum;
	memcpy(&checksum, &temp_data[DATA_STORAGE_SIZE], CHECKSUM_SIZE);
	uint32_t calculated_checksum = crc32(temp_data, DATA_STORAGE_SIZE);
	if (checksum != calculated_checksum) {
		local_storage_save_default();
	} else {
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
