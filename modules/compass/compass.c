#include "compass.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "bmm350.h"

/* 
 * --- COMPASS CALIBRATION GUIDE ---
 * 
 * 1. SETUP
 *    - Set ENABLE_COMPASS_MONITOR_LOG to 2 (Raw Data) in this file.
 *    - Flash firmware and connect via USB.
 *    - Run 'python3 pytest/calibrate_compass.py'.
 * 
 * 2. DATA COLLECTION
 *    - Click "Start Stream" in the Python tool.
 *    - Rotate the drone in ALL directions (figure-8 motion).
 *    - Ensure you cover the entire sphere surface.
 *    - Collect at least 1000+ points.
 * 
 * 3. CALIBRATION
 *    - The tool automatically computes B (Bias) and S (Scale) in real-time.
 *    - Once the "Corrected Data" (Green) forms a perfect sphere centered at 0,
 *      stop the stream.
 * 
 * 4. SAVING
 *    - Copy the resulting Hard Iron Bias (B) and Soft Iron Matrix (S) 
 *      into the g_mag_offset and g_mag_scale variables below.
 *    - Set ENABLE_COMPASS_MONITOR_LOG back to 0 (or 1 for debug).
 * 
 * 5. MATHEMATICAL MODEL
 *    The calibration applies the following linear correction:
 *    
 *    V_cal = S * (V_raw - B)
 *    
 *    Where:
 *    - V_raw: Raw magnetometer reading vector [x, y, z]
 *    - B (Hard Iron Bias): Offset vector [bx, by, bz]
 *      Corrects for permanent magnetic fields on the PCB (e.g. nearby screws, traces).
 *      Shifts the sphere center to (0,0,0).
 *    - S (Soft Iron Matrix): 3x3 Matrix
 *      [ Sxx Sxy Sxz ]
 *      [ Syx Syy Syz ]
 *      [ Szx Szy Szz ]
 *      Corrects for ferromagnetic materials that distort the magnetic field 
 *      (stretching/squashing the sphere into an ellipsoid).
 */

/* Macro to enable/disable sending MONITOR_DATA via logger 
 * 0: Disable
 * 1: Calibrated
 * 2: Raw
 */
#define ENABLE_COMPASS_MONITOR_LOG 0

typedef struct {
	double x;
	double y;
	double z;
} compass_data_t;

struct bmm350_dev BMMdev = {0};
struct bmm350_mag_temp_data mag_temp_data;

/* Static variables for compass values */
static float g_compass_x = 0.0f;
static float g_compass_y = 0.0f;
static float g_compass_z = 0.0f;
static float g_compass_raw_x = 0.0f;
static float g_compass_raw_y = 0.0f;
static float g_compass_raw_z = 0.0f;

/* Calibration parameters: B (Offset) and S (Soft Iron / Scale) */
static float g_mag_offset[3] = {-26.13f, -25.83f, 15.53f};
static float g_mag_scale[3][3] = {
	{0.02f, 0.0f, 0.0f},
	{0.0f, 0.02f, 0.0f},
	{0.0f, 0.0f, 0.02f}
};

BMM350_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	platform_i2c_write_read(I2C_PORT2, BMM350_I2C_ADSEL_SET_LOW_, &reg_addr, 1, reg_data, len, 4);
	return BMM350_OK;
}

BMM350_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	uint8_t buffer[len + 1];
	buffer[0] = reg_addr;
	memcpy(&buffer[1], reg_data, len);
	platform_i2c_write(I2C_PORT2, BMM350_I2C_ADSEL_SET_LOW_, buffer, len + 1);
	return BMM350_OK;
}

BMM350_INTF_RET_TYPE i2c_write_byte(uint8_t reg_addr, uint8_t reg_data) {
	uint8_t buffer[2];
	buffer[0] = reg_addr;
	buffer[1] = reg_data;
	platform_i2c_write(I2C_PORT2, BMM350_I2C_ADSEL_SET_LOW_, buffer, 2);
	return BMM350_OK;
}

void delay_us(uint32_t period, void *intf_ptr) {
	platform_delay(period/1000);
}

static void loop_25hz(uint8_t *data, size_t size) {
	bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &BMMdev);
	
	float x_raw = (float)mag_temp_data.x;
	float y_raw = (float)mag_temp_data.y;
	float z_raw = (float)mag_temp_data.z;

	g_compass_raw_x = x_raw;
	g_compass_raw_y = y_raw;
	g_compass_raw_z = z_raw;

	/* Apply Calibration: V_cal = S * (V_raw - B) */
	float x_off = x_raw - g_mag_offset[0];
	float y_off = y_raw - g_mag_offset[1];
	float z_off = z_raw - g_mag_offset[2];

	float x_cal = g_mag_scale[0][0] * x_off + g_mag_scale[0][1] * y_off + g_mag_scale[0][2] * z_off;
	float y_cal = g_mag_scale[1][0] * x_off + g_mag_scale[1][1] * y_off + g_mag_scale[1][2] * z_off;
	float z_cal = g_mag_scale[2][0] * x_off + g_mag_scale[2][1] * y_off + g_mag_scale[2][2] * z_off;

	/* Store values in global variables */
	g_compass_x = x_cal;
	g_compass_y = y_cal;
	g_compass_z = z_cal;
	
	/* Publish values */
	compass_data_t compass_data = {
		.x = x_cal,
		.y = y_cal,
		.z = z_cal
	};
	publish(SENSOR_COMPASS, (uint8_t*)&compass_data, sizeof(compass_data_t));
}

#if ENABLE_COMPASS_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	/* Send compass data as MONITOR_DATA */
	uint8_t out_msg[12]; /* 3 * 4 bytes (float32) */
#if ENABLE_COMPASS_MONITOR_LOG == 2
	memcpy(&out_msg[0], &g_compass_raw_x, sizeof(float));
	memcpy(&out_msg[4], &g_compass_raw_y, sizeof(float));
	memcpy(&out_msg[8], &g_compass_raw_z, sizeof(float));
#else
	memcpy(&out_msg[0], &g_compass_x, sizeof(float));
	memcpy(&out_msg[4], &g_compass_y, sizeof(float));
	memcpy(&out_msg[8], &g_compass_z, sizeof(float));
#endif
	
	publish(MONITOR_DATA, out_msg, sizeof(out_msg));
}
#endif

void compass_setup(void) {
	BMMdev.read = i2c_read;
	BMMdev.write = i2c_write;
	BMMdev.delay_us = delay_us;
	bmm350_init(&BMMdev);

	/* Set ODR and performance */
	bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &BMMdev);
	/* Enable all axis */
	bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &BMMdev);
	bmm350_set_powermode(BMM350_NORMAL_MODE, &BMMdev);

	subscribe(SCHEDULER_25HZ, loop_25hz);
#if ENABLE_COMPASS_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
