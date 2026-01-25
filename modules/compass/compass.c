#include "compass.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <macro.h>
#include "bmm350.h"

/* 
 * --- COMPASS CALIBRATION GUIDE ---
 * 
 * 1. SETUP
 *    - Set ENABLE_COMPASS_MONITOR_LOG to 2 (Raw Data) in this file.
 *    - Flash firmware and connect via USB.
 *    - Run 'python3 pytest/compass_calibrate.py'.
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

struct bmm350_dev BMMdev = {0};
struct bmm350_mag_temp_data mag_temp_data;

/* Static variables for compass values */
static vector3d_t g_compass_cal = {0};
static vector3d_t g_compass_raw = {0};

#if ENABLE_COMPASS_MONITOR_LOG > 0
static uint8_t g_monitor_msg[12] = {0};
#endif

/* Calibration parameters: B (Offset) and S (Soft Iron / Scale) */
static double g_mag_offset[3] = {-26.13, -25.83, 15.53};
static double g_mag_scale[3][3] = {
	{0.02, 0.0, 0.0},
	{0.0, 0.02, 0.0},
	{0.0, 0.0, 0.02}
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
	
	g_compass_raw.x = (double)mag_temp_data.x;
	g_compass_raw.y = (double)mag_temp_data.y;
	g_compass_raw.z = (double)mag_temp_data.z;

	/* Apply Calibration: V_cal = S * (V_raw - B) */
	double x_off = g_compass_raw.x - g_mag_offset[0];
	double y_off = g_compass_raw.y - g_mag_offset[1];
	double z_off = g_compass_raw.z - g_mag_offset[2];

	double x_cal = g_mag_scale[0][0] * x_off + g_mag_scale[0][1] * y_off + g_mag_scale[0][2] * z_off;
	double y_cal = g_mag_scale[1][0] * x_off + g_mag_scale[1][1] * y_off + g_mag_scale[1][2] * z_off;
	double z_cal = g_mag_scale[2][0] * x_off + g_mag_scale[2][1] * y_off + g_mag_scale[2][2] * z_off;

	/* Normalize to ensure unit vector */
	vector3d_t mag_vec = { .x = x_cal, .y = y_cal, .z = z_cal };
	vector3d_normalize(&mag_vec, &mag_vec);

	/* Store values in global variables */
	memcpy(&g_compass_cal, &mag_vec, sizeof(vector3d_t));
	
	/* Publish values */
	publish(SENSOR_COMPASS, (uint8_t*)&mag_vec, sizeof(vector3d_t));
}

#if ENABLE_COMPASS_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	/* Send compass data as MONITOR_DATA
	   Format: 3 floats (x, y, z) */
	
#if ENABLE_COMPASS_MONITOR_LOG == 2
	float raw_x = (float)g_compass_raw.x;
	float raw_y = (float)g_compass_raw.y;
	float raw_z = (float)g_compass_raw.z;

	memcpy(&g_monitor_msg[0], &raw_x, sizeof(float));
	memcpy(&g_monitor_msg[4], &raw_y, sizeof(float));
	memcpy(&g_monitor_msg[8], &raw_z, sizeof(float));
#else
	float cal_x = (float)g_compass_cal.x;
	float cal_y = (float)g_compass_cal.y;
	float cal_z = (float)g_compass_cal.z;

	memcpy(&g_monitor_msg[0], &cal_x, sizeof(float));
	memcpy(&g_monitor_msg[4], &cal_y, sizeof(float));
	memcpy(&g_monitor_msg[8], &cal_z, sizeof(float));
#endif
	
	publish(MONITOR_DATA, g_monitor_msg, sizeof(g_monitor_msg));
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
	
	// Publish module initialized status
	module_initialized_t module_initialized;
	module_initialized.id = MODULE_ID_COMPASS;
	module_initialized.initialized = 1;
	publish(MODULE_INITIALIZED_UPDATE, (uint8_t*)&module_initialized, sizeof(module_initialized_t));
}
