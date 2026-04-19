#include "compass.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <macro.h>
#include "bmm350.h"
#include <messages.h>

/*
 * --- BMM350 COMPASS MODULE ---
 *
 * Reads raw magnetic field data from the Bosch BMM350 magnetometer at 25 Hz,
 * applies hard/soft iron calibration, normalizes the result to a unit vector,
 * corrects the sensor axis mapping to NED body frame, and publishes via PubSub.
 *
 * Calibration model:  V_cal = S × (V_raw − B)
 *   B = hard iron bias vector   (3-element, double)
 *   S = soft iron scale matrix  (3×3, double)
 *
 * Calibration values are loaded from flash at boot via CALIBRATION_MAG_READY.
 *
 * I2C NOTE (BMM350 dummy bytes):
 *   The BMM350 prepends BMM350_DUMMY_BYTES (2) dummy bytes to every burst read,
 *   even on I2C. Bosch's bmm350_get_regs() already handles this internally:
 *   it requests (len + 2) bytes and copies starting at offset 2. Our i2c_read
 *   wrapper passes the full requested length straight to the HAL so Bosch
 *   receives all bytes — including the leading dummies — as expected.
 */

/* -------------------------------------------------------------------------
 * State
 * ---------------------------------------------------------------------- */

struct bmm350_dev         BMMdev       = {0};
struct bmm350_mag_temp_data mag_temp_data;

static vector3d_t g_compass_raw = {0};   /* µT, sensor frame */
static vector3d_t g_compass_cal = {0};   /* unit vector, NED body frame */

static double g_mag_offset[3]      = {0};
static double g_mag_scale[3][3]    = {{1,0,0},{0,1,0},{0,0,1}};

static uint8_t g_bmm_addr          = BMM350_I2C_ADSEL_SET_LOW_;
static uint8_t g_active_log_class  = 0;
static uint8_t g_log_buf[12]       = {0};

/* -------------------------------------------------------------------------
 * Bosch bmm350_dev interface callbacks
 * ---------------------------------------------------------------------- */

static BMM350_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                     uint32_t len, void *intf_ptr) {
	if (platform_i2c_write_read(I2C_PORT2, g_bmm_addr,
	                            &reg_addr, 1, reg_data, len, 100) == PLATFORM_OK) {
		return BMM350_OK;
	}
	return BMM350_E_COM_FAIL;
}

static BMM350_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr) {
	uint8_t buf[len + 1];
	buf[0] = reg_addr;
	memcpy(&buf[1], reg_data, len);
	if (platform_i2c_write(I2C_PORT2, g_bmm_addr, buf, len + 1) == PLATFORM_OK) {
		return BMM350_OK;
	}
	return BMM350_E_COM_FAIL;
}

static void delay_us(uint32_t period, void *intf_ptr) {
	platform_delay(period / 1000);
}

/* -------------------------------------------------------------------------
 * 1 Hz calibration request loop (polls until values arrive)
 * ---------------------------------------------------------------------- */

static uint8_t g_mag_cal_ready = 0;

static void loop_calibration_request(uint8_t *data, size_t size) {
	if (!g_mag_cal_ready) {
		publish(CALIBRATION_MAG_REQUEST, NULL, 0);
	}
}

/* -------------------------------------------------------------------------
 * 25 Hz sensor loop
 * ---------------------------------------------------------------------- */

static void loop_25hz(uint8_t *data, size_t size) {
	bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &BMMdev);

	g_compass_raw.x = (double)mag_temp_data.x;
	g_compass_raw.y = (double)mag_temp_data.y;
	g_compass_raw.z = (double)mag_temp_data.z;

	/* Hard iron subtract */
	double ox = g_compass_raw.x - g_mag_offset[0];
	double oy = g_compass_raw.y - g_mag_offset[1];
	double oz = g_compass_raw.z - g_mag_offset[2];

	/* Soft iron scale */
	vector3d_t mag_vec = {
		.x = g_mag_scale[0][0]*ox + g_mag_scale[0][1]*oy + g_mag_scale[0][2]*oz,
		.y = g_mag_scale[1][0]*ox + g_mag_scale[1][1]*oy + g_mag_scale[1][2]*oz,
		.z = g_mag_scale[2][0]*ox + g_mag_scale[2][1]*oy + g_mag_scale[2][2]*oz,
	};

	/* Normalize to unit vector */
	vector3d_normalize(&mag_vec, &mag_vec);

	/* Sensor-to-body axis mapping (BMM350):
	 * BMM350 on PCB: sensor X=Right, sensor Y=Forward, sensor Z=Down
	 * Body frame:    X=Forward, Y=Right, Z=Down
	 * Mapping: body_x = sensor_y, body_y = -sensor_x, body_z = sensor_z */
	// FC2
	double tmp = mag_vec.x;
	mag_vec.x = mag_vec.y;
	mag_vec.y = -tmp;

	// FC1
	// double tmp = mag_vec.x;
	// mag_vec.x = -mag_vec.y;
	// mag_vec.y = -tmp;
	// mag_vec.z = -mag_vec.z;

	g_compass_cal = mag_vec;
	publish(SENSOR_COMPASS, (uint8_t *)&mag_vec, sizeof(vector3d_t));
}

/* -------------------------------------------------------------------------
 * Logger
 * ---------------------------------------------------------------------- */

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	uint8_t cls = data[0];
	g_active_log_class = (cls == LOG_CLASS_COMPASS || cls == LOG_CLASS_COMPASS_CALIB) ? cls : 0;
}

static void loop_logger(uint8_t *data, size_t size) {
	if (g_active_log_class == 0) return;

	/* LOG_CLASS_COMPASS      → raw µT values (for calibration tool)
	 * LOG_CLASS_COMPASS_CALIB → calibrated NED unit vector */
	vector3d_t *src = (g_active_log_class == LOG_CLASS_COMPASS)
	                  ? &g_compass_raw : &g_compass_cal;

	float x = (float)src->x;
	float y = (float)src->y;
	float z = (float)src->z;
	memcpy(&g_log_buf[0], &x, 4);
	memcpy(&g_log_buf[4], &y, 4);
	memcpy(&g_log_buf[8], &z, 4);

	publish(SEND_LOG, g_log_buf, sizeof(g_log_buf));
}

/* -------------------------------------------------------------------------
 * Calibration receiver
 * ---------------------------------------------------------------------- */

static void on_mag_calibration_ready(uint8_t *data, size_t size) {
	if (size < sizeof(calibration_mag_t)) return;
	calibration_mag_t *cal = (calibration_mag_t *)data;
	memcpy(g_mag_offset, cal->offset, sizeof(g_mag_offset));
	memcpy(g_mag_scale,  cal->scale,  sizeof(g_mag_scale));
	g_mag_cal_ready = 1;
}

/* -------------------------------------------------------------------------
 * Setup
 * ---------------------------------------------------------------------- */

void compass_setup(void) {
	BMMdev.read     = i2c_read;
	BMMdev.write    = i2c_write;
	BMMdev.delay_us = delay_us;

	bmm350_init(&BMMdev);
	bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &BMMdev);
	bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &BMMdev);
	bmm350_set_powermode(BMM350_NORMAL_MODE, &BMMdev);

	subscribe(SCHEDULER_25HZ,        loop_25hz);
	subscribe(SCHEDULER_25HZ,        loop_logger);
	subscribe(SCHEDULER_1HZ,         loop_calibration_request);
	subscribe(NOTIFY_LOG_CLASS,      on_notify_log_class);
	subscribe(CALIBRATION_MAG_READY, on_mag_calibration_ready);
}
