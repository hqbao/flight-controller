#include "compass.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <stdbool.h>
#include "bmm350.h"

#include <stdio.h>
#include <math.h>

typedef struct {
    double x;
    double y;
    double z;
} compass_data_t;

struct bmm350_dev BMMdev = {0};
struct bmm350_mag_temp_data mag_temp_data;

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
	compass_data_t compass_data = {
		.x = mag_temp_data.x,
		.y = mag_temp_data.y,
		.z = mag_temp_data.z
	};
	publish(SENSOR_COMPASS, (uint8_t*)&compass_data, sizeof(compass_data_t));
	/* Track running min/max and send scaled values mapped into [-1000,1000].
	   We keep publishing SENSOR_COMPASS (original doubles) for internal use,
	   and send MONITOR_DATA as three float32 values scaled to target range. */
	float x_f = (float)mag_temp_data.x;
	float y_f = (float)mag_temp_data.y;
	float z_f = (float)mag_temp_data.z;

	/* static running observations */
	static float min_x = 1e30f, max_x = -1e30f;
	static float min_y = 1e30f, max_y = -1e30f;
	static float min_z = 1e30f, max_z = -1e30f;
	static int samples = 0;
	const int PRINT_EVERY = 200;

	if (x_f < min_x) min_x = x_f;
	if (x_f > max_x) max_x = x_f;
	if (y_f < min_y) min_y = y_f;
	if (y_f > max_y) max_y = y_f;
	if (z_f < min_z) min_z = z_f;
	if (z_f > max_z) max_z = z_f;

	samples++;
	if ((samples % PRINT_EVERY) == 0) {
		platform_console("MAG OBS ranges after %d samples:\n", samples);
		platform_console("  X: %f .. %f\n", min_x, max_x);
		platform_console("  Y: %f .. %f\n", min_y, max_y);
		platform_console("  Z: %f .. %f\n", min_z, max_z);
	}

	/* Map each axis linearly from [min,max] -> [-TARGET, TARGET] */
	const float TARGET = 1000.0f;
	const float EPS = 1e-6f;
	float sx = 0.0f, sy = 0.0f, sz = 0.0f;

	float range_x = max_x - min_x;
	if (fabs(range_x) > EPS) {
		/* normalize to 0..1 then scale to -TARGET..TARGET */
		float nx = (x_f - min_x) / range_x; /* 0..1 */
		sx = ((nx * 2.0f) - 1.0f) * TARGET; /* -TARGET..TARGET */
	}

	float range_y = max_y - min_y;
	if (fabs(range_y) > EPS) {
		float ny = (y_f - min_y) / range_y;
		sy = ((ny * 2.0f) - 1.0f) * TARGET;
	}

	float range_z = max_z - min_z;
	if (fabs(range_z) > EPS) {
		float nz = (z_f - min_z) / range_z;
		sz = ((nz * 2.0f) - 1.0f) * TARGET;
	}

	/* clamp just in case */
	if (sx > TARGET) sx = TARGET;
	if (sx < -TARGET) sx = -TARGET;
	if (sy > TARGET) sy = TARGET;
	if (sy < -TARGET) sy = -TARGET;
	if (sz > TARGET) sz = TARGET;
	if (sz < -TARGET) sz = -TARGET;

	uint8_t out_msg[12]; /* 3 * 4 bytes (float32) */
	memcpy(&out_msg[0], &sx, sizeof(float));
	memcpy(&out_msg[4], &sy, sizeof(float));
	memcpy(&out_msg[8], &sz, sizeof(float));
	publish(MONITOR_DATA, out_msg, sizeof(out_msg));
}

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
}
