#include "compass.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <stdbool.h>
#include "bmm350.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

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
	HAL_Delay(period/1000);
}

static void loop_25hz(uint8_t *data, size_t size) {
	bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &BMMdev);
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
