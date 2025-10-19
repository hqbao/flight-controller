#include "dps310_platform.h"
#include "dps310_errors.h"
#include <platform.h>

void dps310_i2c_init(void) {}

void dps310_i2c_release(void) {}

void dps310_enable(void) {}

int8_t dps310_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count) {
    uint8_t buff[1] = {reg};
    int8_t ret = platform_i2c_write(I2C_PORT3, address, buff, 1);
    if (ret != 0) return DPS310_I2C_FAIL_ERROR;

    //dps310_i2c_delay_ms(1); //10ms
    platform_delay(1);

    //ret = i2c4Recv((uint16_t)address, data, count);
    platform_i2c_read(I2C_PORT3, address, data, count);
    if (ret != 0) return DPS310_I2C_FAIL_ERROR;

    return DPS310_OK;
}

int8_t dps310_i2c_write(uint8_t address, uint8_t reg, const uint8_t *data, uint16_t count) {
	uint16_t count_with_reg = count + 1;
	uint8_t buff[DPS310_I2C_MAX_BUFF_SIZE];
	buff[0] = reg;
	for (uint8_t i = 1; i < count_with_reg; i++) {
		buff[i] = data[i - 1];
	}

	int8_t ret = platform_i2c_write(I2C_PORT3, address, buff, count_with_reg);
	if (ret != 0) return DPS310_I2C_FAIL_ERROR;

	return DPS310_OK;
}
