#include "dps310_platform.h"
#include "main.h"

// Sensor driver includes
#include "dps310_errors.h"

extern I2C_HandleTypeDef hi2c4;

int8_t i2c4Recv(uint16_t addr, uint8_t *data, uint16_t size) {
	return HAL_I2C_Master_Receive(&hi2c4, addr, data, size, 10);
}

int8_t i2c4Trans(uint16_t addr, uint8_t *data, uint16_t size) {
	return HAL_I2C_Master_Transmit(&hi2c4, addr, data, size, 100);
}

void dps310_i2c_delay_ms(uint32_t delay) {
  HAL_Delay(delay);
}

void dps310_i2c_init(void) {}

void dps310_i2c_release(void) {}

void dps310_enable(void) {}

int8_t dps310_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count) {
    int8_t ret;
    uint8_t buff[1];

    buff[0] = reg;

    ret = i2c4Trans((uint16_t)address, buff, 1);
    if (ret != 0) {
      return DPS310_I2C_FAIL_ERROR;
    }

    dps310_i2c_delay_ms(1); //10ms

    ret = i2c4Recv((uint16_t)address, data, count);
    if (ret != 0) return DPS310_I2C_FAIL_ERROR;

    return DPS310_OK;
}

int8_t dps310_i2c_write(uint8_t address, uint8_t reg, const uint8_t *data, uint16_t count) {
	int8_t ret;
	uint16_t count_with_reg = count + 1;
	uint8_t buff[DPS310_I2C_MAX_BUFF_SIZE];

	buff[0] = reg;

	for (uint8_t i = 1; i < count_with_reg; i++) {
		buff[i] = data[i - 1];
	}

	ret = i2c4Trans((uint16_t)address, buff, count_with_reg);
	if (ret != 0) return DPS310_I2C_FAIL_ERROR;

	return DPS310_OK;
}
