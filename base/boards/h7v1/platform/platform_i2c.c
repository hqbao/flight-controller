/**
 ******************************************************************************
 * @file           : platform_i2c.c
 * @brief          : I2C platform driver — STM32H7 HAL implementation
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>
#include <pubsub.h>

/* Port mapping: I2C_PORT1→I2C1, I2C_PORT2→I2C3, I2C_PORT3→I2C4 */
static I2C_HandleTypeDef *i2c_ports[3] = {&hi2c1, &hi2c3, &hi2c4};

/* --- Platform API -------------------------------------------------------- */

char platform_i2c_write_read_dma(i2c_port__t port, uint8_t address,
		uint8_t *input, uint16_t input_size,
		uint8_t *output, uint16_t output_size) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(i2c_ports[port], address,
			*(uint16_t *)input, input_size, output, output_size);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_i2c_write_read(i2c_port__t port, uint8_t address,
		uint8_t *input, uint16_t input_size,
		uint8_t *output, uint16_t output_size, uint32_t timeout) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(i2c_ports[port], address,
			*(uint16_t *)input, input_size, output, output_size, timeout);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_i2c_read(i2c_port__t port, uint8_t address,
		uint8_t *output, uint16_t output_size) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(i2c_ports[port], address,
			output, output_size, 1000);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_i2c_write(i2c_port__t port, uint8_t address,
		uint8_t *input, uint16_t input_size) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c_ports[port], address,
			input, input_size, 1000);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

/* --- HAL Callback -------------------------------------------------------- */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		i2c_port__t port = I2C_PORT1;
		publish(I2C_CALLBACK_UPDATE, (uint8_t*)&port, 1);
	} else if (hi2c->Instance == I2C3) {
		i2c_port__t port = I2C_PORT2;
		publish(I2C_CALLBACK_UPDATE, (uint8_t*)&port, 1);
	} else if (hi2c->Instance == I2C4) {
		i2c_port__t port = I2C_PORT3;
		publish(I2C_CALLBACK_UPDATE, (uint8_t*)&port, 1);
	}
}
