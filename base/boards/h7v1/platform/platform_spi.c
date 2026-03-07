/**
 ******************************************************************************
 * @file           : platform_spi.c
 * @brief          : SPI platform driver — STM32H7 HAL implementation
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>

/* --- Platform API -------------------------------------------------------- */

char platform_spi_write(spi_port_t spi_port, uint8_t *input, uint8_t size) {
	return PLATFORM_NOT_SUPPORT;
}

char platform_spi_write_read(spi_port_t spi_port,
		uint8_t *input, uint16_t input_size,
		uint8_t *output, uint16_t output_size) {
	return PLATFORM_NOT_SUPPORT;
}

/* --- HAL Callback -------------------------------------------------------- */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		platform_spi_data_dma_callback(SPI_PORT1);
	} else if (hspi->Instance == SPI2) {
		platform_spi_data_dma_callback(SPI_PORT2);
	} else if (hspi->Instance == SPI3) {
		platform_spi_data_dma_callback(SPI_PORT3);
	}
}
