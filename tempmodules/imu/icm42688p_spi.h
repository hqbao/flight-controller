#ifndef ICM42688P_SPI_H
#define ICM42688P_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "icm42688p.h"

void icm42688p_init_spi(icm42688p_t *icm42688p, 
	uint8_t accel_scale, uint8_t gyro_scale,
	uint8_t accel_odr, uint8_t gyro_odr, 
	uint8_t accel_mode, uint8_t gyro_mode);
void icm42688p_read_spi(icm42688p_t *icm42688p);
void icm42688p_get_spi(icm42688p_t *icm42688p, float *data);

#endif