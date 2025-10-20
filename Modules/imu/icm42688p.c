#include "icm42688p.h"
#include <platform.h>
#include "icm42688p_i2c.h"
#include "icm42688p_spi.h"

void icm42688p_init(icm42688p_t *icm42688p, 
    uint8_t accel_scale, uint8_t gyro_scale,
	uint8_t accel_odr, uint8_t gyro_odr, 
    uint8_t accel_mode, uint8_t gyro_mode) {
    if (icm42688p->use_port == 0) {
        icm42688p_init_i2c(icm42688p, accel_scale, gyro_scale, accel_odr, gyro_odr, accel_mode, gyro_mode);
    } else if (icm42688p->use_port == 1) {
        icm42688p_init_spi(icm42688p, accel_scale, gyro_scale, accel_odr, gyro_odr, accel_mode, gyro_mode);
    }
}

void icm42688p_read(icm42688p_t *icm42688p) {
    if (icm42688p->use_port == 0) {
        icm42688p_read_i2c(icm42688p);
    } else if (icm42688p->use_port == 1) {
        icm42688p_read_spi(icm42688p);
    }
}
