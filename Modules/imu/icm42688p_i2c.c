#include "icm42688p_i2c.h"
#include <platform.h>

void icm42688p_init_i2c(icm42688p_t *icm42688p, 
    uint8_t accel_scale, uint8_t gyro_scale,
	uint8_t accel_odr, uint8_t gyro_odr, 
    uint8_t accel_mode, uint8_t gyro_mode) {
    // Ensure register bank 0 is selected
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Verify WHO_AM_I (optional but recommended)
    icm42688p->buffer[0] = ICM42688_WHO_AM_I;
    platform_i2c_write_read(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 1, &icm42688p->buffer[1], 1, 1000);
    print("WHO_AM_I = 0x%02X\n", icm42688p->buffer[1]);

    // Set power modes (gyro in low-noise mode, accel off if not needed)
    icm42688p->buffer[0] = ICM42688_PWR_MGMT0;
    icm42688p->buffer[1] = (gyro_mode << 2) | accel_mode;  // gyro_mode = 0x03 (LN mode), accel_mode = 0x00 (off)
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);
    platform_delay(1);

    // Configure accelerometer (if used, else skip)
    if (accel_mode != 0x00) {
        icm42688p->buffer[0] = ICM42688_ACCEL_CONFIG0;
        icm42688p->buffer[1] = (accel_scale << 5) | accel_odr;
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);
    }

    // Configure gyro: 8 kHz ODR + desired scale (e.g., 2000dps)
    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG0;
    icm42688p->buffer[1] = (gyro_scale << 5) | gyro_odr;  // gyro_odr = 0x08 (8 kHz)
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Set gyro bandwidth to ODR/2 (4 kHz) for minimal filtering
    icm42688p->buffer[0] = ICM42688_GYRO_ACCEL_CONFIG0;
    icm42688p->buffer[1] = 0x07;  // Gyro BW = 000 (ODR/2), Accel BW = 111 (ODR/320)
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Enable FIFO for gyro data (critical for 8 kHz streaming)
    icm42688p->buffer[0] = ICM42688_FIFO_CONFIG1;
    icm42688p->buffer[1] = 0x03;  // FIFO mode + gyro data stored
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_TMST_CONFIG;
    icm42688p->buffer[1] = 0x01; // Enable temp compensation
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Configure interrupts (data-ready on INT1)
    icm42688p->buffer[0] = ICM42688_INT_CONFIG;
    icm42688p->buffer[1] = 0x18 | 0x03;  // Push-pull, pulsed, active HIGH
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_INT_SOURCE0;
    icm42688p->buffer[1] = 0x08;  // Data-ready interrupt to INT1
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Bank 2
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x02;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG_STATIC2;
    icm42688p->buffer[1] = 0x00; // Disable AA filter
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Return to Bank 0
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Prepare for reading (temperature register as placeholder)
    icm42688p->buffer[0] = ICM42688_TEMP_DATA1;
}

void icm42688p_read_i2c(icm42688p_t *icm42688p) {
	platform_i2c_write_read_dma(icm42688p->i2c_port, ICM42688_ADDRESS,
			icm42688p->buffer, 1, &icm42688p->buffer[2], 14);
}

void icm42688p_get_i2c(icm42688p_t *icm42688p, float *data) {
    uint8_t *buffer = &icm42688p->buffer[2];
    int16_t ax = (int16_t) (buffer[2] << 8 | buffer[3]);
    int16_t ay = (int16_t) (buffer[4] << 8 | buffer[5]);
    int16_t az = (int16_t) (buffer[6] << 8 | buffer[7]);
    int16_t gx = (int16_t) (buffer[8] << 8 | buffer[9]);
    int16_t gy = (int16_t) (buffer[10] << 8 | buffer[11]);
    int16_t gz = (int16_t) (buffer[12] << 8 | buffer[13]);

    data[0] = ax;
    data[1] = ay;
    data[2] = az;
    data[3] = gx;
    data[4] = gy;
    data[5] = gz;
}
