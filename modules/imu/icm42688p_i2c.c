#include "icm42688p_i2c.h"
#include <platform.h>

void icm42688p_init_i2c(icm42688p_t *icm42688p, 
    uint8_t accel_scale, uint8_t gyro_scale,
	uint8_t accel_odr, uint8_t gyro_odr, 
    uint8_t accel_mode, uint8_t gyro_mode) {

    // ---------------- BANK 0 CONFIGURATION ----------------
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // [Change 1] ENABLE Accel Analog Anti-Aliasing Filter (AAF)
    // Bit 0 = 0 (ACCEL_UI_AAF_DIS = Enabled)
    icm42688p->buffer[0] = ICM42688_ACCEL_CONFIG1;
    icm42688p->buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Set power modes
    icm42688p->buffer[0] = ICM42688_PWR_MGMT0;
    icm42688p->buffer[1] = (gyro_mode << 2) | accel_mode;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);
    platform_delay(1);

    // Configure Accel Scale & ODR
    if (accel_mode != 0x00) {
        icm42688p->buffer[0] = ICM42688_ACCEL_CONFIG0;
        icm42688p->buffer[1] = (accel_scale << 5) | accel_odr;
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);
    }

    // Configure Gyro Scale & ODR
    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG0;
    icm42688p->buffer[1] = (gyro_scale << 5) | gyro_odr;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // [Change 2] ENABLE Digital Low Pass Filters for BOTH
    // Gyro BW (Bits 7:4)  = 1 (ODR/4 - Standard LPF, low latency)
    // Accel BW (Bits 3:0) = 4 (ODR/32 - Smooth LPF for Position Hold)
    // Value = 0x14 - gyro: 0001 accel: 0100
    icm42688p->buffer[0] = ICM42688_GYRO_ACCEL_CONFIG0;
    icm42688p->buffer[1] = 0x33;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Enable FIFO
    icm42688p->buffer[0] = ICM42688_FIFO_CONFIG1;
    icm42688p->buffer[1] = 0x03;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Enable Timestamp
    icm42688p->buffer[0] = ICM42688_TMST_CONFIG;
    icm42688p->buffer[1] = 0x01;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Configure Interrupts
    icm42688p->buffer[0] = ICM42688_INT_CONFIG;
    icm42688p->buffer[1] = 0x18 | 0x03;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_INT_SOURCE0;
    icm42688p->buffer[1] = 0x08;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // ---------------- BANK 2 CONFIGURATION ----------------
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x02;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // [Change 3] ENABLE Gyro Analog AAF
    // Bit 1 = 0 (Enable AAF)
    // Bit 0 = 0 (Enable Notch - inactive if params are 0)
    // Value = 0x00
    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG_STATIC2;
    icm42688p->buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // ---------------- CLEANUP ----------------
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->buffer, 2);

    // Prepare for reading
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
