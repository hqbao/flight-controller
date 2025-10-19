#include "icm42688p.h"
#include <platform.h>

void icm42688p_init(icm42688p_t *icm42688p, uint8_t Ascale, uint8_t Gscale,
		uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode, bool CLKIN) {
    // Ensure register bank 0 is selected
    icm42688p->i2c_buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->i2c_buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Verify WHO_AM_I (optional but recommended)
    icm42688p->i2c_buffer[0] = ICM42688_WHO_AM_I;
    platform_i2c_write_read(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 1, &icm42688p->i2c_buffer[1], 1, 1000);

    // Set power modes (gyro in low-noise mode, accel off if not needed)
    icm42688p->i2c_buffer[0] = ICM42688_PWR_MGMT0;
    icm42688p->i2c_buffer[1] = (gMode << 2) | aMode;  // gMode = 0x03 (LN mode), aMode = 0x00 (off)
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);
    platform_delay(1);

    // Configure accelerometer (if used, else skip)
    if (aMode != 0x00) {
        icm42688p->i2c_buffer[0] = ICM42688_ACCEL_CONFIG0;
        icm42688p->i2c_buffer[1] = (Ascale << 5) | AODR;
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);
    }

    // Configure gyro: 8 kHz ODR + desired scale (e.g., 2000dps)
    icm42688p->i2c_buffer[0] = ICM42688_GYRO_CONFIG0;
    icm42688p->i2c_buffer[1] = (Gscale << 5) | GODR;  // GODR = 0x08 (8 kHz)
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Set gyro bandwidth to ODR/2 (4 kHz) for minimal filtering
    icm42688p->i2c_buffer[0] = ICM42688_GYRO_ACCEL_CONFIG0;
    icm42688p->i2c_buffer[1] = 0x07;  // Gyro BW = 000 (ODR/2), Accel BW = 111 (ODR/320)
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Enable FIFO for gyro data (critical for 8 kHz streaming)
    icm42688p->i2c_buffer[0] = ICM42688_FIFO_CONFIG1;
    icm42688p->i2c_buffer[1] = 0x03;  // FIFO mode + gyro data stored
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    icm42688p->i2c_buffer[0] = ICM42688_TMST_CONFIG;
    icm42688p->i2c_buffer[1] = 0x01; // Enable temp compensation
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Configure interrupts (data-ready on INT1)
    icm42688p->i2c_buffer[0] = ICM42688_INT_CONFIG;
    icm42688p->i2c_buffer[1] = 0x18 | 0x03;  // Push-pull, pulsed, active HIGH
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    icm42688p->i2c_buffer[0] = ICM42688_INT_SOURCE0;
    icm42688p->i2c_buffer[1] = 0x08;  // Data-ready interrupt to INT1
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Optional: External clock (if CLKIN=true)
    if (CLKIN) {
        icm42688p->i2c_buffer[0] = ICM42688_INTF_CONFIG1;
        icm42688p->i2c_buffer[1] = 0x95;  // Enable RTC
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

        icm42688p->i2c_buffer[0] = ICM42688_REG_BANK_SEL;
        icm42688p->i2c_buffer[1] = 0x01;  // Bank 1
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

        icm42688p->i2c_buffer[0] = ICM42688_INTF_CONFIG5;
        icm42688p->i2c_buffer[1] = 0x04;  // Use CLKIN
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

        // Return to bank 0
        icm42688p->i2c_buffer[0] = ICM42688_REG_BANK_SEL;
        icm42688p->i2c_buffer[1] = 0x00;
        platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);
    }

    // Bank 2
    icm42688p->i2c_buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->i2c_buffer[1] = 0x02;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    icm42688p->i2c_buffer[0] = ICM42688_GYRO_CONFIG_STATIC2;
    icm42688p->i2c_buffer[1] = 0x00; // Disable AA filter
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Return to Bank 0
    icm42688p->i2c_buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->i2c_buffer[1] = 0x00;
    platform_i2c_write(icm42688p->i2c_port, ICM42688_ADDRESS, icm42688p->i2c_buffer, 2);

    // Prepare for reading (temperature register as placeholder)
    icm42688p->i2c_buffer[0] = ICM42688_TEMP_DATA1;
}

void icm42688p_read(icm42688p_t *icm42688p, float *data) {
	uint8_t *buffer = &icm42688p->i2c_buffer[2];
	platform_i2c_write_read_dma(icm42688p->i2c_port, ICM42688_ADDRESS,
			icm42688p->i2c_buffer, 1, buffer, 14);

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
