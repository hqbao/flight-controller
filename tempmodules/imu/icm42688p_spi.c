#include "icm42688p_spi.h"
#include <platform.h>

// Register Values
#define ICM42688_WHO_AM_I_VALUE     0x47
#define ICM42688_ACCEL_CONFIG0_8G   (0x03 << 5)  // ±8g range
#define ICM42688_GYRO_CONFIG0_1000DPS (0x03 << 5) // ±1000 dps range

// SPI Read/Write flags
#define ICM42688_READ_FLAG          0x80
#define ICM42688_WRITE_FLAG         0x00

void icm42688p_init_spi(icm42688p_t *icm42688p, 
    uint8_t accel_scale, uint8_t gyro_scale,
    uint8_t accel_odr, uint8_t gyro_odr, 
    uint8_t accel_mode, uint8_t gyro_mode) {

    // Read WHO_AM_I register
    icm42688p->buffer[0] = ICM42688_WHO_AM_I | ICM42688_READ_FLAG;
    icm42688p->buffer[1] = 0x00;
    platform_spi_write_read(SPI_PORT1, icm42688p->buffer, 2, &icm42688p->buffer[2], 1);
    print("WHO_AM_I = 0x%02X\n", icm42688p->buffer[3]);
    
    // Software reset (optional
    icm42688p->buffer[0] = 0x4B | ICM42688_READ_FLAG;
    icm42688p->buffer[1] = 0x00;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);
    platform_delay(100);
    
    // Configure power management
    icm42688p->buffer[0] = ICM42688_PWR_MGMT0 | ICM42688_WRITE_FLAG;
    icm42688p->buffer[1] = (gyro_mode << 2) | accel_mode;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);
    
    platform_delay(10);
    
    // Configure accelerometer
    icm42688p->buffer[0] = ICM42688_ACCEL_CONFIG0 | ICM42688_WRITE_FLAG;
    icm42688p->buffer[1] = (accel_scale << 5) | accel_odr;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);
    
    // Configure gyroscope
    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG0 | ICM42688_WRITE_FLAG;
    icm42688p->buffer[1] = (gyro_scale << 5) | gyro_odr;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    // Set gyro bandwidth to ODR/2 (4 kHz) for minimal filtering
    icm42688p->buffer[0] = ICM42688_GYRO_ACCEL_CONFIG0;
    icm42688p->buffer[1] = 0x07;  // Gyro BW = 000 (ODR/2), Accel BW = 111 (ODR/320)
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    // Enable FIFO for gyro data (critical for 8 kHz streaming)
    icm42688p->buffer[0] = ICM42688_FIFO_CONFIG1;
    icm42688p->buffer[1] = 0x03;  // FIFO mode + gyro data stored
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_TMST_CONFIG;
    icm42688p->buffer[1] = 0x01; // Enable temp compensation
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    // Configure interrupts (data-ready on INT1)
    icm42688p->buffer[0] = ICM42688_INT_CONFIG;
    icm42688p->buffer[1] = 0x18 | 0x03;  // Push-pull, pulsed, active HIGH
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_INT_SOURCE0;
    icm42688p->buffer[1] = 0x08;  // Data-ready interrupt to INT1
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    // Bank 2
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x02;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG_STATIC2;
    icm42688p->buffer[1] = 0x00; // Disable AA filter
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    // Return to Bank 0
    icm42688p->buffer[0] = ICM42688_REG_BANK_SEL;
    icm42688p->buffer[1] = 0x00;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);

    icm42688p->buffer[0] = ICM42688_TEMP_DATA1 | ICM42688_READ_FLAG;
    icm42688p->buffer[1] = 0x00;
}

void icm42688p_read_spi(icm42688p_t *icm42688p) {
    platform_spi_write_read(SPI_PORT1, icm42688p->buffer, 2, &icm42688p->buffer[2], 14);
}

void icm42688p_get_spi(icm42688p_t *icm42688p, float *data) {
    uint8_t *buffer = &icm42688p->buffer[3];
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
