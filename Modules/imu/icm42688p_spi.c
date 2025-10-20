#include "icm42688p_spi.h"
#include <platform.h>

// Register Values
#define ICM42688_WHO_AM_I_VALUE     0x47
#define ICM42688_PWR_MGMT0_ACCEL_LN (0x0F << 0)  // Accel LN mode
#define ICM42688_PWR_MGMT0_GYRO_LN  (0x0F << 4)  // Gyro LN mode
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
    icm42688p->buffer[1] = ICM42688_PWR_MGMT0_ACCEL_LN | ICM42688_PWR_MGMT0_GYRO_LN;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);
    
    platform_delay(10);
    
    // Configure accelerometer
    icm42688p->buffer[0] = ICM42688_ACCEL_CONFIG0 | ICM42688_WRITE_FLAG;
    icm42688p->buffer[1] = ICM42688_ACCEL_CONFIG0_8G;
    platform_spi_write(SPI_PORT1, icm42688p->buffer, 2);
    
    // Configure gyroscope
    icm42688p->buffer[0] = ICM42688_GYRO_CONFIG0 | ICM42688_WRITE_FLAG;
    icm42688p->buffer[1] = ICM42688_GYRO_CONFIG0_1000DPS;
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
    print("gx: %.5f\t gy: %.5f\t gz: %.5f\t ax: %.5f\t ay: %.5f\t az: %.5f\n", 
        (float)gx/20000, (float)gy/20000, (float)gz/20000, 
        (float)ax/20000, (float)ay/20000, (float)az/20000);

    data[0] = ax;
    data[1] = ay;
    data[2] = az;
    data[3] = gx;
    data[4] = gy;
    data[5] = gz;
}
