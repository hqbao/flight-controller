#ifndef DPS310_H
#define DPS310_H

#include <stdint.h>
#include <stdbool.h>

// DPS310 I2C Address
#define DPS310_I2C_ADDR          0xEC

// DPS310 Register Map
#define DPS310_REG_PRS_CFG       0x06
#define DPS310_REG_TMP_CFG       0x07
#define DPS310_REG_MEAS_CFG      0x08
#define DPS310_REG_CFG_REG       0x09
#define DPS310_REG_RESET         0x0C
#define DPS310_REG_PRODUCT_ID    0x0D
#define DPS310_REG_COEF          0x10
#define DPS310_REG_SRC_COEF      0x28

// Pressure and Temperature Result Registers
#define DPS310_REG_PSR_B2        0x00
#define DPS310_REG_PSR_B1        0x01
#define DPS310_REG_PSR_B0        0x02
#define DPS310_REG_TMP_B2        0x03
#define DPS310_REG_TMP_B1        0x04
#define DPS310_REG_TMP_B0        0x05

// Measurement Configuration Bits
#define DPS310_MEAS_CTRL_PRS     (1 << 0)
#define DPS310_MEAS_CTRL_TMP     (1 << 1)
#define DPS310_COEF_RDY          (1 << 7)
#define DPS310_SENSOR_RDY        (1 << 6)
#define DPS310_TMP_RDY           (1 << 5)
#define DPS310_PRS_RDY           (1 << 4)

// Sensor Configuration
typedef enum {
    DPS310_RATE_1HZ = 0,
    DPS310_RATE_2HZ = 1,
    DPS310_RATE_4HZ = 2,
    DPS310_RATE_8HZ = 3,
    DPS310_RATE_16HZ = 4,
    DPS310_RATE_32HZ = 5,
    DPS310_RATE_64HZ = 6,
    DPS310_RATE_128HZ = 7
} dps310_rate_t;

typedef enum {
    DPS310_OVERSAMPLING_1 = 0,
    DPS310_OVERSAMPLING_2 = 1,
    DPS310_OVERSAMPLING_4 = 2,
    DPS310_OVERSAMPLING_8 = 3,
    DPS310_OVERSAMPLING_16 = 4,
    DPS310_OVERSAMPLING_32 = 5,
    DPS310_OVERSAMPLING_64 = 6,
    DPS310_OVERSAMPLING_128 = 7
} dps310_oversampling_t;

// Sensor Data Structure
typedef struct {
    double pressure;     // Pressure in Pa
    double temperature;  // Temperature in °C
    bool data_ready;
    uint32_t timestamp;
} dps310_data_t;

// Sensor Handle
typedef struct {
    dps310_data_t data;
    int32_t c00, c10, c20, c30, c01, c11, c21;
    int32_t c0, c1;
    uint8_t sensor_id;
    bool initialized;
} dps310_handle_t;

// Function Prototypes
bool dps310_init(dps310_handle_t *dps);
bool dps310_start_continuous(dps310_handle_t *dps, dps310_rate_t rate, dps310_oversampling_t oversampling);
bool dps310_stop_continuous(dps310_handle_t *dps);
bool dps310_read_data(dps310_handle_t *dps);
void dps310_timer_callback(dps310_handle_t *dps);

#endif // DPS310_H
