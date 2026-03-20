#ifndef SENSOR_DPS310_H
#define SENSOR_DPS310_H

#include "stdint.h"
#include "dps310_registers.h"

#define DPS310_READ_WAIT_FOR_REG_ATTEMPTS 3

typedef enum {
    IDLE = DPS310_MEAS_CFG_MEAS_CTRL_IDLE,
    PRESSURE_MEASUREMENT = DPS310_MEAS_CFG_MEAS_CTRL_PRS,
    TEMPERATURE_MEASUREMENT = DPS310_MEAS_CFG_MEAS_CTRL_TMP
} dps310_mode_t;

int16_t dps310_probe();

int16_t dps310_configure_temperature(uint8_t data);

int16_t dps310_configure_pressure(uint8_t data);

int16_t dps310_read(float *p_temperature, float *p_pressure);

int16_t dps310_wakeup(dps310_mode_t mode);

int16_t dps310_sleep();

int16_t dps310_reset();

// --- Continuous (background) mode ---
// Start autonomous pressure+temperature measurements at configured rates.
// Result registers update automatically — no blocking trigger/poll needed.
int16_t dps310_start_continuous(void);

// ISR-safe read of latest pressure+temperature from result registers.
// Uses platform_i2c_write_read (HAL_I2C_Mem_Read) — no SysTick dependency.
// Requires dps310_start_continuous() to have been called first.
int16_t dps310_read_continuous(float *p_temperature, float *p_pressure);

float get_altitude(void);

#endif
