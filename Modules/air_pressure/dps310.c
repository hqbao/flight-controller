#include "dps310.h"
#include <platform.h>

static bool dps310_read_coefficients(dps310_handle_t *dps);
static bool dps310_write_register(dps310_handle_t *dps, uint8_t reg, uint8_t data);
static bool dps310_read_registers(dps310_handle_t *dps, uint8_t reg, uint8_t *data, uint8_t len);
static double dps310_calculate_temperature(dps310_handle_t *dps, int32_t raw_temp);
static double dps310_calculate_pressure(dps310_handle_t *dps, int32_t raw_pressure, int32_t raw_temp);

bool dps310_init(dps310_handle_t *dps) {
    dps->initialized = false;
    dps->data.data_ready = false;

    uint8_t product_id = 0;

    // Read product ID to verify communication
    if (!dps310_read_registers(dps, DPS310_REG_PRODUCT_ID, &product_id, 1)) {
        return false;
    }

    if (product_id != 0x10) {  // DPS310 product ID
        return false;
    }

    dps->sensor_id = product_id;

    // Reset the sensor
    if (!dps310_write_register(dps, DPS310_REG_RESET, 0x89)) {
        return false;
    }

    platform_delay(50);  // Wait for reset to complete

    // Read calibration coefficients
    if (!dps310_read_coefficients(dps)) {
        return false;
    }

    dps->initialized = true;
    return true;
}

bool dps310_start_continuous(dps310_handle_t *dps, dps310_rate_t rate, dps310_oversampling_t oversampling) {
    if (!dps->initialized) return false;

    // Stop any ongoing measurements first
    dps310_write_register(dps, DPS310_REG_MEAS_CFG, 0x00);
    platform_delay(10);

    // Set pressure configuration
    uint8_t prs_cfg = (rate << 4) | oversampling;
    if (!dps310_write_register(dps, DPS310_REG_PRS_CFG, prs_cfg)) {
        return false;
    }

    // Set temperature configuration
    uint8_t tmp_cfg = (rate << 4) | oversampling;
    if (!dps310_write_register(dps, DPS310_REG_TMP_CFG, tmp_cfg)) {
        return false;
    }

    // Set configuration register for continuous mode
    uint8_t cfg_reg = 0x00; // Continuous mode, no shift
    if (!dps310_write_register(dps, DPS310_REG_CFG_REG, cfg_reg)) {
        return false;
    }

    platform_delay(10);

    // Start continuous measurement of both pressure and temperature
    uint8_t meas_cfg = 0x07; // Continuous pressure and temperature
    if (!dps310_write_register(dps, DPS310_REG_MEAS_CFG, meas_cfg)) {
        return false;
    }

    // Wait for sensor to be ready
    platform_delay(50);

    return true;
}

bool dps310_stop_continuous(dps310_handle_t *dps) {
    if (!dps->initialized) return false;

    // Stop measurements
    if (!dps310_write_register(dps, DPS310_REG_MEAS_CFG, 0x00)) {
        return false;
    }

    return true;
}

bool dps310_read_data(dps310_handle_t *dps) {
    if (!dps->initialized) return false;

    uint8_t meas_cfg = 0;
    if (!dps310_read_registers(dps, DPS310_REG_MEAS_CFG, &meas_cfg, 1)) {
        return false;
    }

    // Check if sensor data is ready
    if (!(meas_cfg & (DPS310_PRS_RDY | DPS310_TMP_RDY))) {
        return false;
    }

    // Read pressure and temperature raw data
    uint8_t raw_data[6];
    if (!dps310_read_registers(dps, DPS310_REG_PSR_B2, raw_data, 6)) {
        return false;
    }

    // Combine pressure bytes (24-bit signed)
    int32_t raw_pressure = (int32_t)((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]);
    if (raw_pressure & 0x800000) {  // Sign extend for 24-bit
        raw_pressure |= 0xFF000000;
    }

    // Combine temperature bytes (24-bit signed)
    int32_t raw_temperature = (int32_t)((raw_data[3] << 16) | (raw_data[4] << 8) | raw_data[5]);
    if (raw_temperature & 0x800000) {  // Sign extend for 24-bit
        raw_temperature |= 0xFF000000;
    }

    // Calculate compensated values
    dps->data.temperature = dps310_calculate_temperature(dps, raw_temperature);
    dps->data.pressure = dps310_calculate_pressure(dps, raw_pressure, raw_temperature);
    dps->data.timestamp = platform_time_ms();
    dps->data.data_ready = true;

    return true;
}

void dps310_timer_callback(dps310_handle_t *dps) {
    if (dps->initialized) {
        dps310_read_data(dps);
    }
}

// Private helper functions
static bool dps310_read_coefficients(dps310_handle_t *dps) {
    uint8_t coef[18];

    if (!dps310_read_registers(dps, DPS310_REG_COEF, coef, 18)) {
        return false;
    }

    // Parse coefficients (little endian)
    dps->c0 = (int32_t)((coef[0] << 4) | (coef[1] >> 4));
    if (dps->c0 & 0x800) {  // Sign extend
        dps->c0 |= 0xFFFFF000;
    }

    dps->c1 = (int32_t)(((coef[1] & 0x0F) << 8) | coef[2]);
    if (dps->c1 & 0x800) {  // Sign extend
        dps->c1 |= 0xFFFFF000;
    }

    dps->c00 = (int32_t)((coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4));
    if (dps->c00 & 0x80000) {  // Sign extend
        dps->c00 |= 0xFFF00000;
    }

    dps->c10 = (int32_t)(((coef[5] & 0x0F) << 16) | (coef[6] << 8) | coef[7]);
    if (dps->c10 & 0x80000) {  // Sign extend
        dps->c10 |= 0xFFF00000;
    }

    // Read remaining coefficients from second source
    if (!dps310_read_registers(dps, DPS310_REG_SRC_COEF, coef, 3)) {
        return false;
    }

    dps->c01 = (int32_t)((coef[0] << 8) | coef[1]);
    dps->c11 = (int32_t)((coef[2] << 8) | coef[3]);
    dps->c20 = (int32_t)((coef[4] << 8) | coef[5]);
    dps->c21 = (int32_t)((coef[6] << 8) | coef[7]);
    dps->c30 = (int32_t)((coef[8] << 8) | coef[9]);

    return true;
}

static double dps310_calculate_temperature(dps310_handle_t *dps, int32_t raw_temp) {
    double scaled_temp = (double)raw_temp / 1048576.0;  // 2^20
    return scaled_temp * dps->c1 + dps->c0 / 2.0;
}

static double dps310_calculate_pressure(dps310_handle_t *dps, int32_t raw_pressure, int32_t raw_temp) {
    double scaled_pressure = (double)raw_pressure / 1048576.0;  // 2^20
    double scaled_temp = (double)raw_temp / 1048576.0;         // 2^20

    double temp_comp = dps->c00 +
    		scaled_pressure * (dps->c10 +
			scaled_pressure * (dps->c20 +
			scaled_pressure * dps->c30)) +
			scaled_temp * (dps->c01 +
			scaled_pressure * (dps->c11 +
			scaled_pressure * dps->c21));

    return temp_comp;
}

static bool dps310_write_register(dps310_handle_t *dps, uint8_t reg, uint8_t data) {
	uint8_t tx[2] = {reg, data};
	return platform_i2c_write(I2C_PORT3, DPS310_I2C_ADDR, tx, 2) == PLATFORM_OK;
}

static bool dps310_read_registers(dps310_handle_t *dps, uint8_t reg, uint8_t *data, uint8_t len) {
	return platform_i2c_write_read(I2C_PORT3, DPS310_I2C_ADDR, &reg, 1, data, len, 100) == PLATFORM_OK;
}
