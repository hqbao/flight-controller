# IMU Module

## Overview

Drives the **ICM-42688P** inertial measurement unit. Reads gyroscope (1 kHz) and accelerometer (500 Hz), and applies calibration values received from the calibration module via PubSub.

## Data Flow

```
ICM-42688P (I2C/SPI)
    │
    ▼  SCHEDULER_1KHZ → icm42688p_read()
  I2C/SPI DMA callback
    │
    ├─ Gyro: offset subtract → ÷ SSF_GYRO → deg/s
    │   └─► SENSOR_IMU1_GYRO_UPDATE (float[3], 1 kHz)
    │
    └─ Accel: offset subtract → scale matrix → calibrated
        └─► SENSOR_IMU1_ACCEL_UPDATE (float[3], 500 Hz)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Sensor | ICM-42688P |
| Interface | I2C (`I2C_PORT1`) or SPI (`SPI_PORT1`) |
| Gyro range | ±2000 dps |
| Accel range | ±2G |
| Gyro ODR | 4 kHz (hardware), read at 1 kHz |
| Accel ODR | 500 Hz |
| Mode | Low-Noise |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `SSF_GYRO` | 16.4 | Gyro sensitivity scale factor (LSB/dps at 2000 dps) |


## Calibration

All calibration is managed by the `calibration` module and delivered via PubSub:
- **Gyro temperature compensation** → `CALIBRATION_GYRO_READY` (polynomial coefficients `a·T² + b·T + c` per axis, uploaded via Python tool, saved to flash)
- **Accel bias + scale** → `CALIBRATION_ACCEL_READY` (uploaded via Python tool, saved to flash)

### Accelerometer (Manual)
1. Flash firmware and connect via USB
2. Run `python3 tools/calibration_accel.py`
3. Click **"Start Log"** to see live readings
4. Place drone in 6 orientations, click **"Capture Position"** for each
5. Click **"Compute Calib"**, then click **"Upload to FC"** — saves to flash automatically

Model: `V_cal = S × (V_raw − B)`

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SCHEDULER_1KHZ` | 1 kHz | Trigger gyro sensor read |
| `SCHEDULER_500HZ` | 500 Hz | Publish calibrated accel |
| `I2C_CALLBACK_UPDATE` | Event | Process I2C DMA completion |
| `SPI_CALLBACK_UPDATE` | Event | Process SPI DMA completion |
| `CALIBRATION_GYRO_READY` | Event | Receive gyro bias from calibration module |
| `CALIBRATION_ACCEL_READY` | Event | Receive accel bias + scale from calibration module |
| `NOTIFY_LOG_CLASS` | Event | Activate/deactivate logging |
| `SCHEDULER_25HZ` | 25 Hz | Stream accel log data |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SENSOR_IMU1_GYRO_UPDATE` | `float[3]` — gx, gy, gz (deg/s) | 1 kHz |
| `SENSOR_IMU1_GYRO_RAW` | `float[3]` — raw gyro (LSB) | 1 kHz |
| `SENSOR_IMU1_ACCEL_UPDATE` | `float[3]` — ax, ay, az (calibrated) | 500 Hz |
| `SEND_LOG` | `float[4]` — sensor values + temperature (16 bytes) | 25 Hz |

## Log Classes

| Log Class | ID | Data |
|-----------|----|------|
| `LOG_CLASS_IMU_ACCEL_RAW` | 0x01 | Raw accelerometer + temperature (4 floats) |
| `LOG_CLASS_IMU_ACCEL_CALIB` | 0x0A | Calibrated accelerometer + temperature (4 floats) |
| `LOG_CLASS_IMU_GYRO_RAW` | 0x0B | Raw gyroscope + temperature (4 floats, LSB + °C) |
| `LOG_CLASS_IMU_GYRO_CALIB` | 0x0C | Calibrated gyroscope + temperature (4 floats, °/s + °C) |

## Files

| File | Purpose |
|------|---------|
| `imu.c` / `imu.h` | Module logic, calibration, logging |
| `icm42688p.c` / `icm42688p.h` | Sensor driver (register config) |
| `icm42688p_i2c.c` / `icm42688p_i2c.h` | I2C transport |
| `icm42688p_spi.c` / `icm42688p_spi.h` | SPI transport |
