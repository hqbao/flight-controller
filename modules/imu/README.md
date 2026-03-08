# IMU Module

## Overview

Drives the **ICM-42688P** inertial measurement unit. Reads gyroscope (1 kHz) and accelerometer (500 Hz), handles automatic gyro calibration at startup, and supports manual accelerometer calibration via an external Python tool.

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
| `CALIBRATION_FREQ` | 2000 | Samples for calibration (2 seconds at 1 kHz) |
| `IMU_MOTION` | 32 | Max gyro spread (LSB) for valid calibration (~2 dps) |

## Calibration

### Gyroscope (Automatic)
- Runs automatically on startup or when triggered
- Collects 2000 samples over 2 seconds
- Requires drone to be **stationary** — if max-min > 32 LSB on any axis, retries
- Computes average offset per axis, saves to flash via `LOCAL_STORAGE`
- Restores saved calibration on subsequent boots

### Accelerometer (Manual)
1. Flash firmware and connect via USB
2. Run `python3 tools/imu_calibrate_accel.py`
3. Click **"Start Log"** to see live readings
4. Place drone in 6 orientations, click **"Capture Position"** for each
5. Click **"Compute Calib"**, copy Bias (B) and Scale Matrix (S) into `imu.c`

Model: `V_cal = S × (V_raw − B)`

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SCHEDULER_1KHZ` | 1 kHz | Trigger sensor read |
| `SCHEDULER_500HZ` | 500 Hz | Publish calibrated accel |
| `I2C_CALLBACK_UPDATE` | Event | Process I2C DMA completion |
| `SPI_CALLBACK_UPDATE` | Event | Process SPI DMA completion |
| `NOTIFY_LOG_CLASS` | Event | Activate/deactivate logging |
| `SCHEDULER_25HZ` | 25 Hz | Stream accel log data |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SENSOR_IMU1_GYRO_UPDATE` | `float[3]` — gx, gy, gz (deg/s) | 1 kHz |
| `SENSOR_IMU1_ACCEL_UPDATE` | `float[3]` — ax, ay, az (calibrated) | 500 Hz |
| `SEND_LOG` | `float[3]` — ax, ay, az (12 bytes) | 25 Hz |

## Log Class

`LOG_CLASS_IMU_ACCEL` (0x01) — streams 3 accelerometer floats at 25 Hz.

## Files

| File | Purpose |
|------|---------|
| `imu.c` / `imu.h` | Module logic, calibration, logging |
| `icm42688p.c` / `icm42688p.h` | Sensor driver (register config) |
| `icm42688p_i2c.c` / `icm42688p_i2c.h` | I2C transport |
| `icm42688p_spi.c` / `icm42688p_spi.h` | SPI transport |
