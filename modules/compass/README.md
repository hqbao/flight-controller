# Compass Module

## Overview

Reads the **BMM350** 3-axis magnetometer at 25 Hz, applies hard/soft iron calibration, normalizes the result to a unit vector, and publishes the calibrated sensor-frame vector. `state_estimation.c` currently performs the final sensor-to-body mapping for diagnostics only; compass data is not fused into attitude.

## Data Flow

```
BMM350 (I2C_PORT2 = I2C3)
    │
    ▼  SCHEDULER_25HZ
  bmm350_get_compensated_mag_xyz_temp_data()
    │
    ▼  V_cal = S × (V_raw − B)
    ▼  normalize → calibrated sensor-frame unit vector
    │
    ├─► SENSOR_COMPASS  (vector3d_t, calibrated sensor-frame unit vector, 25 Hz)
    └─► SEND_LOG        (float[3], 12 bytes, 25 Hz when logging active)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Sensor    | Bosch BMM350 |
| Interface | I2C (`I2C_PORT2` → STM32 I2C3) |
| Address   | `0x14` / `0x28` (ADSEL low, left-shifted) |
| ODR       | 25 Hz |
| Averaging | 8× |
| Mode      | Normal |

### I2C Dummy Bytes

The BMM350 prepends **2 dummy bytes** before real register data on every burst read, even over I2C. Bosch's `bmm350_get_regs()` handles this internally by requesting `len + 2` bytes and copying from offset 2. The `i2c_read` callback passes the full requested length to the HAL unchanged.

## Calibration

Run `python3 tools/calibration_compass.py`:
1. Flash firmware and connect via USB.
2. Click **"Start Log"** (sends `LOG_CLASS_COMPASS` to stream raw data).
3. Rotate the drone through all orientations (figure-8 / tumble).
4. Click **"Calibrate & Upload"** — computes B and S, saves to flash.

Calibration model: `V_cal = S × (V_raw − B)`, normalized to unit vector.

## Axis Mapping

The compass module does **not** apply the board/body axis mapping. It publishes the calibrated sensor-frame unit vector so the estimator owns all sensor-to-body conversions in one place. `state_estimation.c` maps the BMM350 via `mag_axis_map()` in `sensor_unit.h`:

```c
body_x =  sensor_y;
body_y = -sensor_x;
body_z =  sensor_z;
```

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SCHEDULER_25HZ`        | 25 Hz | Read sensor, apply calibration, publish |
| `SCHEDULER_25HZ`        | 25 Hz | Stream log data (if log class active) |
| `SCHEDULER_1HZ`         | 1 Hz  | Poll for calibration data from flash |
| `NOTIFY_LOG_CLASS`      | Event | Activate / deactivate logging |
| `CALIBRATION_MAG_READY` | Event | Load new calibration values from flash |

### Publications
| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `SENSOR_COMPASS`          | `vector3d_t` | 25 Hz | Calibrated sensor-frame unit vector |
| `CALIBRATION_MAG_REQUEST` | —            | 1 Hz  | Requests mag calibration from flash (until loaded) |
| `SEND_LOG`                | `float[3]`   | 25 Hz | 12 bytes; raw or calibrated (see log class) |

## Log Classes

| Class | Content |
|-------|---------|
| `LOG_CLASS_COMPASS`       | Raw µT values — used by calibration tool |
| `LOG_CLASS_COMPASS_CALIB` | Calibrated sensor-frame unit vector |
