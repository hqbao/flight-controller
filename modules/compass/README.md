# Compass Module

## Overview

Reads the **BMM350** 3-axis magnetometer at 25 Hz, applies hard/soft iron calibration, normalizes the vector, and publishes a calibrated compass heading for attitude estimation.

## Data Flow

```
BMM350 (I2C)
    │
    ▼  SCHEDULER_25HZ
  Read raw mag vector
    │
    ▼  Hard iron subtract → Soft iron matrix multiply → Normalize
    │
    └─► SENSOR_COMPASS (vector3d_t, 25 Hz)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Sensor | BMM350 |
| Interface | I2C (`I2C_PORT2`) |
| ODR | 25 Hz |
| Averaging | 8x |
| Mode | Normal |

## Calibration

Manual via `python3 tools/compass_calibrate.py`:
1. Flash firmware and connect via USB
2. Run the tool, click **"Start Log"**
3. Click **"Start Stream"**, rotate drone in all directions (figure-8 motion)
4. Copy hard iron bias (B) and soft iron matrix (S) into `compass.c`

Model: `V_cal = S × (V_raw − B)`, then normalize to unit vector.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `COMPASS_MONITOR_MODE` | 1 or 2 | 1: calibrated data, 2: raw data |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SCHEDULER_25HZ` | 25 Hz | Read sensor and publish |
| `NOTIFY_LOG_CLASS` | Event | Activate/deactivate logging |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SENSOR_COMPASS` | `vector3d_t` — calibrated, normalized | 25 Hz |
| `SEND_LOG` | `float[3]` — compass vector (12 bytes) | 25 Hz |

## Log Class

`LOG_CLASS_COMPASS` (0x02) — streams 3 compass floats at 25 Hz.
