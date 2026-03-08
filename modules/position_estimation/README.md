# Position Estimation Module

## Overview

Estimates the drone's 3D position and velocity by fusing IMU linear acceleration with optical flow horizontal velocity and barometer/laser altitude. Uses three independent **Fusion6** (Scalar Cascaded Complementary Filter) instances — one per axis.

## Data Flow

```
Linear Accel (500 Hz)    Optical Flow (~25 Hz)    Baro / Laser
    │                         │                        │
    ▼                         ▼                        ▼
  Predict step            Update X/Y               Update Z
  (acceleration →         (flow velocity →          (altitude →
   velocity → position)    position correction)      height correction)
    │
    └─► POSITION_STATE_UPDATE (position + velocity, vector3d_t each)
```

## Fusion Architecture

Three independent `fusion6_t` instances for X, Y, Z axes:

| Axis | Predict | Update Source |
|------|---------|---------------|
| X (North) | Linear accel × g | Optical flow velocity |
| Y (East) | Linear accel × g | Optical flow velocity |
| Z (Down) | Linear accel × g | Barometer or laser altitude delta |

Coordinate transform from body frame:
```c
nav_x = -body_y * GRAVITY_MSS;
nav_y = -body_x * GRAVITY_MSS;
```

Optical flow velocity: `vel = angular_displacement × 5.0` (empirical gain).

## Altitude Source Switching

Automatic hysteresis-based switching between laser range finder and barometer:

| Condition | Source |
|-----------|--------|
| Range < 0.25 m | Switch to laser |
| Range > 0.5 m | Switch to barometer |
| Between 0.25–0.5 m | Keep current source |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `GRAVITY_MSS` | `9.80665` | Gravity constant (m/s²) |
| `BARO_ALPHA_HIGH_ACCEL` | `0.05` | Baro LPF under high accel |
| `BARO_ALPHA_LOW_ACCEL` | `0.005` | Baro LPF under low accel |
| `ACCEL_Z_THRESHOLD` | `0.2` | Threshold for baro filter switching |
| `RANGE_SWITCH_TO_LASER_THRESHOLD` | `0.25` | Switch to laser below (m) |
| `RANGE_SWITCH_TO_BARO_THRESHOLD` | `0.5` | Switch to baro above (m) |

Fusion6 init: `(1.0, 0.5, 1.0, 20.0, 0.1)` for all axes.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SENSOR_LINEAR_ACCEL` | 500 Hz | Predict (acceleration input) |
| `SENSOR_AIR_PRESSURE` | Event | Z altitude reference |
| `EXTERNAL_SENSOR_GPS` | Event | GPS position (outdoor) |
| `EXTERNAL_SENSOR_GPS_VELOC` | Event | GPS velocity (outdoor) |
| `EXTERNAL_SENSOR_OPTFLOW` | ~25 Hz | XY velocity reference |
| `SCHEDULER_10HZ` | 10 Hz | Check altitude source |
| `NOTIFY_LOG_CLASS` | Event | Activate/deactivate logging |
| `SCHEDULER_25HZ` | 25 Hz | Stream log data |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `POSITION_STATE_UPDATE` | `position_state_t` — pos + vel (`vector3d_t`) | 500 Hz |
| `SEND_LOG` | 6 floats: pos XYZ + vel XYZ | 25 Hz |

## Log Classes

| Log Class | ID | Data |
|-----------|----|------|
| `LOG_CLASS_POSITION` | 0x04 | Position & velocity (6 floats, 24 bytes) |
| `LOG_CLASS_POSITION_OPTFLOW` | 0x06 | Optical flow & altitude (6 floats, 24 bytes) |

Both are runtime-selectable — no recompilation needed.

## Tools

| Tool | Purpose |
|------|---------|
| `position_estimation_2d_and_z.py` | 2D XY + altitude chart |
| `position_estimation_chart_xy.py` | XY time-series |
| `position_estimation_chart_z.py` | Altitude time-series |
| `position_estimation_optflow.py` | Optical flow position view |
