# Flight State Module

## Overview

Flight state machine. Manages transitions between flight states based on RC input, sensor status, and safety checks.

## State Machine

```
         ┌─── ARM button ────┐
         │                    ▼
    DISARMED ◄──────────── ARMED
         ▲                    │
         │              Stick combo:
         │              left-yaw + bottom-throttle
         │              + right-roll + bottom-pitch
         │                    │
         │                    ▼
         │                 READY (motors idle)
         │                    │
         │              rc_state != 1
         │                    │
         │                    ▼
         │               TESTING (RC motor passthrough)
         │                    │
         │              rc_state == 1
         │                    │
         │              Throttle > 5°
         │                    │
         │                    ▼
    Emergency           TAKING_OFF
    (tilt > 60°)              │
         ▲              Range > 100mm
         │                    │
         │                    ▼
         └─────────────── FLYING ◄─── cancel landing (rc_state != 2)
                              │                    ▲
                         RC landing                │
                              │                    │
                              ▼                    │
                          LANDING ─────────────────┘
                              │
                         Range < 10mm (500ms)
                              │
                              ▼
                          DISARMED ◄── FLYING (range < 10mm + throttle min)
```

## Safety Features

- **Emergency disarm**: roll or pitch > 60° → immediate DISARMED
- **Landing confirmation**: range < 10mm for 50 consecutive iterations (500ms at 100 Hz)
- **Missing module LED**: flash count indicates missing modules (see fault_handler)

## Arming Gate

DISARMED → ARMED requires ALL of:
- `g_gyro_calibrated` and `g_accel_calibrated` (compass calibration is currently commented out)
- `g_sensor_health.optflow_down` (downward optical flow healthy)
- `g_sensor_health.downward_range` (downward range finder healthy)

**Note:** The upward optical flow (`optflow_up`) is NOT required for arming — it is optional and only used for ceiling/obstacle avoidance during flight. Takeoff is allowed without it.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `DISARM_RANGE_WHEN_LANDING` | 10 | Range to confirm landed (mm) |
| `DISARM_TIME_WHEN_LANDING` | 50 | Iterations at 100 Hz (~500ms) |
| `DISARM_IF_EXCEEDED_ANGLE_RANGE` | 60 | Emergency disarm tilt (degrees) |
| `ALLOWED_LANDING_RANGE` | 500 | Min range to allow landing (mm) |
| `TOOK_OFF_RANGE` | 100 | Range to confirm takeoff (mm) |
| `TAKEOFF_THROTTLE` | 5 | Throttle threshold for takeoff (degrees) |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `CALIBRATION_GYRO_STATUS` | Event | Gyro calibration result |
| `CALIBRATION_ACCEL_STATUS` | Event | Accel calibration result |
| `CALIBRATION_MAG_STATUS` | Event | Mag calibration result |
| `RC_STATE_UPDATE` | Event | Arm/mode commands |
| `RC_MOVE_IN_UPDATE` | Event | Stick positions |
| `SENSOR_HEALTH_UPDATE` | Event | Sensor health bitmask (from fault_detector) |
| `EXTERNAL_SENSOR_OPTFLOW` | Event | Optflow data (range + health check) |
| `ANGULAR_STATE_UPDATE` | Event | Tilt monitoring (emergency disarm) |
| `SCHEDULER_100HZ` | 100 Hz | State machine loop |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `FLIGHT_STATE_UPDATE` | `state_t` (uint8_t) | On state change |
