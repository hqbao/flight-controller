# State Detector Module

## Overview

Flight state machine. Manages transitions between flight states based on RC input, sensor status, and safety checks. Also drives the status LED indicator and triggers gyro calibration.

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
         │              Throttle > 5°
         │                    │
         │                    ▼
    Emergency           TAKING_OFF
    (tilt > 60°)              │
         ▲              Range > 100mm
         │                    │
         │                    ▼
         └─────────────── FLYING
                              │
                         RC landing
                              │
                              ▼
                          LANDING
                              │
                         Range < 10mm (500ms)
                              │
                              ▼
                          DISARMED
```

## Safety Features

- **Emergency disarm**: roll or pitch > 60° → immediate DISARMED
- **Landing confirmation**: range < 10mm for 50 consecutive iterations (500ms at 100 Hz)
- **Missing module LED**: flash count indicates missing modules (5=IMU, 4=baro, 3=range, 2=optflow, 1=GPS, 0=OK)

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
| `MODULE_INITIALIZED_UPDATE` | Event | Track module readiness |
| `SENSOR_IMU1_GYRO_CALIBRATION_UPDATE` | Event | Gyro calibration result |
| `RC_STATE_UPDATE` | Event | Arm/mode commands |
| `RC_MOVE_IN_UPDATE` | Event | Stick positions |
| `EXTERNAL_SENSOR_OPTFLOW` | Event | Sensor health |
| `SENSOR_AIR_PRESSURE` | Event | Sensor health |
| `ANGULAR_STATE_UPDATE` | Event | Tilt monitoring |
| `SCHEDULER_100HZ` | 100 Hz | State machine loop |
| `SCHEDULER_50HZ` | 50 Hz | LED status update |
| `SCHEDULER_1HZ` | 1 Hz | Calibration trigger |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `STATE_DETECTION_UPDATE` | `state_t` (uint8_t) | 100 Hz |
| `SENSOR_CHECK_GYRO_CALIBRATION` | Trigger — start gyro calibration | Once |
