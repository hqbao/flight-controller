# Position Control Module

## Overview

Outer-loop position hold controller. Converts position/velocity error into roll/pitch/yaw angle targets for the inner-loop attitude controller. Supports RC stick override and two flight modes.

## Data Flow

```
POSITION_STATE_UPDATE (500 Hz)
    │
    ▼
  Position error = target − current
    │
    ▼
  P-controller + velocity damping
    │
    ▼
  Output LPF → clamp to ±30°
    │
    ├─► ANGULAR_TARGET_UPDATE (roll, pitch, yaw)
    └─► ALTITUDE_CONTROL_UPDATE (altitude + takeoff speed)
```

### Position→Angle Mapping
- **X error** (forward of target) → positive pitch (nose up) to fly back → **no negation**
- **Y error** (right of target) → negative roll (right wing up) to fly left → **negated**
- **Z error** (above target) → less throttle to descend → **negated**

## Flight Modes

| Mode | RC Button | Behavior |
|------|-----------|----------|
| Mode 0 | X (North) | Stabilize — direct angle control, altitude hold |
| Mode 2 | B (East) | Position hold — GPS/optflow position lock |

In Mode 0/1: RC stick offsets the hold position.
In Mode 2: RC provides rate commands (stick → velocity).

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `POS_CTL_XY_P` | 100.0 | XY position P-gain |
| `POS_CTL_Z_P` | 2000.0 | Z position P-gain |
| `POS_CTL_VELOC_XY_SCALE` | 50.0 | XY velocity damping |
| `POS_CTL_VELOC_Z_SCALE` | 2000.0 | Z velocity damping |
| `POS_CTL_ANGLE_LIMIT` | 30.0 | Max tilt output (degrees) |
| `RC_DEADBAND` | 0.1 | RC stick deadband |
| `RC_XY_SCALE` | 0.01 | RC stick → XY position target scale |
| `RC_Z_SCALE` | 0.04 | RC stick → Z position target scale |
| `RC_YAW_SCALE` | -0.5 | RC stick → yaw velocity scale |
| `LANDING_RANGE_THRESHOLD` | 2000.0 | Landing auto-speed range (mm) |
| `MIN_LANDING_SPEED` | 50 | Minimum landing descent speed |

## Automated Landing

When landing state is active:
- Adaptive descent speed based on range finder rate
- Below `LANDING_RANGE_THRESHOLD` → slower descent
- Minimum speed clamped to `MIN_LANDING_SPEED`

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `POSITION_STATE_UPDATE` | 500 Hz | Current position/velocity |
| `POSITION_TARGET_UPDATE` | Event | Navigation waypoint target |
| `FLIGHT_STATE_UPDATE` | Event | Flight state changes |
| `RC_STATE_UPDATE` | Event | Arm/mode from RC |
| `RC_MOVE_IN_UPDATE` | Event | RC stick inputs |
| `EXTERNAL_SENSOR_OPTFLOW` | Event | Optflow for sensor health check |
| `SCHEDULER_100HZ` | 100 Hz | Manual control update |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `ANGULAR_TARGET_UPDATE` | `double[3]` — roll, pitch, yaw targets | Up to 100 Hz |
| `ALTITUDE_CONTROL_UPDATE` | `double[2]` — altitude + takeoff speed | Up to 100 Hz |
