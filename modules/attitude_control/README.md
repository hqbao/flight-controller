# Attitude Control Module

## Overview

PID-based inner-loop attitude stabilization. Computes roll/pitch/yaw corrections from three independent PID controllers at 1 kHz and publishes them to the mix control module.

## Data Flow

```
ANGULAR_STATE_UPDATE (1 kHz)    ANGULAR_TARGET_UPDATE    ALTITUDE_CONTROL_UPDATE
    │                                   │                        │
    ▼                                   ▼                        ▼
  Current angles              Position controller target   Altitude + throttle
    │                                   │                        │
    └───────────────────────┬───────────┘────────────────────────┘
                            │
                      PID Controllers
                     (Roll / Pitch / Yaw)
                            │
                      ► MIX_CONTROL_UPDATE (mix_control_input_t)
```

## PID Configuration

| Parameter | Roll | Pitch | Yaw |
|-----------|------|-------|-----|
| P | 6.0 | 6.0 | 20.0 |
| I | 1.0 | 1.0 | 1.0 |
| D | 3.0 | 3.0 | 10.0 |
| I limit | 5.0 | 5.0 | 5.0 |

| Constant | Value | Description |
|----------|-------|-------------|
| `ATT_CTL_FREQ` | 500 | Control loop frequency (Hz), defined in `macro.h` |
| `ATT_CTL_SCHEDULER` | `SCHEDULER_500HZ` | Scheduler alias, defined in `macro.h` |

## State Machine Behavior

| Flight State | Behavior |
|--------------|----------|
| DISARMED / ARMED / READY / TESTING | No PID output |
| TAKING_OFF | PID active, yaw setpoint locked to current heading |
| FLYING / LANDING | PID active |

PID controllers are reset on ARM/READY/TAKING_OFF transitions.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `ANGULAR_STATE_UPDATE` | 1 kHz | Current attitude (roll, pitch, yaw) |
| `SCHEDULER_500HZ` | 500 Hz | Control loop tick |
| `FLIGHT_STATE_UPDATE` | Event | Flight state changes |
| `ANGULAR_TARGET_UPDATE` | Event | Target angles from position control |
| `ALTITUDE_CONTROL_UPDATE` | Event | Altitude + throttle target |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `MIX_CONTROL_UPDATE` | `mix_control_input_t` — PID outputs + altitude | 500 Hz (flight states only) |
