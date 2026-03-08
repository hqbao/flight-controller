# Attitude Control Module

## Overview

PID-based inner-loop attitude stabilization. Computes motor speed commands for 8 motors from roll/pitch/yaw error using three independent PID controllers at 1 kHz.

## Data Flow

```
ANGULAR_STATE_UPDATE (1 kHz)    RC_MOVE_IN_UPDATE    ANGULAR_TARGET_UPDATE
    │                               │                        │
    ▼                               ▼                        ▼
  Current angles              RC stick input          Position controller target
    │                               │                        │
    └──────────────────────┬────────┘────────────────────────┘
                           │
                     PID Controllers
                    (Roll / Pitch / Yaw)
                           │
                     Motor Mixer (8-motor)
                           │
                     ► SPEED_CONTROL_UPDATE (int[8])
```

## Motor Mixing

8-motor X-configuration octocopter. Each motor receives:
```
speed[i] = base_throttle ± roll_output ± pitch_output ± yaw_output
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
| `MIN_SPEED` | 150 | Minimum motor command (brushless) |
| `MAX_SPEED` | 1800 | Maximum motor command (brushless) |
| `MOTOR_TYPE` | 1 | 1=Brushless, 2=Brushed |
| `ATT_CTL_FREQ` | 1000 | Control loop frequency (Hz) |

## State Machine Behavior

| Flight State | Motor Output |
|--------------|-------------|
| DISARMED / ARMED | 0 (motors off) |
| READY | MIN_SPEED (idle) |
| TAKING_OFF / FLYING / LANDING | PID control active |
| TESTING | Direct RC passthrough |

PID controllers are reset on ARM/READY/TAKING_OFF transitions.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `ANGULAR_STATE_UPDATE` | 1 kHz | Current attitude |
| `SCHEDULER_1KHZ` | 1 kHz | Control loop tick |
| `STATE_DETECTION_UPDATE` | Event | Flight state changes |
| `RC_MOVE_IN_UPDATE` | Event | RC stick inputs |
| `ANGULAR_TARGET_UPDATE` | Event | Target angles from position control |
| `ALTITUDE_CONTROL_UPDATE` | Event | Altitude/throttle target |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SPEED_CONTROL_UPDATE` | `int[8]` — motor speeds | 1 kHz |
| `SPEED_CONTROL_SETUP` | Trigger — initialize motor drivers | Once |
