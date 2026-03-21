# Mix Control Module

## Overview

Motor mixing for an 8-motor X-frame octocopter. Receives PID outputs and altitude from attitude control, applies per-motor sign mixing, clamps to speed limits, and publishes motor commands to speed control. Also handles state-based motor behavior (off, idle, testing).

## Data Flow

```
MIX_CONTROL_UPDATE            FLIGHT_STATE_UPDATE       RC_MOVE_IN_UPDATE
    │                              │                        │
    ▼                              ▼                        ▼
  PID outputs + altitude      Flight state              RC stick input
    │                              │                        │
    └──────────────────┬───────────┘────────────────────────┘
                       │
                 Motor Mixer (8-motor)
                       │
                 ► SPEED_CONTROL_UPDATE (int[8])
```

## Motor Mixing

8-motor X-configuration octocopter (CW numbering, top view, nose up):
```
  m1(FL,CW)    m2(FR,CCW)
       \  ^  /
        \ | /
         X-X
        /   \
       /     \
  m4(BL,CCW)   m3(BR,CW)
```

Layer 2 (coaxial, counter-rotating): m5(FL,CCW), m6(FR,CW), m7(BR,CCW), m8(BL,CW)

Each motor receives:
```
speed[i] = MIN_SPEED + altitude ± roll ± pitch ± yaw
```

Sign convention (PID error = state − target → positive error = positive output):
- **Roll**: positive output → decrease left (m1,m4), increase right (m2,m3)
- **Pitch**: positive output → decrease front (m1,m2), increase rear (m3,m4)
- **Yaw**: positive output → increase CCW (m2,m4), decrease CW (m1,m3)

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `MIN_SPEED` | 150 | Minimum motor command (brushless) |
| `MAX_SPEED` | 1800 | Maximum motor command (brushless) |
| `MOTOR_TYPE` | 1 | 1=Brushless, 2=Brushed |

## State Machine Behavior

| Flight State | Motor Output |
|--------------|-------------|
| DISARMED / ARMED | 0 (motors off) |
| READY | MIN_SPEED (idle) |
| TAKING_OFF / FLYING / LANDING | PID mixing active (driven by `MIX_CONTROL_UPDATE`) |
| TESTING | Direct RC passthrough per motor |

## Logging

Subscribes to `NOTIFY_LOG_CLASS`. When `LOG_CLASS_MIX_CONTROL` (0x11) is active, publishes 8 motor speeds as `float[8]` via `SEND_LOG` at 10 Hz.

Visualize with:
```bash
python3 tools/mix_control_test.py
```

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `MIX_CONTROL_UPDATE` | 500 Hz | PID outputs + altitude from attitude control |
| `SCHEDULER_500HZ` | 500 Hz | Drives non-flight states (off, idle, testing) |
| `FLIGHT_STATE_UPDATE` | Event | Flight state changes |
| `RC_MOVE_IN_UPDATE` | Event | RC stick inputs (testing mode) |
| `NOTIFY_LOG_CLASS` | Event | Runtime log class activation |
| `SCHEDULER_10HZ` | 10 Hz | Log motor speeds when active |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SPEED_CONTROL_UPDATE` | `int[8]` — motor speeds | 500 Hz |
| `SPEED_CONTROL_SETUP` | Trigger — initialize motor drivers | Once |
| `SEND_LOG` | `float[8]` — motor speeds for telemetry | 10 Hz (when logging active) |
