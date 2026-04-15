# Mix Control Module

## Overview

Motor/servo mixing dispatcher. Selects the active mixer at compile time based on `AIRCRAFT_TYPE` and delegates to the appropriate implementation. Each mixer receives PID outputs and altitude from attitude control, applies per-output mixing, clamps to limits, and publishes output commands to speed control.

## Architecture

```
mix_control.c (dispatcher)
    │
    ├── AIRCRAFT_QUADCOPTER → quadcopter.c  (8 DShot motors)
    └── AIRCRAFT_BICOPTER   → bicopter.c   (2 DShot motors + 2 PWM servos)
```

### Aircraft Type Selection

Defined in `mix_control.h`. Defaults to quadcopter if not overridden.

**Option 1 — Edit the header** (simplest):

Change `AIRCRAFT_TYPE` in `modules/mix_control/mix_control.h`:
```c
#define AIRCRAFT_TYPE AIRCRAFT_BICOPTER
```

**Option 2 — build.sh** (no source edits):

Add `-DAIRCRAFT_TYPE=2` to the gcc flags in the mix_control `subdir.mk` section of `base/boards/h7v1/build.sh`. The `-D` flag overrides the `#ifndef` guard in the header.

**Option 3 — STM32CubeIDE** (GUI):

1. Right-click the project → **Properties**
2. **C/C++ Build → Settings → MCU GCC Compiler → Preprocessor**
3. Under **Defined symbols (-D)**, click **Add** (+)
4. Enter: `AIRCRAFT_TYPE=2`
5. Click **Apply and Close**
6. **Project → Build** (or Ctrl+B)

The `#ifndef AIRCRAFT_TYPE` guard in `mix_control.h` ensures the IDE-provided `-D` takes precedence over the default.

| Constant | Value | Description |
|----------|-------|-------------|
| `AIRCRAFT_QUADCOPTER` | 1 | X-frame quad/octocopter, 8 motors (DShot) |
| `AIRCRAFT_BICOPTER` | 2 | Dual tilt-rotor, 2 motors (DShot) + 2 servos (PWM) |

## Data Flow

```
MIX_CONTROL_UPDATE            FLIGHT_STATE_UPDATE       RC_MOVE_IN_UPDATE
    │                              │                        │
    ▼                              ▼                        ▼
  PID outputs + altitude      Flight state              RC stick input
    │                              │                        │
    └──────────────────┬───────────┘────────────────────────┘
                       │
                 Aircraft Mixer
                       │
           ┌───────────┴────────────┐
           ▼                        ▼
   SPEED_CONTROL_SETUP       SPEED_CONTROL_UPDATE
   (per-port protocol)           (int[8])
```

## Quadcopter Mixer (`quadcopter.c`)

8-motor X-configuration (CW numbering, top view, nose up):
```
  m1(FL,CW)    m2(FR,CCW)       m5(FL2,CCW)   m6(FR2,CW)
       \  ^  /                       \  ^  /
        \ | /                         \ | /
         X-X           or              X-X        (octocopter)
        /   \                         /   \
       /     \                       /     \
  m4(BL,CCW)   m3(BR,CW)       m8(BL2,CW)   m7(BR2,CCW)
```

Each motor: `speed = MIN_SPEED + altitude ± roll ± pitch ± yaw`

Port config: all 8 ports → DShot.

## Bicopter Mixer (`bicopter.c`)

Dual tilt-rotor (V-22 style):
```
  m1(L,CCW)              m2(R,CW)
  s1(L tilt)             s2(R tilt)
      |         ^         |
      |---------|---------|'
                |
```

- **Roll**: differential motor thrust (`m1 = base - roll`, `m2 = base + roll`)
- **Pitch**: collective servo tilt — opposite PWM signs produce same physical tilt due to mirrored S2 (`s1 = center + pitch`, `s2 = center - pitch`)
- **Yaw**: differential servo tilt — same PWM sign, but mirrored S2 makes them tilt opposite (`s1 -= yaw`, `s2 -= yaw`)

Output array: `[0]=left motor, [1]=right motor, [2]=left servo, [3]=right servo, [4-7]=0`

Port config: ports 0–1 → DShot, ports 2–3 → PWM, ports 4–7 → disabled.

## Common Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `MIN_SPEED` | 150 | Minimum motor command (brushless) |
| `MAX_SPEED` | 1800 | Maximum motor command (brushless) |
| `MOTOR_TYPE` | 1 | 1=Brushless, 2=Brushed |
| `SERVO_MIN` | 1000 | Servo PWM minimum (µs, bicopter only) |
| `SERVO_MAX` | 2000 | Servo PWM maximum (µs, bicopter only) |
| `SERVO_CENTER` | 1500 | Servo PWM neutral (µs, bicopter only) |

## State Machine Behavior

| Flight State | Quadcopter Output | Bicopter Output |
|--------------|-------------------|------------------|
| DISARMED / ARMED | All motors 0 | Motors 0, servos at center |
| READY | All motors MIN_SPEED | Motors MIN_SPEED, servos at center |
| TAKING_OFF / FLYING / LANDING | PID mixing active | PID mixing active |
| TESTING | RC passthrough per motor | RC: yaw→m1, alt→m2, roll→s1, pitch→s2 |

## Logging

Subscribes to `NOTIFY_LOG_CLASS`. When `LOG_CLASS_MIX_CONTROL` (0x11) is active, publishes 8 output values as `float[8]` via `SEND_LOG` at 10 Hz.

Visualize with:
```bash
python3 tools/mix_control_quadcopter_test.py   # quadcopter: 8-motor diagram + bar chart
python3 tools/mix_control_bicopter_test.py     # bicopter: top-view tilt-rotor + bars
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
| `SCHEDULER_10HZ` | 10 Hz | Log output values when active |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SPEED_CONTROL_SETUP` | `speed_control_config_t` — per-port protocol | Once |
| `SPEED_CONTROL_UPDATE` | `int[8]` — motor/servo output values | 500 Hz |
| `SEND_LOG` | `float[8]` — output values for telemetry | 10 Hz (when logging active) |
