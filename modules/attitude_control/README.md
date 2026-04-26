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

All gains are tunable at runtime via `tuning_board.py` (Attitude PID category).

| Parameter | Roll | Pitch | Yaw |
|-----------|------|-------|-----|
| P | 4.0 | 4.0 | 10.0 |
| I | 1.0 | 1.0 | 1.0 |
| D | 2.0 | 2.0 | 5.0 |
| I limit | 5.0 | 5.0 | 5.0 |
| P-Term Limit | 1000000 | 1000000 | 1000000 |
| Output Limit | 1000000 | 1000000 | 1000000 |

| Parameter | Default | Description |
|-----------|---------|-------------|
| Smooth Input | 1.0 | Input low-pass weight (1.0 = no smoothing) |
| Smooth P Term | 1.0 | P-value low-pass weight |
| Smooth Output | 1.0 | Output low-pass weight |
| Gain Ramp Time | 1.0 | I-gain accumulation rate |

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

## Yaw Wrap-Around Handling

`g_angular_state.yaw` is in degrees wrapped to ±180° (output of `quat_to_euler`).
A naïve PID would see a ~360° error whenever yaw crosses the ±180° boundary
(e.g. yaw flips from +179° to −179°), commanding a full 360° spin.

Two guards run inside `pid_loop()` before the yaw PID update:

1. **Setpoint shortest-path normalization** — if `setpoint − yaw > 180°`,
   shift setpoint by −360° (and vice versa). The PID error is always the
   short way around.
2. **Derivative spike suppression** — if the measured yaw differs from the
   PID's previous internal value by more than 180° (a wrap just happened),
   snap `pid.value` and `pid.value_prev` to the new measurement so the
   derivative term doesn't see a fake 360°/dt spike on the wrap tick.

No unwrapped accumulator or extra state is kept — the yaw stays in its
native ±180° space and the setpoint is allowed to drift outside that range
(e.g. to −181° == +179°) only as needed to track the shortest path.

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
