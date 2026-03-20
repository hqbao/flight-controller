# Speed Control Module

## Overview

Motor output driver. Translates motor speed commands from mix control into hardware signals (DShot or PWM) for up to 8 motors. Pure passthrough — no filtering or processing.

## Data Flow

```
SPEED_CONTROL_SETUP (once)     SPEED_CONTROL_UPDATE (1 kHz)
    │                               │
    ▼                               ▼
  Initialize DShot/PWM ports    Copy speed array → send to motors
```

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `SPEED_CONTROL_PROTOCOL` | `1` | 1=DShot, 2=PWM |
| `MAX_MOTORS` | `8` | Number of motor outputs |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SPEED_CONTROL_SETUP` | Once | Initialize motor driver ports |
| `SPEED_CONTROL_UPDATE` | 1 kHz | Apply motor speed commands |

### Publications

None.
