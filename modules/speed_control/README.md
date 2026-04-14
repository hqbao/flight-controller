# Speed Control Module

## Overview

Motor/servo output driver. Translates output commands from mix control into hardware signals for up to 8 ports. Each port is independently configured as DShot, PWM, or disabled based on the aircraft type. Pure passthrough — no filtering or processing.

## Data Flow

```
SPEED_CONTROL_SETUP (once)              SPEED_CONTROL_UPDATE (500 Hz)
    │                                        │
    ▼                                        ▼
  Receive speed_control_config_t         Copy output array → per-port dispatch
  Configure each port as:                   │
    PORT_DSHOT → platform_dshot_init()       ├─ PORT_DSHOT → platform_dshot_send()
    PORT_PWM   → platform_pwm_init()        ├─ PORT_PWM   → platform_pwm_send()
    PORT_DISABLED → skip                    └─ PORT_DISABLED → skip
```

## Per-Port Protocol

The mixer publishes a `speed_control_config_t` (defined in `messages.h`) via `SPEED_CONTROL_SETUP`, specifying the protocol for each of the 8 ports:

| Protocol | Value | Hardware | Typical Use |
|----------|-------|----------|-------------|
| `PORT_DISABLED` | 0 | None | Unused port |
| `PORT_DSHOT` | 1 | DShot600 via DMA | Brushless motor ESC |
| `PORT_PWM` | 2 | PWM (direct CCR) | Servo |

Example configurations:
- **Quadcopter**: all 8 ports → `PORT_DSHOT`
- **Bicopter**: ports 0–1 → `PORT_DSHOT` (motors), ports 2–3 → `PORT_PWM` (servos), ports 4–7 → `PORT_DISABLED`

## PubSub Interface

### Subscriptions
| Topic | Rate | Payload | Purpose |
|-------|------|---------|--------|
| `SPEED_CONTROL_SETUP` | Once | `speed_control_config_t` | Configure per-port protocol and initialize hardware |
| `SPEED_CONTROL_UPDATE` | 500 Hz | `int[8]` | Apply output commands to configured ports |

### Publications

None.
