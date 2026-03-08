# RC Receiver Module

## Overview

Processes RC (Remote Control) commands received via UART DB protocol. Decodes stick positions and arm/mode state, then publishes them for control modules.

## Data Flow

```
RC Transmitter → UART (DB protocol)
    │
    ▼  DB_MESSAGE_UPDATE (CMD=0x02, SubCMD=0x00)
  Decode: 4× int32 (roll, pitch, yaw, alt) + state + mode
    │
    ▼  Convert pulse width → degrees (×90/490)
    │
    ├─► RC_STATE_UPDATE (state + mode)
    └─► RC_MOVE_IN_UPDATE (roll, pitch, yaw, alt as doubles)
```

## Wire Format

| Field | Type | Description |
|-------|------|-------------|
| roll | int32 | Pulse width |
| pitch | int32 | Pulse width |
| yaw | int32 | Pulse width |
| alt | int32 | Pulse width |
| state | uint8 | 0=disarmed, 1=armed |
| mode | uint8 | Flight mode (0, 1, 2) |

Conversion: `degrees = pulse_width × (90.0 / 490)`

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `PULSE_WIDTH_TO_DEGREE` | 90.0/490 | Pulse → degree conversion |
| `RC_CMD_ID` | 0x02 | DB protocol command ID |
| `RC_SUB_CMD_MOVE` | 0x00 | Move sub-command |
| `RC_DATA_SIZE` | 18 | Payload size |

## PubSub Interface

### Subscriptions
| Topic | Purpose |
|-------|---------|
| `DB_MESSAGE_UPDATE` | Receive DB-framed RC commands |
| `SCHEDULER_25HZ` | Process and publish at 25 Hz |

### Publications
| Topic | Data |
|-------|------|
| `RC_STATE_UPDATE` | `rc_state_ctl_t` — state + mode |
| `RC_MOVE_IN_UPDATE` | `rc_att_ctl_t` — roll, pitch, yaw, alt (doubles) |
