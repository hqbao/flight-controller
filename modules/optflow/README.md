# Optical Flow Module

## Overview

Receives binary optical flow packets from the external **flight-optflow** ESP32 module via UART. Validates checksums, converts scaled integer values back to radians, applies texture-quality gain, and publishes flow data for position estimation.

## Data Flow

```
flight-optflow ESP32 (any UART port, 38400 baud)
    │
    ▼  DMA ring buffer → auto-detect DB/UBX parser
  DB_MESSAGE_UPDATE (msg_id = 0x01)
    │
    ▼  Validate checksum → Unpack int32 → Convert to radians
    │
    └─► EXTERNAL_SENSOR_OPTFLOW (optflow_data_t)
```

## Wire Protocol

Message ID `0x01`, payload: 4× `int32_t` + 1 direction byte.

| Field | Type | Conversion |
|-------|------|------------|
| dx | int32 | ÷ 100,000 → radians |
| dy | int32 | ÷ 100,000 → radians |
| z (range) | int32 | Direct mm |
| clarity | int32 | ÷ 10 → quality metric |
| direction | uint8 | 0=downward, 1=upward |

Checksum: UBX-style Fletcher-8 over bytes 2 through end of payload.

## Texture Quality Gain

| Clarity | Gain | Effect |
|---------|------|--------|
| < 70 | 0.5 | Halve dx/dy — low-texture surface, reduce trust |
| ≥ 70 | 1.0 | Full dx/dy — good texture |

## PubSub Interface

### Subscriptions
| Topic | Purpose |
|-------|---------|
| `DB_MESSAGE_UPDATE` | Receive DB-framed messages from UART DMA |

### Publications
| Topic | Data |
|-------|------|
| `EXTERNAL_SENSOR_OPTFLOW` | `optflow_data_t` — dx, dy (rad), z (mm), clarity, dt_us, direction |

## Log Class

None.

## Related

- [flight-optflow/ARCHITECTURE.md](../../../flight-optflow/ARCHITECTURE.md) — optical flow sensor module
