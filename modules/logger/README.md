# Logger Module

## Overview

UART telemetry framing module. Receives log data from other modules via `SEND_LOG`, wraps it in DB-framed binary packets, and transmits over UART. Also receives commands from Python tools to activate log classes at runtime — **no firmware recompilation needed**.

## Data Flow

```
Python tool (USB)                    Module (e.g. attitude_estimation)
    │                                        │
    ▼  DB_CMD_LOG_CLASS (0x03)              ▼  SEND_LOG
  UART → DMA → DB parser              Data payload (floats/int16s)
    │                                        │
    ▼                                        ▼
  on_db_message()                     send_log()
    │                                        │
    ▼                                        ▼
  NOTIFY_LOG_CLASS ──broadcast──►     Frame: ['d']['b'][ID][Class][Len][Payload][Checksum]
  (all modules listen)                       │
                                             ▼
                                        UART_PORT1 → USB → Python tool
```

## Wire Protocol

```
['d']['b'][ID=0x00][Class][Length_LE 2B][Payload][Checksum_LE 2B]
```

- Header: 6 bytes
- Checksum: 16-bit sum of bytes [2 .. end of payload]
- UART: `UART_PORT1` at 9600 baud (960 bytes/sec)

## Log Class System

Only one log class is active at a time. Python tools send `DB_CMD_LOG_CLASS` (0x03) to select:

| Log Class | ID | Module |
|-----------|----|--------|
| `LOG_CLASS_NONE` | 0x00 | Stop all logging |
| `LOG_CLASS_IMU_ACCEL` | 0x01 | imu — 3 accel floats |
| `LOG_CLASS_COMPASS` | 0x02 | compass — 3 mag floats |
| `LOG_CLASS_ATTITUDE` | 0x03 | attitude_estimation — 9 floats |
| `LOG_CLASS_POSITION` | 0x04 | position_estimation — 6 floats (pos + vel) |
| `LOG_CLASS_IMU_GYRO` | 0x05 | fft — 50× int16 batch |
| `LOG_CLASS_POSITION_OPTFLOW` | 0x06 | position_estimation — 6 floats (optflow + alt) |
| `LOG_CLASS_ATTITUDE_MAG` | 0x07 | attitude_estimation — 9 floats (mag debug) |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `LOGGER_MAX_PAYLOAD` | 120 | Maximum payload bytes per frame |
| `LOGGER_OUTPUT_SIZE` | 128 | Output buffer size |
| `LOGGER_HEADER_SIZE` | 6 | Frame header size |
| `LOGGER_CHECKSUM_SIZE` | 2 | Checksum size |

## Bandwidth Budget

At 9600 baud = 960 bytes/sec:
- 25 Hz frames → max 38 bytes payload each
- 10 Hz frames → max 96 bytes payload each
- 5 Hz frames → max 192 bytes payload each (limited by `LOGGER_MAX_PAYLOAD`)

## PubSub Interface

### Subscriptions
| Topic | Purpose |
|-------|---------|
| `DB_MESSAGE_UPDATE` | Receive commands from Python tools |
| `SEND_LOG` | Receive data payloads to frame and transmit |

### Publications
| Topic | Data |
|-------|------|
| `NOTIFY_LOG_CLASS` | `uint8_t` — active log class ID (broadcast) |
