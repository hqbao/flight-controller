# Local Storage Module

## Overview

Persistent parameter storage using flash memory. Stores and loads calibration data (gyro bias, calibration status) with CRC32 integrity validation.

## Storage Layout

```
┌──────────────────────────────────┬──────────┐
│         Data (128 bytes)         │ CRC32 (4)│
│  param[0] param[1] ... param[31] │          │
└──────────────────────────────────┴──────────┘
```

Each parameter is 4 bytes (float). Addressed by `param_id × 4`.

## Stored Parameters

| Parameter ID | Purpose |
|--------------|---------|
| `PARAM_ID_IMU1_GYRO_CALIBRATED` | Calibration valid flag |
| `PARAM_ID_IMU1_GYRO_BIAS_X` | Gyro X-axis bias |
| `PARAM_ID_IMU1_GYRO_BIAS_Y` | Gyro Y-axis bias |
| `PARAM_ID_IMU1_GYRO_BIAS_Z` | Gyro Z-axis bias |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `SHOULD_CLEAR_STORAGE` | 0 | Set to 1 for factory reset on boot |
| `DATA_STORAGE_SIZE` | 128 | Data area size (bytes) |
| `CHECKSUM_SIZE` | 4 | CRC32 checksum size |
| `LOCAL_STORAGE_SIZE` | 132 | Total: data + checksum |
| `PARAM_SIZE` | 4 | Bytes per parameter |

## Boot Sequence

1. Read flash block (132 bytes)
2. Validate CRC32 (polynomial 0xEDB88320)
3. If invalid → write defaults, recalculate CRC32

## Write Coalescing

Flash writes are **deferred and coalesced** in `loop_flush` (called from `LOOP`). On `LOCAL_STORAGE_SAVE` the module updates the RAM shadow and records a timestamp. The actual sector erase+write only runs **500 ms after the most recent save**, so a burst of rapid saves (e.g. `tuning_board.py` "Upload Defaults" sends 71 params at 50 ms apart) collapses into a single ~1.3 s flash write instead of dozens of back-to-back ones. This keeps `LOOP` responsive — important because FFT spectrum analysis and other diagnostic streams also run from `LOOP`.

Flash writes run in **thread mode** (from `LOOP`), never from a scheduler ISR — a full sector erase on STM32H743 takes ~1.3 s and would freeze all interrupts (PID, UART DMA, sensors) if run from ISR context.

## PubSub Interface

### Subscriptions
| Topic | Purpose |
|-------|---------|
| `LOCAL_STORAGE_SAVE` | Save parameter (id + value) to flash |
| `LOCAL_STORAGE_LOAD` | Load parameter from flash |

### Publications
| Topic | Data |
|-------|------|
| `LOCAL_STORAGE_RESULT` | `param_storage_t` — id + float value |
