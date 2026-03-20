# Air Pressure Module

## Overview

Reads the **DPS310** barometric pressure sensor over I2C and converts pressure to altitude. The sensor runs in continuous (background) measurement mode — the host simply reads the latest result registers. Automatically computes a zero-altitude baseline by averaging the first 100 readings after a brief warmup period.

If the DPS310 is not detected at boot (I2C error or sensor absent), `air_pressure_setup()` returns early and the rest of the system runs normally.

## Data Flow

```
DPS310 (I2C4, continuous mode: 32 Hz pressure, 4 Hz temperature)
    │
    ▼  LOOP (~25 Hz, main-thread context)
  Read pressure+temp registers (6-byte burst) → Compensate → Altitude
    │
    ▼  Subtract baseline offset, scale to mm
    │
    └─► SENSOR_AIR_PRESSURE (double, altitude in mm)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Sensor | DPS310 barometric pressure |
| Interface | I2C (I2C_PORT3 → I2C4) |
| I2C Address | 0xEC |
| Mode | Continuous pressure + temperature |
| Pressure rate | 32 measurements/sec, 4× oversampling |
| Temperature rate | 4 measurements/sec, single oversampling |
| Warmup | 100 ms delay, skip first 10 readings |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `ALT_SAMPLES` | 100 | Number of readings for baseline average |
| `WARMUP_DELAY_MS` | 100 | Initial sensor warmup delay (ms) |
| `WARMUP_COUNTER_INIT` | -10 | Skip first 10 readings (sensor settling) |
| `ALTITUDE_SCALE_FACTOR` | 1000.0 | Output in mm |
| `LOOP_INTERVAL_MS` | 40 | Rate limit: ~25 Hz from main loop |

## Auto-Calibration

On boot, the module:
1. Waits 100 ms for sensor warmup
2. Probes the DPS310 (product ID check, coefficient read, temperature correction)
3. Starts DPS310 continuous measurement mode
4. Discards the first 10 readings
5. Averages the next 100 readings as zero-altitude reference
6. All subsequent readings are relative to this baseline

## ISR Safety — Why LOOP Instead of SCHEDULER

The loop runs from `LOOP` (main-thread context), **not** from `SCHEDULER_25HZ` (TIM8 ISR):

- `dps310_read_continuous()` calls `HAL_I2C_Mem_Read()` (polling mode)
- Polling timeout relies on `HAL_GetTick()` which is driven by SysTick
- TIM8 ISR has priority 0; SysTick has priority 15 — SysTick cannot preempt TIM8
- Any I2C bus glitch inside the ISR → `HAL_GetTick()` frozen → timeout never fires → permanent hang
- In main-thread context, SysTick runs normally → timeout works → I2C errors recover gracefully

## PubSub Interface

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SENSOR_AIR_PRESSURE` | `double` — altitude in mm relative to boot baseline | ~25 Hz |

## Log Class

None.
