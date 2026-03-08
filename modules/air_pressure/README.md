# Air Pressure Module

## Overview

Reads the **DPS310** barometric pressure sensor and converts pressure to altitude. Automatically computes a zero-altitude baseline by averaging the first 100 readings after a brief warmup period.

## Data Flow

```
DPS310 (I2C)
    │
    ▼  LOOP (continuous)
  Read pressure → Convert to altitude
    │
    ▼  Subtract baseline offset
    │
    └─► SENSOR_AIR_PRESSURE (double, altitude in mm)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Sensor | DPS310 |
| Interface | I2C |
| Warmup | 100 ms delay, skip first 10 readings |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `ALT_SAMPLES` | 100 | Number of readings for baseline average |
| `WARMUP_DELAY_MS` | 100 | Initial sensor warmup delay |
| `WARMUP_COUNTER_INIT` | -10 | Skip first 10 readings |
| `ALTITUDE_SCALE_FACTOR` | 1000.0 | Output in mm |

## Auto-Calibration

On boot, the module:
1. Waits 100 ms for sensor warmup
2. Discards the first 10 readings
3. Averages the next 100 readings as zero-altitude reference
4. All subsequent readings are relative to this baseline

## PubSub Interface

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SENSOR_AIR_PRESSURE` | `double` — altitude in mm | Continuous |

## Log Class

None.
