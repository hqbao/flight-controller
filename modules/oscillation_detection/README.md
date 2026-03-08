# Oscillation Detection Module

## Overview

Detects high-frequency oscillations in the airframe by analyzing gyroscope zero-crossing frequency. Useful for detecting PID-induced vibrations or structural resonances during flight.

## Algorithm

1. Accumulate gyro data at 1 kHz
2. Downsample to 100 Hz for zero-crossing analysis
3. Count consecutive samples with the same sign
4. Estimate frequency: `freq = (100 × 0.5) / count`
5. Publish frequency estimates at 5 Hz

Independent X and Y axis tracking.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SENSOR_IMU1_GYRO_UPDATE` | 1 kHz | Raw gyro data |
| `SCHEDULER_100HZ` | 100 Hz | Zero-crossing analysis |
| `SCHEDULER_5HZ` | 5 Hz | Publish frequency estimate |

### Publications
| Topic | Data |
|-------|------|
| `OSCILLATION_FREQ_DETECTED` | `vector3d_t` — x=freq_x Hz, y=freq_y Hz, z=0 |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `SHAKE_DETECTION_FREQ` | 100.0 | Sampling frequency for analysis (Hz) |
