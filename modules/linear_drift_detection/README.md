# Linear Drift Detection Module

## Overview

Detects sustained linear drift (unidirectional movement) by analyzing optical flow data. Measures how long the drift direction persists on each axis. Useful for detecting wind drift or sensor bias.

## Algorithm

1. Accumulate optical flow `dx`/`dy` between 5 Hz ticks
2. At each tick: check if accumulated flow has the same sign as previous tick
3. Same direction → `drift_time += 0.2s`
4. Direction changed → `drift_time = 0.2s` (reset)
5. Publish drift duration per axis at 5 Hz

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `EXTERNAL_SENSOR_OPTFLOW` | ~25 Hz | Optical flow data |
| `SCHEDULER_5HZ` | 5 Hz | Drift analysis tick |

### Publications
| Topic | Data |
|-------|------|
| `LINEAR_DRIFT_DETECTION` | `vector3d_t` — x=drift_time_x sec, y=drift_time_y sec, z=0 |
