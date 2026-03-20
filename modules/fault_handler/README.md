# Fault Handler Module

## Overview

Safety module with two responsibilities:
1. **Fault response** — immediately disarms all motors on any `FAULT_DETECTION` event
2. **LED status indicator** — flashes LED at 50 Hz to indicate which sensor subsystem is unhealthy

## Fault Response

On `FAULT_DETECTION` event:
1. Increment fault counter
2. Toggle LED
3. Force `FLIGHT_STATE_UPDATE` → DISARMED (state = 0)
4. Print fault count to console

## LED Status Indicator

Runs at 50 Hz via `SCHEDULER_50HZ`. Checks `sensor_health_t` from fault_detector and flashes LED with priority-ordered patterns:

| Flash Count | Condition |
|-------------|-----------|
| 5 | IMU unhealthy (gyro or accel or compass) |
| 4 | Barometer unhealthy |
| 3 | Downward range unhealthy |
| 2 | Optical flow (down) unhealthy |
| 1 | GPS unhealthy |
| 0 (LED off) | All systems nominal |

Timing: N flashes in 500 ms (25 ticks at 50 Hz), then 500 ms pause. Higher-priority faults (more flashes) take precedence.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `FAULT_DETECTION` | Event | Fault event from any module |
| `SENSOR_HEALTH_UPDATE` | 1 Hz | Sensor health bitmask (from fault_detector) |
| `SCHEDULER_50HZ` | 50 Hz | LED flash pattern driver |

### Publications
| Topic | Data |
|-------|------|
| `FLIGHT_STATE_UPDATE` | `state_t` = DISARMED (0) |
