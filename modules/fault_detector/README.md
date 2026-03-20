# Fault Detector Module

Monitors all sensors for health using a sliding window. Publishes a `sensor_health_t` bitmask on `SENSOR_HEALTH_UPDATE` at 1 Hz.

## Detection Method

All sensors use the same uniform design:

1. **Callbacks** store the latest value when data arrives
2. **25 Hz loop** proactively pushes the latest value into a ring buffer every tick
3. **1 Hz loop** checks if all values in the window are identical (max − min < ε)

If a sensor disconnects, its stale value keeps getting pushed → the window fills with identical values → stuck detected → unhealthy. No separate "received" flags needed.

### Monitored Values

| Sensor | Window Type | Tracked Value |
|--------|-------------|---------------|
| Gyro | 3-axis float | gx, gy, gz (rad/s) |
| Accel | 3-axis float | ax, ay, az (m/s²) |
| Compass | 3-axis float | mx, my, mz (µT) |
| Barometer | 1-axis double | pressure (Pa) |
| Optflow down | 1-axis double | clarity (0–100+) |
| Optflow up | 1-axis double | clarity (0–100+) |
| Downward range | 1-axis double | range (mm) |
| GPS | 1-axis double | num_sv (satellite count) |

### Parameters

| Constant | Value | Description |
|----------|-------|-------------|
| `WINDOW_SIZE` | 8 | Samples in ring buffer |
| `STUCK_EPSILON_F` | 1e-6 | Float stuck threshold |
| `STUCK_EPSILON_D` | 1e-9 | Double stuck threshold |

## Topics

| Topic | Direction | Payload | Description |
|---|---|---|---|
| `SENSOR_HEALTH_UPDATE` | Publishes | `sensor_health_t` | Sensor health bitmask (1 Hz) |
| `SENSOR_HEALTH_REQUEST` | Subscribes | — | Re-publishes current health on demand |
| `SENSOR_IMU1_GYRO_UPDATE` | Subscribes | 3×float | Gyro data |
| `SENSOR_IMU1_ACCEL_UPDATE` | Subscribes | 3×float | Accel data |
| `SENSOR_COMPASS` | Subscribes | `vector3d_t` | Compass data |
| `SENSOR_AIR_PRESSURE` | Subscribes | double | Barometer data |
| `EXTERNAL_SENSOR_OPTFLOW` | Subscribes | `optflow_data_t` | Optflow data (down + up) |
| `EXTERNAL_SENSOR_GPS_QUALITY` | Subscribes | `gps_quality_t` | GPS quality data |
| `SCHEDULER_25HZ` | Subscribes | — | Push latest values into windows |
| `SCHEDULER_1HZ` | Subscribes | — | Evaluate and publish health |

## Health Struct

```c
typedef struct {
    uint8_t gyro;           // 1 = healthy
    uint8_t accel;
    uint8_t compass;
    uint8_t baro;
    uint8_t downward_range;
    uint8_t optflow_down;
    uint8_t optflow_up;
    uint8_t gps;
} sensor_health_t;
```

## Consumer: flight_state

`flight_state` subscribes to `SENSOR_HEALTH_UPDATE` and uses the bitmask for:
- **Arming gate** (100 Hz): blocks ARMED transition if calibration incomplete
