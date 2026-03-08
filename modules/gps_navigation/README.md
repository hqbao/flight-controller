# GPS Navigation Module

## Overview

GPS waypoint navigation for outdoor flight. Converts GPS lat/lon to local NED frame, tracks waypoints sequentially, and publishes position targets for the position controller.

## Data Flow

```
EXTERNAL_SENSOR_GPS (lat, lon, alt)
    │
    ▼  Convert to local NED (flat-earth approximation)
  local_x = R × (lat_rad − home_lat)    (North)
  local_y = R × (lon_rad − home_lon) × cos(home_lat)  (East)
    │
    ▼  SCHEDULER_10HZ: compute distance to next waypoint
    │
    └─► POSITION_TARGET_UPDATE (target in local NED)
```

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `WAYPOINT_THRESHOLD` | 5.0 | Waypoint reached distance (meters) |
| `MAX_WAYPOINTS` | 10 | Maximum stored waypoints |
| Earth radius | 6,371,000 | Flat-earth approximation (m) |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `EXTERNAL_SENSOR_GPS` | Event | Current GPS position |
| `EXTERNAL_SENSOR_GPS_QUALITY` | Event | Fix quality monitoring |
| `SCHEDULER_10HZ` | 10 Hz | Navigation update loop |

### Publications
| Topic | Data |
|-------|------|
| `POSITION_TARGET_UPDATE` | `vector3d_t` — target in local NED (meters) |
