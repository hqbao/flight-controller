# GPS Module

## Overview

Parses incoming **UBX NAV-PVT** messages from a u-blox GPS receiver and publishes structured position, velocity, and quality data.

## Data Flow

```
u-blox ZED-F9P (UART 38400 baud)
    │
    ▼  DMA ring buffer → auto-detect DB/UBX parser
  UBX_MESSAGE_UPDATE
    │
    ▼  Validate: NAV class (0x01), PVT message (0x07)
    │
    ├─► EXTERNAL_SENSOR_GPS       (gps_position_t)
    ├─► EXTERNAL_SENSOR_GPS_VELOC (gps_velocity_t)
    └─► EXTERNAL_SENSOR_GPS_QUALITY (gps_quality_t)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Receiver | u-blox ZED-F9P |
| Interface | UART (38400 baud) |
| Protocol | UBX (NMEA disabled on receiver; parser also auto-detects DB) |
| Update rate | 10 Hz |
| Configuration tool | `python3 tools/gps_config_f9p.py` |

## Reliability Criteria

A GPS fix is considered `reliable` when all conditions are met:
- `fix_type == 3` (3D fix)
- `num_sv >= 6` (at least 6 satellites)
- `h_acc < 5000` mm (horizontal accuracy < 5 m)

## PubSub Interface

### Subscriptions
| Topic | Purpose |
|-------|---------|
| `UBX_MESSAGE_UPDATE` | Receive parsed UBX frames from UART DMA |

### Publications
| Topic | Data |
|-------|------|
| `EXTERNAL_SENSOR_GPS` | `gps_position_t` — lon, lat (deg×1e7), alt (mm) |
| `EXTERNAL_SENSOR_GPS_VELOC` | `gps_velocity_t` — velN, velE, velD (mm/s) |
| `EXTERNAL_SENSOR_GPS_QUALITY` | `gps_quality_t` — fix_type, num_sv, h_acc, reliable |

## Log Class

None.

## Tools

| Tool | Purpose |
|------|---------|
| `gps_config_f9p.py` | Configure ZED-F9P for 10 Hz UBX-only at 38400 baud |
| `gps_read_ubx.py` | Real-time GPS monitor and visualizer |
| `gps_sim_ubx.py` | GPS simulator (sends fake UBX NAV-PVT) |
