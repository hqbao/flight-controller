# Flight Telemetry Module

## Overview

Centralizes key flight data into a single 66-byte frame sent via `SEND_LOG` at 25 Hz. Designed for real-time dashboard monitoring over the 38400 baud UART link.

## Bandwidth

At 38400 baud (3840 bytes/sec): 66 payload + 8 overhead = 74 bytes/frame × 25 Hz = **1850 bytes/sec** (~48% utilization).

## Frame Layout

| Offset | Size | Field | Type | Unit |
|--------|------|-------|------|------|
| 0 | 12 | Attitude: roll, pitch, yaw | 3 × float | degrees |
| 12 | 12 | Position: x, y, z | 3 × float | meters |
| 24 | 12 | Velocity: vx, vy, vz | 3 × float | m/s |
| 36 | 16 | Motors: m1–m8 | 8 × int16_t | speed units |
| 52 | 12 | PID outputs: roll, pitch, yaw | 3 × float | — |
| 64 | 1 | Flight state | uint8_t | enum |
| 65 | 1 | Sensor health bitmask | uint8_t | bitmask |

### Health Bitmask

| Bit | Sensor |
|-----|--------|
| 0 | Gyro |
| 1 | Accel |
| 2 | Compass |
| 3 | Baro |
| 4 | Downward range |
| 5 | Optical flow (down) |
| 6 | Optical flow (up) |
| 7 | GPS |

## Logging

Activated by `NOTIFY_LOG_CLASS` with `LOG_CLASS_FLIGHT_TELEMETRY` (0x12).

Visualize with:
```bash
python3 tools/flight_telemetry_view.py            # Quadcopter
python3 tools/flight_telemetry_bicopter_view.py    # Bicopter
```

### Dashboard Layout (HUD Style)

Both viewers use a full-screen 3D aircraft model as the main background with semi-transparent overlay panels. The quadcopter version shows 4 arms with L1/L2 prop discs; the bicopter version shows a horizontal beam with tilting nacelles and servo angles.

- **Left data panel** — Vertical strip showing numeric readouts in sections: ATTITUDE (roll/pitch/yaw in degrees), POSITION (x/y/z in meters), VELOCITY (vx/vy/vz in m/s), MOTORS (FL/FR/BR/BL with Layer 1 and Layer 2 speeds), PID OUT (roll/pitch/yaw)
- **3D quadcopter** — Center background, NED-rotated wireframe with arm lines, nose indicator (red), prop discs (L1 blue / L2 orange), ground reference ring with North arrow, motor labels (M1–M4)
- **Position XY mini-map** — Top-right overlay with trail history, auto-scaling, and mouse scroll zoom (scroll up = zoom in, scroll down = zoom out)
- **Velocity chart** — Right-side overlay, time-series plot of Vx/Vy/Vz
- **Altitude chart** — Right-side overlay below velocity, time-series plot of altitude
- **Status bar** — Bottom strip with Start/Stop Log toggle button, flight state, sensor health indicator dots, and FPS counter

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `ANGULAR_STATE_UPDATE` | 1 kHz | Attitude (roll, pitch, yaw in degrees) |
| `POSITION_STATE_UPDATE` | 500 Hz | Position + velocity |
| `SPEED_CONTROL_UPDATE` | 1 kHz | Motor speeds (8 × int) |
| `MIX_CONTROL_UPDATE` | 1 kHz | PID outputs (roll, pitch, yaw) |
| `FLIGHT_STATE_UPDATE` | Event | Flight state enum |
| `SENSOR_HEALTH_UPDATE` | 1 Hz | Sensor health bitmask |
| `NOTIFY_LOG_CLASS` | Event | Runtime log class activation |
| `SCHEDULER_25HZ` | 25 Hz | Pack and send telemetry frame |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `SEND_LOG` | 66 bytes (see frame layout) | 25 Hz (when logging active) |
