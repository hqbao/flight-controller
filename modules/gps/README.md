# GPS Module

Parses **UBX NAV-PVT** frames from a u-blox GPS receiver (default target:
**ZED-F9P**) and publishes structured position, velocity, quality and
log data on the PubSub bus.

## Data Flow

```
ZED-F9P (UART1 / UART2 / USB, all 38400 baud, UBX-only)
    │
    ▼  STM32 USART2/3/4 DMA → UART_RAW_RECEIVED
    │     (USART1 carries telemetry to host PC and is also valid)
  dblink/db_reader  — auto-detects UBX (0xB5 0x62) on every port
    │
    ▼  UBX_MESSAGE_UPDATE  (class | id | len | payload | ck_a | ck_b)
modules/gps   — selects NAV-PVT (class 0x01, id 0x07) only
    │
    ├─► EXTERNAL_SENSOR_GPS         (gps_position_t)
    ├─► EXTERNAL_SENSOR_GPS_VELOC   (gps_velocity_t)
    ├─► EXTERNAL_SENSOR_GPS_QUALITY (gps_quality_t)
    └─► SEND_LOG  (gps_log_t, 48 B)   ← only when LOG_CLASS_GPS active
```

The GPS module is port-agnostic: it parses any UBX NAV-PVT frame regardless
of which STM32 USART it arrived on. ZED-F9P UART1 and UART2 are both wired
to publish the same data — connect either one to the FC.

## Hardware / Receiver Configuration

| Parameter      | Value |
|----------------|-------|
| Receiver       | u-blox ZED-F9P |
| Wire interface | UART, 8N1 |
| Baud rate      | 38400 (matches STM32 USART2/3/4) |
| Output         | UBX only — every NMEA + extra UBX message disabled |
| Active message | `UBX-NAV-PVT` (class 0x01, id 0x07, 92 B payload) |
| Update rate    | 10 Hz (CFG-RATE-MEAS = 100 ms; max safe with multi-GNSS) |

Apply this configuration with:

```bash
python3 tools/gps_config_f9p.py            # auto-detect serial port
python3 tools/gps_config_f9p.py --rate-ms 200   # drop to 5 Hz if needed
```

The script writes via UBX-CFG-VALSET to RAM + BBR + Flash, so the receiver
boots into the same configuration after a power cycle. Run it once per
receiver.

## Reliability Criteria

`gps_quality_t.reliable` is set to 1 only when **all** of:

- `flags & 0x01` (gnssFixOK bit set by the receiver)
- `fix_type == 3` (3-D fix)
- `num_sv >= 6`
- `h_acc < 5000` mm (horizontal accuracy < 5 m)

## PubSub Interface

### Subscriptions

| Topic                | Purpose |
|----------------------|---------|
| `UBX_MESSAGE_UPDATE` | Receive parsed UBX frames from `dblink/db_reader` |
| `NOTIFY_LOG_CLASS`   | Activate / deactivate `LOG_CLASS_GPS` streaming |

### Publications

| Topic                          | Payload          | Notes |
|--------------------------------|------------------|-------|
| `EXTERNAL_SENSOR_GPS`          | `gps_position_t` | lat / lon (deg × 1e7), alt MSL (mm) |
| `EXTERNAL_SENSOR_GPS_VELOC`    | `gps_velocity_t` | NED velocity (mm/s) |
| `EXTERNAL_SENSOR_GPS_QUALITY`  | `gps_quality_t`  | fix_type, num_sv, h_acc, v_acc, p_dop, flags, reliable |
| `SEND_LOG`                     | `gps_log_t` (48 B) | Only emitted while `LOG_CLASS_GPS` is active (10 Hz) |

## Log Class

`LOG_CLASS_GPS = 0x1E` — packed `gps_log_t`:

| Offset | Type    | Field           | Unit |
|--------|---------|-----------------|------|
| 0..3   | float   | `lat_deg`       | degrees |
| 4..7   | float   | `lon_deg`       | degrees |
| 8..11  | float   | `alt_msl_m`     | metres above MSL |
| 12..15 | float   | `vel_n_mps`     | m/s |
| 16..19 | float   | `vel_e_mps`     | m/s |
| 20..23 | float   | `vel_d_mps`     | m/s |
| 24..27 | float   | `g_speed_mps`   | 2-D ground speed (m/s) |
| 28..31 | float   | `head_mot_deg`  | heading of motion (deg) |
| 32..35 | float   | `h_acc_m`       | horizontal accuracy estimate (m) |
| 36..39 | float   | `v_acc_m`       | vertical accuracy estimate (m) |
| 40..41 | uint16  | `p_dop`         | position DOP × 100 |
| 42     | uint8   | `num_sv`        | satellites used in fix |
| 43     | uint8   | `fix_type`      | 0=no fix, 2=2D, 3=3D |
| 44     | uint8   | `flags`         | NAV-PVT flags byte |
| 45     | uint8   | `reliable`      | 0 / 1 |
| 46..47 | uint16  | `_pad`          | padding to 48 bytes |

## Tools

| Tool                       | Purpose |
|----------------------------|---------|
| `tools/gps_config_f9p.py`  | One-shot ZED-F9P configurator (UBX-CFG-VALSET, saves to BBR + Flash). Disables all NMEA + extra UBX, enables NAV-PVT @ 10 Hz on UART1/UART2/USB at 38400. |
| `tools/gps_view.py`        | Real-time dashboard: 2-D position map, altitude bar, NED + ground-speed line charts, sat-count history, accuracy / pDOP / heading panel. Sends `LOG_CLASS_GPS` over the dblink. |
