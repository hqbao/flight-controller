# Troubleshoot Module

Ad-hoc diagnostic-only module for one-off data collection. Designed to be
edited / repurposed as new questions arise. Always passive — never affects
the control path.

## Current Diagnostic: Accelerometer Clip Detector

Subscribes to `SENSOR_IMU1_ACCEL_RAW_UPDATE` (int16 LSB at 500 Hz, published
by the `imu` module pre-calibration). Tracks per-axis min, max, and clip-count
over a 1-second window.

A "clip" is any sample within ~1% of the INT16 rail (`|v| >= 32440`). When
the configured accel full-scale range (`AFS_*`) is too small, vibration or
maneuver peaks saturate the sensor, producing asymmetric data that the
fusion estimator rectifies into a DC linear-accel bias — which causes the
position estimator to drift on the affected axis.

### Usage

1. Activate the diagnostic from a Python tool by sending `DB_CMD_LOG_CLASS`
   with class id `LOG_CLASS_TROUBLESHOOT_ACCEL` (`0x1D`).
2. The module publishes `SEND_LOG` once per second with a 20-byte payload:

   | Offset | Bytes | Type | Field |
   |--------|-------|------|-------|
   | 0      | 6     | `int16[3]`  | min[X,Y,Z] |
   | 6      | 6     | `int16[3]`  | max[X,Y,Z] |
   | 12     | 6     | `uint16[3]` | clip_count[X,Y,Z] |
   | 18     | 2     | `uint16`    | sample_count in window |

3. The window resets after each publish.
4. Companion viewer: `flight-controller/tools/troubleshoot_accel_clip_view.py`.

### Verdict guide (at AFS_16G; 1 g = 2048 LSB, rail = ±32767 ≈ ±16 g)

| Condition | Verdict | Action |
|-----------|---------|--------|
| `clip_count > 0` on any axis | CLIPPING | Already at AFS_16G — investigate mechanical source (bent prop, loose mount, broken bearing). |
| `\|min\|` or `\|max\| > 30000` | NEAR RAIL | Within ~10% of ±16 g — reduce vibration source. |
| `\|min\|` and `\|max\| < 25000` | OK | Configured range has comfortable margin. |

## Files

| File | Purpose |
|------|---------|
| `troubleshoot.c` / `troubleshoot.h` | Module logic. Edit this file to add new diagnostics. |

## Subscriptions

| Topic | Rate | Purpose |
|-------|------|---------|
| `SENSOR_IMU1_ACCEL_RAW_UPDATE` | 500 Hz | Per-sample min/max/clip tracking |
| `NOTIFY_LOG_CLASS` | Event | Activate/deactivate when `LOG_CLASS_TROUBLESHOOT_ACCEL` is selected |
| `SCHEDULER_1HZ` | 1 Hz | Publish window summary, then reset window |

## Publications

| Topic | Data | Rate |
|-------|------|------|
| `SEND_LOG` | 20-byte clip diagnostic payload | 1 Hz (only when active) |
