# Flight Controller

A modular embedded flight control system for multi-rotor aircraft, written in **C** for STM32H7 and ESP32 microcontrollers. Integrates multiple sensors, performs real-time attitude and position estimation, and executes stabilization control loops.

## Overview

The Flight Controller is the core autopilot that:
- Reads and calibrates sensor data (IMU, magnetometer, barometer, GPS, optical flow)
- Performs sensor fusion to estimate aircraft attitude and position
- Executes PID-based stabilization and control loops
- Drives motor ESCs via PWM
- Logs telemetry over UART for real-time visualization
- Supports GPS-based outdoor and optical-flow-based indoor navigation

## Hardware Support

| Component | Driver | Interface |
| :--- | :--- | :--- |
| **IMU** | ICM-42688P | I2C / SPI |
| **Magnetometer** | BMM350 | I2C |
| **Barometer** | DPS310 | I2C |
| **GPS** | u-blox ZED-F9P (UBX) | UART |
| **Optical Flow** | External module (flight-optflow) | UART |
| **MCU** | STM32H7 / ESP32-S3 / ESP32-P4 / macOS (SITL) | — |

## Demo Videos

[![Video 1](https://img.youtube.com/vi/rbQvsm3T5Mc/0.jpg)](https://www.youtube.com/shorts/rbQvsm3T5Mc)
[![Video 2](https://img.youtube.com/vi/Ewxq5O-b1gY/0.jpg)](https://www.youtube.com/shorts/Ewxq5O-b1gY)
[![Video 3](https://img.youtube.com/vi/MipMW2Ulwu0/0.jpg)](https://www.youtube.com/shorts/MipMW2Ulwu0)

## Project Structure

```
flight-controller/
├── base/
│   ├── boards/                    # Board-specific HAL implementations
│   │   ├── h7v1/                  #   STM32H7 board
│   │   ├── p4v1/                  #   ESP32-P4 board
│   │   └── s3v1/                  #   ESP32-S3 board
│   └── foundation/                # Platform abstraction, pub/sub, macros
│
├── libs/                          # Pre-compiled static libraries
│   ├── optflow/                   #   Optical flow (from ../optflow/)
│   └── robotkit/                  #   Math, fusion, PID (from ../robotkit/)
│
├── modules/                       # Flight control modules
│   ├── imu/                       #   IMU driver (ICM-42688P)
│   ├── compass/                   #   Magnetometer driver & calibration (BMM350)
│   ├── air_pressure/              #   Barometer driver (DPS310)
│   ├── gps/                       #   GPS receiver (u-blox UBX protocol)
│   ├── optflow/                   #   Optical flow sensor interface
│   ├── attitude_estimation/       #   Sensor fusion (Mahony/EKF/Madgwick)
│   ├── attitude_control/          #   Attitude stabilization PID loops
│   ├── position_estimation/       #   Position/velocity estimation (GPS + optflow fusion)
│   ├── position_control/          #   Position hold PID loops
│   ├── speed_control/             #   Motor speed / thrust controller
│   ├── gps_navigation/            #   GPS waypoint navigation (outdoor)
│   ├── gps_denied_navigation/     #   Optical flow navigation (indoor)
│   ├── rc_receiver/               #   RC receiver input processing
│   ├── state_detector/            #   Flight state machine
│   ├── fault_handler/             #   Safety and error handling
│   ├── linear_drift_detection/    #   Sustained linear drift detector
│   ├── oscillation_detection/     #   High-frequency oscillation detector
│   ├── logger/                    #   UART telemetry framing
│   └── local_storage/             #   Persistent configuration storage
│
├── tools/                         # Python host tools
│   ├── gps_config_f9p.py          #   ZED-F9P GPS configuration (UBX-CFG-VALSET)
│   ├── gps_read_upx.py            #   GPS real-time monitor & visualizer
│   ├── gps_sim_ubx.py             #   GPS simulator (UBX NAV-PVT)
│   ├── imu_calibrate_accel.py     #   Accelerometer calibration (6-position)
│   ├── compass_calibrate.py       #   Compass calibration (ellipsoid fit)
│   ├── attitude_estimation_view.py#   3D attitude visualization
│   ├── position_estimation_view.py#   3D position visualization
│   ├── position_estimation_2d.py  #   2D position chart (XY)
│   ├── position_estimation_2d_and_z.py # 2D + altitude chart
│   ├── position_estimation_chart_xy.py # XY position time-series
│   ├── position_estimation_chart_z.py  # Z position time-series
│   └── position_estimation_optflow.py  # Optical flow position view
│
└── simulation/                    # Software-in-the-loop (SITL)
    ├── install.sh                 #   Install SITL dependencies
    ├── uninstall.sh               #   Remove SITL environment
    ├── run_sitl_macos.sh          #   Launch SITL on macOS
    ├── sitl_bridge.py             #   Sensor bridge for SITL
    ├── test_fly.py                #   Automated flight test
    └── models/                    #   Physics models
```

## Architecture

### Module System
Modules follow a strict **setup → subscribe → publish** pattern:
- Each module exports `<module>_setup(void)`, called from `base/foundation/module.c`
- Modules subscribe to topics (sensor data, scheduler ticks) and publish results
- **No direct calls between modules** — all communication via pub/sub

```c
// Example: modules/imu/imu.c
void imu_setup(void) {
    subscribe(SCHEDULER_100HZ, imu_poll_callback);
}

static void imu_poll_callback(uint8_t *data, size_t size) {
    // Read sensor, then publish
    publish(SENSOR_IMU1_GYRO_UPDATE, (uint8_t*)&gyro, sizeof(gyro));
}
```

### Platform Abstraction
Hardware access only through `base/foundation/platform.h`:
- `platform_i2c_write_read()`, `platform_uart_send()`, `platform_spi_write_read()`
- Port enums: `I2C_PORT1`, `UART_PORT2`, `SPI_PORT3`
- **Never** include HAL headers (`stm32h7xx_hal.h`, `driver/i2c.h`) in module code

### Scheduler Topics
| Rate | Usage |
|------|-------|
| `SCHEDULER_2KHZ` | Gyro readout, attitude fusion prediction |
| `SCHEDULER_500HZ` | Accel readout, fusion correction |
| `SCHEDULER_100HZ` | IMU polling |
| `SCHEDULER_50HZ` | PID control loops |
| `SCHEDULER_25HZ` | Compass, attitude logging |
| `SCHEDULER_10HZ` | Navigation, control loops |
| `SCHEDULER_5HZ` | GPS navigation, low-rate updates |
| `SCHEDULER_1HZ` | State updates |

### Event-Driven Topics
- `SENSOR_IMU1_GYRO_UPDATE` / `SENSOR_IMU1_ACCEL_UPDATE` — IMU data
- `SENSOR_COMPASS` — Calibrated compass vector
- `ANGULAR_STATE_UPDATE` — Estimated attitude (roll, pitch, yaw)
- `POSITION_STATE_UPDATE` — Estimated position and velocity
- `EXTERNAL_SENSOR_GPS` / `EXTERNAL_SENSOR_GPS_VELOC` — GPS data
- `EXTERNAL_SENSOR_OPTFLOW` — Optical flow data (from UART → DMA → parser)
- `MONITOR_DATA` — Telemetry for UART streaming

### UART Receive Architecture (STM32H7)
All 4 UART ports use **DMA circular ring buffers** (32 bytes each) instead of per-byte DMA:
- DMA hardware fills the buffer continuously with zero CPU cost
- `HAL_UART_RxHalfCpltCallback` processes bytes 0–15 (first half)
- `HAL_UART_RxCpltCallback` processes bytes 16–31 (second half)
- At 38400 baud, each half provides **~4.2ms of buffering** before data loss
- Reduces total UART ISR rate from ~12,500/s to ~720/s
- Protocol parser validates `payload_size` against buffer bounds to prevent overflow from corrupted length fields

| UART | Baud | Device | Protocol |
|------|------|--------|----------|
| USART1 | 9600 | Telemetry output | DB |
| USART2 | 38400 | GPS | UBX |
| USART3 | 38400 | GPS/external | UBX |
| UART4 | 38400 | Optical flow (flight-optflow) | DB |

## Building & Flashing

### STM32H7
```bash
cd flight-controller
make BOARD=h7v1
make flash BOARD=h7v1
```

### ESP32 (requires ESP-IDF v5.x)
```bash
. $HOME/esp/esp-idf/export.sh
cd base/boards/s3v1   # or p4v1
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

### SITL (macOS)
```bash
cd simulation
./install.sh
./run_sitl_macos.sh
```

## Coding Conventions

- **Module independence:** No cross-module header includes. Communicate via pub/sub only.
- **Shared structs:** Defined in `base/foundation/messages.h` (never duplicated in module headers).
- **Shared macros:** Defined in `base/foundation/macro.h`.
- **Data types:** Use `vector3d_t`, `quat_t` from robotkit. Append `_t` to all typedefs.
- **Functions:** `<module>_<action>()` naming (e.g., `imu_setup()`, `compass_calibrate()`).
- **Memory:** No `malloc()` in flight-critical modules — use static buffers.
- **Static variables:** Prefer file-scope `static` over function-scope `static`.
- **Struct handling:** Use `memset`/`memcpy` for clearing and copying to avoid padding issues.

## Sensor Calibration

### Accelerometer (Static Multi-Position)
1. Set `#define ENABLE_ACCEL_MONITOR_LOG 1` in `modules/imu/imu.c`, build & flash
2. Run `python3 tools/imu_calibrate_accel.py`
3. Place drone in 6 orientations (flat, left, right, nose up/down, inverted), capture each
4. Click "Compute Calib", copy bias vector & scale matrix into `modules/imu/imu.c`
5. Set `ENABLE_ACCEL_MONITOR_LOG 0`, re-flash

### Compass (Ellipsoid Fit)
1. Set `#define ENABLE_COMPASS_MONITOR_LOG 2` in `modules/compass/compass.c`, build & flash
2. Run `python3 tools/compass_calibrate.py`
3. Rotate drone in all directions (figure-8 motion)
4. Copy hard iron bias & soft iron matrix into `modules/compass/compass.c`
5. Set `ENABLE_COMPASS_MONITOR_LOG 0`, re-flash

### Gyroscope
- **Automatic** on every boot — keep drone stationary for 2 seconds after power-on

### GPS (ZED-F9P)
1. Connect ZED-F9P via USB
2. Run `python3 tools/gps_config_f9p.py` to configure 10 Hz UBX-only output at 38400 baud
3. Run `python3 tools/gps_read_upx.py` to verify output

## Telemetry & Visualization

### UART Frame Format
```
'd' 'b' [ID] [Class] [Length_LE] [Payload] [Checksum_LE]
```
- Class `0x00`: `MONITOR_DATA` — float32 sensor/fusion values

### Python Viewers
Install dependencies: `pip install pyserial matplotlib numpy`

| Tool | Purpose |
|------|---------|
| `attitude_estimation_view.py` | 3D attitude (gravity, linear accel) |
| `position_estimation_view.py` | 3D position & velocity vectors |
| `position_estimation_2d.py` | 2D XY position chart |
| `position_estimation_chart_z.py` | Altitude time-series |
| `gps_read_upx.py` | GPS satellite/position monitor |

> **Note:** Enable exactly **one** `ENABLE_*_MONITOR_LOG` macro at a time. Multiple enabled macros cause UART corruption.

| Macro | Module | Streams |
|-------|--------|---------|
| `ENABLE_ACCEL_MONITOR_LOG` | `imu.c` | Raw accelerometer data |
| `ENABLE_COMPASS_MONITOR_LOG` | `compass.c` | Raw (2) or calibrated (1) compass |
| `ENABLE_ATTITUDE_MONITOR_LOG` | `attitude_estimation.c` | Attitude vectors |
| `ENABLE_POSITION_ESTIMATION_MONITOR_LOG` | `position_estimation.c` | Position (1) or velocity (2) |

## Related Projects

| Project | Description |
|---------|-------------|
| [`../optflow/`](../optflow/) | Optical flow library (Lucas-Kanade dense) |
| [`../robotkit/`](../robotkit/) | Math, sensor fusion, PID control library |
| [`../flight-optflow/`](../flight-optflow/) | ESP32 optical flow sensor application |

## License

Proprietary. See LICENSE file for details.
