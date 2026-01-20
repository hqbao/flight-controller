# Flight Controller

A modular embedded flight control system for multi-rotor aircraft, written in **C** for STM32H7 and ESP32 microcontrollers. This project integrates multiple sensors, performs real-time attitude estimation, and executes stabilization control algorithms.

## Overview

The Flight Controller (FC) is the core autopilot system that:
- Reads and calibrates sensor data (IMU, magnetometer, barometer, GPS)
- Performs sensor fusion to estimate aircraft attitude and position
- Executes PID-based stabilization and control algorithms
- Sends commands to motor ESCs via PWM signals
- Logs diagnostic data and telemetry over UART

## Key Features

- **Multi-Sensor Fusion:** Combines gyro, accelerometer, compass, barometer, GPS, and optical flow data for robust attitude and position estimation
- **Adaptive Gain Control:** Dynamic optical flow trust based on drift and oscillation detection
  - Increases trust during sustained linear drift
  - Decreases trust (dampens) during high-frequency oscillations (>5Hz)
- **Hardware Abstraction:** Platform-independent code that runs on STM32H7 and ESP32
- **Modular Architecture:** Cleanly separated modules for each sensor and control function
- **Navigation Modes:** GPS-based outdoor navigation and optical flow-based indoor navigation
- **Real-Time Calibration:** Automatic compass and IMU calibration with range validation
- **UART Telemetry:** Streams sensor data and estimation results to host PC for visualization
- **PID Control:** Attitude, position, and speed stabilization loops
- **Pub/Sub System:** Inter-module communication via publish-subscribe messaging

## Hardware Support

| Component | Driver | Status |
| :--- | :--- | :--- |
| **IMU** | ICM-42688P (I2C/SPI) | ✓ Active |
| **Magnetometer** | BMM350 (I2C) | ✓ Active, Calibrating |
| **Barometer** | DPS310 (I2C) | ✓ Active |
| **Optical Flow** | External Module (UART) | ✓ Supported |
| **GPS** | u-blox UBX Protocol (UART) | ✓ Supported |
| **Microcontroller** | STM32H7 / ESP32 | ✓ Both Supported |

## Demo Videos

[![Video 1](https://img.youtube.com/vi/rbQvsm3T5Mc/0.jpg)](https://www.youtube.com/shorts/rbQvsm3T5Mc)
[![Video 2](https://img.youtube.com/vi/Ewxq5O-b1gY/0.jpg)](https://www.youtube.com/shorts/Ewxq5O-b1gY)
[![Video 3](https://img.youtube.com/vi/MipMW2Ulwu0/0.jpg)](https://www.youtube.com/shorts/MipMW2Ulwu0)

## Coding Guidelines and Rules

- **Module Independence:** Do NOT include headers from one module into another (e.g., `modules/A/A.h` should not be included in `modules/B/B.c`). Modules should be loosely coupled and communicate via the Pub/Sub system. If a data structure is shared, define it in a common header (like `libs/robotkit`) or duplicate the definition if it is specific to the message payload.
- **Static Variables:** Prefer file-scope `static` variables over function-scope `static` variables for safety and clarity.
- **Memory Safety:** Use `memset` and `memcpy` for clearing and copying structs to avoid uninitialized memory or padding issues.

## Project Structure

```
flight-controller/
├── base/                      # Core platform abstraction layer
│   ├── boards/                # Board-specific HAL implementations
│   │   ├── h7v1/              # STM32H7 board variant 1
│   │   ├── p4v1/              # ESP32-P4 board variant 1
│   │   └── s3v1/              # ESP32-S3 board variant 1
│   └── foundation/            # HAL, pubsub, platform utilities
│
├── libs/                      # Reusable math and signal processing libraries
│   ├── optflow/               # Optical flow algorithms
│   └── robotkit/              # Quaternions, matrices, PID controllers
│
├── modules/                   # Flight control functional modules
│   ├── imu/                   # IMU driver and data acquisition
│   ├── compass/               # Magnetometer driver & calibration
│   ├── air_pressure/          # Barometer driver
│   ├── gps/                   # GPS receiver (UBX protocol)
│   ├── optflow/               # Optical flow sensor interface
│   ├── attitude_estimation/   # Sensor fusion (Kalman/complementary filter)
│   ├── attitude_control/      # Attitude stabilization PID loops
│   ├── speed_control/         # Motor speed / thrust controller
│   ├── position_estimation/   # Position estimation (GPS/optical flow fusion)
│   ├── position_control/      # Position control PID loops
│   ├── gps_navigation/        # GPS-based waypoint navigation (outdoor)
│   ├── gps_denied_navigation/ # Optical flow waypoint navigation (indoor)
│   ├── position_target/       # Target position generator and smoothing
│   ├── rc_receiver/           # RC receiver input processing
│   ├── state_detector/        # Flight state machine
│   ├── fault_handler/         # Safety and error handling
│   ├── linear_drift_detection/# Detects sustained linear drift
│   ├── oscillation_detection/ # Detects high-frequency oscillations
│   ├── logger/                # UART telemetry framing
│   └── local_storage/         # Persistent configuration storage

│
└── pytest/                    # Python3 visualization and testing tools
    ├── calibrate_accel.py     # Accelerometer calibration (Static Multi-Position)
    ├── calibrate_compass.py   # Compass calibration (Ellipsoid Fit)
    ├── gps_config.py          # U-Blox GPS configuration script
    ├── gps_read_upx.py        # Tool to read and parse UBX messages
    ├── gps_sim_ubx.py         # GPS simulator (UBX NAV-PVT messages)
    ├── GPS_SIMULATOR_VALIDATION.md # Documentation for GPS simulator
    ├── view_attitude.py       # 3D attitude visualization
    ├── view_position.py       # 3D position visualization
    └── view_charts.py         # Real-time sensor data plotting
```


- **Threshold:** Requires ≥50 µT range on each axis to enable publishing
- **Output Range:** [-1.0, +1.0] unit vectors (normalized)
- **Max Range Check:** Outliers beyond 500 µT are rejected to prevent corruption

### IMU (ICM-42688P)
- **Gyro Bias:** Calculated during startup (static period)
- **Accelerometer:** Offset compensation applied per axis
- **Sample Rate:** 2000 Hz gyro, 500 Hz accelerometer

## Telemetry & Visualization

### UART Data Format
```
Frame structure: 'd' 'b' [ID] [Class] [Length_LE] [Payload] [Checksum_LE]
```

### Message Classes
- `0x00`: MONITOR_DATA - Real-time sensor/fusion data as float32 values
- Checksum: 16-bit sum of ID + Class + length bytes + payload

### Python Viewers

**view_attitude.py** - 3D Attitude Visualization
- Displays the drone's orientation in 3D space.
- Useful for tuning PID loops and verifying sensor fusion.

**view_position.py** - 3D Position Visualization
- Plots the estimated position and velocity vectors.
- Useful for testing optical flow and navigation logic.

**view_charts.py** - Time-series plotting
- Plots up to 6 float32 time-series (128-sample history)
- Color-coded lines for each channel
- Adapts to payload size (4, 8, 12, 16, 20, or 24 bytes)

**gps_read_upx.py** - GPS Data Viewer
- Reads and displays detailed satellite and position data form u-blox F9P
- Includes matplotlib UI for visualizing GPS status

## Building & Flashing

### STM32H7
```bash
cd flight-controller
# Build for H7V1 board
make BOARD=h7v1
# Flash via ST-Link or debugger
make flash BOARD=h7v1
```

### ESP32 (ESP-IDF)
```bash
cd flight-streamer
idf.py build
idf.py flash
idf.py monitor
```

## Module Communication

Modules communicate via a publish-subscribe system:

```c
// Publish sensor data
publish(SENSOR_COMPASS, (uint8_t*)&data, sizeof(data));

// Subscribe to scheduler ticks
subscribe(SCHEDULER_25HZ, callback_function);
```

### Scheduler Topics
- `SCHEDULER_1HZ`: Low-frequency state updates
- `SCHEDULER_5HZ`: Navigation updates
- `SCHEDULER_10HZ`: Navigation and control loops
- `SCHEDULER_25HZ`: Compass calibration/publishing, attitude fusion logging
- `SCHEDULER_50HZ`: Control loops
- `SCHEDULER_100HZ`: IMU polling
- `SCHEDULER_250HZ`: High-rate sensor fusion
- `SCHEDULER_500HZ`: Motor speed updates
- `SCHEDULER_1KHZ`: 1000 Hz control loop
- `SCHEDULER_2KHZ`: 2000 Hz gyro rate
- `SCHEDULER_4KHZ`: 4000 Hz high-speed control
- `SCHEDULER_8KHZ`: 8000 Hz ultra-high-rate processing

### Sensor Topics (Interrupt-Driven)
- `SENSOR_IMU1_GYRO_UPDATE`: IMU1 gyroscope interrupt (2000 Hz)
- `SENSOR_IMU1_ACCEL_UPDATE`: IMU1 accelerometer interrupt (500 Hz)
- `SENSOR_IMU1_GYRO_CALIBRATION_UPDATE`: IMU1 gyro calibration result
- `SENSOR_IMU2_GYRO_UPDATE`: IMU2 gyroscope interrupt
- `SENSOR_IMU2_ACCEL_UPDATE`: IMU2 accelerometer interrupt
- `SENSOR_IMU2_GYRO_CALIBRATION_UPDATE`: IMU2 gyro calibration result

### Data Publication Topics
- `SENSOR_COMPASS`: Calibrated compass vector (published by compass module)
- `ANGULAR_STATE_UPDATE`: Estimated attitude angles (roll, pitch, yaw)
- `ANGULAR_TARGET_UPDATE`: Target attitude angles for control
- `ALTITUDE_CONTROL_UPDATE`: Altitude control commands
- `SENSOR_LINEAR_ACCEL`: Linear acceleration estimate
- `POSITION_STATE_UPDATE`: Estimated position and velocity (from position_estimation)
- `POSITION_TARGET_UPDATE`: Target position (from navigation modules)
- `EXTERNAL_SENSOR_GPS`: GPS position data (lat, lon, alt)
- `EXTERNAL_SENSOR_GPS_VELOC`: GPS velocity data (velN, velE, velD)
- `EXTERNAL_SENSOR_OPTFLOW`: Optical flow sensor data
- `MONITOR_DATA`: Telemetry data for UART streaming (float32 values)
- `SENSOR_ATTITUDE_VECTOR`: Attitude estimation vector (fusion1 predictions)

### Control Topics
- `RC_STATE_UPDATE`: Remote control state changes
- `RC_MOVE_IN_UPDATE`: Remote control movement commands
- `STATE_DETECTION_UPDATE`: Flight state machine updates
- `SPEED_CONTROL_SETUP`: Motor initialization
- `SPEED_CONTROL_UPDATE`: Motor speed commands

### System Topics
- `DB_MESSAGE_UPDATE`: DB protocol messages (internal communication)
- `UBX_MESSAGE_UPDATE`: UBX protocol messages (GPS/external devices)
- `FAULT_DETECTION`: Fault and safety alerts
- `LOOP`: Main loop tick
- `I2C_CALLBACK_UPDATE`: I2C transaction completion
- `SPI_CALLBACK_UPDATE`: SPI transaction completion
- `UART_CALLBACK_UPDATE`: UART reception completion

## Macros for Feature Control

These macros (found in respective module `.c` files) control which data is streamed to the `MONITOR_DATA` topic for visualization. Only one should be enabled at a time to avoid data corruption.

| Macro | Module | Purpose |
| :--- | :--- | :--- |
| `ENABLE_ACCEL_MONITOR_LOG` | `imu.c` | Stream raw accelerometer data for calibration |
| `ENABLE_COMPASS_MONITOR_LOG` | `compass.c` | Stream raw (2) or calibrated (1) compass data |
| `ENABLE_ATTITUDE_MONITOR_LOG` | `attitude_estimation.c` | Stream estimated attitude vectors |
| `ENABLE_POSITION_ESTIMATION_MONITOR_LOG` | `position_estimation.c` | Stream estimated position (1) or velocity (2) |
| `ENABLE_IMU_MONITOR_LOG` | `imu.c` | (Deprecated) Stream raw IMU data |

## Visualization Tools

The `pytest/` directory contains tools to visualize the drone's state in real-time.

1. **Install dependencies:**
   ```bash
   pip install pyserial matplotlib numpy
   ```

2. **Run a Viewer:**
   *   **Attitude**: `python3 view_attitude.py` (Shows 3D orientation)
   *   **Position**: `python3 view_position.py` (Shows 3D position/velocity)
   *   **Charts**: `python3 view_charts.py` (Shows raw sensor graphs)

3. **Note**: Ensure the corresponding `ENABLE_..._MONITOR_LOG` macro is enabled in the firmware module you want to visualize.

## Sensor Calibration

Accurate sensor calibration is critical for stable flight. Follow these procedures for the Accelerometer and Magnetometer.

### 1. Accelerometer Calibration (Static Multi-Position)
Corrects for sensor bias and axis scaling/misalignment.

1.  **Prepare Firmware**:
    *   Open `modules/imu/imu.c`.
    *   Set `#define ENABLE_ACCEL_MONITOR_LOG 1`.
    *   Build and flash the firmware.
2.  **Run Tool**:
    *   Connect the drone via USB.
    *   Run: `python3 pytest/calibrate_accel.py`
3.  **Capture Data**:
    *   Place the drone **STATIC** in an orientation (e.g., Flat).
    *   Click **"Capture Position"** and wait for it to finish.
    *   Rotate the drone to a new orientation (Left, Right, Nose Up, Nose Down, Upside Down).
    *   Repeat for at least **6 different positions** covering the sphere.
4.  **Compute & Save**:
    *   Click **"Compute Calib"**.
    *   Copy the resulting **Bias Vector (B)** and **Scale Matrix (S)**.
    *   Paste them into the `g_imu1` struct in `modules/imu/imu.c`.
    *   Set `#define ENABLE_ACCEL_MONITOR_LOG 0` and re-flash.

### 2. Compass Calibration (Ellipsoid Fit)
Corrects for Hard Iron (offsets) and Soft Iron (distortions) effects.

1.  **Prepare Firmware**:
    *   Open `modules/compass/compass.c`.
    *   Set `#define ENABLE_COMPASS_MONITOR_LOG 2` (Raw Mode).
    *   Build and flash the firmware.
2.  **Run Tool**:
    *   Connect the drone via USB.
    *   Run: `python3 pytest/calibrate_compass.py`
3.  **Capture Data**:
    *   Click **"Start Stream"**.
    *   Rotate the drone in **ALL directions** (Figure-8 motion).
    *   Ensure you cover the entire surface of the sphere.
    *   Watch the "Corrected Data" (Green) plot converge to a perfect sphere.
4.  **Save**:
    *   Stop the stream.
    *   Copy the resulting **Hard Iron Bias (B)** and **Soft Iron Matrix (S)**.
    *   Paste them into `g_mag_offset` and `g_mag_scale` in `modules/compass/compass.c`.
    *   Set `#define ENABLE_COMPASS_MONITOR_LOG 0` and re-flash.

### 3. Gyroscope Calibration
*   **Automatic**: Performed automatically on every boot.
*   **Requirement**: Keep the drone **completely stationary** for 2 seconds after powering on.
*   **Logic**: The system checks for stability (max spread < 2 dps). If moved, it retries.

## Testing & Debugging

- Use `pytest/view_charts.py` to monitor raw sensor values
- Use `pytest/view_attitude.py` to validate attitude estimation
- Enable debug output on UART for real-time monitoring
- Check module logs for calibration status and error conditions

## Future Enhancements

- [ ] Extended Kalman Filter for better state estimation
- [ ] Wind estimation and compensation
- [ ] Multi-IMU redundancy
- [ ] Advanced motor mixing algorithms
- [ ] Waypoint mission planning and execution
- [ ] Ground control station integration
- [ ] Real-time GPS/optical flow switching
- [ ] Obstacle detection and avoidance

## License

Proprietary. See LICENSE file for details.

## Contributing

For questions or contributions, contact the development team or open an issue in the repository.
