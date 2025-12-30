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

- **Multi-Sensor Fusion:** Combines gyro, accelerometer, compass, and barometer data for robust attitude estimation
- **Hardware Abstraction:** Platform-independent code that runs on STM32H7 and ESP32
- **Modular Architecture:** Cleanly separated modules for each sensor and control function
- **Real-Time Calibration:** Automatic compass and IMU calibration with range validation
- **UART Telemetry:** Streams sensor data and estimation results to host PC for visualization
- **PID Control:** Attitude and speed stabilization loops
- **Pub/Sub System:** Inter-module communication via publish-subscribe messaging

## Hardware Support

| Component | Driver | Status |
| :--- | :--- | :--- |
| **IMU** | ICM-42688P (I2C/SPI) | ✓ Active |
| **Magnetometer** | BMM350 (I2C) | ✓ Active, Calibrating |
| **Barometer** | DPS310 (I2C) | ✓ Active |
| **GPS** | Generic UART | ✓ Supported |
| **Microcontroller** | STM32H7 / ESP32 | ✓ Both Supported |

## Demo Videos

[![Video 1](https://img.youtube.com/vi/rbQvsm3T5Mc/0.jpg)](https://www.youtube.com/shorts/rbQvsm3T5Mc)
[![Video 2](https://img.youtube.com/vi/Ewxq5O-b1gY/0.jpg)](https://www.youtube.com/shorts/Ewxq5O-b1gY)
[![Video 3](https://img.youtube.com/vi/MipMW2Ulwu0/0.jpg)](https://www.youtube.com/shorts/MipMW2Ulwu0)

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
│   ├── attitude_fusion/       # Sensor fusion (Kalman/complementary filter)
│   ├── attitude_control/      # Attitude stabilization PID loops
│   ├── speed_control/         # Motor speed / thrust controller
│   ├── nav_fusion/            # Position estimation
│   ├── nav_control/           # Navigation / waypoint guidance
│   ├── remote_control/        # RC receiver input processing
│   ├── state_detector/        # Flight state machine
│   ├── fault_handler/         # Safety and error handling
│   ├── logger/                # UART telemetry framing
│   ├── imu_calibrator/        # IMU bias calibration
│   ├── factory_calibration/   # One-time calibration utilities
│   └── local_storage/         # Persistent configuration storage
│
└── pytest/                    # Python3 visualization and testing tools
    ├── view_vectors.py        # 3D vector viewer (dual vectors)
    └── view_charts.py         # Time-series plotting (up to 6 channels)
```

## Sensor Calibration

### Compass (BMM350)
- **Calibration Method:** Continuous min/max tracking during motion
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

**view_vectors.py** - 3D vector visualization
- Displays two 3D vectors in real-time using matplotlib
- Red vector: First 3 float32 values (e.g., compass)
- Blue vector: Second 3 float32 values (e.g., attitude)
- Validates values in range [-1, 1]

**view_charts.py** - Time-series plotting
- Plots up to 6 float32 time-series (128-sample history)
- Color-coded lines for each channel
- Adapts to payload size (4, 8, 12, 16, 20, or 24 bytes)

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
- `SENSOR_ATTITUDE_ANGLE`: Estimated attitude angles (roll, pitch, yaw)
- `SENSOR_LINEAR_ACCEL`: Linear acceleration estimate
- `MONITOR_DATA`: Telemetry data for UART streaming (float32 values)
- `SENSOR_ATTITUDE_VECTOR`: Attitude estimation vector (fusion1 predictions)

### System Topics
- `INTERNAL_MESSAGE`: Internal module communication
- `EXTERNAL_MESSAGE`: External/ground station messages
- `FAULT_DETECTION`: Fault and safety alerts
- `LOOP`: Main loop tick
- `I2C_CALLBACK_UPDATE`: I2C transaction completion
- `SPI_CALLBACK_UPDATE`: SPI transaction completion
- `UART_CALLBACK_UPDATE`: UART reception completion

## Macros for Feature Control

| Macro | Default | Purpose |
| :--- | :--- | :--- |
| `ENABLE_COMPASS_MONITOR_LOG` | 0 | Send compass data to MONITOR_DATA |
| `ENABLE_ATTITUDE_MONITOR_LOG` | 1 | Send attitude (v_pred, v_true) to MONITOR_DATA |

## Example: Viewing Compass Data

1. **Enable compass logging** in [modules/compass/compass.c](modules/compass/compass.c):
   ```c
   #define ENABLE_COMPASS_MONITOR_LOG 1
   ```

2. **Build and flash** firmware to STM32H7

3. **Run Python viewer**:
   ```bash
   cd pytest
   python3 view_vectors.py
   ```

4. **Rotate the device** to calibrate (move through 50+ µT range on all axes)

5. **Observe calibrated compass vectors** in red and attitude vectors in blue

## Testing & Debugging

- Use `pytest/view_charts.py` to monitor raw sensor values
- Use `pytest/view_vectors.py` to validate 3D vector outputs
- Enable debug output on UART for real-time monitoring
- Check module logs for calibration status and error conditions

## Future Enhancements

- [ ] Extended Kalman Filter for better state estimation
- [ ] Wind estimation and compensation
- [ ] Multi-IMU redundancy
- [ ] Advanced motor mixing algorithms
- [ ] Autonomous flight modes
- [ ] Ground control station integration

## License

Proprietary. See LICENSE file for details.

## Contributing

For questions or contributions, contact the development team or open an issue in the repository.
