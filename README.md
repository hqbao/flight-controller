# Flight Controller

A modular embedded flight control system for multi-rotor aircraft, written in **C** for STM32H7 and ESP32 microcontrollers. Integrates multiple sensors, performs real-time attitude and position estimation, and executes stabilization control loops.

## Overview

The Flight Controller is the core autopilot that:
- Reads and calibrates sensor data (IMU, magnetometer, barometer, GPS, optical flow)
- Performs sensor fusion to estimate aircraft attitude and position
- Executes PID-based stabilization and control loops
- Drives motor ESCs via DShot/PWM
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
│   │   │   ├── Core/              #     CubeIDE-generated code (main.c, IRQ, MSP)
│   │   │   ├── platform/          #     Platform HAL drivers (see below)
│   │   │   └── Drivers/           #     STM32 HAL/CMSIS drivers
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
│   ├── mix_control/               #   Motor mixing (PID → per-motor speeds)
│   ├── position_estimation/       #   Position/velocity estimation (GPS + optflow fusion)
│   ├── position_control/          #   Position hold PID loops
│   ├── speed_control/             #   Motor output driver (DShot/PWM)
│   ├── gps_navigation/            #   GPS waypoint navigation (outdoor)
│   ├── gps_denied_navigation/     #   Optical flow navigation (indoor)
│   ├── rc_receiver/               #   RC receiver input processing
│   ├── calibration/               #   Sensor calibration (gyro/accel/mag)
│   ├── flight_state/              #   Flight state machine
│   ├── flight_telemetry/          #   Centralized telemetry (66-byte frame at 10 Hz)
│   ├── fault_detector/            #   Sensor health monitoring (stuck detection)
│   ├── fault_handler/             #   Safety and error handling
│   ├── fft/                       #   FFT vibration analysis
│   ├── logger/                    #   UART telemetry framing
│   └── local_storage/             #   Persistent configuration storage
│
├── tools/                         # Python host tools
│   ├── gps_config_f9p.py          #   ZED-F9P GPS configuration (UBX-CFG-VALSET)
│   ├── gps_read_ubx.py            #   GPS real-time monitor & visualizer
│   ├── gps_sim_ubx.py             #   GPS simulator (UBX NAV-PVT)
│   ├── calibration_gyro.py         #   Gyro temperature compensation tool
│   ├── calibration_accel.py        #   Accelerometer calibration (6-position ellipsoid fit)
│   ├── calibration_compass.py      #   Compass calibration (ellipsoid fit)
│   ├── mix_control_test.py        #   Real-time motor speed visualizer
│   ├── fft_view.py                #   Real-time FFT spectrum viewer
│   ├── fft_spectrogram.py         #   Waterfall spectrogram viewer
│   ├── attitude_estimation_view.py#   Attitude fusion dashboard (3D vectors + data panel)
│   ├── attitude_estimation_mag_view.py # Magnetometer debug dashboard (3D + heading)
│   ├── position_estimation_2d_and_z.py # Position dashboard (2D map + altitude + velocity)
│   ├── position_estimation_chart.py    # Position/velocity time-series (2×2 grid)
│   ├── position_estimation_optflow.py  # Optical flow & altitude sensor viewer
│   └── flight_telemetry_view.py   #   Flight telemetry HUD
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
    subscribe(SCHEDULER_1KHZ, on_sensor_read_tick);
    subscribe(SCHEDULER_500HZ, on_scheduler_500hz);
}

static void on_sensor_read_tick(uint8_t *data, size_t size) {
    icm42688p_read(&g_imu_sensor);  // triggers I2C/SPI → callback → publish gyro
}

static void on_scheduler_500hz(uint8_t *data, size_t size) {
    // Apply accel calibration, then publish
    publish(SENSOR_IMU1_ACCEL_UPDATE, (uint8_t*)g_imu_data, 12);
}
```

### Platform Abstraction
Hardware access only through `base/foundation/platform.h`:
- `platform_i2c_write_read()`, `platform_uart_send()`, `platform_spi_write_read()`
- Port enums: `I2C_PORT1`, `UART_PORT2`, `SPI_PORT3`
- **Never** include HAL headers (`stm32h7xx_hal.h`, `driver/i2c.h`) in module code

#### STM32H7 Platform Drivers (`base/boards/h7v1/platform/`)
All STM32 HAL implementations are separated from CubeIDE-generated code into dedicated driver files:

| File | Purpose |
|------|---------|
| `platform_hw.h` | Extern declarations for all HAL peripheral handles |
| `platform_i2c.c` | I2C read/write/DMA + `HAL_I2C_MemRxCpltCallback` |
| `platform_spi.c` | SPI stubs + `HAL_SPI_TxRxCpltCallback` |
| `platform_uart.c` | UART TX (USART1) + RX DMA ring buffers, auto-detect DB/UBX parser, error recovery |
| `platform_pwm.c` | PWM + DShot + DShot Extended motor protocols |
| `platform_rc.c` | RC PPM decoder via TIM16 input capture |
| `platform_common.c` | LED toggle, delay, time, console, reset |
| `dshot.c` / `dshot.h` | DShot600 protocol (16-bit frames, 8 ports on TIM1+TIM2) |
| `dshot_ex.c` / `dshot_ex.h` | DShot Extended protocol (32-bit frames, 4 ports on TIM1) |

### Scheduler Topics
| Rate | Usage |
|------|-------|
| `SCHEDULER_8KHZ` | *(unused — available for future use)* |
| `SCHEDULER_4KHZ` | *(unused — available for future use)* |
| `SCHEDULER_2KHZ` | *(unused — available for future use)* |
| `SCHEDULER_1KHZ` | IMU gyro readout (1 kHz sensor read) |
| `SCHEDULER_500HZ` | IMU accel processing, attitude PID, motor mixing |
| `SCHEDULER_250HZ` | *(unused — available for future use)* |
| `SCHEDULER_250HZ` | *(unused — available for future use)* |
| `SCHEDULER_100HZ` | Flight state machine, position control |
| `SCHEDULER_50HZ` | LED status indicator (fault_handler) |
| `SCHEDULER_25HZ` | Compass readout, IMU/position/compass logging, fault detection, RC |
| `SCHEDULER_10HZ` | Attitude/mix/telemetry logging, navigation (GPS + GPS-denied) |
| `SCHEDULER_5HZ` | *(unused — available for future use)* |
| `SCHEDULER_1HZ` | Calibration requests, heartbeat, flash readback, sensor health |

> **ISR safety warning:** All `SCHEDULER_*` callbacks run inside the TIM8 ISR (priority 0). SysTick (priority 15) cannot preempt them, so `HAL_GetTick()` is frozen during execution. Any HAL polling function that uses `HAL_GetTick()` for timeout (e.g., `HAL_I2C_Mem_Read`, `HAL_I2C_Master_Transmit`) will hang forever on a bus error. Modules that need polling I2C should subscribe to `LOOP` (main-thread context) instead, or use DMA.

### LOOP Topic
The `LOOP` topic fires from `main()` (thread context) and is used by modules that need polling I2C or other operations requiring working SysTick timeouts:
- `air_pressure` — DPS310 barometer (I2C polling, ~25 Hz rate-limited)

### Event-Driven Topics
- `SENSOR_IMU1_GYRO_UPDATE` / `SENSOR_IMU1_ACCEL_UPDATE` — IMU data
- `SENSOR_COMPASS` — Calibrated compass vector
- `ANGULAR_STATE_UPDATE` — Estimated attitude (roll, pitch, yaw)
- `POSITION_STATE_UPDATE` — Estimated position and velocity
- `EXTERNAL_SENSOR_GPS` / `EXTERNAL_SENSOR_GPS_VELOC` — GPS data
- `EXTERNAL_SENSOR_OPTFLOW` — Optical flow data (from UART → DMA → parser)
- `SEND_LOG` — Immediate telemetry output (module publishes data → logger sends UART frame)
- `NOTIFY_LOG_CLASS` — Runtime log class selection (from Python tool → UART → logger → all modules)

### UART Architecture (STM32H7)
Implemented in `base/boards/h7v1/platform/platform_uart.c`.

All 4 UART ports use **DMA circular ring buffers** (32 bytes each) for reception. The protocol parser auto-detects both **DB** and **UBX** framing on every port — no port is locked to a single protocol.

**Receive (DMA):**
- DMA hardware fills the buffer continuously with zero CPU cost
- `HAL_UART_RxHalfCpltCallback` processes bytes 0–15 (first half)
- `HAL_UART_RxCpltCallback` processes bytes 16–31 (second half)
- At 38400 baud, each half provides **~4.2ms of buffering** before data loss
- Reduces total UART ISR rate from ~12,500/s to ~720/s
- Protocol parser validates `payload_size` against buffer bounds to prevent overflow from corrupted length fields
- `HAL_UART_ErrorCallback` auto-restarts DMA after any UART error (overrun, framing, noise) — prevents permanent reception loss

**Transmit:** USART1 also handles telemetry TX and Python tool commands. USART2–4 are receive-only.

All ports auto-detect both **DB** and **UBX** framing — any device (GPS, optical flow, external sensor) can be connected to any port.

| UART | Baud | Direction | Notes |
|------|------|-----------|-------|
| USART1 | 9600 | TX + RX | Telemetry output + Python tool commands (USB) |
| USART2 | 38400 | RX only | General-purpose sensor input |
| USART3 | 38400 | RX only | General-purpose sensor input |
| UART4 | 38400 | RX only | General-purpose sensor input |

## Building & Flashing

### STM32H7
```bash
cd flight-controller/base/boards/h7v1
./build.sh            # Build (auto-patches CubeIDE makefiles)
./build-flash.sh      # Build + flash
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
- **Byte buffers:** Never cast `uint8_t*` to `float*`/`int32_t*` — use `memcpy`. See [docs/HARDFAULT_ALIGNMENT_BUG.md](docs/HARDFAULT_ALIGNMENT_BUG.md).

## Coordinate Frame (NED)

The system uses the **NED (North-East-Down)** convention:

### Navigation & Body Frame
- **Navigation frame**: X=North, Y=East, Z=Down
- **Body frame**: X=Forward, Y=Right, Z=Down
- **Euler angles**: Roll (right→+), Pitch (nose-up→+), Yaw (clockwise→+)
- **Gravity vector** at rest: `v_pred = (0, 0, -1)` in NED body frame

### Linear Acceleration Convention (Positive = Direction of Motion)
The robotkit fusion library outputs `v_linear_acc` with **positive = direction of motion**:
- **Move forward then stop** → X goes positive then negative
- **Move right then stop** → Y goes positive then negative
- **Move up then stop** → Z goes positive then negative

Z is positive-up (opposite to NED Z-down). This convention is applied inside the fusion algorithms (fusion1–5). Consumers receive it directly via `SENSOR_LINEAR_ACCEL` — no manual negation needed.

### Position Estimation Frames
- **X/Y (horizontal)**: Uses **body-frame** linear acceleration (`la.body.x/y`) — matches optical flow sensor which measures in the body frame
- **Z (altitude)**: Uses **earth-frame** linear acceleration (`la.earth.z`) — gravity-aligned for barometer/rangefinder fusion
- **Axis mapping**: X = Forward (Pitch correction), Y = Right (Roll correction)

### Sensor-to-Body Axis Mapping (ICM-42688P)
IMU sensor axes (X=right, Y=forward on PCB) differ from body frame. The mapping in `attitude_estimation.c`:
```c
body_gx = -raw_gy;  body_gy = -raw_gx;  body_gz = -raw_gz;
body_ax = -raw_ay;  body_ay = -raw_ax;  body_az = -raw_az;
```

### Sensor-to-Body Axis Mapping (BMM350)
Magnetometer axis mapping in `compass.c` after calibration:
- BMM350 on PCB: sensor X=Right, sensor Y=Forward, sensor Z=Down (same X/Y orientation as ICM-42688P)
- Mapping: `body_x = sensor_y`, `body_y = -sensor_x`, `body_z = sensor_z`
```c
double tmp = mag_vec.x;
mag_vec.x = mag_vec.y;
mag_vec.y = -tmp;
```

### Tilt-Compensated Heading
Magnetometer heading uses tilt compensation in `attitude_estimation.c`:
1. Extract roll/pitch from attitude quaternion
2. Build tilt-only quaternion: `quat_from_euler(roll, pitch, 0)` — removes tilt, preserves nose direction
3. Rotate body-frame mag vector to level frame: `quat_rotate_vector(&mag_earth, &q_tilt, &mag_body)`
4. Compute heading: `atan2(-mag_earth.y, mag_earth.x)` — angle of nose relative to North

## Sensor Calibration

### Accelerometer (6-Position Ellipsoid Fit)
Fits an axis-aligned ellipsoid `V_cal = S × (V_raw − B)` to correct bias and scale per axis.
Temperature compensation is **not needed** — ICM-42688P accel thermal drift (~0.15 mg/°C) is negligible vs flight vibration noise.

1. Build & flash firmware, connect via USB
2. Run `python3 tools/calibration_accel.py`
3. Click **"Test Stor"** to verify FC serial + flash round-trip works
4. Click **"Start Log"** — yellow arrow shows live accelerometer reading
5. Place drone in 6 orientations, click **"Capture"** for each:
   Flat (Z up), Inverted (Z down), Left (Y up), Right (Y down), Nose up (X up), Nose down (X down)
6. Click **"Compute"** — red dots = raw, green dots = corrected
7. Click **"Upload"** — sends bias + scale (12 floats) to FC flash
8. Click **"Verify"** — reads back flash to confirm persistence
9. Click **"View Cal"** — switches to calibrated stream; magnitude should read ~16384 in all orientations
- **"Save CSV"** / **"Load CSV"** persist captured positions to `tools/data/` with chip ID in filename

### Compass (Ellipsoid Fit)
Fits a full ellipsoid `V_cal = S × (V_raw − B)` with cross-coupling terms for soft iron correction.

1. Build & flash firmware, connect via USB
2. Run `python3 tools/calibration_compass.py`
3. Click **"Test Stor"** to verify FC serial + flash round-trip works
4. Click **"Start Log"** — yellow arrow shows live magnetometer reading
5. Click **"Stream"**, then rotate drone in all directions (figure-8 motion)
6. Watch green dots form a sphere — auto-fit runs every 1 second with ≥20 points
7. Click **"Upload"** — sends bias + scale matrix (12 floats) to FC flash
8. Click **"Verify"** — reads back flash to confirm persistence
9. Click **"View Cal"** — switches to calibrated stream; magnitude should stay ~1.0 in all orientations
- **"Save CSV"** / **"Load CSV"** persist collected raw points to `tools/data/` with chip ID in filename
- **"Chip ID"** displays the FC's unique hardware identifier

### Gyroscope (Temperature Compensation)
Fits a quadratic polynomial `bias(T) = a·T² + b·T + c` per axis, allowing the FC to compensate gyro drift across the operating temperature range.

1. Build & flash firmware, connect via USB
2. Run `python3 tools/calibration_gyro.py`
3. Click **"Test Stor"** to verify FC serial + flash round-trip works
4. Click **"Start Log"** — raw gyro + temperature data streams at 25 Hz
5. Click **"Record"** — begins recording gyro/temperature samples
6. Wait for temperature to vary (cold start → warm-up gives best spread)
7. Click **"Record"** again to stop, or let it auto-stop; data is saved as CSV in `tools/data/`
8. Click **"Load CSV"** to load a saved recording, then review the polynomial fit
9. Click **"Upload to FC"** — sends 9 float coefficients (3 axes × a,b,c) to flash
10. Click **"Verify"** to read back stored coefficients, **"View Cal"** to see compensated output
- Recordings include chip ID in the filename for multi-board tracking
- Subsequent boots load coefficients from flash and apply temperature compensation automatically

### GPS (ZED-F9P)
1. Connect ZED-F9P via USB
2. Run `python3 tools/gps_config_f9p.py` to configure 10 Hz UBX-only output at 38400 baud
3. Run `python3 tools/gps_read_ubx.py` to verify output

## Telemetry & Visualization

### UART Frame Format
```
'd' 'b' [ID] [Class] [Length_LE] [Payload] [Checksum_LE]
```
- ID `0x00`: `SEND_LOG` — float32 sensor/fusion values (log class set in Class byte)
- ID `0x03`: `DB_CMD_LOG_CLASS` — command from Python tool to select active log class
- ID `0x05`: `DB_CMD_CALIBRATE_ACCEL` — command from Python tool to upload accel calibration (12 floats)
- ID `0x06`: `DB_CMD_CALIBRATE_MAG` — command from Python tool to upload mag calibration (12 floats)
- ID `0x07`: `DB_CMD_RESET` — command from Python tool to reset the flight controller
- ID `0x08`: `DB_CMD_CALIBRATE_GYRO_TEMP` — command from Python tool to upload gyro temp compensation (9 floats: 3 axes × 3 polynomial coefficients)
- ID `0x09`: `DB_CMD_CHIP_ID` — request 8-byte unique chip ID (response sent via `SEND_LOG`)

### Runtime Log Class Selection
Python tools send a `DB_CMD_LOG_CLASS` command over UART to activate logging from a specific module at runtime — **no firmware recompilation needed**. Each tool has a **"Start Log"** button that sends the appropriate class and a **"Reset FC"** button that sends `DB_CMD_RESET` to perform a hardware reset (STM32: `NVIC_SystemReset()`, SITL: `exit(0)`).

| Log Class | Value | Module | Data |
|-----------|-------|--------|------|
| `LOG_CLASS_NONE` | `0x00` | — | Stops all logging |
| `LOG_CLASS_IMU_ACCEL_RAW` | `0x01` | `imu.c` | Raw accelerometer + temperature (4 floats) |
| `LOG_CLASS_COMPASS` | `0x02` | `compass.c` | Raw magnetometer (3 floats) |
| `LOG_CLASS_ATTITUDE` | `0x03` | `attitude_estimation.c` | Attitude vectors (9 floats) |
| `LOG_CLASS_POSITION` | `0x04` | `position_estimation.c` | Position & velocity (6 floats) |
| `LOG_CLASS_FFT_GYRO_Z` | `0x05` | `fft.c` | Gyro Z batch (50× int16) |
| `LOG_CLASS_POSITION_OPTFLOW` | `0x06` | `position_estimation.c` | Optical flow & altitude (6 floats) |
| `LOG_CLASS_ATTITUDE_MAG` | `0x07` | `attitude_estimation.c` | Mag debug: raw, earth, attitude (9 floats) |
| `LOG_CLASS_GYRO_CAL` | `0x08` | — | *(Reserved — gyro calibration moved to Python tool)* |
| `LOG_CLASS_HEART_BEAT` | `0x09` | `logger.c` | Heartbeat counter (1 float, 1 Hz) — active by default on power-up |
| `LOG_CLASS_IMU_ACCEL_CALIB` | `0x0A` | `imu.c` | Calibrated accelerometer + temperature (4 floats) |
| `LOG_CLASS_IMU_GYRO_RAW` | `0x0B` | `imu.c` | Raw gyroscope + temperature (4 floats, LSB + °C) |
| `LOG_CLASS_IMU_GYRO_CALIB` | `0x0C` | `imu.c` | Calibrated gyroscope + temperature (4 floats, °/s + °C) |
| `LOG_CLASS_COMPASS_CALIB` | `0x0D` | `compass.c` | Calibrated magnetometer (3 floats) |
| `LOG_CLASS_FFT_GYRO_X` | `0x0E` | `fft.c` | Gyro X batch (50× int16) |
| `LOG_CLASS_FFT_GYRO_Y` | `0x0F` | `fft.c` | Gyro Y batch (50× int16) |
| `LOG_CLASS_STORAGE` | `0x10` | `local_storage.c` | Stored calibration params (48 params in 2 pages: 30 + 18 floats, auto-stops) |
| `LOG_CLASS_MIX_CONTROL` | `0x11` | `mix_control.c` | Motor speeds (8 floats, 10 Hz) |
| `LOG_CLASS_FLIGHT_TELEMETRY` | `0x12` | `flight_telemetry.c` | Full telemetry frame (66 bytes, 10 Hz) |
| `LOG_CLASS_ATTITUDE_EARTH` | `0x13` | `attitude_estimation.c` | Earth-frame attitude vectors (9 floats): v_pred, v_true, v_linear_acc_earth_frame |

> **Note:** Only one log class is active at a time. Selecting a new class automatically deactivates the previous one. On power-up, `LOG_CLASS_HEART_BEAT` is active by default so the flight controller is always sending data.

### Python Viewers
Install dependencies: `pip install pyserial matplotlib numpy`

| Tool | Purpose |
|------|---------|  
| `attitude_estimation_view.py` | Attitude fusion dashboard: 3D vectors (gyro prediction, fused estimate, linear accel), data panel with error/tilt metrics |
| `attitude_estimation_mag_view.py` | Magnetometer debug dashboard: 3D vectors (raw mag, tilt-compensated, gravity), heading readout with compass direction |
| `position_estimation_2d_and_z.py` | Position dashboard: 2D map with trail + velocity arrow, data panel, altitude & velocity charts |
| `position_estimation_chart.py` | Position/velocity time-series (2×2 grid) with live value annotations |
| `position_estimation_optflow.py` | Optical flow (downward/upward) & altitude sensors (range finder/barometer) time-series |
| `fft_view.py` | Real-time FFT spectrum viewer |
| `fft_spectrogram.py` | Waterfall spectrogram viewer |
| `calibration_gyro.py` | Gyro temperature compensation (polynomial fit, test storage, upload) |
| `calibration_accel.py` | Accelerometer 6-position ellipsoid calibration (test storage, chip ID, CSV save/load) |
| `calibration_compass.py` | Compass ellipsoid fit calibration (test storage, chip ID, CSV save/load) |
| `mix_control_test.py` | Real-time motor speed visualizer (8 motors) |
| `flight_telemetry_view.py` | Flight telemetry HUD: 3D quadcopter, data panel, position/velocity/altitude overlays |
| `gps_read_ubx.py` | GPS satellite/position monitor |

> **Common controls:** All tools include **Start/Stop Log** (toggles data streaming), **Chip ID** (displays the FC's unique hardware identifier), and **Reset FC** (hardware-resets the flight controller via `DB_CMD_RESET`). Calibration tools additionally include **Test Storage** — uploads known test values to flash and reads them back to verify the serial + storage round-trip before real calibration.

## Related Projects

| Project | Description |
|---------|-------------|
| [flight-optflow](https://github.com/hqbao/flight-optflow) | ESP32 optical flow sensor application |
| [flight-vision](https://github.com/hqbao/flight-vision) | OAK-D W visual navigation module |
| robotkit | Math, sensor fusion, PID control library |
| optflow | Optical flow library (Lucas-Kanade dense) |

## License

Proprietary. See LICENSE file for details.
