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
| **MCU** | STM32H7 / ESP32-S3 / ESP32-P4 | — |

## Getting Started

- **STM32CubeIDE setup:** [Importing to STM32CubeIDE](base/boards/h7v1/SETUP_CUBEIDE.md)
- **WSL2 setup (Windows):** [WSL2 Setup Guide](base/boards/h7v1/SETUP_WSL2.md)

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
│   ├── mix_control/               #   Motor/servo mixing (quad/bicopter, build-time selectable)
│   ├── position_estimation/       #   Position/velocity estimation (Fusion5 + Fusion4 parallel, configurable)
│   ├── position_control/          #   Position hold P-control loops
│   ├── speed_control/             #   Motor/servo output driver (per-port DShot/PWM)
│   ├── gps_navigation/            #   GPS waypoint navigation (outdoor)
│   ├── gps_denied_navigation/     #   Optical flow navigation (indoor)
│   ├── rc_receiver/               #   RC receiver input processing
│   ├── calibration/               #   Sensor calibration (gyro/accel/mag)
│   ├── flight_state/              #   Flight state machine
│   ├── flight_telemetry/          #   Centralized telemetry (66-byte frame at 10 Hz)
│   ├── fault_detector/            #   Sensor health monitoring (stuck detection)
│   ├── fault_handler/             #   Safety and error handling
│   ├── fft/                       #   FFT vibration analysis
│   ├── notch_filter/              #   Gyro notch filter (motor vibration rejection)
│   ├── config/                    #   Runtime tuning parameters (71 params, flash-persistent)
│   ├── dblink/                    #   UART protocol parser + TX queue (DB/UBX framing)
│   └── local_storage/             #   Persistent configuration storage
│
├── tools/                         # Python host tools
│   ├── calibration_gyro.py         #   Gyro temperature compensation tool
│   ├── calibration_accel.py        #   Accelerometer calibration (6-position ellipsoid fit)
│   ├── calibration_compass.py      #   Compass calibration (ellipsoid fit)
│   ├── mix_control_quadcopter_test.py #   Quadcopter motor output visualizer
│   ├── mix_control_bicopter_test.py #   Bicopter tilt-rotor output visualizer
│   ├── fft_spectrum_view.py        #   Real-time spectrogram + peak overlay
│   ├── attitude_estimation_view.py#   Attitude fusion dashboard (3D vectors + data panel)
│   ├── attitude_estimation_mag_view.py # Magnetometer debug dashboard (3D + heading)
│   ├── position_estimation_2d_and_z.py # Position dashboard (2D map + altitude + velocity)
│   ├── position_estimation_chart.py    # Position/velocity time-series (2×2 grid)
│   ├── position_estimation_optflow.py  # Optical flow & altitude sensor viewer
│   ├── position_estimation_compare.py  # Fusion5 vs Fusion4 comparison
│   ├── rc_receiver_view.py        #   RC receiver debug (channels + state/mode)
│   ├── tuning_board.py            #   Parameter tuning GUI (71 params, query/upload/defaults)
│   ├── linear_accel_view.py       #   Linear-accel time-series (vibration bias diagnostic)
│   ├── troubleshoot_accel_clip_view.py # Accel clip / FS-range diagnostic (raw INT16 LSB)
│   ├── test_dblink.py             #   Automated UART data path test
│   ├── flight_telemetry_view.py   #   Flight telemetry HUD (quadcopter)
│   └── flight_telemetry_bicopter_view.py # Flight telemetry HUD (bicopter)
```

## Architecture

### Module System
Modules follow a strict **setup → subscribe → publish** pattern:
- Each module exports `<module>_setup(void)`, called from the board-specific `module.c` (e.g. `base/boards/h7v1/platform/module.c`)
- Modules subscribe to topics (sensor data, scheduler ticks) and publish results
- **No direct calls between modules** — all communication via pub/sub

```c
// Example: modules/imu/imu.c
void imu_setup(void) {
    subscribe(GYRO_SCHEDULER, on_gyro_read);    // GYRO_SCHEDULER = SCHEDULER_1KHZ
    subscribe(ACCEL_SCHEDULER, on_accel_process); // ACCEL_SCHEDULER = SCHEDULER_500HZ
}

static void on_gyro_read(uint8_t *data, size_t size) {
    icm42688p_read(&g_imu_sensor);  // triggers I2C/SPI → callback → publish gyro
}

static void on_accel_process(uint8_t *data, size_t size) {
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
| `platform_uart.c` | UART TX (USART1) + RX DMA ring buffers with IDLE detection, publishes `UART_RAW_RECEIVED`, error recovery |
| `platform_pwm.c` | Servo PWM output (50 Hz, 1–2 ms pulse width) |
| `platform_rc.c` | RC PPM decoder via TIM16 input capture |
| `platform_common.c` | LED toggle, delay, time, console, reset |
| `dshot.c` / `dshot.h` | DShot600 protocol + platform API (8 ports on TIM1+TIM2) |
| `dshot_ex.c` / `dshot_ex.h` | DShot Extended protocol + platform API (4 ports on TIM1) |

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
| `SCHEDULER_10HZ` | Attitude/mix/telemetry logging, navigation (GPS + GPS-denied), FFT trigger flag |
| `SCHEDULER_5HZ` | *(unused — available for future use)* |
| `SCHEDULER_1HZ` | Calibration requests, heartbeat, flash readback, sensor health |

> **ISR safety warning:** All `SCHEDULER_*` callbacks run inside the TIM8 ISR (priority 0). SysTick (priority 15) cannot preempt them, so `HAL_GetTick()` is frozen during execution. Any HAL polling function that uses `HAL_GetTick()` for timeout (e.g., `HAL_I2C_Mem_Read`, `HAL_I2C_Master_Transmit`) will hang forever on a bus error. Modules that need polling I2C should subscribe to `LOOP` (main-thread context) instead, or use DMA.

### LOOP Topic
The `LOOP` topic fires from `main()` (thread context) and is used by modules that need non-ISR execution:
- `air_pressure` — DPS310 barometer (I2C polling, ~25 Hz rate-limited)
- `fft` — 256-point FFT vibration analysis (SNR=3.0, min separation 40 Hz, lock-and-hold EMA, axis-focused streaming); too expensive for ISR — SCHEDULER_10HZ sets a flag, LOOP runs the computation

### Event-Driven Topics
- `SENSOR_IMU1_GYRO_UPDATE` / `SENSOR_IMU1_ACCEL_UPDATE` — IMU data
- `SENSOR_IMU1_GYRO_FILTERED_UPDATE` — Notch-filtered gyro (from notch_filter module)
- `SENSOR_COMPASS` — Calibrated compass vector
- `ANGULAR_STATE_UPDATE` — Estimated attitude (roll, pitch, yaw)
- `LINEAR_ACCEL_UPDATE` — Gravity-removed acceleration, body + earth frame (from fusion)
- `POSITION_STATE_UPDATE` — Estimated position and velocity
- `EXTERNAL_SENSOR_GPS` / `EXTERNAL_SENSOR_GPS_VELOC` — GPS data
- `EXTERNAL_SENSOR_OPTFLOW` — Optical flow data (from UART → DMA → dblink)
- `UART_RAW_RECEIVED` — Raw UART DMA bytes (from platform_uart → dblink)
- `UART_RAW_SEND` / `UART_TX_COMPLETE` — UART TX queue management (dblink)
- `SEND_LOG` — Immediate telemetry output (module publishes data → dblink sends UART frame)
- `NOTIFY_LOG_CLASS` — Runtime log class selection (from Python tool → UART → dblink → all modules)

### UART Architecture (STM32H7)
Implemented in `base/boards/h7v1/platform/platform_uart.c`.

All 4 UART ports use **DMA circular ring buffers** (32 bytes each) with **IDLE line detection** for reception. Raw bytes are published via `UART_RAW_RECEIVED`, and the `dblink` module performs protocol auto-detection (both **DB** and **UBX** framing) on every port — no port is locked to a single protocol.

**Receive (IDLE+DMA):**
- DMA hardware fills the buffer continuously with zero CPU cost
- `HAL_UARTEx_ReceiveToIdle_DMA` fires `HAL_UARTEx_RxEventCallback` when the UART line goes idle (sender stops transmitting), providing the exact byte count received
- `g_last_pos[port]` tracks the last processed position in the circular buffer, handling wraparound correctly
- Half-transfer interrupt is disabled (`__HAL_DMA_DISABLE_IT(DMA_IT_HT)`) — only IDLE events trigger processing
- `dblink` validates `payload_size` against buffer bounds to prevent overflow from corrupted length fields
- `HAL_UART_ErrorCallback` resets `g_last_pos`, restarts IDLE+DMA reception after any UART error (overrun, framing, noise) — prevents permanent reception loss

> **Why IDLE detection instead of half/complete callbacks?** Fixed half/complete DMA boundaries (e.g., 16-byte halves) cause bytes to get stuck when frame sizes don't align with boundaries. A 9-byte frame in a 32-byte buffer leaves 7 bytes waiting for more data before the next callback fires. IDLE detection solves this by firing whenever the sender finishes transmitting, regardless of byte count — commands are processed immediately with no boundary-dependent delays.

**Transmit:** USART1 also handles telemetry TX and Python tool commands. USART2–4 are receive-only.

All ports route raw bytes to `dblink`, which auto-detects both **DB** and **UBX** framing — any device (GPS, optical flow, external sensor) can be connected to any port.

| UART | Baud | Direction | Notes |
|------|------|-----------|-------|
| USART1 | 38400 | TX + RX | Telemetry output + Python tool commands (USB) |
| USART2 | 38400 | RX only | General-purpose sensor input |
| USART3 | 38400 | RX only | General-purpose sensor input |
| UART4 | 38400 | RX only | General-purpose sensor input |

## Building & Flashing

### STM32H7
```bash
cd flight-controller/base/boards/h7v1
./build.sh              # Build bicopter (default)
./build.sh quad         # Build quadcopter
./build.sh bicopter     # Build bicopter (explicit)
./build-flash.sh        # Build bicopter + flash
./build-flash.sh quad   # Build quadcopter + flash
```

### ESP32 (requires ESP-IDF v5.x)
```bash
. $HOME/esp/esp-idf/export.sh
cd base/boards/s3v1   # or p4v1
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
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
- **Tuning params:** Stale `0.0` values in flash silently break attitude estimation (`v_true` frozen at (0,0,-1)). After any firmware upgrade that changes `tuning_params_t` layout, run **"Upload Defaults"** in `tuning_board.py`. See [docs/TUNING_PARAM_ZERO_BUG.md](docs/TUNING_PARAM_ZERO_BUG.md).

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

Z is positive-up (opposite to NED Z-down). This convention is applied inside the fusion algorithms (fusion1–3). Consumers receive it directly via `LINEAR_ACCEL_UPDATE` — no manual negation needed.

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
3. Click **"Start Log"** — yellow arrow shows live accelerometer reading
4. Place drone in 6 orientations, click **"Capture"** for each:
   Flat (Z up), Inverted (Z down), Left (Y up), Right (Y down), Nose up (X up), Nose down (X down)
5. Click **"Compute"** — red dots = raw, green dots = corrected
6. Click **"Upload"** — sends bias + scale (12 floats) to FC flash
7. Click **"Query FC"** — reads back flash to confirm persistence
8. Click **"View Cal"** — switches to calibrated stream; magnitude should read ~`MAX_IMU_ACCEL` LSB (currently **2048** at AFS_16G) in all orientations
- **"Default"** uploads identity calibration (zero bias, identity scale) — flyable baseline that leaves raw LSB unchanged so fusion's `accel_scale = MAX_IMU_ACCEL` divides out gravity correctly
- **"Save CSV"** / **"Load CSV"** persist captured positions to `tools/.calibration_data/` with chip ID in filename

### Compass (Ellipsoid Fit)
Fits a full ellipsoid `V_cal = S × (V_raw − B)` with cross-coupling terms for soft iron correction.

1. Build & flash firmware, connect via USB
2. Run `python3 tools/calibration_compass.py`
3. Click **"Start Log"** — yellow arrow shows live magnetometer reading
4. Click **"Stream"**, then rotate drone in all directions (figure-8 motion)
5. Watch red dots fill the sphere and green dots converge — auto-fit runs every 1 second with ≥20 points
6. Click **"Stream"** again to stop when coverage is complete
7. Click **"Upload"** — sends bias + scale matrix (12 floats) to FC flash
8. Click **"Query FC"** — reads back flash to confirm persistence
9. Click **"View Cal"** — switches to calibrated stream; magnitude should stay ~1.0 in all orientations
- **"Default"** uploads identity calibration (zero bias, identity scale) to reset
- **"Save CSV"** / **"Load CSV"** persist collected raw points to `tools/.calibration_data/` with chip ID in filename

### Gyroscope (Temperature Compensation)
Fits a quadratic polynomial `bias(T) = a·T² + b·T + c` per axis, allowing the FC to compensate gyro drift across the operating temperature range.

1. Build & flash firmware, connect via USB
2. Run `python3 tools/calibration_gyro.py`
3. Click **"Start Log"** — raw gyro + temperature data streams at 25 Hz
4. Click **"Record"** — begins recording gyro/temperature samples
5. Wait for temperature to vary (cold start → warm-up gives best spread)
6. Click **"Record"** again to stop
7. Click **"Compute"** — fits polynomial, shows curves on chart
8. Click **"Upload"** — sends 9 float coefficients (3 axes × a,b,c) to flash
9. Click **"Query FC"** to read back stored coefficients, **"View Cal"** to see compensated output
- **"Save CSV"** / **"Load CSV"** persist recordings to `tools/.calibration_data/` with chip ID in filename
- Subsequent boots load coefficients from flash and apply temperature compensation automatically

### GPS (ZED-F9P)
1. Connect the ZED-F9P USB port to the host PC.
2. Run `python3 tools/gps_config_f9p.py` once per receiver. It writes UBX-CFG-VALSET to RAM + BBR + Flash:
   - 10 Hz measurement rate (`--rate-ms` to override; e.g. `--rate-ms 200` for 5 Hz)
   - UART1 + UART2 baud = 38400 (matches STM32 USART2/3/4)
   - UBX-only on UART1 / UART2 / USB (every NMEA standard message + extra UBX-NAV messages disabled)
   - `UBX-NAV-PVT` enabled on UART1 / UART2 / USB at every nav epoch
   - Saved to BBR + Flash so the configuration survives power cycles
3. Wire the F9P TX → any STM32 USART2/3/4 RX (the parser is port-agnostic; both F9P UART1 and UART2 outputs work).
4. Flash the firmware (`gps_setup()` is enabled on `h7v1`) and run `python3 tools/gps_view.py` for the live dashboard (2-D NED map, altitude bar, velocity time-series, sat-count history, accuracy / pDOP / heading panel).

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
- ID `0x0A`: `DB_CMD_TUNING` — upload single tuning parameter (4-byte param_id LE + 4-byte float LE)

### Runtime Log Class Selection
Python tools send a `DB_CMD_LOG_CLASS` command over UART to activate logging from a specific module at runtime — **no firmware recompilation needed**. On startup, every tool sends `LOG_CLASS_HEART_BEAT` immediately after connecting — this stops whatever the previous tool was streaming and restores the firmware to its default state. Each tool has a **"Start Log"** button that sends the appropriate class and a **"Reset FC"** button that sends `DB_CMD_RESET` to perform a hardware reset (`NVIC_SystemReset()`).

| Log Class | Value | Module | Data |
|-----------|-------|--------|------|
| `LOG_CLASS_NONE` | `0x00` | — | Stops all logging |
| `LOG_CLASS_IMU_ACCEL_RAW` | `0x01` | `imu.c` | Raw accelerometer + temperature (4 floats) |
| `LOG_CLASS_COMPASS` | `0x02` | `compass.c` | Raw magnetometer (3 floats) |
| `LOG_CLASS_ATTITUDE` | `0x03` | `attitude_estimation.c` | Attitude vectors (9 floats) |
| `LOG_CLASS_POSITION` | `0x04` | `position_estimation.c` | Position & velocity (6 floats) |
| `LOG_CLASS_FFT_GYRO_Z` | `0x05` | — | *(Removed — was host-side FFT raw gyro streaming)* |
| `LOG_CLASS_POSITION_OPTFLOW` | `0x06` | `position_estimation.c` | Optical flow & altitude (6 floats) |
| `LOG_CLASS_ATTITUDE_MAG` | `0x07` | `attitude_estimation.c` | Mag debug: raw, earth, attitude (9 floats) |
| `LOG_CLASS_GYRO_CAL` | `0x08` | — | *(Reserved — gyro calibration moved to Python tool)* |
| `LOG_CLASS_HEART_BEAT` | `0x09` | `dblink` | Heartbeat counter (1 float, 1 Hz) — active by default on power-up |
| `LOG_CLASS_IMU_ACCEL_CALIB` | `0x0A` | `imu.c` | Calibrated accelerometer + temperature (4 floats) |
| `LOG_CLASS_IMU_GYRO_RAW` | `0x0B` | `imu.c` | Raw gyroscope + temperature (4 floats, LSB + °C) |
| `LOG_CLASS_IMU_GYRO_CALIB` | `0x0C` | `imu.c` | Calibrated gyroscope + temperature (4 floats, °/s + °C) |
| `LOG_CLASS_COMPASS_CALIB` | `0x0D` | `compass.c` | Calibrated magnetometer (3 floats) |
| `LOG_CLASS_FFT_GYRO_X` | `0x0E` | — | *(Removed — was host-side FFT raw gyro streaming)* |
| `LOG_CLASS_FFT_GYRO_Y` | `0x0F` | — | *(Removed — was host-side FFT raw gyro streaming)* |
| `LOG_CLASS_STORAGE` | `0x10` | `local_storage.c` | Stored params (104 params in 4 pages: 26 floats × 4, auto-stops) |
| `LOG_CLASS_MIX_CONTROL` | `0x11` | `quadcopter.c` / `bicopter.c` | Motor/servo outputs (8 floats, 10 Hz) |
| `LOG_CLASS_FLIGHT_TELEMETRY` | `0x12` | `flight_telemetry.c` | Full telemetry frame (66 bytes, 10 Hz) |
| `LOG_CLASS_ATTITUDE_EARTH` | `0x13` | `attitude_estimation.c` | Earth-frame attitude vectors (9 floats): v_pred, v_true, v_linear_acc_earth_frame |
| `LOG_CLASS_FFT_GYRO_FILTERED_X` | `0x14` | — | *(Removed — was host-side FFT filtered gyro streaming)* |
| `LOG_CLASS_FFT_GYRO_FILTERED_Y` | `0x15` | — | *(Removed — was host-side FFT filtered gyro streaming)* |
| `LOG_CLASS_FFT_GYRO_FILTERED_Z` | `0x16` | — | *(Removed — was host-side FFT filtered gyro streaming)* |
| `LOG_CLASS_FFT_PEAKS` | `0x17` | `fft.c` | Smoothed peak frequencies (6 floats: 3 axes × 2 peaks, 10 Hz) |
| `LOG_CLASS_FFT_SPECTRUM_X` | `0x18` | `fft.c` | Spectrum + peaks combined frame, X axis (112 bytes: 1 + 103 bins + 8 peak, 10 Hz) |
| `LOG_CLASS_FFT_SPECTRUM_Y` | `0x19` | `fft.c` | Spectrum + peaks combined frame, Y axis (112 bytes: 1 + 103 bins + 8 peak, 10 Hz) |
| `LOG_CLASS_FFT_SPECTRUM_Z` | `0x1A` | `fft.c` | Spectrum + peaks combined frame, Z axis (112 bytes: 1 + 103 bins + 8 peak, 10 Hz) |
| `LOG_CLASS_RC_RECEIVER` | `0x1B` | `rc_receiver.c` | RC inputs (7 floats: roll, pitch, yaw, alt, state, mode, msg_count, 25 Hz) |
| `LOG_CLASS_POSITION_COMPARE` | `0x1C` | `position_estimation.c` | Fusion5 vs Fusion4 comparison (12 floats: F5 pos/vel + F4 pos/vel, 25 Hz) |
| `LOG_CLASS_TROUBLESHOOT_ACCEL` | `0x1D` | `troubleshoot.c` | Per-axis raw INT16 accel min/max + clip count over 1 s window |
| `LOG_CLASS_GPS` | `0x1E` | `gps.c` | Packed `gps_log_t` (48 B): lat/lon/alt, NED + ground speed, heading, h/v acc, pDOP, num_sv, fix_type, flags, reliable (10 Hz from a configured ZED-F9P) |

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
| `position_estimation_compare.py` | Fusion5 vs Fusion4 side-by-side comparison (2×3 grid: position + velocity, all axes) |
| `fft_spectrum_view.py` | Real-time spectrogram with dynamic notch peak overlay (replaces old fft_view.py / fft_spectrogram.py) |
| `linear_accel_view.py` | Linear-acceleration time-series (X/Y/Z) with running mean/std/peak — diagnoses vibration-induced DC bias and accel noise. Reuses `LOG_CLASS_ATTITUDE` / `LOG_CLASS_ATTITUDE_EARTH`. |
| `troubleshoot_accel_clip_view.py` | Accelerometer full-scale-range diagnostic: per-axis raw INT16 LSB min/max + clip-count over 1 s window. Confirms whether the configured `AFS_*` range is being saturated under flight vibration / maneuvers. Pairs with `LOG_CLASS_TROUBLESHOOT_ACCEL` from the `troubleshoot` module. |
| `rc_receiver_view.py` | RC receiver debug tool: roll/pitch/yaw/alt time-series, state/mode display, message counter |
| `calibration_gyro.py` | Gyro temperature compensation (polynomial fit, upload, query, CSV) |
| `calibration_accel.py` | Accelerometer 6-position ellipsoid calibration (upload, query, default, CSV) |
| `calibration_compass.py` | Compass ellipsoid fit calibration (upload, query, default, CSV) |
| `mix_control_quadcopter_test.py` | Quadcopter motor output visualizer (8 motors) |
| `mix_control_bicopter_test.py` | Bicopter tilt-rotor output visualizer (2 motors + 2 servos) |
| `flight_telemetry_view.py` | Flight telemetry HUD: 3D quadcopter, data panel, position/velocity/altitude overlays |
| `flight_telemetry_bicopter_view.py` | Flight telemetry HUD: 3D bicopter with tilting nacelles, motors/servos, overlays |
| `gps_config_f9p.py` | One-shot ZED-F9P configurator (UBX-CFG-VALSET to RAM + BBR + Flash) — disables NMEA + extra UBX, enables NAV-PVT @ 10 Hz on UART1/UART2/USB at 38400 baud |
| `gps_view.py` | GPS dashboard: 2-D NED position map with trail + velocity arrow, altitude bar with min/max history, NED + ground-speed line charts, sat-count time-series, fix / accuracy / pDOP / heading panel. Uses `LOG_CLASS_GPS` |
| `test_dblink.py` | Automated test of all log classes — validates full UART data path (chip ID, heartbeat, all sensor/state classes) |
| `tuning_board.py` | Parameter tuning GUI: 71 flash-persistent params in 7 categories, query/upload/defaults with confirmation |

> **Common controls:** All tools include **Start/Stop Log** (toggles data streaming) and **Reset FC** (hardware-resets the flight controller via `DB_CMD_RESET`). Calibration tools include **Upload** (send calibration to flash), **Query FC** (read back stored coefficients), **Default** (upload identity/zero calibration), and **Save/Load CSV** (persist data to `tools/.calibration_data/` with chip ID in filename). Chip ID is auto-detected on connect. All tools use the macOS-native backend on darwin and TkAgg on other platforms.

## Related Projects

| Project | Description |
|---------|-------------|
| [flight-optflow](https://github.com/hqbao/flight-optflow) | ESP32 optical flow sensor application |
| [flight-vision](https://github.com/hqbao/flight-vision) | OAK-D W visual navigation module |
| robotkit | Math, sensor fusion, PID control library |
| optflow | Optical flow library (Lucas-Kanade dense) |

## License

Proprietary. See LICENSE file for details.
