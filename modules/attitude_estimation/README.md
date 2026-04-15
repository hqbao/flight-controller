# Attitude Estimation Module

## Overview

Estimates the drone's 3D orientation (roll, pitch, yaw) by fusing gyroscope and accelerometer data. Supports **3 selectable sensor fusion algorithms**. Also computes tilt-compensated magnetometer heading and body-frame linear acceleration.

## Data Flow

```
Gyro (1 kHz)           Accel (500 Hz)         Compass (25 Hz)
    │                      │                       │
    ▼                      ▼                       ▼
  Predict step         Correct step          Tilt-compensated
  (gyro integration)   (gravity reference)    heading via quaternion
    │                      │                       │
    ├─► ANGULAR_STATE_UPDATE (roll, pitch, yaw)    │
    │                      │                       │
    │                 Linear accel extraction       │
    │                      │                       │
    │                 ► LINEAR_ACCEL_UPDATE         │
    │                                              │
    └──────────────────────────────────────────────┘
                                                   │
                                        g_mag_earth (used for logging)
```

## Fusion Algorithms

| ID | Algorithm | Description |
|----|-----------|-------------|
| 1 | Mahony | Complementary filter with PI correction |
| 2 | **7-State EKF** | **EKF with gyro bias estimation (active)** |
| 3 | Madgwick+Bias | Madgwick with gyro bias tracking |

Select via `FUSION_ALGO` constant in the source.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `FUSION_ALGO` | `2` | Active algorithm (7-State EKF) |
| `DT` | 1/1000.0 | Gyro prediction timestep |
| `GYRO_NOISE` | 0.001 | EKF process noise (Q diagonal for quaternion states) |
| `ACCEL_NOISE` | 100.0 | EKF measurement noise (R diagonal = accel_noise²). Higher = slower correction, more gyro trust. See robotkit/FUSION2_EKF_7STATE.md for tuning guide |
| `BIAS_NOISE` | 0.0001 | Gyro bias random walk noise (Q diagonal for bias states) |
| `ATT_ACCEL_SMOOTH` | 4.0 | Accelerometer LPF gain (raw; multiplied by `dt` internally) |

### Innovation Clamping (Fusion2 EKF)

The EKF uses **innovation clamping** to bound the per-step correction magnitude, making it robust to linear acceleration (the same way Madgwick's `beta×dt` naturally bounds corrections). The `max_innovation` parameter (default `0.1` ≈ 6°) caps the innovation vector norm before applying the Kalman gain. See `robotkit/FUSION2_EKF_7STATE.md` for details and tuning rationale.

## Sensor Frame Convention

### NED Body Frame
The system uses the **NED (North-East-Down)** convention:
- **Navigation frame**: X=North, Y=East, Z=Down
- **Body frame**: X=Forward, Y=Right, Z=Down
- **Euler angles**: Roll (right→+), Pitch (nose-up→+), Yaw (clockwise→+)
- **Gravity vector** at rest: `v_pred = (0, 0, -1)`

### Sensor-to-Body Axis Mapping (ICM-42688P)
IMU sensor axes (X=right, Y=forward on PCB) differ from body frame:
```c
body_gx = -raw_gy;  body_gy = -raw_gx;  body_gz = -raw_gz;
body_ax = -raw_ay;  body_ay = -raw_ax;  body_az = -raw_az;
```

### Linear Acceleration Convention (Positive = Direction of Motion)
The fusion algorithms output `v_linear_acc` with **positive = direction of motion**:
- **Move forward then stop** → X goes positive then negative
- **Move right then stop** → Y goes positive then negative
- **Move up then stop** → Z goes positive then negative

Z is positive-up (opposite to NED Z-down). This convention is applied inside the fusion algorithm — consumers receive it directly via `LINEAR_ACCEL_UPDATE`.

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SENSOR_IMU1_GYRO_UPDATE` | 1 kHz | Predict step (gyro integration) |
| `SENSOR_IMU1_ACCEL_UPDATE` | 500 Hz | Correct step (gravity reference) |
| `SENSOR_COMPASS` | 25 Hz | Tilt-compensated heading |
| `NOTIFY_LOG_CLASS` | Event | Activate/deactivate logging |
| `SCHEDULER_10HZ` | 10 Hz | Stream log data |

### Publications
| Topic | Data | Rate |
|-------|------|------|
| `ANGULAR_STATE_UPDATE` | `angle3d_t` — roll, pitch, yaw (degrees) | 1 kHz |
| `LINEAR_ACCEL_UPDATE` | `linear_accel_data_t` — body + earth frame | 500 Hz |
| `SEND_LOG` | 9 floats: v_pred, v_true, v_linear_acc | 10 Hz |

## Log Classes

| Log Class | ID | Data |
|-----------|----|------|
| `LOG_CLASS_ATTITUDE` | 0x03 | Fusion debug: v_pred, v_true, v_linear_acc (9 floats, 36 bytes) |
| `LOG_CLASS_ATTITUDE_MAG` | 0x07 | Mag debug: raw mag, earth mag, attitude vector (9 floats, 36 bytes) |

Both are runtime-selectable — no recompilation needed.

## Tools

| Tool | Purpose |
|------|---------|
| `attitude_estimation_view.py` | 3D visualization of attitude vectors || `attitude_estimation_mag_view.py` | 3D magnetometer debug visualization |