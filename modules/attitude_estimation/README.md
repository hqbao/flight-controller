# Attitude Estimation Module

## Overview

Estimates the drone's 3D orientation (roll, pitch, yaw) by fusing gyroscope and accelerometer data. Supports **5 selectable sensor fusion algorithms**. Also computes tilt-compensated magnetometer heading and body-frame linear acceleration.

## Data Flow

```
Gyro (1 kHz)           Accel (500 Hz)         Compass (25 Hz)
    │                      │                       │
    ▼                      ▼                       ▼
  Predict step         Correct step          Tilt-compensated
  (gyro integration)   (gravity reference)    heading via quaternion
    │                      │                       │
    ├─► ANGULAR_STATE_UPDATE (roll, pitch, yaw)    │
    ├─► SENSOR_ATTITUDE_VECTOR (predicted gravity)  │
    │                      │                       │
    │                 Linear accel extraction       │
    │                      │                       │
    │                 ► SENSOR_LINEAR_ACCEL         │
    │                                              │
    └──────────────────────────────────────────────┘
                                                   │
                                    ► SENSOR_MAG_HEADING_UPDATE
```

## Fusion Algorithms

| ID | Algorithm | Description |
|----|-----------|-------------|
| 1 | Mahony | Complementary filter with PI correction |
| 2 | EKF | Extended Kalman Filter (4-state quaternion) |
| 3 | Madgwick | Gradient descent optimization |
| 4 | **7-State EKF** | EKF with gyro bias estimation (active) |
| 5 | Madgwick+Bias | Madgwick with gyro bias tracking |

Select via `FUSION_ALGO` constant in the source.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `FUSION_ALGO` | `4` | Active algorithm (7-State EKF) |
| `ATTITUDE_MONITOR_MODE` | `1` | 1=Fusion debug, 2=Mag debug |
| `DT` | 1/1000.0 | Gyro prediction timestep |
| `GYRO_NOISE` | 0.0001 | EKF process noise |
| `ACCEL_NOISE` | 100.0 | EKF measurement noise |
| `BIAS_NOISE` | 0.00001 | Gyro bias random walk noise |
| `ATT_ACCEL_SMOOTH` | 4.0 | Accelerometer LPF bandwidth |
| `ATT_LIN_ACC_DECAY` | 0.5 | Linear acceleration decay factor |

## Sensor Frame Convention

Body frame conversion from raw IMU:
```c
gx = -raw_gx;  gy = -raw_gy;  gz = raw_gz;
```

Euler extraction:
```c
roll  = -euler.y * RAD2DEG;
pitch =  euler.x * RAD2DEG;
yaw   =  euler.z * RAD2DEG;
```

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
| `SENSOR_ATTITUDE_VECTOR` | `vector3d_t` — predicted gravity vector | 1 kHz |
| `SENSOR_LINEAR_ACCEL` | `linear_accel_data_t` — body + earth frame | 500 Hz |
| `SENSOR_MAG_HEADING_UPDATE` | `double` — heading (degrees) | 25 Hz |
| `SEND_LOG` | 9 floats: v_pred, v_true, v_linear_acc | 10 Hz |

## Log Class

`LOG_CLASS_ATTITUDE` (0x03) — streams predicted gravity, true gravity, and linear acceleration vectors.

## Tools

| Tool | Purpose |
|------|---------|
| `attitude_estimation_view.py` | 3D visualization of attitude vectors |
