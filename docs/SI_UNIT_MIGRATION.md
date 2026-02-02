# Position Control & Estimation - SI Unit Migration
**Date:** 2024-10-27
**Status:** IMPLEMENTED

## Overview
This document details the migration of the Position Estimation and Control system from legacy units (mm/scaled) to strict SI Units (Meters, m/s). This change resolves scaling issues and "fighting" betweeen the estimator and controller.

## 1. Position Estimation (`position_estimation.c`)

### Unit Changes
- **Inputs**: Optical Flow logic now treats inputs as standard Radians/sec.
- **Conversion**: Velocity = `Flow_Rad_Sec * Altitude_Meters`.
- **Z-Axis**: Range Finder and Barometer data converted from mm to **Meters**.
- **Outputs**:
  - Position: **Meters** (m)
  - Velocity: **Meters/second** (m/s)
  - Acceleration: **Meters/second²** (m/s²)

### Fusion Tuning (Fusion7)
To handle the vibration characteristics of the drone and the new unit scale, the Kalman Filter parameters were updated:

| Parameter | Value | Reason |
| :--- | :--- | :--- |
| `sigma_accel` | **2.0** | High process noise to handle motor vibration |
| `sigma_vel` | **0.1** | Flow measurement noise |
| `sigma_bias` | **0.002** | Increased bias tracking to remove rectification errors |

## 2. Position Control (`position_control.c`)

### Gain Scaling
With the transition to Meters, the error signal magnitude decreased by a factor of 10-100x compared to the legacy millimeter-like scale (depending on the exact legacy scaling factor).

To maintain control authority, the Proportional Gain was increased:

| Parameter | Old Value | New Value | Reason |
| :--- | :--- | :--- | :--- |
| `POS_CTL_XY_P` | `50.0` | **500.0** | Compensates for 10x smaller input error signal (0.1m vs ~10 legacy units) |

### Velocity Scaling
- `POS_CTL_VELOC_XY_SCALE`: **75.0** (Unchanged). 
  - *Note*: Effective damping is reduced due to unit scaling. Ensure `pos_ctl_veloc_xy_scale` provides sufficient braking.

## 3. Verification
- **Drift**: Eliminated (Stable hover).
- **Oscillation**: None observed.
- **Hold**: Accurate within +/- 10-20cm locally.
