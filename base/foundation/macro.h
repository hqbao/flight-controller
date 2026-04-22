#ifndef MACRO_H
#define MACRO_H

#include <stdint.h>

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

// System Frequencies (Hz) and Scheduler Topic Aliases
// Change the scheduler alias here to switch the frequency for all modules.
#define GYRO_FREQ       1000
#define GYRO_SCHEDULER  SCHEDULER_1KHZ

#define ACCEL_FREQ          500
#define ACCEL_SCHEDULER     SCHEDULER_500HZ

#define ATT_CTL_FREQ        500
#define ATT_CTL_SCHEDULER   SCHEDULER_500HZ

#define PILOT_CTL_FREQ        100
#define PILOT_CTL_SCHEDULER   SCHEDULER_100HZ

// Math Constants
#define DEG2RAD (0.01745329251f)
#define RAD2DEG (57.2957795131f)
// Accelerometer 1g reference in raw LSB units. Used by attitude_estimation as
// fusion's accel_scale to remove gravity from calibrated accel readings.
// MUST match the IMU full-scale setting in modules/imu/imu.c:
//   AFS_2G  -> 16384
//   AFS_4G  -> 8192
//   AFS_8G  -> 4096
//   AFS_16G -> 2048
// Calibration outputs (S * (raw - B)) preserve raw-LSB scale, so this constant
// directly determines how fusion converts calibrated accel into g-units.
#define MAX_IMU_ACCEL 2048

#endif
