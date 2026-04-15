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
#define MAX_IMU_ACCEL 16384

#endif
