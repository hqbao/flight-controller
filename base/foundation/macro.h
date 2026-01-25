#ifndef MACRO_H
#define MACRO_H

#include <stdint.h>

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

// System Frequencies (Hz)
#define GYRO_FREQ 1000
#define ACCEL_FREQ 500
#define ATT_CTL_FREQ 1000
#define POS_CTL_FREQ 500
#define CTL_FREQ 100

// Math Constants
#define DEG2RAD (0.01745329251f)
#define RAD2DEG (57.2957795131f)
#define MAX_IMU_ACCEL 16384

#endif
