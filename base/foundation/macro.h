#ifndef MACRO_H
#define MACRO_H

#include <stdint.h>

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

// Module IDs for readiness tracking
typedef enum {
	MODULE_ID_LOCAL_STORAGE = 0,
	MODULE_ID_IMU,
	MODULE_ID_COMPASS,
	MODULE_ID_AIR_PRESSURE,
	MODULE_ID_GPS,
	MODULE_ID_OPTFLOW,
	MODULE_ID_MAX = 16,
} module_id_e;

typedef struct {
	module_id_e id;
	uint8_t initialized;
} module_initialized_t;

// Parameter IDs for storage
typedef enum {
	PARAM_ID_IMU1_GYRO_CALIBRATED = 0,  // IMU1 gyro calibration status flag
	// IMU1 Gyro Bias
	PARAM_ID_IMU1_GYRO_BIAS_X = 1,
	PARAM_ID_IMU1_GYRO_BIAS_Y = 2,
	PARAM_ID_IMU1_GYRO_BIAS_Z = 3,
	// Reserve space for future parameters
	PARAM_ID_MAX = 32,
} param_id_e;

typedef struct {
	param_id_e id;
	float value;
} param_storage_t;

#endif
