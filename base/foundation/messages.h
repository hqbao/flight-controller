#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <vector3d.h>

// --- System Messages ---

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


// --- Control Messages ---

typedef enum {
	DISARMED = 0,
	ARMED,
	READY,
	TAKING_OFF,
	FLYING,
	LANDING,
	TESTING,
} state_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float alt;
} rc_att_ctl_t;

typedef struct {
	uint8_t state;
	uint8_t mode;
} rc_state_ctl_t;


// --- Sensor & State Messages ---

typedef struct {
	vector3d_t body;
	vector3d_t earth;
} linear_accel_data_t;

typedef enum {
	OPTFLOW_DOWNWARD = 0,
	OPTFLOW_UPWARD = 1,
} optflow_direction_t;

typedef struct {
    double dx;      // Angular displacement X (radians)
    double dy;      // Angular displacement Y (radians)
    double z;       // Range finder altitude (mm)
    optflow_direction_t direction; 
} optflow_data_t;

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;


// --- GPS Messages ---

// GPS position data structure (from NAV-PVT)
typedef struct {
	int32_t lon;        // Longitude (deg * 1e7)
	int32_t lat;        // Latitude (deg * 1e7)
	int32_t alt;        // Height above mean sea level (mm)
} gps_position_t;

// GPS velocity data structure (from NAV-PVT)
typedef struct {
	int32_t velN;       // North velocity (mm/s)
	int32_t velE;       // East velocity (mm/s)
	int32_t velD;       // Down velocity (mm/s)
} gps_velocity_t;

// GPS quality data structure
typedef struct {
	uint8_t fix_type;   // 0=no fix, 2=2D, 3=3D
	uint8_t num_sv;     // Number of satellites
	uint32_t h_acc;     // Horizontal accuracy (mm)
	uint8_t reliable;   // 1=reliable, 0=not reliable
} gps_quality_t;

#endif // MESSAGES_H
