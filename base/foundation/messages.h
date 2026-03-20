#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <vector3d.h>

// --- System Messages ---

// Parameter IDs for storage (4 bytes each, 192 bytes = 48 params max)
typedef enum {
	// IDs 0-3 reserved (previously gyro bias, now unused)
	// Accelerometer
	PARAM_ID_ACCEL_CALIBRATED = 4,
	PARAM_ID_ACCEL_BIAS_X  = 5,
	PARAM_ID_ACCEL_BIAS_Y  = 6,
	PARAM_ID_ACCEL_BIAS_Z  = 7,
	PARAM_ID_ACCEL_SCALE_00 = 8,   // scale[0][0]
	PARAM_ID_ACCEL_SCALE_01 = 9,   // scale[0][1]
	PARAM_ID_ACCEL_SCALE_02 = 10,  // scale[0][2]
	PARAM_ID_ACCEL_SCALE_10 = 11,  // scale[1][0]
	PARAM_ID_ACCEL_SCALE_11 = 12,  // scale[1][1]
	PARAM_ID_ACCEL_SCALE_12 = 13,  // scale[1][2]
	PARAM_ID_ACCEL_SCALE_20 = 14,  // scale[2][0]
	PARAM_ID_ACCEL_SCALE_21 = 15,  // scale[2][1]
	PARAM_ID_ACCEL_SCALE_22 = 16,  // scale[2][2]
	// Magnetometer
	PARAM_ID_MAG_CALIBRATED = 17,
	PARAM_ID_MAG_OFFSET_X  = 18,
	PARAM_ID_MAG_OFFSET_Y  = 19,
	PARAM_ID_MAG_OFFSET_Z  = 20,
	PARAM_ID_MAG_SCALE_00  = 21,   // scale[0][0]
	PARAM_ID_MAG_SCALE_01  = 22,   // scale[0][1]
	PARAM_ID_MAG_SCALE_02  = 23,   // scale[0][2]
	PARAM_ID_MAG_SCALE_10  = 24,   // scale[1][0]
	PARAM_ID_MAG_SCALE_11  = 25,   // scale[1][1]
	PARAM_ID_MAG_SCALE_12  = 26,   // scale[1][2]
	PARAM_ID_MAG_SCALE_20  = 27,   // scale[2][0]
	PARAM_ID_MAG_SCALE_21  = 28,   // scale[2][1]
	PARAM_ID_MAG_SCALE_22  = 29,   // scale[2][2]
	// Gyro temperature compensation (degree-2 polynomial per axis)
	PARAM_ID_GYRO_TEMP_CALIBRATED = 30,
	PARAM_ID_GYRO_TEMP_X_A = 31,   // X: a*T²
	PARAM_ID_GYRO_TEMP_X_B = 32,   // X: b*T
	PARAM_ID_GYRO_TEMP_X_C = 33,   // X: c
	PARAM_ID_GYRO_TEMP_Y_A = 34,   // Y: a*T²
	PARAM_ID_GYRO_TEMP_Y_B = 35,   // Y: b*T
	PARAM_ID_GYRO_TEMP_Y_C = 36,   // Y: c
	PARAM_ID_GYRO_TEMP_Z_A = 37,   // Z: a*T²
	PARAM_ID_GYRO_TEMP_Z_B = 38,   // Z: b*T
	PARAM_ID_GYRO_TEMP_Z_C = 39,   // Z: c
	// Max (40 of 48 used = 160 of 192 bytes)
	PARAM_ID_MAX = 48,
} param_id_e;

typedef struct {
	param_id_e id;
	float value;
} param_storage_t;

// Calibration data delivery structs (calibration module → sensor modules)

typedef struct {
	float temp_coeff[3][3];     /* [axis][0]*T² + [axis][1]*T + [axis][2] (LSB) */
} calibration_gyro_t;

typedef struct {
	float bias[3];
	float scale[3][3];
} calibration_accel_t;

typedef struct {
	double offset[3];
	double scale[3][3];
} calibration_mag_t;


// --- Control Messages ---

typedef struct {
	vector3d_t position;
	vector3d_t velocity;
} position_state_t;

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
    double clarity; // Quality metric (0-100+)
    uint32_t dt_us; // Time delta (microseconds)
    optflow_direction_t direction; 
} optflow_data_t;

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

typedef struct {
	double roll;
	double pitch;
	double yaw;
	double altitude;
} mix_control_input_t;


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


// --- Sensor Health (fault_detector → flight_state) ---

typedef struct {
	uint8_t gyro;
	uint8_t accel;
	uint8_t compass;
	uint8_t baro;
	uint8_t downward_range;
	uint8_t optflow_down;
	uint8_t optflow_up;
	uint8_t gps;
} sensor_health_t;


// --- Log Class IDs (runtime-selectable via NOTIFY_LOG_CLASS) ---
#define LOG_CLASS_NONE                  0x00
#define LOG_CLASS_IMU_ACCEL_RAW         0x01
#define LOG_CLASS_COMPASS               0x02
#define LOG_CLASS_ATTITUDE              0x03  // v_pred, v_true, v_linear_acc (body)
#define LOG_CLASS_POSITION              0x04
#define LOG_CLASS_FFT_GYRO_Z            0x05
#define LOG_CLASS_POSITION_OPTFLOW      0x06
#define LOG_CLASS_ATTITUDE_MAG          0x07  // raw mag, earth mag, v_pred
#define LOG_CLASS_GYRO_CAL              0x08
#define LOG_CLASS_HEART_BEAT            0x09
#define LOG_CLASS_IMU_ACCEL_CALIB       0x0A
#define LOG_CLASS_IMU_GYRO_RAW          0x0B
#define LOG_CLASS_IMU_GYRO_CALIB        0x0C
#define LOG_CLASS_COMPASS_CALIB         0x0D
#define LOG_CLASS_FFT_GYRO_X            0x0E
#define LOG_CLASS_FFT_GYRO_Y            0x0F
#define LOG_CLASS_STORAGE               0x10
#define LOG_CLASS_MIX_CONTROL           0x11
#define LOG_CLASS_FLIGHT_TELEMETRY     0x12
#define LOG_CLASS_ATTITUDE_EARTH       0x13  // v_pred, v_true, v_linear_acc (earth)

// DB message command IDs (from Python tools via UART)
#define DB_CMD_LOG_CLASS                0x03
#define DB_CMD_CALIBRATE_ACCEL          0x05
#define DB_CMD_CALIBRATE_MAG            0x06
#define DB_CMD_RESET                    0x07
#define DB_CMD_CALIBRATE_GYRO_TEMP      0x08
#define DB_CMD_CHIP_ID                  0x09

#endif // MESSAGES_H
