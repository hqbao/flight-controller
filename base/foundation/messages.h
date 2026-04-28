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

	// Tuning schema version stamp (ID 47).
	// On boot, local_storage compares the stored value against the compiled-in
	// TUNING_SCHEMA_VERSION; on mismatch, all tuning slots (IDs 48..TUNING_LAST)
	// are forced back to compiled defaults and the new version is written.
	// Bump TUNING_SCHEMA_VERSION (in local_storage.c) any time tuning_params_t
	// field IDs change meaning. Calibration params (IDs <48) are never touched.
	PARAM_ID_TUNING_SCHEMA = 47,

	// === Tuning Parameters (IDs 48-128) ===

	// Attitude PID (22 params: 6 per axis + 4 shared)
	PARAM_ID_ATT_ROLL_P = 48,
	PARAM_ID_ATT_ROLL_I = 49,
	PARAM_ID_ATT_ROLL_D = 50,
	PARAM_ID_ATT_ROLL_I_LIMIT = 51,
	PARAM_ID_ATT_ROLL_P_LIMIT = 52,
	PARAM_ID_ATT_ROLL_O_LIMIT = 53,
	PARAM_ID_ATT_PITCH_P = 54,
	PARAM_ID_ATT_PITCH_I = 55,
	PARAM_ID_ATT_PITCH_D = 56,
	PARAM_ID_ATT_PITCH_I_LIMIT = 57,
	PARAM_ID_ATT_PITCH_P_LIMIT = 58,
	PARAM_ID_ATT_PITCH_O_LIMIT = 59,
	PARAM_ID_ATT_YAW_P = 60,
	PARAM_ID_ATT_YAW_I = 61,
	PARAM_ID_ATT_YAW_D = 62,
	PARAM_ID_ATT_YAW_I_LIMIT = 63,
	PARAM_ID_ATT_YAW_P_LIMIT = 64,
	PARAM_ID_ATT_YAW_O_LIMIT = 65,
	PARAM_ID_ATT_SMOOTH_INPUT = 66,
	PARAM_ID_ATT_SMOOTH_P_TERM = 67,
	PARAM_ID_ATT_SMOOTH_OUTPUT = 68,
	PARAM_ID_ATT_GAIN_TIME = 69,

	// Position Control (8 params)
	PARAM_ID_POS_XY_P = 70,
	PARAM_ID_POS_Z_P = 71,
	PARAM_ID_POS_VELOC_XY_SCALE = 72,
	PARAM_ID_POS_VELOC_Z_SCALE = 73,
	PARAM_ID_POS_LPF_XY = 74,
	PARAM_ID_POS_LPF_Z = 75,
	PARAM_ID_POS_ANGLE_LIMIT = 76,
	PARAM_ID_POS_RC_DEADBAND = 77,

	// Motor/Servo Limits (5 params)
	PARAM_ID_MOTOR_MIN = 78,
	PARAM_ID_MOTOR_MAX = 79,
	PARAM_ID_SERVO_MIN = 80,
	PARAM_ID_SERVO_MAX = 81,
	PARAM_ID_SERVO_CENTER = 82,

	// Attitude Estimation (8 params)
	PARAM_ID_ATT_MAHONY_KP = 83,
	PARAM_ID_ATT_MAHONY_KI = 84,
	PARAM_ID_ATT_F3_BETA = 85,
	PARAM_ID_ATT_F3_ZETA = 86,
	PARAM_ID_ATT_ACCEL_SMOOTH = 87,
	PARAM_ID_ATT_LIN_ACC_DECAY = 88,
	PARAM_ID_ATT_LIN_ACCEL_MIN = 89,
	PARAM_ID_ATT_LIN_ACCEL_MAX = 90,

	// Position Estimation (11 params)
	PARAM_ID_PE_XY_S1_INTEG = 91,
	PARAM_ID_PE_XY_S1_CORR = 92,
	PARAM_ID_PE_XY_S2_INTEG = 93,
	PARAM_ID_PE_XY_S2_CORR = 94,
	PARAM_ID_PE_XY_V_FB = 95,
	PARAM_ID_PE_Z_S1_INTEG = 96,
	PARAM_ID_PE_Z_S1_CORR = 97,
	PARAM_ID_PE_Z_S2_INTEG = 98,
	PARAM_ID_PE_Z_S2_CORR = 99,
	PARAM_ID_PE_Z_V_FB = 100,
	PARAM_ID_PE_OPTFLOW_GAIN = 101,

	// FFT/Notch Filter (3 params)
	PARAM_ID_NOTCH_Q = 102,
	PARAM_ID_NOTCH_MIN_HZ = 103,
	PARAM_ID_FFT_FREQ_ALPHA = 104,

	// Flight State (4 params)
	PARAM_ID_DISARM_ANGLE = 105,
	PARAM_ID_DISARM_RANGE = 106,
	PARAM_ID_ALLOWED_LANDING_RANGE = 107,
	PARAM_ID_TOOK_OFF_RANGE = 108,

	// Servo Bias (4 params)
	PARAM_ID_SERVO_BIAS_1 = 109,
	PARAM_ID_SERVO_BIAS_2 = 110,
	PARAM_ID_SERVO_BIAS_3 = 111,
	PARAM_ID_SERVO_BIAS_4 = 112,

	// Thrust Linearization (2 params)
	PARAM_ID_THRUST_P1 = 113,
	PARAM_ID_THRUST_P2 = 114,

	// RC Scale (3 params)
	PARAM_ID_RC_XY_SCALE  = 115,
	PARAM_ID_RC_Z_SCALE   = 116,
	PARAM_ID_RC_YAW_SCALE = 117,

	// Sensor latency compensation for state estimator (4 params, ms)
	PARAM_ID_EST_LATENCY_BARO_MS    = 118,
	PARAM_ID_EST_LATENCY_LIDAR_MS   = 119,
	PARAM_ID_EST_LATENCY_OPTFLOW_MS = 120,
	PARAM_ID_EST_LATENCY_GPS_MS     = 121,

	// Sensor health timeouts + filter divergence + control loop fade (7 params)
	PARAM_ID_EST_TIMEOUT_GPS_MS     = 122,
	PARAM_ID_EST_TIMEOUT_OPTFLOW_MS = 123,
	PARAM_ID_EST_TIMEOUT_LIDAR_MS   = 124,
	PARAM_ID_EST_TIMEOUT_MAG_MS     = 125,
	PARAM_ID_EST_TIMEOUT_BARO_MS    = 126,
	PARAM_ID_EST_P_RUNAWAY_POS_M2   = 127,
	PARAM_ID_CTL_POSITION_LOOP_FADE_S = 128,

	// Tuning boundary markers
	PARAM_ID_TUNING_FIRST = PARAM_ID_ATT_ROLL_P,                    // 48
	PARAM_ID_TUNING_LAST  = PARAM_ID_CTL_POSITION_LOOP_FADE_S,      // 128
	PARAM_ID_TUNING_COUNT = 81,

	PARAM_ID_MAX = 144,
} param_id_e;

typedef struct {
	param_id_e id;
	float value;
} param_storage_t;

// Tuning parameters struct — fields are in 1:1 sequential order with PARAM_IDs 48-118
typedef struct {
	// Attitude PID (22 floats, IDs 48-69)
	float att_roll_p;
	float att_roll_i;
	float att_roll_d;
	float att_roll_i_limit;
	float att_roll_p_limit;
	float att_roll_o_limit;
	float att_pitch_p;
	float att_pitch_i;
	float att_pitch_d;
	float att_pitch_i_limit;
	float att_pitch_p_limit;
	float att_pitch_o_limit;
	float att_yaw_p;
	float att_yaw_i;
	float att_yaw_d;
	float att_yaw_i_limit;
	float att_yaw_p_limit;
	float att_yaw_o_limit;
	float att_smooth_input;
	float att_smooth_p_term;
	float att_smooth_output;
	float att_gain_time;
	// Position Control (8 floats, IDs 70-77)
	float pos_xy_p;
	float pos_z_p;
	float pos_veloc_xy_scale;
	float pos_veloc_z_scale;
	float pos_lpf_xy;
	float pos_lpf_z;
	float pos_angle_limit;
	float pos_rc_deadband;
	// Motor/Servo (5 floats, IDs 78-82)
	float motor_min;
	float motor_max;
	float servo_min;
	float servo_max;
	float servo_center;
	// Attitude Estimation (8 floats, IDs 83-90)
	float att_mahony_kp;
	float att_mahony_ki;
	float att_f3_beta;
	float att_f3_zeta;
	float att_accel_smooth;
	float att_lin_acc_decay;
	float att_lin_accel_min;
	float att_lin_accel_max;
	// Position Estimation (11 floats, IDs 91-101)
	float pe_xy_s1_integ;
	float pe_xy_s1_corr;
	float pe_xy_s2_integ;
	float pe_xy_s2_corr;
	float pe_xy_v_fb;
	float pe_z_s1_integ;
	float pe_z_s1_corr;
	float pe_z_s2_integ;
	float pe_z_s2_corr;
	float pe_z_v_fb;
	float pe_optflow_gain;
	// FFT/Notch (3 floats, IDs 102-104)
	float notch_q;
	float notch_min_hz;
	float fft_freq_alpha;
	// Flight State (4 floats, IDs 105-108)
	float disarm_angle;
	float disarm_range;
	float allowed_landing_range;
	float took_off_range;
	// Servo Bias (4 floats, IDs 109-112)
	float servo_bias_1;
	float servo_bias_2;
	float servo_bias_3;
	float servo_bias_4;
	// Thrust Linearization (2 floats, IDs 113-114)
	float thrust_p1;
	float thrust_p2;
	// RC Scale (3 floats, IDs 115-117)
	float rc_xy_scale;
	float rc_z_scale;
	float rc_yaw_scale;
	// Sensor latency compensation (4 floats, IDs 118-121, milliseconds).
	// Used by state_estimation to time-align measurements with IMU history
	// (Stage-2 GPS rewind in fusion6 + first-order lag for the rest).
	float est_latency_baro_ms;
	float est_latency_lidar_ms;
	float est_latency_optflow_ms;
	float est_latency_gps_ms;
	// Sensor freshness timeouts (5 floats, IDs 122-126, milliseconds).
	// fusion6_check_health clears the corresponding *_fresh bit in state_t.flags
	// when (now - last_update_us) exceeds these.
	float est_timeout_gps_ms;
	float est_timeout_optflow_ms;
	float est_timeout_lidar_ms;
	float est_timeout_mag_ms;
	float est_timeout_baro_ms;
	// Filter divergence threshold (1 float, ID 127, m^2). When trace(P_pos) exceeds
	// this, position_diverged bit is set in state_t.flags.
	float est_p_runaway_pos_m2;
	// Position outer-loop fade-out time (1 float, ID 128, seconds). When the
	// graceful-degradation contract disengages the position loop, commands ramp
	// to zero over this duration.
	float ctl_position_loop_fade_s;
} tuning_params_t;

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

// --- Speed Control Configuration ---

typedef enum {
	PORT_DISABLED = 0,
	PORT_DSHOT,
	PORT_PWM
} port_protocol_t;

typedef struct {
	uint8_t protocol[8]; // port_protocol_t per port
} speed_control_config_t;


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
	uint8_t num_sv;     // Number of satellites used in fix
	uint32_t h_acc;     // Horizontal accuracy estimate (mm)
	uint32_t v_acc;     // Vertical accuracy estimate (mm)
	uint16_t p_dop;     // Position DOP (scaled 0.01)
	uint8_t flags;      // NAV-PVT flags byte (bit0=gnssFixOK, bit1=diffSoln, ...)
	uint8_t reliable;   // 1=reliable, 0=not reliable
} gps_quality_t;

// Packed GPS log frame (48 bytes incl. padding) — streamed via SEND_LOG when
// LOG_CLASS_GPS is active. Field order is fixed; host-side tools parse it as-is.
typedef struct {
	float lat_deg;       // Latitude (degrees)
	float lon_deg;       // Longitude (degrees)
	float alt_msl_m;     // Altitude above mean sea level (m)
	float vel_n_mps;     // North velocity (m/s)
	float vel_e_mps;     // East velocity (m/s)
	float vel_d_mps;     // Down velocity (m/s)
	float g_speed_mps;   // 2-D ground speed (m/s)
	float head_mot_deg;  // Heading of motion (deg, 0..360)
	float h_acc_m;       // Horizontal accuracy estimate (m)
	float v_acc_m;       // Vertical accuracy estimate (m)
	uint16_t p_dop;      // Position DOP × 100
	uint8_t  num_sv;     // Number of satellites used
	uint8_t  fix_type;   // 0=no fix, 2=2D, 3=3D
	uint8_t  flags;      // NAV-PVT flags byte
	uint8_t  reliable;   // 1=reliable, 0=not reliable
	uint16_t _pad;       // tail padding to 48 bytes (host parser reads exactly 48 bytes)
} gps_log_t;


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


// --- Unified Navigation State (state_estimation → consumers, @ 25 Hz) ---
//
// Snapshot of the full ESKF (fusion6) estimate. NED frame throughout.
// Published on STATE_UPDATE. All modules that need position / velocity /
// attitude / bias should subscribe here instead of legacy QUAT/ANGULAR/
// LINEAR_ACCEL/POSITION_STATE topics. Phase 4 deletes the legacy estimators.

typedef struct {
	vector3d_t position;       // NED, m  (X=North, Y=East, Z=Down)
	vector3d_t velocity;       // NED, m/s
	float      q[4];           // quaternion body→earth, [w,x,y,z]
	angle3d_t  euler;          // roll/pitch/yaw, degrees
	vector3d_t accel_body;     // linear acceleration, body frame, m/s² (gravity removed)
	vector3d_t accel_earth;    // linear acceleration, earth frame, m/s² (positive-up Z)
	vector3d_t gyro_bias;      // rad/s
	vector3d_t accel_bias;     // m/s²
	double     baro_bias;      // m
	double     trace_P_pos;    // m²
	double     trace_P_vel;    // (m/s)²
	double     trace_P_att;    // rad²
	uint16_t   health_flags;   // FUSION6_HF_* bitmask (mirrored)
	uint8_t    init_done;      // 1 once static init completed and filter is trustworthy
	uint8_t    _pad;
} nav_state_t;


// --- FFT Peak Detection (fft module → notch_filter module) ---

#define FFT_NUM_PEAKS  3   /* Per-band peak slots (low / mid / high) */

typedef struct {
	uint8_t axis;                    /* 0=X, 1=Y, 2=Z */
	float freq[FFT_NUM_PEAKS];       /* Peak frequencies in Hz (0 = no peak) */
} fft_peaks_t;


// --- Log Class IDs (runtime-selectable via NOTIFY_LOG_CLASS) ---
#define LOG_CLASS_NONE                  0x00
#define LOG_CLASS_IMU_ACCEL_RAW         0x01
#define LOG_CLASS_COMPASS               0x02
#define LOG_CLASS_ATTITUDE              0x03  // v_pred, v_true, v_linear_acc (body)
#define LOG_CLASS_POSITION              0x04
#define LOG_CLASS_FFT_GYRO_Z            0x05  // removed (was host-side FFT)
#define LOG_CLASS_POSITION_OPTFLOW      0x06
#define LOG_CLASS_ATTITUDE_MAG          0x07  // raw mag, earth mag, v_pred
#define LOG_CLASS_GYRO_CAL              0x08
#define LOG_CLASS_HEART_BEAT            0x09
#define LOG_CLASS_IMU_ACCEL_CALIB       0x0A
#define LOG_CLASS_IMU_GYRO_RAW          0x0B
#define LOG_CLASS_IMU_GYRO_CALIB        0x0C
#define LOG_CLASS_COMPASS_CALIB         0x0D
#define LOG_CLASS_FFT_GYRO_X            0x0E  // removed (was host-side FFT)
#define LOG_CLASS_FFT_GYRO_Y            0x0F  // removed (was host-side FFT)
#define LOG_CLASS_STORAGE               0x10
#define LOG_CLASS_MIX_CONTROL           0x11
#define LOG_CLASS_FLIGHT_TELEMETRY     0x12
#define LOG_CLASS_ATTITUDE_EARTH       0x13  // v_pred, v_true, v_linear_acc (earth)
#define LOG_CLASS_FFT_SPECTRUM_DUAL_X  0x14  // raw + filtered spectrum side-by-side, X axis
#define LOG_CLASS_FFT_SPECTRUM_DUAL_Y  0x15  // raw + filtered spectrum side-by-side, Y axis
#define LOG_CLASS_FFT_SPECTRUM_DUAL_Z  0x16  // raw + filtered spectrum side-by-side, Z axis
#define LOG_CLASS_FFT_PEAKS            0x17
#define LOG_CLASS_FFT_SPECTRUM_X       0x18
#define LOG_CLASS_FFT_SPECTRUM_Y       0x19
#define LOG_CLASS_FFT_SPECTRUM_Z       0x1A
#define LOG_CLASS_RC_RECEIVER          0x1B
#define LOG_CLASS_POSITION_COMPARE     0x1C  // fusion5 vs fusion4 comparison (12 floats)
#define LOG_CLASS_TROUBLESHOOT_ACCEL   0x1D  // accel raw LSB min/max/clip-count diagnostic
#define LOG_CLASS_GPS                  0x1E  // gps_log_t (48 bytes) at GPS update rate

// DB message command IDs (from Python tools via UART)
#define DB_CMD_LOG_CLASS                0x03
#define DB_CMD_CALIBRATE_ACCEL          0x05
#define DB_CMD_CALIBRATE_MAG            0x06
#define DB_CMD_RESET                    0x07
#define DB_CMD_CALIBRATE_GYRO_TEMP      0x08
#define DB_CMD_CHIP_ID                  0x09
#define DB_CMD_TUNING                   0x0A

// UART frame constants
// Sized to fit the largest SEND_LOG payload (FFT dual spectrum = 223 B)
// plus 6 B header + 2 B checksum = 231 B wire frame, rounded up to 256.
#define UART_FRAME_MAX_SIZE 256

// Raw UART data (protocol-agnostic)
typedef struct {
	uint8_t port;                       // UART port index (0-3)
	uint16_t size;                      // Byte count
	uint8_t data[UART_FRAME_MAX_SIZE];  // Raw bytes
} uart_raw_t;

#endif // MESSAGES_H
