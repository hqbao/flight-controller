/**
 * ATTITUDE ESTIMATION MODULE
 * 
 * This module estimates the drone's 3D orientation (roll, pitch, yaw) by fusing
 * gyroscope and accelerometer data from the IMU sensor.
 * 
 * ALGORITHM SELECTION:
 * - FUSION_ALGO = 1: Mahony complementary filter (lightweight, fast)
 * - FUSION_ALGO = 2: Extended Kalman Filter (more sophisticated, heavier)
 * - FUSION_ALGO = 3: Madgwick Filter (Gradient Descent Optimization)
 * 
 * DATA FLOW:
 * 1. SENSOR_IMU1_GYRO_UPDATE (1000Hz) -> gyro_update()
 *    - Integrate gyroscope to predict orientation
 *    - Extract roll/pitch/yaw from quaternion
 *    - Publish ANGULAR_STATE_UPDATE and SENSOR_ATTITUDE_VECTOR
 * 
 * 2. SENSOR_IMU1_ACCEL_UPDATE (500Hz) -> accel_update()
 *    - Correct gyro drift using gravity direction
 *    - Update filter corrections
 * 
 * COORDINATE FRAMES:
 * - Body Frame: X=forward, Y=right, Z=down (aircraft convention)
 * - Sensor axes are negated to match body frame:
 *   gx = -raw_gx, gy = -raw_gy, gz = raw_gz
 *   ax = -raw_ax, ay = -raw_ay, az = raw_az
 * 
 * OUTPUT:
 * - ANGULAR_STATE_UPDATE: {roll, pitch, yaw} in degrees
 * - SENSOR_ATTITUDE_VECTOR: v_pred (predicted gravity vector in body frame)
 * - SENSOR_LINEAR_ACCEL: {v_linear_acc (body), v_linear_acc_earth_frame}
 * - MONITOR_DATA (if enabled): v_pred, v_true, v_linear_acc for visualization
 * 
 * VECTOR NAMING (unified across all fusion algorithms):
 * - v_pred: Predicted gravity from quaternion
 * - v_true: Measured gravity from accelerometer
 * - v_linear_acc: Linear acceleration (gravity removed)
 */
#include "attitude_estimation.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>

/* Select Fusion Algorithm: 1 = Fusion1 (Mahony), 2 = Fusion2 (EKF), 3 = Fusion3 (Madgwick) */
#define FUSION_ALGO 1

#if FUSION_ALGO == 1
#include <fusion1.h>
#elif FUSION_ALGO == 2
#include <fusion2.h>
#elif FUSION_ALGO == 3
#include <fusion3.h>
#endif

/* Macro to enable/disable sending MONITOR_DATA via logger */
#define ENABLE_ATTITUDE_MONITOR_LOG 1

#define MAX_IMU_ACCEL 16384
#define GYRO_FREQ 1000
#define ACCEL_FREQ 500
#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131
#define DT (1.0 / GYRO_FREQ)

// Fusion 1 (Mahony) Gains
#if FUSION_ALGO == 1
#define ATT_F1_GAIN_PROP 1.5
#define ATT_F1_GAIN_INT 0.15
#endif

#if FUSION_ALGO == 2
#define GYRO_NOISE 0.0001
#define ACCEL_NOISE 100.0
#endif

// Fusion 3 (Madgwick) Gains
#if FUSION_ALGO == 3
#define ATT_F3_BETA 0.001
#endif

// Shared Accelerometer Parameters (Used by Fusion 1, 2, and 3)
#if FUSION_ALGO == 1 || FUSION_ALGO == 2 || FUSION_ALGO == 3
#define ATT_ACCEL_SMOOTH 4.0 // LPF Gain (Bandwidth factor). Higher = Faster response, Less smoothing.
#define ATT_F1_LIN_ACC_DECAY 0.5
#define ATT_F1_LIN_ACCEL_MIN 0.5
#define ATT_F1_LIN_ACCEL_MAX 2.0
#endif

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

#if FUSION_ALGO == 1
static fusion1_t g_f11;
#elif FUSION_ALGO == 2
static fusion2_t g_f11;
#elif FUSION_ALGO == 3
static fusion3_t g_f11;
#endif

static angle3d_t g_angular_state = {0, 0, 0};
static vector3d_t g_raw_accel = {0, 0, 0};
static linear_accel_data_t g_linear_accel_out;
static uint8_t g_monitor_msg[36];

/**
 * GYRO UPDATE: Called at 1000Hz
 * Integrates gyroscope to predict orientation change
 */
/**
 * GYRO UPDATE: Called at 1000Hz
 * Integrates gyroscope to predict orientation change
 */
static void gyro_update(uint8_t *data, size_t size) {
    float raw_gx, raw_gy, raw_gz;
    memcpy(&raw_gx, &data[0], sizeof(float));
    memcpy(&raw_gy, &data[4], sizeof(float));
    memcpy(&raw_gz, &data[8], sizeof(float));

	// Convert sensor frame to body frame (negate X and Y)
	float gx = -raw_gx;
	float gy = -raw_gy;
	float gz = raw_gz;

	// Run prediction step (algorithm-specific)
#if FUSION_ALGO == 1
	fusion1_predict(&g_f11, gx, gy, gz, DT);
#elif FUSION_ALGO == 2
	fusion2_predict(&g_f11, gx, gy, gz, DT);
#elif FUSION_ALGO == 3
	fusion3_predict(&g_f11, gx, gy, gz, DT);
#endif

	// Extract Euler angles from quaternion (common for all algorithms)
	vector3d_t euler;
	quat_to_euler(&euler, &g_f11.q);
	g_angular_state.roll = -euler.y * RAD2DEG;
	g_angular_state.pitch = euler.x * RAD2DEG;
	g_angular_state.yaw = euler.z * RAD2DEG;

	// All fusion algorithms now use v_pred consistently
	publish(SENSOR_ATTITUDE_VECTOR, (uint8_t*)&g_f11.v_pred, sizeof(vector3d_t));

	publish(ANGULAR_STATE_UPDATE, (uint8_t*)&g_angular_state, sizeof(angle3d_t));
}

/**
 * ACCEL UPDATE: Called at 500Hz
 * Corrects gyro drift using gravity direction from accelerometer
 */
/**
 * ACCEL UPDATE: Called at 500Hz  
 * Corrects gyro drift using gravity direction from accelerometer
 */
static void accel_update(uint8_t *data, size_t size) {
    float raw_ax, raw_ay, raw_az;
    memcpy(&raw_ax, &data[0], sizeof(float));
    memcpy(&raw_ay, &data[4], sizeof(float));
    memcpy(&raw_az, &data[8], sizeof(float));

	// Convert sensor frame to body frame (negate X and Y)
	float ax = -raw_ax;
	float ay = -raw_ay;
	float az = raw_az;

	g_raw_accel.x = ax;
	g_raw_accel.y = ay;
	g_raw_accel.z = az;

#if FUSION_ALGO == 1
	fusion1_update(&g_f11, ax, ay, az);
#elif FUSION_ALGO == 2
	fusion2_update(&g_f11, ax, ay, az);
#elif FUSION_ALGO == 3
	fusion3_update(&g_f11, ax, ay, az);
#endif

	memcpy(&g_linear_accel_out.body, &g_f11.v_linear_acc, sizeof(vector3d_t));
	memcpy(&g_linear_accel_out.earth, &g_f11.v_linear_acc_earth_frame, sizeof(vector3d_t));
	publish(SENSOR_LINEAR_ACCEL, (uint8_t*)&g_linear_accel_out, sizeof(linear_accel_data_t));
}

#if ENABLE_ATTITUDE_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	/* Pack 3 vectors into MONITOR_DATA message for visualization
	   Format: 9 floats - v_pred(3), v_true(3), v_linear_acc(3) */

	// Predicted gravity vector (from quaternion)
	float pred_x = (float)g_f11.v_pred.x;
	float pred_y = (float)g_f11.v_pred.y;
	float pred_z = (float)g_f11.v_pred.z;
	
	// True/measured gravity vector (from accelerometer)
	float true_x = (float)g_f11.v_true.x;
	float true_y = (float)g_f11.v_true.y;
	float true_z = (float)g_f11.v_true.z;
	
	// Linear acceleration (body frame, gravity removed)
	float lin_x = (float)g_f11.v_linear_acc.x;
	float lin_y = (float)g_f11.v_linear_acc.y;
	float lin_z = (float)g_f11.v_linear_acc.z;
	
	memcpy(&g_monitor_msg[0], &pred_x, sizeof(float));
	memcpy(&g_monitor_msg[4], &pred_y, sizeof(float));
	memcpy(&g_monitor_msg[8], &pred_z, sizeof(float));
	memcpy(&g_monitor_msg[12], &true_x, sizeof(float));
	memcpy(&g_monitor_msg[16], &true_y, sizeof(float));
	memcpy(&g_monitor_msg[20], &true_z, sizeof(float));
	memcpy(&g_monitor_msg[24], &lin_x, sizeof(float));
	memcpy(&g_monitor_msg[28], &lin_y, sizeof(float));
	memcpy(&g_monitor_msg[32], &lin_z, sizeof(float));

	publish(MONITOR_DATA, g_monitor_msg, sizeof(g_monitor_msg));
}
#endif

void attitude_estimation_setup(void) {
#if FUSION_ALGO == 1
	// Initialize Fusion1 (Mahony filter)
	// Parameters: gain_acc_smooth, kp (Mahony P-gain), Ki, decay, accel_freq
	// ATT_ACCEL_SMOOTH controls gravity vector smoothing stiffness
	fusion1_init(&g_f11, ATT_ACCEL_SMOOTH, ATT_F1_GAIN_PROP, ATT_F1_GAIN_INT, ATT_F1_LIN_ACC_DECAY, MAX_IMU_ACCEL, ACCEL_FREQ);
	fusion1_remove_linear_accel(&g_f11, ATT_F1_LIN_ACCEL_MIN, ATT_F1_LIN_ACCEL_MAX);
#elif FUSION_ALGO == 2
	// Initialize Fusion2 (EKF)
	// Parameters: gyro_noise, accel_noise, accel_scale, lpf_gain
	// Passing ATT_ACCEL_SMOOTH converted to per-sample gain (gain / freq)
	fusion2_init(&g_f11, GYRO_NOISE, ACCEL_NOISE, MAX_IMU_ACCEL, ATT_ACCEL_SMOOTH / (double)ACCEL_FREQ);
#elif FUSION_ALGO == 3
	// Initialize Fusion3 (Madgwick filter)
	// Parameters: k0 (accel smoothing gain), beta, freq
	// Uses ATT_ACCEL_SMOOTH for consistent gravity noise rejection
	fusion3_init(&g_f11, ATT_ACCEL_SMOOTH, ATT_F3_BETA, MAX_IMU_ACCEL, ACCEL_FREQ);
	fusion3_remove_linear_accel(&g_f11, ATT_F1_LIN_ACC_DECAY, ATT_F1_LIN_ACCEL_MIN, ATT_F1_LIN_ACCEL_MAX);
#endif

	subscribe(SENSOR_IMU1_GYRO_UPDATE, gyro_update);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, accel_update);

#if ENABLE_ATTITUDE_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}

