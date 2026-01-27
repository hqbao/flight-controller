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
#include <messages.h>
#include <macro.h>

/* Select Fusion Algorithm: 1 = Fusion1 (Mahony), 2 = Fusion2 (EKF), 3 = Fusion3 (Madgwick), 4 = Fusion4 (7-State EKF), 5 = Fusion5 (Madgwick+Bias) */
#define FUSION_ALGO 4

#if FUSION_ALGO == 1
#include <fusion1.h>
#elif FUSION_ALGO == 2
#include <fusion2.h>
#elif FUSION_ALGO == 3
#include <fusion3.h>
#elif FUSION_ALGO == 4
#include <fusion4.h>
#elif FUSION_ALGO == 5
#include <fusion5.h>
#endif

/* Macro to enable/disable sending MONITOR_DATA via logger */
#define ENABLE_ATTITUDE_MONITOR_LOG 0

#define DT (1.0 / GYRO_FREQ)

// Fusion 1 (Mahony) Gains
#if FUSION_ALGO == 1
#define ATT_F1_GAIN_PROP 1.5  // Kp: Proportional correction (not frequency-scaled, gentle response)
#define ATT_F1_GAIN_INT 0.15  // Ki: Integral for gyro bias (divided by freq for time integration)
#endif

#if FUSION_ALGO == 2 || FUSION_ALGO == 4
#define GYRO_NOISE 0.0001
#define ACCEL_NOISE 100.0
#endif

#if FUSION_ALGO == 4
#define BIAS_NOISE 0.00001
#endif

// Fusion 3 (Madgwick) Gains
#if FUSION_ALGO == 3
#define ATT_F3_BETA 0.001
#endif

// Fusion 5 (Madgwick w/ Bias) Gains
#if FUSION_ALGO == 5
#define ATT_F5_BETA 0.001
#define ATT_F5_ZETA 0.00001
#endif

// Shared Accelerometer Parameters (Used by Fusion 1, 2, 3, 4, 5)
#if FUSION_ALGO == 1 || FUSION_ALGO == 2 || FUSION_ALGO == 3 || FUSION_ALGO == 4 || FUSION_ALGO == 5
#define ATT_ACCEL_SMOOTH 4.0  // Accel LPF bandwidth factor (divided by freq for frequency-invariance)
#define ATT_F1_LIN_ACC_DECAY 0.5
#define ATT_F1_LIN_ACCEL_MIN 0.5
#define ATT_F1_LIN_ACCEL_MAX 2.0
#endif

#if FUSION_ALGO == 1
static fusion1_t g_f11;
#elif FUSION_ALGO == 2
static fusion2_t g_f11;
#elif FUSION_ALGO == 3
static fusion3_t g_f11;
#elif FUSION_ALGO == 4
static fusion4_t g_f11;
#elif FUSION_ALGO == 5
static fusion5_t g_f11;
#endif

static angle3d_t g_angular_state = {0, 0, 0};
static vector3d_t g_raw_accel = {0, 0, 0};
static vector3d_t g_mag_vec = {0, 0, 0};
static vector3d_t g_mag_earth = {0, 0, 0};
static double g_mag_heading = 0.0;
static linear_accel_data_t g_linear_accel_out;

#if ENABLE_ATTITUDE_MONITOR_LOG
static uint8_t g_monitor_msg[36] = {0};
#endif

/* 
 * MAGNETOMETER UPDATE
 * Calculates tilt-compensated heading (Yaw)
 */
static void mag_update(uint8_t *data, size_t size) {
    if (size != sizeof(vector3d_t)) return;
    memcpy(&g_mag_vec, data, sizeof(vector3d_t));

    // 1. Get Roll/Pitch from attitude estimate
    vector3d_t euler;
    quat_to_euler(&euler, &g_f11.q);

    // 2. Create Tilt-Only quaternion (Yaw=0)
    quaternion_t q_tilt;
    quat_from_euler(&q_tilt, euler.y, euler.x, 0);

    // 3. Un-tilt mag vector to Earth frame horizontal plane
    quaternion_t q_tilt_conj;
    quat_conjugate(&q_tilt_conj, &q_tilt);
    quat_rotate_vector(&g_mag_earth, &q_tilt_conj, &g_mag_vec);

    // 4. Calculate Heading
    g_mag_heading = atan2(-g_mag_earth.y, g_mag_earth.x) * RAD2DEG;

    publish(SENSOR_MAG_HEADING_UPDATE, (uint8_t*)&g_mag_heading, sizeof(double));
}

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
#elif FUSION_ALGO == 4
	fusion4_predict(&g_f11, gx, gy, gz, DT);
#elif FUSION_ALGO == 5
	fusion5_predict(&g_f11, gx, gy, gz, DT);
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
#elif FUSION_ALGO == 4
	fusion4_update(&g_f11, ax, ay, az);
#elif FUSION_ALGO == 5
	fusion5_update(&g_f11, ax, ay, az);
#endif

	memcpy(&g_linear_accel_out.body, &g_f11.v_linear_acc, sizeof(vector3d_t));
	memcpy(&g_linear_accel_out.earth, &g_f11.v_linear_acc_earth_frame, sizeof(vector3d_t));
	publish(SENSOR_LINEAR_ACCEL, (uint8_t*)&g_linear_accel_out, sizeof(linear_accel_data_t));
}

#if ENABLE_ATTITUDE_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	float v1_x, v1_y, v1_z;
	float v2_x, v2_y, v2_z;
	float v3_x, v3_y, v3_z;

#if ENABLE_ATTITUDE_MONITOR_LOG == 2
	// Mode 2: Magnetometer Debugging (Raw Mag, Earth Mag, Attitude Vector)
	v1_x = (float)g_mag_vec.x;
	v1_y = (float)g_mag_vec.y;
	v1_z = (float)g_mag_vec.z;

	v2_x = (float)g_mag_earth.x;
	v2_y = (float)g_mag_earth.y;
	v2_z = (float)g_mag_earth.z;

	v3_x = (float)g_f11.v_pred.x;
	v3_y = (float)g_f11.v_pred.y;
	v3_z = (float)g_f11.v_pred.z;
#else
	// Mode 1: Fusion Debugging (Attitude Vector, Measured Gravity, Linear Accel)
	/* Pack 3 vectors into MONITOR_DATA message for visualization
	   Format: 9 floats - v_pred(3), v_true(3), v_linear_acc(3) */

	// Predicted gravity vector (from quaternion)
	v1_x = (float)g_f11.v_pred.x;
	v1_y = (float)g_f11.v_pred.y;
	v1_z = (float)g_f11.v_pred.z;
	
	// True/measured gravity vector (from accelerometer)
	v2_x = (float)g_f11.v_true.x;
	v2_y = (float)g_f11.v_true.y;
	v2_z = (float)g_f11.v_true.z;
	
	// Linear acceleration (body frame, gravity removed)
	v3_x = (float)g_f11.v_linear_acc.x;
	v3_y = (float)g_f11.v_linear_acc.y;
	v3_z = (float)g_f11.v_linear_acc.z;
#endif
	
	memcpy(&g_monitor_msg[0], &v1_x, sizeof(float));
	memcpy(&g_monitor_msg[4], &v1_y, sizeof(float));
	memcpy(&g_monitor_msg[8], &v1_z, sizeof(float));
	memcpy(&g_monitor_msg[12], &v2_x, sizeof(float));
	memcpy(&g_monitor_msg[16], &v2_y, sizeof(float));
	memcpy(&g_monitor_msg[20], &v2_z, sizeof(float));
	memcpy(&g_monitor_msg[24], &v3_x, sizeof(float));
	memcpy(&g_monitor_msg[28], &v3_y, sizeof(float));
	memcpy(&g_monitor_msg[32], &v3_z, sizeof(float));

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
#elif FUSION_ALGO == 4
    // Initialize Fusion4 (7-State EKF)
    // Parameters: gyro_noise, bias_noise, accel_noise, accel_scale, lpf_gain
	// Bias noise set to 1e-5 to allow slow walk estimation
    fusion4_init(&g_f11, GYRO_NOISE, BIAS_NOISE, ACCEL_NOISE, MAX_IMU_ACCEL, ATT_ACCEL_SMOOTH / (double)ACCEL_FREQ);
#elif FUSION_ALGO == 5
	// Initialize Fusion 5 (Madgwick with Bias)
	// Parameters: k0 (accel filter), beta (gain), zeta (bias gain), accel_scale, freq
	fusion5_init(&g_f11, ATT_ACCEL_SMOOTH, ATT_F5_BETA, ATT_F5_ZETA, MAX_IMU_ACCEL, ACCEL_FREQ);
    fusion5_remove_linear_accel(&g_f11, ATT_F1_LIN_ACC_DECAY, ATT_F1_LIN_ACCEL_MIN, ATT_F1_LIN_ACCEL_MAX);
#endif

	subscribe(SENSOR_IMU1_GYRO_UPDATE, gyro_update);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, accel_update);
	subscribe(SENSOR_COMPASS, mag_update);

#if ENABLE_ATTITUDE_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}

