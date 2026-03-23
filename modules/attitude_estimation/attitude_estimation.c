/**
 * ATTITUDE ESTIMATION MODULE
 * 
 * This module estimates the drone's 3D orientation (roll, pitch, yaw) by fusing
 * gyroscope and accelerometer data from the IMU sensor.
 * 
 * ALGORITHM SELECTION:
 * - FUSION_ALGO = 1: Mahony complementary filter (lightweight, fast)
 * - FUSION_ALGO = 2: 7-State EKF with gyro bias estimation
 * - FUSION_ALGO = 3: Madgwick Filter with Bias Estimation
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
 * - Body Frame: NED/FRD (X=forward, Y=right, Z=down)
 * - Earth Frame: NED (X=north, Y=east, Z=down)
 * - ICM-42688P sensor axes: X=right, Y=forward on PCB
 * - PCB mounted face-up, sensor X/Y swapped relative to body:
 *   Sensor Y (forward) → body X (forward), Sensor X (right) → body Y (right)
 *   body_gx = -raw_gy, body_gy = -raw_gx, body_gz = -raw_gz
 *   body_ax = -raw_ay, body_ay = -raw_ax, body_az = -raw_az
 * - At rest, accel reads (0, 0, -1g) in NED body frame
 * 
 * OUTPUT:
 * - ANGULAR_STATE_UPDATE: {roll, pitch, yaw} in degrees
 * - SENSOR_ATTITUDE_VECTOR: v_pred (predicted gravity vector in body frame)
 * - SENSOR_LINEAR_ACCEL: {v_linear_acc (body), v_linear_acc_earth_frame}
 * - SEND_LOG (if enabled):
 *   LOG_CLASS_ATTITUDE (0x03):       v_pred, v_true, v_linear_acc (body frame)
 *   LOG_CLASS_ATTITUDE_EARTH (0x13): v_pred, v_true, v_linear_acc_earth_frame
 *   LOG_CLASS_ATTITUDE_MAG (0x07):   raw mag, earth mag, v_pred
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

/* Select Fusion Algorithm: 1 = Fusion1 (Mahony), 2 = Fusion2 (7-State EKF), 3 = Fusion3 (Madgwick+Bias) */
#define FUSION_ALGO 3

#if FUSION_ALGO == 1
#include <fusion1.h>
#elif FUSION_ALGO == 2
#include <fusion2.h>
#elif FUSION_ALGO == 3
#include <fusion3.h>
#endif

#define DT (1.0 / GYRO_FREQ)

// Fusion 1 (Mahony) Gains
#if FUSION_ALGO == 1
#define ATT_MAHONY_KP 0.1     // Kp: Proportional correction (not frequency-scaled, gentle response)
#define ATT_MAHONY_KI 0.001    // Ki: Integral for gyro bias (divided by freq for time integration)
#endif

// Fusion 2 (7-State EKF) Gains
#if FUSION_ALGO == 2
#define GYRO_NOISE 0.001     // Process noise rate for quaternion (Q[0:3], scaled by dt internally)
#define ACCEL_NOISE 100.0    // Measurement noise std dev (R = accel_noise² = 10000). On unit vectors,
                             // higher = more gyro-dominant. See FUSION2_EKF_7STATE.md tuning guide.
#define BIAS_NOISE 0.0001    // Process noise rate for gyro bias random walk (Q[4:6], scaled by dt)
#endif

// Fusion 3 (Madgwick w/ Bias) Gains
#if FUSION_ALGO == 3
#define ATT_F3_BETA 0.001
#define ATT_F3_ZETA 0.0001
#endif

// Shared Accelerometer Parameters (Used by all fusion algorithms)
#if FUSION_ALGO == 1 || FUSION_ALGO == 2 || FUSION_ALGO == 3
#define ATT_ACCEL_SMOOTH 4.0      // Accel LPF gain (raw; multiplied by dt internally)
#define ATT_LIN_ACC_DECAY 0.5     // Linear acceleration decay factor
#define ATT_LIN_ACCEL_MIN 0.5     // Min accel magnitude for linear accel removal (G)
#define ATT_LIN_ACCEL_MAX 2.0     // Max accel magnitude for linear accel removal (G)
#endif

#if FUSION_ALGO == 1
static fusion1_t g_f11;
#elif FUSION_ALGO == 2
static fusion2_t g_f11;
#elif FUSION_ALGO == 3
static fusion3_t g_f11;
#endif

static angle3d_t g_angular_state = {0, 0, 0};
static vector3d_t g_raw_gyro = {0, 0, 0};
static vector3d_t g_raw_accel = {0, 0, 0};
static vector3d_t g_mag_vec = {0, 0, 1};
static vector3d_t g_mag_earth = {0, 0, 0};
static double g_mag_heading = 0.0;
static linear_accel_data_t g_linear_accel_out;

static uint8_t g_monitor_msg[36] = {0};
static uint8_t g_log_class = LOG_CLASS_NONE;

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

    // 2. Create Tilt-Only quaternion (Yaw=0) — NED: roll=euler.x, pitch=euler.y
    // q_tilt represents body-to-earth rotation (tilt only, no yaw)
    quaternion_t q_tilt;
    quat_from_euler(&q_tilt, euler.x, euler.y, 0);

    // 3. Rotate body-frame mag vector to Earth frame horizontal plane
    // q_tilt rotates body→earth, which un-tilts the body-frame mag reading
    quat_rotate_vector(&g_mag_earth, &q_tilt, &g_mag_vec);

    // 4. Calculate Heading
    // After tilt compensation, g_mag_earth is in a level frame with X=nose direction.
    // atan2(y, x) gives the angle of magnetic North relative to the nose.
    // We want heading = angle of nose relative to North, which is the negative:
    //   heading = -atan2(y, x) = atan2(-y, x)
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

	// Convert sensor frame to NED body frame (FRD: X=fwd, Y=right, Z=down)
	// Sensor Y (forward) → body X, Sensor X (right) → body Y, both negated
	g_raw_gyro.x = -raw_gy;
	g_raw_gyro.y = -raw_gx;
	g_raw_gyro.z = -raw_gz;

	// Run prediction step (algorithm-specific)
#if FUSION_ALGO == 1
	fusion1_predict(&g_f11, g_raw_gyro.x, g_raw_gyro.y, g_raw_gyro.z, DT);
#elif FUSION_ALGO == 2
	fusion2_predict(&g_f11, g_raw_gyro.x, g_raw_gyro.y, g_raw_gyro.z, DT);
#elif FUSION_ALGO == 3
	fusion3_predict(&g_f11, g_raw_gyro.x, g_raw_gyro.y, g_raw_gyro.z, DT);
#endif

	// Extract Euler angles from quaternion (NED: roll=X, pitch=Y, yaw=Z)
	vector3d_t euler;
	quat_to_euler(&euler, &g_f11.q);
	g_angular_state.roll = euler.x * RAD2DEG;
	g_angular_state.pitch = euler.y * RAD2DEG;
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

	// Convert sensor frame to NED body frame (FRD: X=fwd, Y=right, Z=down)
	// Sensor Y (forward) → body X, Sensor X (right) → body Y, both negated
	float ax = -raw_ay;
	float ay = -raw_ax;
	float az = -raw_az;

	g_raw_accel.x = ax;
	g_raw_accel.y = ay;
	g_raw_accel.z = az;

#if FUSION_ALGO == 1
	fusion1_update(&g_f11, ax, ay, az, 1.0 / ACCEL_FREQ);
#elif FUSION_ALGO == 2
	fusion2_update(&g_f11, ax, ay, az, 1.0 / ACCEL_FREQ);
#elif FUSION_ALGO == 3
	fusion3_update(&g_f11, ax, ay, az, 1.0 / ACCEL_FREQ);
#endif

	memcpy(&g_linear_accel_out.body, &g_f11.v_linear_acc, sizeof(vector3d_t));
	memcpy(&g_linear_accel_out.earth, &g_f11.v_linear_acc_earth_frame, sizeof(vector3d_t));
	publish(SENSOR_LINEAR_ACCEL, (uint8_t*)&g_linear_accel_out, sizeof(linear_accel_data_t));
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	if (data[0] == LOG_CLASS_ATTITUDE || data[0] == LOG_CLASS_ATTITUDE_MAG || data[0] == LOG_CLASS_ATTITUDE_EARTH) {
		g_log_class = data[0];
	} else {
		g_log_class = LOG_CLASS_NONE;
	}
}

static void loop_logger(uint8_t *data, size_t size) {
	if (g_log_class == LOG_CLASS_NONE) return;

	float v1_x, v1_y, v1_z;
	float v2_x, v2_y, v2_z;
	float v3_x, v3_y, v3_z;

	if (g_log_class == LOG_CLASS_ATTITUDE_MAG) {
		// Mag debug: raw mag, earth mag, attitude vector
		v1_x = (float)g_mag_vec.x;
		v1_y = (float)g_mag_vec.y;
		v1_z = (float)g_mag_vec.z;

		v2_x = (float)g_mag_earth.x;
		v2_y = (float)g_mag_earth.y;
		v2_z = (float)g_mag_earth.z;

		v3_x = (float)g_f11.v_pred.x;
		v3_y = (float)g_f11.v_pred.y;
		v3_z = (float)g_f11.v_pred.z;
	} else if (g_log_class == LOG_CLASS_ATTITUDE_EARTH) {
		// Earth frame debug: v_pred, v_true, v_linear_acc_earth_frame
		v1_x = (float)g_f11.v_pred.x;
		v1_y = (float)g_f11.v_pred.y;
		v1_z = (float)g_f11.v_pred.z;

		v2_x = (float)g_f11.v_true.x;
		v2_y = (float)g_f11.v_true.y;
		v2_z = (float)g_f11.v_true.z;

		v3_x = (float)g_f11.v_linear_acc_earth_frame.x;
		v3_y = (float)g_f11.v_linear_acc_earth_frame.y;
		v3_z = (float)g_f11.v_linear_acc_earth_frame.z;
	} else {
		// Fusion debug: v_pred, v_true, v_linear_acc (body frame)
		v1_x = (float)g_f11.v_pred.x;
		v1_y = (float)g_f11.v_pred.y;
		v1_z = (float)g_f11.v_pred.z;

		v2_x = (float)g_f11.v_true.x;
		v2_y = (float)g_f11.v_true.y;
		v2_z = (float)g_f11.v_true.z;

		v3_x = (float)g_f11.v_linear_acc.x;
		v3_y = (float)g_f11.v_linear_acc.y;
		v3_z = (float)g_f11.v_linear_acc.z;
	}
	
	memcpy(&g_monitor_msg[0], &v1_x, sizeof(float));
	memcpy(&g_monitor_msg[4], &v1_y, sizeof(float));
	memcpy(&g_monitor_msg[8], &v1_z, sizeof(float));
	memcpy(&g_monitor_msg[12], &v2_x, sizeof(float));
	memcpy(&g_monitor_msg[16], &v2_y, sizeof(float));
	memcpy(&g_monitor_msg[20], &v2_z, sizeof(float));
	memcpy(&g_monitor_msg[24], &v3_x, sizeof(float));
	memcpy(&g_monitor_msg[28], &v3_y, sizeof(float));
	memcpy(&g_monitor_msg[32], &v3_z, sizeof(float));

	publish(SEND_LOG, g_monitor_msg, sizeof(g_monitor_msg));
}

void attitude_estimation_setup(void) {
#if FUSION_ALGO == 1
	// Initialize Fusion1 (Mahony filter)
	// Parameters: gain_acc_smooth, kp (Mahony P-gain), Ki, accel_scale
	// ATT_ACCEL_SMOOTH controls gravity vector smoothing stiffness
	fusion1_init(&g_f11, ATT_ACCEL_SMOOTH, ATT_MAHONY_KP, ATT_MAHONY_KI, MAX_IMU_ACCEL);
#elif FUSION_ALGO == 2
	// Initialize Fusion2 (7-State EKF)
	// Parameters: gyro_noise, bias_noise, accel_noise, accel_scale, lpf_gain
	// Bias noise set to 1e-4 to allow slow walk estimation
	fusion2_init(&g_f11, GYRO_NOISE, BIAS_NOISE, ACCEL_NOISE, MAX_IMU_ACCEL, ATT_ACCEL_SMOOTH);
#elif FUSION_ALGO == 3
	// Initialize Fusion3 (Madgwick with Bias)
	// Parameters: k0 (accel filter), beta (gain), zeta (bias gain), accel_scale
	fusion3_init(&g_f11, ATT_ACCEL_SMOOTH, ATT_F3_BETA, ATT_F3_ZETA, MAX_IMU_ACCEL);
#endif

	subscribe(SENSOR_IMU1_GYRO_FILTERED_UPDATE, gyro_update);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, accel_update);
	subscribe(SENSOR_COMPASS, mag_update);

	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	// 10 Hz: 36-byte payload (9 floats) exceeds 30-byte max at 25 Hz (9600 baud)
	subscribe(SCHEDULER_10HZ, loop_logger);
}

