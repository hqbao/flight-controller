#include "attitude_estimation.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <fusion2.h>

/* Macro to enable/disable sending MONITOR_DATA via logger */
#define ENABLE_ATTITUDE_MONITOR_LOG 0

#define MAX_IMU_ACCEL 16384
#define GYRO_FREQ 1000
#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131
#define DT (1.0 / GYRO_FREQ)
#define GYRO_NOISE 0.0001
#define ACCEL_NOISE 100.0

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

static fusion2_t g_f11;
static angle3d_t g_angular_state = {0, 0, 0};
static vector3d_t g_raw_accel = {0, 0, 0};

static void gyro_update(uint8_t *data, size_t size) {
    float raw_gx, raw_gy, raw_gz;
    memcpy(&raw_gx, &data[0], sizeof(float));
    memcpy(&raw_gy, &data[4], sizeof(float));
    memcpy(&raw_gz, &data[8], sizeof(float));

	float gx = -raw_gx;
	float gy = -raw_gy;
	float gz = raw_gz;

	fusion2_predict(&g_f11, gx, gy, gz, DT);

	// Extract Euler angles from quaternion (in radians)
	vector3d_t euler;
	quat_to_euler(&euler, &g_f11.q);
	g_angular_state.roll = -euler.y * RAD2DEG;
	g_angular_state.pitch = euler.x * RAD2DEG;
	g_angular_state.yaw = euler.z * RAD2DEG;

	publish(ANGULAR_STATE_UPDATE, (uint8_t*)&g_angular_state, sizeof(angle3d_t));
	publish(SENSOR_ATTITUDE_VECTOR, (uint8_t*)&g_f11.pred_norm_accel, sizeof(vector3d_t));
}

static void accel_update(uint8_t *data, size_t size) {
    float raw_ax, raw_ay, raw_az;
    memcpy(&raw_ax, &data[0], sizeof(float));
    memcpy(&raw_ay, &data[4], sizeof(float));
    memcpy(&raw_az, &data[8], sizeof(float));

	float ax = -raw_ax;
	float ay = -raw_ay;
	float az = raw_az;

	g_raw_accel.x = ax;
	g_raw_accel.y = ay;
	g_raw_accel.z = az;

	fusion2_update(&g_f11, ax, ay, az);
	
	publish(SENSOR_LINEAR_ACCEL, (uint8_t*)&g_f11.v_linear_acc, sizeof(vector3d_t));
}

#if ENABLE_ATTITUDE_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	/* Pack pred_norm_accel and true_norm_accel into MONITOR_DATA message
	   Format: 6 float32 values (pred.x, pred.y, pred.z, true.x, true.y, true.z) */
	static uint8_t out_msg[24]; /* 6 * 4 bytes (float32) */
	
	// Cast double to float before packing
	float pred_x = (float)g_f11.pred_norm_accel.x;
	float pred_y = (float)g_f11.pred_norm_accel.y;
	float pred_z = (float)g_f11.pred_norm_accel.z;
	float raw_x = (float)(g_raw_accel.x / MAX_IMU_ACCEL);
	float raw_y = (float)(g_raw_accel.y / MAX_IMU_ACCEL);
	float raw_z = (float)(g_raw_accel.z / MAX_IMU_ACCEL);
	
	memcpy(&out_msg[0], &pred_x, sizeof(float));
	memcpy(&out_msg[4], &pred_y, sizeof(float));
	memcpy(&out_msg[8], &pred_z, sizeof(float));
	memcpy(&out_msg[12], &raw_x, sizeof(float));
	memcpy(&out_msg[16], &raw_y, sizeof(float));
	memcpy(&out_msg[20], &raw_z, sizeof(float));
	
	publish(MONITOR_DATA, out_msg, sizeof(out_msg));
}
#endif

static void init(void) {
	// Initialize with gyro_noise, accel_noise, and accel_scale
	// gyro_noise: process noise, accel_noise: measurement noise, accel_scale: 1g in sensor units
	fusion2_init(&g_f11, GYRO_NOISE, ACCEL_NOISE, MAX_IMU_ACCEL);
	//g_f11.no_correction = 1;
}

void attitude_estimation_setup(void) {
	init();
	subscribe(SENSOR_IMU1_GYRO_UPDATE, gyro_update);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, accel_update);
#if ENABLE_ATTITUDE_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}

