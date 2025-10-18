#include "attitude_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <fusion1.h>

#define MAX_IMU_ACCEL 16384
#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131
#define IMU_FREQ 4000

#define ACCEL_OFFSET_X 0
#define ACCEL_OFFSET_Y 0
#define ACCEL_OFFSET_Z 0

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

static fusion1_t g_f11;
static vector3d_t g_imu1_gyro = {0, 0, 0};
static vector3d_t g_imu1_accel = {0, 0, MAX_IMU_ACCEL};

static fusion1_t g_f12;
static vector3d_t g_imu2_gyro = {0, 0, 0};
static vector3d_t g_imu2_accel = {0, 0, MAX_IMU_ACCEL};

static fusion1_t g_f13;
static vector3d_t g_imu3_gyro = {0, 0, 0};
static vector3d_t g_imu3_accel = {0, 0, MAX_IMU_ACCEL};

static angle3d_t g_angular_state = {0, 0, 0};

static void gyro_update(uint8_t *data, size_t size) {
	float gx = -(*(float*)&data[0]);
	float gy = -(*(float*)&data[4]);
	float gz = (*(float*)&data[8]);
	double dt = 1.0 / IMU_FREQ;

	vector3d_init(&g_imu1_gyro, gx, gy, gz);
	fusion1_predict(&g_f11,
			dt * g_imu1_gyro.x * DEG2RAD,
			dt * g_imu1_gyro.y * DEG2RAD,
			dt * g_imu1_gyro.z * DEG2RAD);

	vector3d_init(&g_imu2_gyro, gx, gz, -gy);
	fusion1_predict(&g_f12,
			dt * g_imu2_gyro.x * DEG2RAD,
			dt * g_imu2_gyro.y * DEG2RAD,
			dt * g_imu2_gyro.z * DEG2RAD);

	vector3d_init(&g_imu3_gyro, gz, gy, -gx);
	fusion1_predict(&g_f13,
			dt * g_imu3_gyro.x * DEG2RAD,
			dt * g_imu3_gyro.y * DEG2RAD,
			dt * g_imu3_gyro.z * DEG2RAD);

	g_angular_state.roll = asin(g_f11.v_pred.y) * RAD2DEG;
	g_angular_state.pitch = asin(g_f11.v_pred.x) * RAD2DEG;
	g_angular_state.yaw += g_imu1_gyro.z * dt;

	publish(SENSOR_ATTITUDE_ANGLE, (uint8_t*)&g_angular_state, sizeof(angle3d_t));
}

static void accel_update(uint8_t *data, size_t size) {
	float ax = -(*(float*)&data[4]) - ACCEL_OFFSET_X;
	float ay = -(*(float*)&data[0]) - ACCEL_OFFSET_Y;
	float az = (*(float*)&data[8]) - ACCEL_OFFSET_Z;

	vector3d_init(&g_imu1_accel, ax, ay, az);
	fusion1_update(&g_f11, g_imu1_accel.x, g_imu1_accel.y, g_imu1_accel.z);

	vector3d_init(&g_imu2_accel, az, ay, -ax);
	fusion1_update(&g_f12, g_imu2_accel.x, g_imu2_accel.y, g_imu2_accel.z);

	vector3d_init(&g_imu3_accel, ax, az, -ay);
	fusion1_update(&g_f13, g_imu3_accel.x, g_imu3_accel.y, g_imu3_accel.z);

	publish(SENSOR_LINEAR_ACCEL, (uint8_t*)&g_f11.v_linear_acc, sizeof(vector3d_t));
}

static void init(void) {
	fusion1_init(&g_f11, 4.0, 0.5, IMU_FREQ);
	g_f11.accel_scale = MAX_IMU_ACCEL;
	//g_f11.no_correction = 1;

	fusion1_init(&g_f12, 4.0, 0.5, IMU_FREQ);
	fusion1_predict(&g_f12, M_PI_2, 0, 0);
	//g_f12.no_correction = 1;

	fusion1_init(&g_f13, 4.0, 0.5, IMU_FREQ);
	fusion1_predict(&g_f13, 0, -M_PI_2, 0);
	//g_f13.no_correction = 1;
}

void attitude_fusion_setup(void) {
	init();
	subscribe(SENSOR_IMU1_GYRO_UPDATE, gyro_update);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, accel_update);
}

