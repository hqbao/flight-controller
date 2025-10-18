#include "nav_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>
#include <pid_control.h>
#include <string.h>
#include <math.h>
#include <macro.h>

#define IMU_FREQ 4000
#define NAV_FREQ 1000
#define MAX_IMU_ACCEL 16384

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
    double dx;
    double dy;
    double z;
} optflow_t;

typedef struct {
	uint8_t state;
	uint8_t mode;
} rc_state_ctl_t;

static rc_state_ctl_t g_rc_state_ctl;
static rc_state_ctl_t g_rc_state_ctl_prev;
static state_t g_state = DISARMED;
static double g_air_pressure_alt_raw = 0;
static double g_air_pressure_alt = 0;
static double g_alt = 0;
static double g_alt_prev = 0;
static double g_alt_d = 0;
static optflow_t g_optflow = {0, 0, 0};
static vector3d_t g_linear_accel = {0, 0, 0};
static vector3d_t g_linear_veloc = {0, 0, 0};
static vector3d_t g_linear_veloc_bias = {0, 0, 0};
static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_true = {0, 0, 0};
static vector3d_t g_pos = {0, 0, 0};
static vector3d_t g_pos_bias = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

static double coef1 = 1.25;
static double coef2 = 20;
static double coef3 = 0.01;
static double coef41 = 0.01;
static double coef421 = 0.01;
static double coef422 = 0.005;
static double coef43 = 100;
static double coef51 = 0.05;
static double coef52 = 0.005;
static double scale1 = 0.2;
static double threshold1 = 300;

static void state_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_state_ctl, data, size);
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	g_optflow.dx = (double)(*(int*)&data[4]) * coef3;
	g_optflow.dy = (double)(*(int*)&data[0]) * coef3;
	g_optflow.z = (double)(*(int*)&data[8]);

	g_pos_true.x += g_optflow.dx;
	g_pos_true.y += g_optflow.dy;

	g_linear_veloc.x += coef41 * (g_optflow.dx - g_linear_veloc.x);
	g_linear_veloc.y += coef41 * (g_optflow.dy - g_linear_veloc.y);

	if (g_rc_state_ctl.mode == 1) {
		g_alt = g_optflow.z;
		if (g_rc_state_ctl_prev.mode != 1) {
			g_alt_prev = g_alt;
		}

		g_alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_linear_veloc.z += coef421 * (g_alt_d * coef43 - g_linear_veloc.z);
		g_pos_true.z += g_alt_d;

		g_rc_state_ctl_prev.mode = g_rc_state_ctl.mode;
	}
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt_raw = *(double*)data;
	if (fabs(g_linear_accel.z) > threshold1) {
		g_air_pressure_alt += coef51 * (g_air_pressure_alt_raw - g_air_pressure_alt);
	} else {
		g_air_pressure_alt += coef52 * (g_air_pressure_alt_raw - g_air_pressure_alt);
	}

	if (g_rc_state_ctl.mode == 0) {
		g_alt = g_air_pressure_alt;
		if (g_rc_state_ctl_prev.mode != 0) {
			g_alt_prev = g_alt;
		}

		g_alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_linear_veloc.z += coef422 * (g_alt_d * coef43 - g_linear_veloc.z);
		g_pos_true.z += g_alt_d;

		g_rc_state_ctl_prev.mode = g_rc_state_ctl.mode;
	}
}

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	g_linear_accel.x = v.x * MAX_IMU_ACCEL;
	g_linear_accel.y = v.y * MAX_IMU_ACCEL;
	g_linear_accel.z = v.z * MAX_IMU_ACCEL;

	g_linear_veloc.x += 1.0 / IMU_FREQ * g_linear_accel.x;
	g_linear_veloc.y += 1.0 / IMU_FREQ * g_linear_accel.y;
	g_linear_veloc.z += 1.0 / IMU_FREQ * g_linear_accel.z;

	g_pos.x += coef1 / IMU_FREQ * g_linear_veloc.x;
	g_pos.y += coef1 / IMU_FREQ * g_linear_veloc.y;
	//g_pos.z += coef1 / IMU_FREQ * g_linear_veloc.z;

	g_pos.x += coef2 / IMU_FREQ * (g_pos_true.x - g_pos.x);
	g_pos.y += coef2 / IMU_FREQ * (g_pos_true.y - g_pos.y);
	g_pos.z += coef2 / IMU_FREQ * (g_pos_true.z - g_pos.z);
}

static void loop_1khz(uint8_t *data, size_t size) {
	vector3d_sub(&g_pos_final, &g_pos, &g_pos_bias);
	vector3d_scale(&g_pos_final, &g_pos_final, scale1);
	g_pos_final.x = -g_pos_final.x;
	g_pos_final.y = -g_pos_final.y;

	vector3d_sub(&g_linear_veloc_final, &g_linear_veloc, &g_linear_veloc_bias);
	g_linear_veloc_final.x = -g_linear_veloc_final.x;
	g_linear_veloc_final.y = -g_linear_veloc_final.y;

	static uint8_t g_nav_pos_msg[sizeof(vector3d_t) * 2] = {0};
	memcpy(g_nav_pos_msg, &g_pos_final, sizeof(vector3d_t));
	memcpy(&g_nav_pos_msg[sizeof(vector3d_t)], &g_linear_veloc_final, sizeof(vector3d_t));

	publish(NAV_POSITION_UPDATE, (uint8_t*)&g_nav_pos_msg, sizeof(vector3d_t) * 2);

	int number1 = g_alt_d * coef43;
	int number2 = g_alt_d * coef43;
	int number3 = g_linear_veloc.z;
	static uint8_t g_msg[16] = {0};
	memcpy(&g_msg[0], &number1, 4);
	memcpy(&g_msg[4], &number2, 4);
	memcpy(&g_msg[8], &number3, 4);
	publish(MONITOR_DATA, (uint8_t*)g_msg, 12);
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY || g_state == TAKING_OFF) {
		vector3d_set(&g_pos_bias, &g_pos);
		vector3d_set(&g_linear_veloc_bias, &g_linear_veloc);
	}
}

void nav_fusion_setup(void) {
	subscribe(SCHEDULER_1KHZ, loop_1khz);
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(STATE_DETECTION_UPDATE, state_update);
	subscribe(COMMAND_SET_STATE, state_control_update);
}

