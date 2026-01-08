#include "nav_fusion.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>
#include <pid_control.h>
#include <string.h>
#include <math.h>
#include <macro.h>

/* Macro to enable/disable sending MONITOR_DATA via logger 
 * 0: Disable
 * 1: Position
 * 2: Velocity
 */
#define ENABLE_NAV_FUSION_MONITOR_LOG 0

#define ACCEL_FREQ 500
#define MAX_IMU_ACCEL 16384

typedef struct {
    double dx;
    double dy;
    double z;
} optflow_t;

typedef enum {
	OPTFLOW_DOWNWARD = 0,
	OPTFLOW_UPWARD = 1,
} optflow_direction_t;

typedef struct {
	uint8_t state;
	uint8_t mode;
} rc_state_ctl_t;

static rc_state_ctl_t g_rc_state_ctl;
static rc_state_ctl_t g_rc_state_ctl_prev;
static double g_air_pressure_alt_raw = 0;
static double g_air_pressure_alt = 0;
static double g_alt = 0;
static double g_alt_prev = 0;
static double g_alt_d = 0;
static optflow_t g_optflow_down = {0, 0, 0};
static optflow_t g_optflow_up = {0, 0, 0};
static optflow_t g_optflow = {0, 0, 0};
static vector3d_t g_linear_accel = {0, 0, 0};
static vector3d_t g_linear_veloc = {0, 0, 0};
static vector3d_t g_linear_veloc_est = {0, 0, 0};
static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_est2 = {0, 0, 0};
static vector3d_t g_pos_est2_prev = {0, 0, 0};
static vector3d_t g_pos_true = {0, 0, 0};
static vector3d_t g_pos_est1 = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

/* Tuning Parameters */
#define POS_XY_CORRECTION_GAIN      20.0
#define POS_Z_CORRECTION_GAIN       20.0
#define POS_XY_INTEGRATION_GAIN     0.05
#define POS_Z_INTEGRATION_GAIN      1.0
#define POS_XY_TRUE_CORRECTION_GAIN 1.0
#define POS_Z_TRUE_CORRECTION_GAIN  1.0
#define OPTFLOW_UNIT_SCALE          0.02
#define ALT_DERIVATIVE_SCALE    100.0
#define BARO_ALPHA_HIGH_ACCEL   0.05
#define BARO_ALPHA_LOW_ACCEL    0.005
#define OUTPUT_POS_SCALE        1.0
#define OUTPUT_VELOC_SCALE      1.0
#define ACCEL_Z_THRESHOLD       300.0

static void state_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_state_ctl, data, size);
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	int32_t raw_dx, raw_dy, raw_z;

	if (data[1] == OPTFLOW_DOWNWARD) {
		memcpy(&raw_dx, &data[4], sizeof(int32_t));
		memcpy(&raw_dy, &data[8], sizeof(int32_t));
		memcpy(&raw_z, &data[12], sizeof(int32_t));

		g_optflow_down.dx = (double)raw_dx * OPTFLOW_UNIT_SCALE;
		g_optflow_down.dy = (double)raw_dy * OPTFLOW_UNIT_SCALE;
		g_optflow.dx = g_optflow_down.dx;
		g_optflow.dy = g_optflow_down.dy;
		g_optflow.z  = (double)raw_z;
	} else if (data[1] == OPTFLOW_UPWARD) {
		memcpy(&raw_dx, &data[4], sizeof(int32_t));
		memcpy(&raw_dy, &data[8], sizeof(int32_t));

		g_optflow_up.dx = (double)raw_dx * OPTFLOW_UNIT_SCALE;
		g_optflow_up.dy = (double)raw_dy * OPTFLOW_UNIT_SCALE;
		g_optflow.dx = g_optflow_up.dx;
		g_optflow.dy = -g_optflow_up.dy;
	}

	g_pos_true.x += g_optflow.dx;
	g_pos_true.y += g_optflow.dy;

	if (g_rc_state_ctl.mode == 0 && data[1] == OPTFLOW_DOWNWARD) {
		g_alt = g_optflow.z;
		if (g_rc_state_ctl_prev.mode != 0) {
			g_alt_prev = g_alt;
		}

		g_alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_pos_true.z += g_alt_d;

		g_rc_state_ctl_prev.mode = g_rc_state_ctl.mode;
	}
}

static void air_pressure_update(uint8_t *data, size_t size) {
	g_air_pressure_alt_raw = *(double*)data;
	if (fabs(g_linear_accel.z) > ACCEL_Z_THRESHOLD) {
		g_air_pressure_alt += BARO_ALPHA_HIGH_ACCEL * (g_air_pressure_alt_raw - g_air_pressure_alt);
	} else {
		g_air_pressure_alt += BARO_ALPHA_LOW_ACCEL * (g_air_pressure_alt_raw - g_air_pressure_alt);
	}

	if (g_rc_state_ctl.mode >= 1) {
		g_alt = g_air_pressure_alt;
		if (g_rc_state_ctl_prev.mode < 1) {
			g_alt_prev = g_alt;
		}

		g_alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_pos_true.z += g_alt_d;

		g_rc_state_ctl_prev.mode = g_rc_state_ctl.mode;
	}
}

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	g_linear_accel.x = v.x * MAX_IMU_ACCEL;
	g_linear_accel.y = v.y * MAX_IMU_ACCEL;
	g_linear_accel.z = v.z * MAX_IMU_ACCEL;

	g_linear_veloc.x += 1.0 / ACCEL_FREQ * g_linear_accel.x;
	g_linear_veloc.y += 1.0 / ACCEL_FREQ * g_linear_accel.y;
	g_linear_veloc.z += 1.0 / ACCEL_FREQ * g_linear_accel.z;

	g_pos_est2.x += POS_XY_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc.x;
	g_pos_est2.y += POS_XY_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc.y;
	g_pos_est2.z += POS_Z_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc.z;
	
	g_pos_est2.x += POS_XY_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.x - g_pos_est2.x);
	g_pos_est2.y += POS_XY_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.y - g_pos_est2.y);
	g_pos_est2.z += POS_Z_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.z - g_pos_est2.z);

	g_linear_veloc_est.x = (g_pos_est2.x - g_pos_est2_prev.x) * ACCEL_FREQ;
	g_linear_veloc_est.y = (g_pos_est2.y - g_pos_est2_prev.y) * ACCEL_FREQ;
	g_linear_veloc_est.z = (g_pos_est2.z - g_pos_est2_prev.z) * ACCEL_FREQ;
	g_pos_est2_prev.x = g_pos_est2.x;
	g_pos_est2_prev.y = g_pos_est2.y;
	g_pos_est2_prev.z = g_pos_est2.z;

	g_pos_est1.x += POS_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.x - g_pos_est1.x);
	g_pos_est1.y += POS_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.y - g_pos_est1.y);
	g_pos_est1.z += POS_Z_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.z - g_pos_est1.z);

	vector3d_scale(&g_pos_final, &g_pos_est1, OUTPUT_POS_SCALE);
	g_pos_final.x = -g_pos_final.x;
	g_pos_final.y = -g_pos_final.y;

	vector3d_scale(&g_linear_veloc_final, &g_linear_veloc_est, OUTPUT_VELOC_SCALE);
	g_linear_veloc_final.x = -g_linear_veloc_final.x;
	g_linear_veloc_final.y = -g_linear_veloc_final.y;

	static uint8_t g_nav_pos_msg[sizeof(vector3d_t) * 2] = {0};
	memcpy(g_nav_pos_msg, &g_pos_final, sizeof(vector3d_t));
	memcpy(&g_nav_pos_msg[sizeof(vector3d_t)], &g_linear_veloc_final, sizeof(vector3d_t));

	publish(NAV_POSITION_UPDATE, (uint8_t*)&g_nav_pos_msg, sizeof(vector3d_t) * 2);
}

#if ENABLE_NAV_FUSION_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	static uint8_t g_msg[12] = {0};
#if ENABLE_NAV_FUSION_MONITOR_LOG == 1
	float val[3] = {(float)g_pos_est1.x, (float)g_pos_est1.y, (float)g_pos_est1.z};
#elif ENABLE_NAV_FUSION_MONITOR_LOG == 2
	float val[3] = {(float)g_linear_veloc_est.x, (float)g_linear_veloc_est.y, (float)g_linear_veloc_est.z};
#endif
	memcpy(g_msg, val, 12);
	publish(MONITOR_DATA, (uint8_t*)g_msg, 12);
}
#endif

void nav_fusion_setup(void) {
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(COMMAND_SET_STATE, state_control_update);
#if ENABLE_NAV_FUSION_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
