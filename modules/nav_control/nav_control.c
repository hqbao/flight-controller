#include "nav_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define NAV_FREQ 1000
#define CTL_FREQ 100
#define MIN_LANDING_SPEED 50

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
    double dx;
    double dy;
    double z;
} optflow_t;

static state_t g_state = DISARMED;
static rc_att_ctl_t g_rc_att_ctl = {0};
static double g_downward_range = 0;
static double g_downward_range_prev = 0;

static pid_control_t g_pid_nav_x = {0};
static pid_control_t g_pid_nav_y = {0};
static pid_control_t g_pid_nav_z = {0};

static vector3d_t g_pos_final = {0, 0, 0};
static vector3d_t g_pos_target = {0, 0, 0};
static vector3d_t g_nav_veloc = {0, 0, 0};
static vector3d_t g_pos_bias = {0, 0, 0};

static uint8_t g_target_data[32] = {0};

static double g_yaw_veloc = 0;

static double nav_veloc_z_scale = 2.5;

int g_moving_state_roll = 0; // 0: Released, 1: Just control
int g_moving_state_pitch = 0;
int g_moving_state_alt = 0;

static double g_landing_speed = MIN_LANDING_SPEED;

static void move_in_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_att_ctl, data, sizeof(rc_att_ctl_t));
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	g_downward_range = (double)(*(int*)&data[8]);
	if (g_state == LANDING) {
		if (g_downward_range < 2000) {
			if (g_downward_range - g_downward_range_prev >= 0) { // Not moving down
				g_landing_speed += 1;
			} else if (g_downward_range - g_downward_range_prev < -20) { // Too high speed
				g_landing_speed -= 5;
			}
		}
		g_downward_range_prev = g_downward_range;
	}
}

static void pid_setup(void) {
	pid_control_init(&g_pid_nav_x);
	pid_control_set_p_gain(&g_pid_nav_x, 1.0);
	pid_control_set_d_gain(&g_pid_nav_x, 0.2);
	pid_control_set_i_gain(&g_pid_nav_x, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_x, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_nav_y);
	pid_control_set_p_gain(&g_pid_nav_y, 1.0);
	pid_control_set_d_gain(&g_pid_nav_y, 0.2);
	pid_control_set_i_gain(&g_pid_nav_y, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_y, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_nav_z);
	pid_control_set_p_gain(&g_pid_nav_z, 25);
	pid_control_set_d_gain(&g_pid_nav_z, 1);
	pid_control_set_i_gain(&g_pid_nav_z, 0, 1.0);
	pid_control_set_smooth(&g_pid_nav_z, 1.0, 1.0, 0.005);
}

static void nav_control_loop(void) {
	double dt = 1.0 / NAV_FREQ;
	pid_control_update(&g_pid_nav_x, g_pos_final.x, g_pos_target.x, dt);
	pid_control_update(&g_pid_nav_y, g_pos_final.y, g_pos_target.y, dt);
	pid_control_update(&g_pid_nav_z, g_pos_final.z, g_pos_target.z, dt);
}

static void position_update(uint8_t *data, size_t size) {
	memcpy(&g_pos_final, data, sizeof(vector3d_t));
	memcpy(&g_nav_veloc, &data[sizeof(vector3d_t)], sizeof(vector3d_t));
	nav_control_loop();
}

static void reset(void) {
	vector3d_set(&g_pos_target, &g_pos_final);
	vector3d_init(&g_pos_bias, 0, 0, 0);
	pid_control_reset(&g_pid_nav_x, g_pos_final.x);
	pid_control_reset(&g_pid_nav_y, g_pos_final.y);
	pid_control_reset(&g_pid_nav_z, g_pos_final.z);
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY || g_state == TAKING_OFF) {
		reset();
	}
}

static void loop_1khz(uint8_t *data, size_t size) {
	double nav_roll 	= g_pid_nav_y.output;
	double nav_pitch 	= g_pid_nav_x.output;
	double nav_yaw 		= g_yaw_veloc;
	double nav_alt 		= g_pid_nav_z.output + g_nav_veloc.z * nav_veloc_z_scale;

	memcpy(&g_target_data[0], 	&nav_roll, 8);
	memcpy(&g_target_data[8],	&nav_pitch, 8);
	memcpy(&g_target_data[16], 	&nav_yaw, 8);
	memcpy(&g_target_data[24], 	&nav_alt, 8);
	publish(COMMAND_SET_TARGET_ORIENTATION, (uint8_t*)g_target_data, sizeof(double) * 4);
}

static void loop_100hz(uint8_t *data, size_t size) {
	if (fabs(g_rc_att_ctl.pitch) > 0.1) {
		if (g_moving_state_pitch == 0) {
			g_pos_bias.x = g_pos_target.x - g_pos_final.x;
			g_moving_state_pitch = CTL_FREQ;
		}
		g_pos_target.x = g_pos_final.x + g_pos_bias.x + g_rc_att_ctl.pitch * 1.0;
	} else if (g_moving_state_pitch == CTL_FREQ) {
		g_pos_target.x = g_pos_final.x + g_pos_bias.x;
		g_moving_state_pitch = 0;
	}

	if (fabs(g_rc_att_ctl.roll) > 0.1) {
		if (g_moving_state_roll == 0) {
			g_pos_bias.y = g_pos_target.y - g_pos_final.y;
			g_moving_state_roll = CTL_FREQ;
		}
		g_pos_target.y = g_pos_final.y + g_pos_bias.y + g_rc_att_ctl.roll * 1.0;
	} else if (g_moving_state_roll == CTL_FREQ) {
		g_pos_target.y = g_pos_final.y + g_pos_bias.y;
		g_moving_state_roll = 0;
	}

	if (g_state == LANDING) {
		g_pos_target.z = g_pos_final.z + g_pos_bias.z - g_landing_speed;
	} else {
		if (fabs(g_rc_att_ctl.alt) > 0.1) {
			if (g_moving_state_alt == 0) {
				g_pos_bias.z = g_pos_target.z - g_pos_final.z;
				g_moving_state_alt = CTL_FREQ;
			}
			g_pos_target.z = g_pos_final.z + g_pos_bias.z + g_rc_att_ctl.alt * 3.0;
		} else if (g_moving_state_alt > 0) {
			g_pos_target.z = g_pos_final.z + g_pos_bias.z;
			g_moving_state_alt -= 1;
		}
	}

	if (fabs(g_rc_att_ctl.yaw) > 0.1) {
		g_yaw_veloc = g_rc_att_ctl.yaw * (-0.5);
	} else {
		g_yaw_veloc = 0;
	}
}

void nav_control_setup(void) {
	pid_setup();
	subscribe(NAV_POSITION_UPDATE, position_update);
	subscribe(STATE_DETECTION_UPDATE, state_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SCHEDULER_1KHZ, loop_1khz);
	subscribe(SCHEDULER_100HZ, loop_100hz);
}
