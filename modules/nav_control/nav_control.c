#include "nav_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define NAV_FREQ 500
#define CTL_FREQ 100
#define MIN_LANDING_SPEED 50

/* PID Gains */
// Navigation XY (Position)
#define NAV_XY_P 2.0
#define NAV_XY_I 0.0
#define NAV_XY_D 0.2
#define NAV_XY_I_LIMIT 1.0
#define NAV_XY_O_LIMIT 45.0

// Navigation Z (Altitude)
#define NAV_Z_P 20.0
#define NAV_Z_I 0.0
#define NAV_Z_D 10.0
#define NAV_Z_I_LIMIT 1.0
#define NAV_Z_GAIN_TIME 1.0
#define NAV_XY_GAIN_TIME 1.0

// Smoothing
#define NAV_SMOOTH_INPUT 1.0
#define NAV_SMOOTH_P_TERM 1.0
#define NAV_SMOOTH_OUTPUT 1.0
#define NAV_Z_SMOOTH_INPUT 0.005
#define NAV_Z_SMOOTH_OUTPUT 0.005

/* Landing Control */
#define LANDING_RANGE_THRESHOLD 2000.0
#define LANDING_SPEED_INC 1.0
#define LANDING_SPEED_DEC 5.0
#define LANDING_DESCENT_RATE_THRESHOLD -20.0

/* RC Control */
#define RC_DEADBAND 0.1
#define RC_XY_SCALE 0.5
#define RC_Z_SCALE 6.0
#define RC_YAW_SCALE -0.5

/* Velocity Scaling */
#define NAV_VELOC_Z_SCALE 1.5

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

int g_moving_state_roll = 0; // 0: Released, 1: Just control
int g_moving_state_pitch = 0;
int g_moving_state_alt = 0;

static double g_landing_speed = MIN_LANDING_SPEED;

static void move_in_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_att_ctl, data, sizeof(rc_att_ctl_t));
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (data[1] == 0) { // Downward
		g_downward_range = (double)(*(int*)&data[12]);
		if (g_state == LANDING) {
			if (g_downward_range < LANDING_RANGE_THRESHOLD) {
				if (g_downward_range - g_downward_range_prev >= 0) { // Not moving down
					g_landing_speed += LANDING_SPEED_INC;
				} else if (g_downward_range - g_downward_range_prev < LANDING_DESCENT_RATE_THRESHOLD) { // Too high speed
					g_landing_speed -= LANDING_SPEED_DEC;
				}
			}
			g_downward_range_prev = g_downward_range;
		}
	}
}

static void pid_setup(void) {
	pid_control_init(&g_pid_nav_x);
	pid_control_set_p_gain(&g_pid_nav_x, NAV_XY_P);
	pid_control_set_d_gain(&g_pid_nav_x, NAV_XY_D);
	pid_control_set_i_gain(&g_pid_nav_x, NAV_XY_I, NAV_XY_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_nav_x, NAV_XY_I_LIMIT);
	pid_control_set_smooth(&g_pid_nav_x, NAV_SMOOTH_INPUT, NAV_SMOOTH_P_TERM, NAV_SMOOTH_OUTPUT);
	pid_control_set_o_limit(&g_pid_nav_x, NAV_XY_O_LIMIT);

	pid_control_init(&g_pid_nav_y);
	pid_control_set_p_gain(&g_pid_nav_y, NAV_XY_P);
	pid_control_set_d_gain(&g_pid_nav_y, NAV_XY_D);
	pid_control_set_i_gain(&g_pid_nav_y, NAV_XY_I, NAV_XY_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_nav_y, NAV_XY_I_LIMIT);
	pid_control_set_smooth(&g_pid_nav_y, NAV_SMOOTH_INPUT, NAV_SMOOTH_P_TERM, NAV_SMOOTH_OUTPUT);
	pid_control_set_o_limit(&g_pid_nav_y, NAV_XY_O_LIMIT);

	pid_control_init(&g_pid_nav_z);
	pid_control_set_p_gain(&g_pid_nav_z, NAV_Z_P);
	pid_control_set_d_gain(&g_pid_nav_z, NAV_Z_D);
	pid_control_set_i_gain(&g_pid_nav_z, NAV_Z_I, NAV_Z_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_nav_z, NAV_Z_I_LIMIT);
	pid_control_set_smooth(&g_pid_nav_z, NAV_Z_SMOOTH_INPUT, NAV_SMOOTH_P_TERM, NAV_Z_SMOOTH_OUTPUT);
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

static void loop_nav_publish(uint8_t *data, size_t size) {
	double nav_roll 	= g_pid_nav_y.output;
	double nav_pitch 	= g_pid_nav_x.output;
	double nav_yaw 		= g_yaw_veloc;
	double nav_alt 		= g_pid_nav_z.output + g_nav_veloc.z * NAV_VELOC_Z_SCALE;

	memcpy(&g_target_data[0], 	&nav_roll, 8);
	memcpy(&g_target_data[8],	&nav_pitch, 8);
	memcpy(&g_target_data[16], 	&nav_yaw, 8);
	memcpy(&g_target_data[24], 	&nav_alt, 8);
	publish(COMMAND_SET_TARGET_ORIENTATION, (uint8_t*)g_target_data, sizeof(double) * 4);
}

static void loop_100hz(uint8_t *data, size_t size) {
	if (fabs(g_rc_att_ctl.pitch) > RC_DEADBAND) {
		if (g_moving_state_pitch == 0) {
			g_pos_bias.x = g_pos_target.x - g_pos_final.x;
			g_moving_state_pitch = CTL_FREQ;
		}
		g_pos_target.x = g_pos_final.x + g_pos_bias.x + g_rc_att_ctl.pitch * RC_XY_SCALE;
	} else if (g_moving_state_pitch == CTL_FREQ) {
		g_pos_target.x = g_pos_final.x + g_pos_bias.x;
		g_moving_state_pitch = 0;
	}

	if (fabs(g_rc_att_ctl.roll) > RC_DEADBAND) {
		if (g_moving_state_roll == 0) {
			g_pos_bias.y = g_pos_target.y - g_pos_final.y;
			g_moving_state_roll = CTL_FREQ;
		}
		g_pos_target.y = g_pos_final.y + g_pos_bias.y + g_rc_att_ctl.roll * RC_XY_SCALE;
	} else if (g_moving_state_roll == CTL_FREQ) {
		g_pos_target.y = g_pos_final.y + g_pos_bias.y;
		g_moving_state_roll = 0;
	}

	if (g_state == LANDING) {
		g_pos_target.z = g_pos_final.z + g_pos_bias.z - g_landing_speed;
	} else {
		if (fabs(g_rc_att_ctl.alt) > RC_DEADBAND) {
			if (g_moving_state_alt == 0) {
				g_pos_bias.z = g_pos_target.z - g_pos_final.z;
				g_moving_state_alt = CTL_FREQ;
			}
			g_pos_target.z = g_pos_final.z + g_pos_bias.z + g_rc_att_ctl.alt * RC_Z_SCALE;
		} else if (g_moving_state_alt > 0) {
			g_pos_target.z = g_pos_final.z + g_pos_bias.z;
			g_moving_state_alt -= 1;
		}
	}

	if (fabs(g_rc_att_ctl.yaw) > RC_DEADBAND) {
		g_yaw_veloc = g_rc_att_ctl.yaw * RC_YAW_SCALE;
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
	subscribe(SCHEDULER_500HZ, loop_nav_publish);
	subscribe(SCHEDULER_100HZ, loop_100hz);
}
