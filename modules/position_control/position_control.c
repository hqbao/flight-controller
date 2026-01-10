#include "position_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define POS_CTL_FREQ 500
#define CTL_FREQ 100
#define MIN_LANDING_SPEED 50

/* PID Gains */
// Position Control XY
#define POS_CTL_XY_P 0.2
#define POS_CTL_XY_I 0.0
#define POS_CTL_XY_D 0.0
#define POS_CTL_XY_GAIN_TIME 1.0
#define POS_CTL_XY_I_LIMIT 5.0

// Position Control Z (Altitude)
#define POS_CTL_Z_P 4.0
#define POS_CTL_Z_I 0.0
#define POS_CTL_Z_D 2.0
#define POS_CTL_Z_GAIN_TIME 1.0
#define POS_CTL_Z_I_LIMIT 500.0

// Smoothing
#define POS_CTL_XY_SMOOTH_INPUT 1.0
#define POS_CTL_XY_SMOOTH_P_TERM 1.0
#define POS_CTL_XY_SMOOTH_OUTPUT 1.0
#define POS_CTL_Z_SMOOTH_INPUT 0.005
#define POS_CTL_Z_SMOOTH_P_TERM 1.0
#define POS_CTL_Z_SMOOTH_OUTPUT 0.005

/* Landing Control */
#define LANDING_RANGE_THRESHOLD 2000.0
#define LANDING_SPEED_INC 1.0
#define LANDING_SPEED_DEC 5.0
#define LANDING_DESCENT_RATE_THRESHOLD -20.0

/* RC Control */
#define RC_DEADBAND 0.1
#define RC_XY_SCALE 5.0
#define RC_Z_SCALE 20.0
#define RC_YAW_SCALE -0.5

/* Velocity Scaling */
#define POS_CTL_VELOC_X_SCALE 1.2
#define POS_CTL_VELOC_Y_SCALE 1.2
#define POS_CTL_VELOC_Z_SCALE 2.0

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

typedef struct {
    double dx;
    double dy;
    double z;
} optflow_t;

static state_t g_state = DISARMED;
static rc_state_ctl_t g_rc_state_ctl = {0};
static rc_att_ctl_t g_rc_att_ctl = {0};
static double g_downward_range = 0;
static double g_downward_range_prev = 0;

static pid_control_t g_pid_pos_x = {0};
static pid_control_t g_pid_pos_y = {0};
static pid_control_t g_pid_pos_z = {0};

static vector3d_t g_pos_final = {0, 0, 0};
static vector3d_t g_pos_target = {0, 0, 0};
static vector3d_t g_pos_offset = {0, 0, 0};
static vector3d_t g_veloc_final = {0, 0, 0};
static vector3d_t g_veloc_offset = {0, 0, 0};

static uint8_t g_target_data[40] = {0};

static double g_yaw_veloc = 0;
static double g_take_off_speed = 0;

static double g_pos_ctl_roll = 0;
static double g_pos_ctl_pitch = 0;
static double g_pos_ctl_yaw = 0;
static double g_pos_ctl_alt = 0;

static int g_moving_state_roll = 0; // 0: Released, 1: Just control
static int g_moving_state_pitch = 0;
static int g_moving_state_alt = 0;

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
	pid_control_init(&g_pid_pos_x);
	pid_control_set_p_gain(&g_pid_pos_x, POS_CTL_XY_P);
	pid_control_set_d_gain(&g_pid_pos_x, POS_CTL_XY_D);
	pid_control_set_i_gain(&g_pid_pos_x, POS_CTL_XY_I, POS_CTL_XY_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_pos_x, POS_CTL_XY_I_LIMIT);
	pid_control_set_smooth(&g_pid_pos_x, POS_CTL_XY_SMOOTH_INPUT, POS_CTL_XY_SMOOTH_P_TERM, POS_CTL_XY_SMOOTH_OUTPUT);

	pid_control_init(&g_pid_pos_y);
	pid_control_set_p_gain(&g_pid_pos_y, POS_CTL_XY_P);
	pid_control_set_d_gain(&g_pid_pos_y, POS_CTL_XY_D);
	pid_control_set_i_gain(&g_pid_pos_y, POS_CTL_XY_I, POS_CTL_XY_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_pos_y, POS_CTL_XY_I_LIMIT);
	pid_control_set_smooth(&g_pid_pos_y, POS_CTL_XY_SMOOTH_INPUT, POS_CTL_XY_SMOOTH_P_TERM, POS_CTL_XY_SMOOTH_OUTPUT);

	pid_control_init(&g_pid_pos_z);
	pid_control_set_p_gain(&g_pid_pos_z, POS_CTL_Z_P);
	pid_control_set_d_gain(&g_pid_pos_z, POS_CTL_Z_D);
	pid_control_set_i_gain(&g_pid_pos_z, POS_CTL_Z_I, POS_CTL_Z_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_pos_z, POS_CTL_Z_I_LIMIT);
	pid_control_set_smooth(&g_pid_pos_z, POS_CTL_Z_SMOOTH_INPUT, POS_CTL_Z_SMOOTH_P_TERM, POS_CTL_Z_SMOOTH_OUTPUT);
}

static void position_control_loop(void) {
	double dt = 1.0 / POS_CTL_FREQ;
	pid_control_update(&g_pid_pos_x, g_pos_final.x, g_pos_target.x, dt);
	pid_control_update(&g_pid_pos_y, g_pos_final.y, g_pos_target.y, dt);
	pid_control_update(&g_pid_pos_z, g_pos_final.z, g_pos_target.z, dt);
}

static void reset(void) {
	vector3d_set(&g_pos_target, &g_pos_final);
	vector3d_init(&g_pos_offset, 0, 0, 0);
	pid_control_reset(&g_pid_pos_x, g_pos_final.x);
	pid_control_reset(&g_pid_pos_y, g_pos_final.y);
	pid_control_reset(&g_pid_pos_z, g_pos_final.z);
}

static void publish_angular_target(void) {
	memcpy(&g_target_data[0], 	&g_pos_ctl_roll, 8);
	memcpy(&g_target_data[8],	&g_pos_ctl_pitch, 8);
	memcpy(&g_target_data[16], 	&g_pos_ctl_yaw, 8);
	memcpy(&g_target_data[24], 	&g_pos_ctl_alt, 8);
	memcpy(&g_target_data[32], 	&g_take_off_speed, 8);
	publish(ANGULAR_TARGET_UPDATE, (uint8_t*)g_target_data, 40);
}

static void position_update(uint8_t *data, size_t size) {
	memcpy(&g_pos_final, data, sizeof(vector3d_t));
	memcpy(&g_veloc_final, &data[sizeof(vector3d_t)], sizeof(vector3d_t));
	
	if (g_rc_state_ctl.mode == 2) {
		reset();
		vector3d_set(&g_veloc_offset, &g_veloc_final);
		g_pos_ctl_roll 		= -g_rc_att_ctl.roll * 0.5;
		g_pos_ctl_pitch 	= -g_rc_att_ctl.pitch * 0.5;
		g_pos_ctl_yaw 		= g_yaw_veloc;
		g_pos_ctl_alt 		= -g_rc_att_ctl.alt * 2;
	} else {
		position_control_loop();
		g_pos_ctl_roll 		= g_pid_pos_y.output + (g_veloc_final.y - g_veloc_offset.y) * POS_CTL_VELOC_Y_SCALE;
		g_pos_ctl_pitch 	= g_pid_pos_x.output + (g_veloc_final.x - g_veloc_offset.x) * POS_CTL_VELOC_X_SCALE;
		g_pos_ctl_yaw 		= g_yaw_veloc;
		g_pos_ctl_alt 		= g_pid_pos_z.output + (g_veloc_final.z - g_veloc_offset.z) * POS_CTL_VELOC_Z_SCALE;
	}
	
	publish_angular_target();
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY) {
		reset();
		vector3d_set(&g_veloc_offset, &g_veloc_final);
		g_take_off_speed = 0;
		g_pos_ctl_roll = 0;
		g_pos_ctl_pitch = 0;
		g_pos_ctl_yaw = 0;
		g_pos_ctl_alt = 0;
	} else if (g_state == TAKING_OFF) {
		reset();
	}
}

static void update_target(uint8_t *data, size_t size) {
	g_yaw_veloc = fabs(g_rc_att_ctl.yaw) > RC_DEADBAND ? g_rc_att_ctl.yaw * RC_YAW_SCALE : 0;

	if (g_rc_state_ctl.mode == 2) {
		if (fabs(g_rc_att_ctl.alt) > RC_DEADBAND) {
			g_take_off_speed += 0.5 / CTL_FREQ * g_rc_att_ctl.alt;
		}
		return;
	}

	if (fabs(g_rc_att_ctl.pitch) > RC_DEADBAND) {
		if (g_moving_state_pitch == 0) {
			g_pos_offset.x = g_pos_target.x - g_pos_final.x;
			g_moving_state_pitch = CTL_FREQ;
		}
		g_pos_target.x = g_pos_final.x + g_pos_offset.x + g_rc_att_ctl.pitch * RC_XY_SCALE;
	} else if (g_moving_state_pitch == CTL_FREQ) {
		g_pos_target.x = g_pos_final.x + g_pos_offset.x;
		g_moving_state_pitch = 0;
	}

	if (fabs(g_rc_att_ctl.roll) > RC_DEADBAND) {
		if (g_moving_state_roll == 0) {
			g_pos_offset.y = g_pos_target.y - g_pos_final.y;
			g_moving_state_roll = CTL_FREQ;
		}
		g_pos_target.y = g_pos_final.y + g_pos_offset.y + g_rc_att_ctl.roll * RC_XY_SCALE;
	} else if (g_moving_state_roll == CTL_FREQ) {
		g_pos_target.y = g_pos_final.y + g_pos_offset.y;
		g_moving_state_roll = 0;
	}

	if (g_state == LANDING) {
		g_pos_target.z = g_pos_final.z + g_pos_offset.z - g_landing_speed;
	} else {
		if (fabs(g_rc_att_ctl.alt) > RC_DEADBAND) {
			if (g_moving_state_alt == 0) {
				g_pos_offset.z = g_pos_target.z - g_pos_final.z;
				g_moving_state_alt = CTL_FREQ;
			}
			g_pos_target.z = g_pos_final.z + g_pos_offset.z + g_rc_att_ctl.alt * RC_Z_SCALE;
		} else if (g_moving_state_alt > 0) {
			g_pos_target.z = g_pos_final.z + g_pos_offset.z;
			g_moving_state_alt -= 1;
		}
	}
}

static void state_control_update(uint8_t *data, size_t size) {
	rc_state_ctl_t rc_state_ctl = {0};
	memcpy(&rc_state_ctl, data, size);
	if (rc_state_ctl.mode == 2 && g_rc_state_ctl.mode < 2) {
		if (g_state == FLYING) {
			g_take_off_speed -= g_pos_ctl_alt;
			g_pos_ctl_alt = 0;
		}
	}

	memcpy(&g_rc_state_ctl, data, size);
}

void position_control_setup(void) {
	pid_setup();
	subscribe(POSITION_STATE_UPDATE, position_update);
	subscribe(STATE_DETECTION_UPDATE, state_update);
	subscribe(RC_STATE_UPDATE, state_control_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SCHEDULER_100HZ, update_target);
}
