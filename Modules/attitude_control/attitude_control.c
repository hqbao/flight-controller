#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define MIN_SPEED 120
#define MAX_SPEED 1800
#define PID_FREQ 1000

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
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

static int g_output_speed[4] = {0, 0, 0, 0};
static angle3d_t g_angular_state = {0, 0, 0};
static angle3d_t g_angular_target = {0, 0, 0};
static state_t g_state = DISARMED;

static pid_control_t g_pid_att_roll;
static pid_control_t g_pid_att_pitch;
static pid_control_t g_pid_att_yaw;

static rc_att_ctl_t g_rc_att_ctl;

static double g_take_off_speed = MIN_SPEED;
static double g_altitude = 0;
static double g_set_point_yaw = 0;

static void angular_state_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_state, data, size);
}

static void angular_target_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_target, data, sizeof(angle3d_t));
	g_altitude = *(double*)&data[24];
	if (fabs(g_angular_target.yaw) > 1.0) {
		g_set_point_yaw = g_angular_state.yaw + g_angular_target.yaw;
	}
}

static void move_in_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_att_ctl, data, size);
}

static void pid_setup(void) {
	pid_control_init(&g_pid_att_roll);
	pid_control_set_p_gain(&g_pid_att_roll, 40);
	pid_control_set_d_gain(&g_pid_att_roll, 8);
	pid_control_set_i_gain(&g_pid_att_roll, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_att_roll, 5);
	pid_control_set_smooth(&g_pid_att_roll, 1.0, 0.5, 1.0);

	pid_control_init(&g_pid_att_pitch);
	pid_control_set_p_gain(&g_pid_att_pitch, 40);
	pid_control_set_d_gain(&g_pid_att_pitch, 8);
	pid_control_set_i_gain(&g_pid_att_pitch, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_att_pitch, 5);
	pid_control_set_smooth(&g_pid_att_pitch, 1.0, 0.5, 1.0);

	pid_control_init(&g_pid_att_yaw);
	pid_control_set_p_gain(&g_pid_att_yaw, 40);
	pid_control_set_d_gain(&g_pid_att_yaw, 8);
	pid_control_set_i_gain(&g_pid_att_yaw, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_att_yaw, 5);
	pid_control_set_smooth(&g_pid_att_yaw, 1.0, 0.5, 1.0);
}

static void pid_loop(void) {
	double dt = 1.0 / PID_FREQ;
	pid_control_update(&g_pid_att_roll, 	g_angular_state.roll, 	g_angular_target.roll, dt);
	pid_control_update(&g_pid_att_pitch,	g_angular_state.pitch, 	g_angular_target.pitch, dt);
	pid_control_update(&g_pid_att_yaw, 		g_angular_state.yaw, 	g_set_point_yaw, dt);
	double m1 = g_take_off_speed - g_altitude + g_pid_att_roll.output - g_pid_att_pitch.output - g_pid_att_yaw.output;
	double m2 = g_take_off_speed - g_altitude - g_pid_att_roll.output - g_pid_att_pitch.output + g_pid_att_yaw.output;
	double m3 = g_take_off_speed - g_altitude - g_pid_att_roll.output + g_pid_att_pitch.output - g_pid_att_yaw.output;
	double m4 = g_take_off_speed - g_altitude + g_pid_att_roll.output + g_pid_att_pitch.output + g_pid_att_yaw.output;
	g_output_speed[0] = LIMIT((int)m1, MIN_SPEED, MAX_SPEED);
	g_output_speed[1] = LIMIT((int)m2, MIN_SPEED, MAX_SPEED);
	g_output_speed[2] = LIMIT((int)m3, MIN_SPEED, MAX_SPEED);
	g_output_speed[3] = LIMIT((int)m4, MIN_SPEED, MAX_SPEED);
}

static void attitude_control_loop(uint8_t *data, size_t size) {
	if (g_state == DISARMED || g_state == ARMED) {
		g_output_speed[0] = 0;
		g_output_speed[1] = 0;
		g_output_speed[2] = 0;
		g_output_speed[3] = 0;
	}
	else if (g_state == READY) {
		g_output_speed[0] = MIN_SPEED;
		g_output_speed[1] = MIN_SPEED;
		g_output_speed[2] = MIN_SPEED;
		g_output_speed[3] = MIN_SPEED;
	}
	else if (g_state == TAKING_OFF || g_state == FLYING || g_state == LANDING) {
		pid_loop();
		if (g_state == TAKING_OFF) {
			g_set_point_yaw = g_angular_state.yaw;
		}
	}
	else if (g_state == TESTING) {
		g_output_speed[0] = LIMIT(MIN_SPEED + g_rc_att_ctl.yaw * 20, 0, MAX_SPEED);
		g_output_speed[1] = LIMIT(MIN_SPEED + g_rc_att_ctl.alt * 20, 0, MAX_SPEED);
		g_output_speed[2] = LIMIT(MIN_SPEED + g_rc_att_ctl.roll * 20, 0, MAX_SPEED);
		g_output_speed[3] = LIMIT(MIN_SPEED + g_rc_att_ctl.pitch * 20, 0, MAX_SPEED);
	}

	publish(SPEED_CONTROL_UPDATE, (uint8_t*)g_output_speed, sizeof(int) * 4);
}

static void reset(void) {
	pid_control_reset(&g_pid_att_roll, g_angular_state.roll);
	pid_control_reset(&g_pid_att_pitch, g_angular_state.pitch);
	pid_control_reset(&g_pid_att_yaw, g_angular_state.yaw);
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY || g_state == TAKING_OFF) {
		reset();
	}
}

void attitude_control_setup(void) {
	pid_setup();

	publish(SPEED_CONTROL_SETUP, NULL, 0);

	subscribe(SENSOR_ATTITUDE_ANGLE, angular_state_update);
	subscribe(SCHEDULER_1KHZ, attitude_control_loop);
	subscribe(STATE_DETECTION_UPDATE, state_update);
	subscribe(COMMAND_SET_MOVE_IN, move_in_control_update); // For motor testing
	subscribe(COMMAND_SET_TARGET_ORIENTATION, angular_target_update);
}
