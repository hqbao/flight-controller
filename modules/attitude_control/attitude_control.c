#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>
#include <messages.h>

/* PID Gains */
// Roll
#define ATT_ROLL_P 6.0
#define ATT_ROLL_I 1.0
#define ATT_ROLL_D 3.0
#define ATT_ROLL_I_LIMIT 5.0

// Pitch
#define ATT_PITCH_P 6.0
#define ATT_PITCH_I 1.0
#define ATT_PITCH_D 3.0
#define ATT_PITCH_I_LIMIT 5.0

// Yaw
#define ATT_YAW_P 20.0
#define ATT_YAW_I 1.0
#define ATT_YAW_D 10.0
#define ATT_YAW_I_LIMIT 5.0

// Smoothing
#define ATT_SMOOTH_INPUT 1.0
#define ATT_SMOOTH_P_TERM 1.0
#define ATT_SMOOTH_OUTPUT 1.0
#define ATT_GAIN_TIME 1.0

static angle3d_t g_angular_state = {0, 0, 0};
static angle3d_t g_angular_target = {0, 0, 0};
static state_t g_state = DISARMED;

static pid_control_t g_pid_att_roll;
static pid_control_t g_pid_att_pitch;
static pid_control_t g_pid_att_yaw;

static double g_take_off_speed = 0;
static double g_altitude = 0;
static double g_set_point_yaw = 0;

static void angular_state_update(uint8_t *data, size_t size) {
	if (size > sizeof(angle3d_t)) size = sizeof(angle3d_t);
	memcpy(&g_angular_state, data, size);
}

static void angular_target_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_target, data, sizeof(angle3d_t));
	if (fabs(g_angular_target.yaw) > 1.0) {
		g_set_point_yaw = g_angular_state.yaw + g_angular_target.yaw;
	}
}

static void altitude_control_update(uint8_t *data, size_t size) {
	if (size < 16) return;
	memcpy(&g_altitude, &data[0], sizeof(double));
	memcpy(&g_take_off_speed, &data[8], sizeof(double));
}

static void pid_setup(void) {
	pid_control_init(&g_pid_att_roll);
	pid_control_set_p_gain(&g_pid_att_roll, ATT_ROLL_P);
	pid_control_set_d_gain(&g_pid_att_roll, ATT_ROLL_D);
	pid_control_set_i_gain(&g_pid_att_roll, ATT_ROLL_I, ATT_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_att_roll, ATT_ROLL_I_LIMIT);
	pid_control_set_smooth(&g_pid_att_roll, ATT_SMOOTH_INPUT, ATT_SMOOTH_P_TERM, ATT_SMOOTH_OUTPUT);

	pid_control_init(&g_pid_att_pitch);
	pid_control_set_p_gain(&g_pid_att_pitch, ATT_PITCH_P);
	pid_control_set_d_gain(&g_pid_att_pitch, ATT_PITCH_D);
	pid_control_set_i_gain(&g_pid_att_pitch, ATT_PITCH_I, ATT_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_att_pitch, ATT_PITCH_I_LIMIT);
	pid_control_set_smooth(&g_pid_att_pitch, ATT_SMOOTH_INPUT, ATT_SMOOTH_P_TERM, ATT_SMOOTH_OUTPUT);

	pid_control_init(&g_pid_att_yaw);
	pid_control_set_p_gain(&g_pid_att_yaw, ATT_YAW_P);
	pid_control_set_d_gain(&g_pid_att_yaw, ATT_YAW_D);
	pid_control_set_i_gain(&g_pid_att_yaw, ATT_YAW_I, ATT_GAIN_TIME);
	pid_control_set_i_limit(&g_pid_att_yaw, ATT_YAW_I_LIMIT);
	pid_control_set_smooth(&g_pid_att_yaw, ATT_SMOOTH_INPUT, ATT_SMOOTH_P_TERM, ATT_SMOOTH_OUTPUT);
}

static void pid_loop(void) {
	double dt = 1.0 / ATT_CTL_FREQ;
	pid_control_update(&g_pid_att_roll, 	g_angular_state.roll, 	g_angular_target.roll, dt);
	pid_control_update(&g_pid_att_pitch,	g_angular_state.pitch, 	g_angular_target.pitch, dt);
	pid_control_update(&g_pid_att_yaw, 		g_angular_state.yaw, 	g_set_point_yaw, dt);

	mix_control_input_t mix = {
		.roll     = g_pid_att_roll.output,
		.pitch    = g_pid_att_pitch.output,
		.yaw      = g_pid_att_yaw.output,
		.altitude = g_altitude + g_take_off_speed,
	};
	publish(MIX_CONTROL_UPDATE, (uint8_t *)&mix, sizeof(mix));
}

static void attitude_control_loop(uint8_t *data, size_t size) {
	if (g_state == TAKING_OFF || g_state == FLYING || g_state == LANDING) {
		pid_loop();
		if (g_state == TAKING_OFF) {
			g_set_point_yaw = g_angular_state.yaw;
		}
	}
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

	subscribe(ANGULAR_STATE_UPDATE, angular_state_update);
	subscribe(SCHEDULER_500HZ, attitude_control_loop);
	subscribe(FLIGHT_STATE_UPDATE, state_update);
	subscribe(ANGULAR_TARGET_UPDATE, angular_target_update);
	subscribe(ALTITUDE_CONTROL_UPDATE, altitude_control_update);
}
