#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>

#define MOTOR_TYPE 1 // 1: BRUSHLESS, 2: BRUSHED

#if MOTOR_TYPE == 1
#define MIN_SPEED 150
#define MAX_SPEED 1800
#elif  MOTOR_TYPE == 2
#define MIN_SPEED 80
#define MAX_SPEED 3000
#endif // MOTOR_TYPE

#define PID_FREQ 1000

/* PID Gains */
// Roll
#define ATT_ROLL_P 8.0
#define ATT_ROLL_I 1.0
#define ATT_ROLL_D 4.0
#define ATT_ROLL_I_LIMIT 5.0

// Pitch
#define ATT_PITCH_P 8.0
#define ATT_PITCH_I 1.0
#define ATT_PITCH_D 4.0
#define ATT_PITCH_I_LIMIT 5.0

// Yaw
#define ATT_YAW_P 40.0
#define ATT_YAW_I 1.0
#define ATT_YAW_D 20.0
#define ATT_YAW_I_LIMIT 5.0

// Smoothing
#define ATT_SMOOTH_INPUT 1.0
#define ATT_SMOOTH_P_TERM 1.0
#define ATT_SMOOTH_OUTPUT 1.0
#define ATT_GAIN_TIME 1.0

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

static int g_output_speed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static angle3d_t g_angular_state = {0, 0, 0};
static angle3d_t g_angular_target = {0, 0, 0};
static state_t g_state = DISARMED;

static pid_control_t g_pid_att_roll;
static pid_control_t g_pid_att_pitch;
static pid_control_t g_pid_att_yaw;

static rc_att_ctl_t g_rc_att_ctl;

static double g_take_off_speed = 0;
static double g_altitude = 0;
static double g_set_point_yaw = 0;

static void angular_state_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_state, data, size);
}

static void angular_target_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_target, data, sizeof(angle3d_t));
	g_altitude = *(double*)&data[24];
	g_take_off_speed = *(double*)&data[32];
	if (fabs(g_angular_target.yaw) > 1.0) {
		g_set_point_yaw = g_angular_state.yaw + g_angular_target.yaw;
	}
}

static void move_in_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_att_ctl, data, size);
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
	double dt = 1.0 / PID_FREQ;
	pid_control_update(&g_pid_att_roll, 	g_angular_state.roll, 	g_angular_target.roll, dt);
	pid_control_update(&g_pid_att_pitch,	g_angular_state.pitch, 	g_angular_target.pitch, dt);
	pid_control_update(&g_pid_att_yaw, 		g_angular_state.yaw, 	g_set_point_yaw, dt);

	double m1 = MIN_SPEED + g_take_off_speed - g_altitude + g_pid_att_roll.output - g_pid_att_pitch.output - g_pid_att_yaw.output;
	double m2 = MIN_SPEED + g_take_off_speed - g_altitude - g_pid_att_roll.output - g_pid_att_pitch.output + g_pid_att_yaw.output;
	double m3 = MIN_SPEED + g_take_off_speed - g_altitude - g_pid_att_roll.output + g_pid_att_pitch.output - g_pid_att_yaw.output;
	double m4 = MIN_SPEED + g_take_off_speed - g_altitude + g_pid_att_roll.output + g_pid_att_pitch.output + g_pid_att_yaw.output;

	double m5 = MIN_SPEED + g_take_off_speed - g_altitude + g_pid_att_roll.output - g_pid_att_pitch.output + g_pid_att_yaw.output;
	double m6 = MIN_SPEED + g_take_off_speed - g_altitude - g_pid_att_roll.output - g_pid_att_pitch.output - g_pid_att_yaw.output;
	double m7 = MIN_SPEED + g_take_off_speed - g_altitude - g_pid_att_roll.output + g_pid_att_pitch.output + g_pid_att_yaw.output;
	double m8 = MIN_SPEED + g_take_off_speed - g_altitude + g_pid_att_roll.output + g_pid_att_pitch.output - g_pid_att_yaw.output;

	g_output_speed[0] = LIMIT((int)m1, MIN_SPEED, MAX_SPEED);
	g_output_speed[1] = LIMIT((int)m2, MIN_SPEED, MAX_SPEED);
	g_output_speed[2] = LIMIT((int)m3, MIN_SPEED, MAX_SPEED);
	g_output_speed[3] = LIMIT((int)m4, MIN_SPEED, MAX_SPEED);

	g_output_speed[4] = LIMIT((int)m5, MIN_SPEED, MAX_SPEED);
	g_output_speed[5] = LIMIT((int)m6, MIN_SPEED, MAX_SPEED);
	g_output_speed[6] = LIMIT((int)m7, MIN_SPEED, MAX_SPEED);
	g_output_speed[7] = LIMIT((int)m8, MIN_SPEED, MAX_SPEED);
}

static void attitude_control_loop(uint8_t *data, size_t size) {
	if (g_state == DISARMED || g_state == ARMED) {
		g_output_speed[0] = 0;
		g_output_speed[1] = 0;
		g_output_speed[2] = 0;
		g_output_speed[3] = 0;
		g_output_speed[4] = 0;
		g_output_speed[5] = 0;
		g_output_speed[6] = 0;
		g_output_speed[7] = 0;
	}
	else if (g_state == READY) {
		g_output_speed[0] = MIN_SPEED;
		g_output_speed[1] = MIN_SPEED;
		g_output_speed[2] = MIN_SPEED;
		g_output_speed[3] = MIN_SPEED;
		g_output_speed[4] = MIN_SPEED;
		g_output_speed[5] = MIN_SPEED;
		g_output_speed[6] = MIN_SPEED;
		g_output_speed[7] = MIN_SPEED;
	}
	else if (g_state == TAKING_OFF || g_state == FLYING || g_state == LANDING) {
		pid_loop();
		if (g_state == TAKING_OFF) {
			g_set_point_yaw = g_angular_state.yaw;
		}
	}
	else if (g_state == TESTING) {
		g_output_speed[0] = LIMIT(MIN_SPEED + g_rc_att_ctl.yaw / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[1] = LIMIT(MIN_SPEED + g_rc_att_ctl.alt / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[2] = LIMIT(MIN_SPEED + g_rc_att_ctl.roll / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[3] = LIMIT(MIN_SPEED + g_rc_att_ctl.pitch / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[4] = LIMIT(MIN_SPEED + g_rc_att_ctl.yaw / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[5] = LIMIT(MIN_SPEED + g_rc_att_ctl.alt / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[6] = LIMIT(MIN_SPEED + g_rc_att_ctl.roll / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[7] = LIMIT(MIN_SPEED + g_rc_att_ctl.pitch / 90 	* (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
	}

	publish(SPEED_CONTROL_UPDATE, (uint8_t*)g_output_speed, sizeof(int) * 8);
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

	subscribe(ANGULAR_STATE_UPDATE, angular_state_update);
	subscribe(SCHEDULER_1KHZ, attitude_control_loop);
	subscribe(STATE_DETECTION_UPDATE, state_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update); // For motor testing
	subscribe(ANGULAR_TARGET_UPDATE, angular_target_update);
}
