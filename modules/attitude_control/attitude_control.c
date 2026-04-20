#include "attitude_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>
#include <messages.h>

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
	pid_control_set_p_gain(&g_pid_att_roll, 4.0);
	pid_control_set_d_gain(&g_pid_att_roll, 2.0);
	pid_control_set_i_gain(&g_pid_att_roll, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_att_roll, 5.0);
	pid_control_set_p_limit(&g_pid_att_roll, 1000000.0);
	pid_control_set_o_limit(&g_pid_att_roll, 1000000.0);
	pid_control_set_smooth(&g_pid_att_roll, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_att_pitch);
	pid_control_set_p_gain(&g_pid_att_pitch, 4.0);
	pid_control_set_d_gain(&g_pid_att_pitch, 2.0);
	pid_control_set_i_gain(&g_pid_att_pitch, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_att_pitch, 5.0);
	pid_control_set_p_limit(&g_pid_att_pitch, 1000000.0);
	pid_control_set_o_limit(&g_pid_att_pitch, 1000000.0);
	pid_control_set_smooth(&g_pid_att_pitch, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_att_yaw);
	pid_control_set_p_gain(&g_pid_att_yaw, 10.0);
	pid_control_set_d_gain(&g_pid_att_yaw, 5.0);
	pid_control_set_i_gain(&g_pid_att_yaw, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_att_yaw, 5.0);
	pid_control_set_p_limit(&g_pid_att_yaw, 1000000.0);
	pid_control_set_o_limit(&g_pid_att_yaw, 1000000.0);
	pid_control_set_smooth(&g_pid_att_yaw, 1.0, 1.0, 1.0);
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

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t t;
	memcpy(&t, data, sizeof(tuning_params_t));

	pid_control_set_p_gain(&g_pid_att_roll, t.att_roll_p);
	pid_control_set_d_gain(&g_pid_att_roll, t.att_roll_d);
	pid_control_set_i_gain(&g_pid_att_roll, t.att_roll_i, t.att_gain_time);
	pid_control_set_i_limit(&g_pid_att_roll, t.att_roll_i_limit);
	pid_control_set_p_limit(&g_pid_att_roll, t.att_roll_p_limit);
	pid_control_set_o_limit(&g_pid_att_roll, t.att_roll_o_limit);
	pid_control_set_smooth(&g_pid_att_roll, t.att_smooth_input, t.att_smooth_p_term, t.att_smooth_output);

	pid_control_set_p_gain(&g_pid_att_pitch, t.att_pitch_p);
	pid_control_set_d_gain(&g_pid_att_pitch, t.att_pitch_d);
	pid_control_set_i_gain(&g_pid_att_pitch, t.att_pitch_i, t.att_gain_time);
	pid_control_set_i_limit(&g_pid_att_pitch, t.att_pitch_i_limit);
	pid_control_set_p_limit(&g_pid_att_pitch, t.att_pitch_p_limit);
	pid_control_set_o_limit(&g_pid_att_pitch, t.att_pitch_o_limit);
	pid_control_set_smooth(&g_pid_att_pitch, t.att_smooth_input, t.att_smooth_p_term, t.att_smooth_output);

	pid_control_set_p_gain(&g_pid_att_yaw, t.att_yaw_p);
	pid_control_set_d_gain(&g_pid_att_yaw, t.att_yaw_d);
	pid_control_set_i_gain(&g_pid_att_yaw, t.att_yaw_i, t.att_gain_time);
	pid_control_set_i_limit(&g_pid_att_yaw, t.att_yaw_i_limit);
	pid_control_set_p_limit(&g_pid_att_yaw, t.att_yaw_p_limit);
	pid_control_set_o_limit(&g_pid_att_yaw, t.att_yaw_o_limit);
	pid_control_set_smooth(&g_pid_att_yaw, t.att_smooth_input, t.att_smooth_p_term, t.att_smooth_output);
}

void attitude_control_setup(void) {
	pid_setup();

	subscribe(ANGULAR_STATE_UPDATE, angular_state_update);
	subscribe(ATT_CTL_SCHEDULER, attitude_control_loop);
	subscribe(FLIGHT_STATE_UPDATE, state_update);
	subscribe(ANGULAR_TARGET_UPDATE, angular_target_update);
	subscribe(ALTITUDE_CONTROL_UPDATE, altitude_control_update);
	subscribe(TUNING_READY, on_tuning_ready);
}
