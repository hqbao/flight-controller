#include "position_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <macro.h>
#include <messages.h>

#define MIN_LANDING_SPEED 50

/* Control Gains (defaults — overridden by tuning params from flash) */
static double g_pos_xy_p = 50.0;
static double g_pos_z_p = 2000.0;
static double g_pos_veloc_xy_scale = 50.0;
static double g_pos_veloc_z_scale = 2000.0;
static double g_pos_lpf_xy = 1.0;
static double g_pos_lpf_z_raw = 5.0;
static double g_pos_angle_limit = 30.0;
static double g_rc_deadband = 0.1;

/* Landing Control (defaults — overridden by tuning) */
static double g_landing_range_thr = 2000.0;
static double g_landing_speed_inc = 1.0;
static double g_landing_speed_dec = 5.0;
static double g_landing_descent_thr = -20.0;

/* RC Control */
static double g_rc_xy_scale = 0.01;
static double g_rc_z_scale = 0.04;
static double g_rc_yaw_scale = -0.5;

static state_t g_state = DISARMED;
static rc_state_ctl_t g_rc_state_ctl = {0};
static rc_att_ctl_t g_rc_att_ctl = {0};
static double g_downward_range = 0;
static double g_downward_range_prev = 0;

static vector3d_t g_pos_final = {0, 0, 0};
static vector3d_t g_pos_target = {0, 0, 0};
static vector3d_t g_pos_offset = {0, 0, 0};
static vector3d_t g_veloc_final = {0, 0, 0};
static vector3d_t g_veloc_offset = {0, 0, 0};
static vector3d_t g_veloc_applied = {0, 0, 0};

static uint8_t g_target_data[40] = {0};

static double g_yaw_veloc = 0;
static double g_take_off_speed = 0;

static double g_pos_ctl_roll = 0;
static double g_pos_ctl_pitch = 0;
static double g_pos_ctl_yaw = 0;
static double g_pos_ctl_alt = 0;

static vector3d_t g_output_smooth = {0, 0, 0};

static int g_moving_state_roll = 0;  // 0: Released, PILOT_CTL_FREQ: Active stick input
static int g_moving_state_pitch = 0;
static int g_moving_state_alt = 0;   // Counts down from PILOT_CTL_FREQ to 0 after release

static double g_landing_speed = MIN_LANDING_SPEED;

static void move_in_control_update(uint8_t *data, size_t size) {
	if (size < sizeof(rc_att_ctl_t)) return;
	memcpy(&g_rc_att_ctl, data, sizeof(rc_att_ctl_t));
}

static void position_target_update(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	memcpy(&g_pos_target, data, sizeof(vector3d_t));
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	
	optflow_data_t msg;
	memcpy(&msg, data, sizeof(optflow_data_t));
	
	if (msg.direction == OPTFLOW_DOWNWARD) {
		g_downward_range = msg.z;
		if (g_state == LANDING) {
			if (g_downward_range < g_landing_range_thr) {
				if (g_downward_range - g_downward_range_prev >= 0) { // Not moving down
					g_landing_speed += g_landing_speed_inc;
				} else if (g_downward_range - g_downward_range_prev < g_landing_descent_thr) { // Too high speed
					g_landing_speed -= g_landing_speed_dec;
				}
			}
			g_downward_range_prev = g_downward_range;
		}
	}
}

static void reset_pid(void) {
	vector3d_set(&g_pos_target, &g_pos_final);
	vector3d_init(&g_pos_offset, 0, 0, 0);
	vector3d_init(&g_output_smooth, 0, 0, 0);
}

static void publish_angular_target(void) {
	memcpy(&g_target_data[0], 	&g_pos_ctl_roll, 8);
	memcpy(&g_target_data[8],	&g_pos_ctl_pitch, 8);
	memcpy(&g_target_data[16], 	&g_pos_ctl_yaw, 8);
	publish(ANGULAR_TARGET_UPDATE, (uint8_t*)g_target_data, 24);
	
	memcpy(&g_target_data[0], 	&g_pos_ctl_alt, 8);
	memcpy(&g_target_data[8], 	&g_take_off_speed, 8);
	publish(ALTITUDE_CONTROL_UPDATE, (uint8_t*)g_target_data, 16);
}

static void position_update(uint8_t *data, size_t size) {
	if (size < sizeof(position_state_t)) return;

	position_state_t state;
	memcpy(&state, data, sizeof(position_state_t));
	g_pos_final = state.position;
	g_veloc_final = state.velocity;
	
	if (g_rc_state_ctl.mode == 2) {
		reset_pid();
		vector3d_set(&g_veloc_offset, &g_veloc_final);
		g_pos_ctl_roll 		= g_rc_att_ctl.roll * 0.5;
		g_pos_ctl_pitch 	= -g_rc_att_ctl.pitch * 0.5;
		g_pos_ctl_yaw 		= -g_yaw_veloc;
		g_pos_ctl_alt 		= g_rc_att_ctl.alt * 2;
	} else {
		g_veloc_applied.y = (g_veloc_final.y - g_veloc_offset.y) * g_pos_veloc_xy_scale;
		g_veloc_applied.x = (g_veloc_final.x - g_veloc_offset.x) * g_pos_veloc_xy_scale;
		g_veloc_applied.z = (g_veloc_final.z - g_veloc_offset.z) * g_pos_veloc_z_scale;

		// Simple P Control: Output = (Current - Target) * P
		// Note: P-term in PID lib was (feedback - setpoint) * P. 
		// We maintain that structure.
		double x_output = (g_pos_final.x - g_pos_target.x) * g_pos_xy_p;
		double y_output = (g_pos_final.y - g_pos_target.y) * g_pos_xy_p;
		double z_output = (g_pos_final.z - g_pos_target.z) * g_pos_z_p;

		double lpf_z = g_pos_lpf_z_raw / ACCEL_FREQ;
		g_output_smooth.x += g_pos_lpf_xy * (x_output - g_output_smooth.x);
		g_output_smooth.y += g_pos_lpf_xy * (y_output - g_output_smooth.y);
		g_output_smooth.z += lpf_z * (z_output - g_output_smooth.z);

		/*
		 * Body-Frame Position→Angle Mapping:
		 * - Positive X error (forward of target) → need positive pitch (nose up) to fly back → no negation
		 * - Positive Y error (right of target) → need negative roll (right wing up) to fly left → negated
		 * - Positive Z error (above target) → need less throttle to descend → negated
		 */
		g_pos_ctl_roll 		= -LIMIT(g_output_smooth.y + g_veloc_applied.y, -g_pos_angle_limit, g_pos_angle_limit);
		g_pos_ctl_pitch 	= LIMIT(g_output_smooth.x + g_veloc_applied.x, -g_pos_angle_limit, g_pos_angle_limit);
		g_pos_ctl_yaw 		= -g_yaw_veloc;
		g_pos_ctl_alt 		= -(g_output_smooth.z + g_veloc_applied.z);
	}
	
	publish_angular_target();
}

static void state_update(uint8_t *data, size_t size) {
	g_state = (state_t)data[0];
	if (g_state == ARMED || g_state == READY) {
		reset_pid();
		vector3d_set(&g_veloc_offset, &g_veloc_final);
		g_take_off_speed = 0;
		g_landing_speed = MIN_LANDING_SPEED;
		g_pos_ctl_roll = 0;
		g_pos_ctl_pitch = 0;
		g_pos_ctl_yaw = 0;
		g_pos_ctl_alt = 0;
	} else if (g_state == TAKING_OFF) {
		reset_pid();
	}
}

static void manual_control_update(uint8_t *data, size_t size) {
	g_yaw_veloc = fabs(g_rc_att_ctl.yaw) > g_rc_deadband ? g_rc_att_ctl.yaw * g_rc_yaw_scale : 0;

	if (g_rc_state_ctl.mode == 2) {
		if (fabs(g_rc_att_ctl.alt) > g_rc_deadband) {
			g_take_off_speed += 0.5 / PILOT_CTL_FREQ * g_rc_att_ctl.alt;
		}
		return;
	}

	if (fabs(g_rc_att_ctl.roll) > g_rc_deadband) {
		if (g_moving_state_roll == 0) {
			g_pos_offset.y = g_pos_target.y - g_pos_final.y;
			g_moving_state_roll = PILOT_CTL_FREQ;
		}
		g_pos_target.y = g_pos_final.y + g_pos_offset.y + g_rc_att_ctl.roll * g_rc_xy_scale;
	} else if (g_moving_state_roll == PILOT_CTL_FREQ) {
		g_pos_target.y = g_pos_final.y + g_pos_offset.y;
		g_moving_state_roll = 0;
	}

	if (fabs(g_rc_att_ctl.pitch) > g_rc_deadband) {
		if (g_moving_state_pitch == 0) {
			g_pos_offset.x = g_pos_target.x - g_pos_final.x;
			g_moving_state_pitch = PILOT_CTL_FREQ;
		}
		g_pos_target.x = g_pos_final.x + g_pos_offset.x + g_rc_att_ctl.pitch * g_rc_xy_scale;
	} else if (g_moving_state_pitch == PILOT_CTL_FREQ) {
		g_pos_target.x = g_pos_final.x + g_pos_offset.x;
		g_moving_state_pitch = 0;
	}

	if (g_state == LANDING) {
		g_pos_target.z = g_pos_final.z + g_pos_offset.z - g_landing_speed;
	} else {
		if (fabs(g_rc_att_ctl.alt) > g_rc_deadband) {
			if (g_moving_state_alt == 0) {
				g_pos_offset.z = g_pos_target.z - g_pos_final.z;
				g_moving_state_alt = PILOT_CTL_FREQ;
			}
			g_pos_target.z = g_pos_final.z + g_pos_offset.z + g_rc_att_ctl.alt * g_rc_z_scale;
		} else if (g_moving_state_alt > 0) {
			g_pos_target.z = g_pos_final.z + g_pos_offset.z;
			g_moving_state_alt -= 1;
		}
	}
}

static void state_control_update(uint8_t *data, size_t size) {
	rc_state_ctl_t rc_state_ctl = {0};
	if (size > sizeof(rc_state_ctl_t)) size = sizeof(rc_state_ctl_t);
	memcpy(&rc_state_ctl, data, size);
	if (rc_state_ctl.mode == 2 && g_rc_state_ctl.mode < 2) {
		if (g_state == FLYING) {
			g_take_off_speed += g_pos_ctl_alt;
			g_pos_ctl_alt = 0;
		}
	}

	memcpy(&g_rc_state_ctl, data, size);
}

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t t;
	memcpy(&t, data, sizeof(tuning_params_t));
	g_pos_xy_p = t.pos_xy_p;
	g_pos_z_p = t.pos_z_p;
	g_pos_veloc_xy_scale = t.pos_veloc_xy_scale;
	g_pos_veloc_z_scale = t.pos_veloc_z_scale;
	g_pos_lpf_xy = t.pos_lpf_xy;
	g_pos_lpf_z_raw = t.pos_lpf_z;
	g_pos_angle_limit = t.pos_angle_limit;
	g_rc_deadband = t.pos_rc_deadband;
}

void position_control_setup(void) {
	subscribe(POSITION_STATE_UPDATE, position_update);
	subscribe(POSITION_TARGET_UPDATE, position_target_update);
	subscribe(FLIGHT_STATE_UPDATE, state_update);
	subscribe(RC_STATE_UPDATE, state_control_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(PILOT_CTL_SCHEDULER, manual_control_update);
	subscribe(TUNING_READY, on_tuning_ready);
}
