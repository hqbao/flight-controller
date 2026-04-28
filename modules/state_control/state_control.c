/*
 * STATE_CONTROL — cascaded position→attitude→mix controller.
 *
 * Inlined from the deleted attitude_control + position_control modules.
 * Single owner of the control cascade — reads nav_state_t on STATE_UPDATE,
 * publishes mix_control_input_t on MIX_CONTROL_UPDATE.
 *
 * The outer position loop runs on PILOT_CTL_SCHEDULER (100 Hz) and produces
 * angular setpoints (target roll/pitch + altitude) cached in g_target_*.
 * The inner attitude loop runs on ATT_CTL_SCHEDULER (500 Hz), reads the
 * cached state + targets, runs roll/pitch/yaw PIDs, and publishes the
 * mixer input.
 */

#include "state_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <pid_control.h>
#include <macro.h>
#include <messages.h>

#define MIN_LANDING_SPEED 50

/* ============================================================
 * Cached state (populated by callbacks)
 * ============================================================ */

static nav_state_t   g_nav         = {0};
static state_t       g_flight_state = DISARMED;
static rc_state_ctl_t g_rc_state    = {0};
static rc_att_ctl_t   g_rc_in       = {0};

/* ============================================================
 * Inner attitude PID
 * ============================================================ */

static pid_control_t g_pid_roll;
static pid_control_t g_pid_pitch;
static pid_control_t g_pid_yaw;

/* Yaw setpoint in absolute (wrapped) space. */
static double g_yaw_setpoint = 0;

/* ============================================================
 * Outer position cascade — angular & altitude targets produced here,
 * consumed by the inner loop.
 * ============================================================ */

static double g_target_roll  = 0;
static double g_target_pitch = 0;
static double g_target_yaw   = 0;   /* yaw rate command (deg/s style) */
static double g_target_alt   = 0;
static double g_take_off_speed = 0;

/* Outer-loop position state */
static vector3d_t g_pos_target = {0, 0, 0};
static vector3d_t g_pos_offset = {0, 0, 0};
static vector3d_t g_veloc_offset = {0, 0, 0};
static vector3d_t g_output_smooth = {0, 0, 0};
static double     g_yaw_veloc = 0;

/* Stick-release latches (legacy semantics from position_control). */
static int g_moving_state_roll  = 0;
static int g_moving_state_pitch = 0;
static int g_moving_state_alt   = 0;

/* Optflow downward range cache (for landing speed adapt). */
static double g_downward_range      = 0;
static double g_downward_range_prev = 0;
static double g_landing_speed       = MIN_LANDING_SPEED;

/* ============================================================
 * Tuning (mirrored from tuning_params_t at TUNING_READY)
 * ============================================================ */

/* Position cascade defaults (overridden by ctl_pos_* tuning) */
static double g_pos_xy_p           = 100.0;
static double g_pos_z_p            = 2000.0;
static double g_pos_veloc_xy_scale = 50.0;
static double g_pos_veloc_z_scale  = 2000.0;
static double g_pos_lpf_xy         = 1.0;
static double g_pos_lpf_z_raw      = 5.0;
static double g_pos_angle_limit    = 30.0;
static double g_rc_deadband        = 0.1;

/* RC scaling (overridden by rc_*_scale tuning) */
static double g_rc_xy_scale  = 0.01;
static double g_rc_z_scale   = 0.04;
static double g_rc_yaw_scale = -0.5;

/* Landing thresholds (legacy hard-coded; kept here so no behaviour change). */
static double g_landing_range_thr  = 2000.0;
static double g_landing_speed_inc  = 1.0;
static double g_landing_speed_dec  = 5.0;
static double g_landing_descent_thr = -20.0;

/* ============================================================
 * Helpers
 * ============================================================ */

static void pid_init_all(void) {
	pid_control_init(&g_pid_roll);
	pid_control_set_p_gain(&g_pid_roll, 4.0);
	pid_control_set_d_gain(&g_pid_roll, 2.0);
	pid_control_set_i_gain(&g_pid_roll, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_roll, 5.0);
	pid_control_set_p_limit(&g_pid_roll, 1000000.0);
	pid_control_set_o_limit(&g_pid_roll, 1000000.0);
	pid_control_set_smooth(&g_pid_roll, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_pitch);
	pid_control_set_p_gain(&g_pid_pitch, 4.0);
	pid_control_set_d_gain(&g_pid_pitch, 2.0);
	pid_control_set_i_gain(&g_pid_pitch, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_pitch, 5.0);
	pid_control_set_p_limit(&g_pid_pitch, 1000000.0);
	pid_control_set_o_limit(&g_pid_pitch, 1000000.0);
	pid_control_set_smooth(&g_pid_pitch, 1.0, 1.0, 1.0);

	pid_control_init(&g_pid_yaw);
	pid_control_set_p_gain(&g_pid_yaw, 10.0);
	pid_control_set_d_gain(&g_pid_yaw, 5.0);
	pid_control_set_i_gain(&g_pid_yaw, 1.0, 1.0);
	pid_control_set_i_limit(&g_pid_yaw, 5.0);
	pid_control_set_p_limit(&g_pid_yaw, 1000000.0);
	pid_control_set_o_limit(&g_pid_yaw, 1000000.0);
	pid_control_set_smooth(&g_pid_yaw, 1.0, 1.0, 1.0);
}

static void reset_pids_to_state(void) {
	pid_control_reset(&g_pid_roll,  g_nav.euler.roll);
	pid_control_reset(&g_pid_pitch, g_nav.euler.pitch);
	pid_control_reset(&g_pid_yaw,   g_nav.euler.yaw);
	g_yaw_setpoint = g_nav.euler.yaw;
}

static void reset_pos_cascade(void) {
	vector3d_set(&g_pos_target, &g_nav.position);
	vector3d_init(&g_pos_offset, 0, 0, 0);
	vector3d_init(&g_output_smooth, 0, 0, 0);
}

/* ============================================================
 * Callbacks
 * ============================================================ */

static void on_state_update(uint8_t *data, size_t size) {
	if (size < sizeof(nav_state_t)) return;
	memcpy(&g_nav, data, sizeof(nav_state_t));
}

static void on_position_target(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	memcpy(&g_pos_target, data, sizeof(vector3d_t));
}

static void on_rc_in(uint8_t *data, size_t size) {
	if (size < sizeof(rc_att_ctl_t)) return;
	memcpy(&g_rc_in, data, sizeof(rc_att_ctl_t));
}

static void on_rc_state(uint8_t *data, size_t size) {
	rc_state_ctl_t rc = {0};
	if (size > sizeof(rc_state_ctl_t)) size = sizeof(rc_state_ctl_t);
	memcpy(&rc, data, size);
	/* Mode 1→2 transition while flying: fold current alt PID output into
	 * the take-off speed accumulator (legacy behaviour from position_control). */
	if (rc.mode == 2 && g_rc_state.mode < 2) {
		if (g_flight_state == FLYING) {
			g_take_off_speed += g_target_alt;
			g_target_alt = 0;
		}
	}
	memcpy(&g_rc_state, data, size);
}

static void on_optflow(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t msg;
	memcpy(&msg, data, sizeof(optflow_data_t));

	if (msg.direction != OPTFLOW_DOWNWARD) return;

	g_downward_range = msg.z;
	if (g_flight_state != LANDING) return;
	if (g_downward_range >= g_landing_range_thr) return;

	double drng = g_downward_range - g_downward_range_prev;
	if (drng >= 0) {
		g_landing_speed += g_landing_speed_inc;
	} else if (drng < g_landing_descent_thr) {
		g_landing_speed -= g_landing_speed_dec;
	}
	g_downward_range_prev = g_downward_range;
}

static void on_flight_state(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_flight_state = (state_t)data[0];

	if (g_flight_state == ARMED || g_flight_state == READY) {
		reset_pids_to_state();
		reset_pos_cascade();
		g_veloc_offset = g_nav.velocity;
		g_take_off_speed = 0;
		g_landing_speed  = MIN_LANDING_SPEED;
		g_target_roll = g_target_pitch = g_target_yaw = g_target_alt = 0;
	} else if (g_flight_state == TAKING_OFF) {
		reset_pids_to_state();
		reset_pos_cascade();
	}
}

/* ============================================================
 * Outer position cascade — runs on PILOT_CTL_SCHEDULER (100 Hz).
 * Combines manual_control_update + position_update from legacy
 * position_control.c.
 * ============================================================ */

static void run_position_cascade(void) {
	/* Yaw rate command from RC. */
	g_yaw_veloc = (fabs(g_rc_in.yaw) > g_rc_deadband)
	            ? g_rc_in.yaw * g_rc_yaw_scale : 0;

	/* MODE 2 = full manual: stick → angle. Skip outer cascade math. */
	if (g_rc_state.mode == 2) {
		reset_pos_cascade();
		g_veloc_offset = g_nav.velocity;
		g_target_roll  = g_rc_in.roll  * 0.5;
		g_target_pitch = -g_rc_in.pitch * 0.5;
		g_target_yaw   = -g_yaw_veloc;
		g_target_alt   = g_rc_in.alt   * 2;

		/* Take-off speed integration (manual climb stick). */
		if (fabs(g_rc_in.alt) > g_rc_deadband) {
			g_take_off_speed += 0.5 / (double)PILOT_CTL_FREQ * g_rc_in.alt;
		}
		return;
	}

	/* MODE 1 = position-hold. Update target from RC stick deflections. */
	if (fabs(g_rc_in.roll) > g_rc_deadband) {
		if (g_moving_state_roll == 0) {
			g_pos_offset.y = g_pos_target.y - g_nav.position.y;
			g_moving_state_roll = PILOT_CTL_FREQ;
		}
		g_pos_target.y = g_nav.position.y + g_pos_offset.y + g_rc_in.roll * g_rc_xy_scale;
	} else if (g_moving_state_roll == PILOT_CTL_FREQ) {
		g_pos_target.y = g_nav.position.y + g_pos_offset.y;
		g_moving_state_roll = 0;
	}

	if (fabs(g_rc_in.pitch) > g_rc_deadband) {
		if (g_moving_state_pitch == 0) {
			g_pos_offset.x = g_pos_target.x - g_nav.position.x;
			g_moving_state_pitch = PILOT_CTL_FREQ;
		}
		g_pos_target.x = g_nav.position.x + g_pos_offset.x + g_rc_in.pitch * g_rc_xy_scale;
	} else if (g_moving_state_pitch == PILOT_CTL_FREQ) {
		g_pos_target.x = g_nav.position.x + g_pos_offset.x;
		g_moving_state_pitch = 0;
	}

	if (g_flight_state == LANDING) {
		g_pos_target.z = g_nav.position.z + g_pos_offset.z - g_landing_speed;
	} else {
		if (fabs(g_rc_in.alt) > g_rc_deadband) {
			if (g_moving_state_alt == 0) {
				g_pos_offset.z = g_pos_target.z - g_nav.position.z;
				g_moving_state_alt = PILOT_CTL_FREQ;
			}
			g_pos_target.z = g_nav.position.z + g_pos_offset.z + g_rc_in.alt * g_rc_z_scale;
		} else if (g_moving_state_alt > 0) {
			g_pos_target.z = g_nav.position.z + g_pos_offset.z;
			g_moving_state_alt -= 1;
		}
	}

	/* Velocity feed-forward */
	vector3d_t v_app;
	v_app.x = (g_nav.velocity.x - g_veloc_offset.x) * g_pos_veloc_xy_scale;
	v_app.y = (g_nav.velocity.y - g_veloc_offset.y) * g_pos_veloc_xy_scale;
	v_app.z = (g_nav.velocity.z - g_veloc_offset.z) * g_pos_veloc_z_scale;

	/* Position P-term */
	double x_out = (g_nav.position.x - g_pos_target.x) * g_pos_xy_p;
	double y_out = (g_nav.position.y - g_pos_target.y) * g_pos_xy_p;
	double z_out = (g_nav.position.z - g_pos_target.z) * g_pos_z_p;

	/* Output smoothing (1-pole LPF) */
	double lpf_z = g_pos_lpf_z_raw / (double)ACCEL_FREQ;
	g_output_smooth.x += g_pos_lpf_xy * (x_out - g_output_smooth.x);
	g_output_smooth.y += g_pos_lpf_xy * (y_out - g_output_smooth.y);
	g_output_smooth.z += lpf_z       * (z_out - g_output_smooth.z);

	g_target_roll  = -LIMIT(g_output_smooth.y + v_app.y, -g_pos_angle_limit, g_pos_angle_limit);
	g_target_pitch =  LIMIT(g_output_smooth.x + v_app.x, -g_pos_angle_limit, g_pos_angle_limit);
	g_target_yaw   = -g_yaw_veloc;
	g_target_alt   = -(g_output_smooth.z + v_app.z);
}

static void on_pilot_ctl(uint8_t *data, size_t size) {
	(void)data; (void)size;
	run_position_cascade();
}

/* ============================================================
 * Inner attitude PID — runs on ATT_CTL_SCHEDULER (500 Hz).
 * ============================================================ */

static void run_inner_loop(void) {
	double dt = 1.0 / (double)ATT_CTL_FREQ;

	/* Yaw rate command (g_target_yaw) integrates into a wrapped yaw setpoint.
	 * Same logic as legacy attitude_control.c angular_target_update. */
	if (fabs(g_target_yaw) > 1.0) {
		g_yaw_setpoint = g_nav.euler.yaw + g_target_yaw;
	}

	/* Yaw wrap-around handling (yaw is in ±180°). */
	double yaw_err = g_yaw_setpoint - g_nav.euler.yaw;
	if (yaw_err > 180.0)       g_yaw_setpoint -= 360.0;
	else if (yaw_err < -180.0) g_yaw_setpoint += 360.0;
	if (fabs(g_nav.euler.yaw - g_pid_yaw.value) > 180.0) {
		g_pid_yaw.value      = g_nav.euler.yaw;
		g_pid_yaw.value_prev = g_nav.euler.yaw;
	}

	pid_control_update(&g_pid_roll,  g_nav.euler.roll,  g_target_roll,  dt);
	pid_control_update(&g_pid_pitch, g_nav.euler.pitch, g_target_pitch, dt);
	pid_control_update(&g_pid_yaw,   g_nav.euler.yaw,   g_yaw_setpoint, dt);

	mix_control_input_t mix = {
		.roll     = g_pid_roll.output,
		.pitch    = g_pid_pitch.output,
		.yaw      = g_pid_yaw.output,
		.altitude = g_target_alt + g_take_off_speed,
	};
	publish(MIX_CONTROL_UPDATE, (uint8_t *)&mix, sizeof(mix));
}

static void on_att_ctl(uint8_t *data, size_t size) {
	(void)data; (void)size;
	if (g_flight_state == TAKING_OFF || g_flight_state == FLYING || g_flight_state == LANDING) {
		run_inner_loop();
		if (g_flight_state == TAKING_OFF) {
			g_yaw_setpoint = g_nav.euler.yaw;
		}
	}
}

/* ============================================================
 * Tuning
 * ============================================================ */

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t t;
	memcpy(&t, data, sizeof(tuning_params_t));

	/* Inner attitude PID gains */
	pid_control_set_p_gain(&g_pid_roll,  t.ctl_att_roll_p);
	pid_control_set_d_gain(&g_pid_roll,  t.ctl_att_roll_d);
	pid_control_set_i_gain(&g_pid_roll,  t.ctl_att_roll_i, t.ctl_att_gain_time);
	pid_control_set_i_limit(&g_pid_roll, t.ctl_att_roll_i_limit);
	pid_control_set_p_limit(&g_pid_roll, t.ctl_att_roll_p_limit);
	pid_control_set_o_limit(&g_pid_roll, t.ctl_att_roll_o_limit);
	pid_control_set_smooth(&g_pid_roll,  t.ctl_att_smooth_input, t.ctl_att_smooth_p_term, t.ctl_att_smooth_output);

	pid_control_set_p_gain(&g_pid_pitch,  t.ctl_att_pitch_p);
	pid_control_set_d_gain(&g_pid_pitch,  t.ctl_att_pitch_d);
	pid_control_set_i_gain(&g_pid_pitch,  t.ctl_att_pitch_i, t.ctl_att_gain_time);
	pid_control_set_i_limit(&g_pid_pitch, t.ctl_att_pitch_i_limit);
	pid_control_set_p_limit(&g_pid_pitch, t.ctl_att_pitch_p_limit);
	pid_control_set_o_limit(&g_pid_pitch, t.ctl_att_pitch_o_limit);
	pid_control_set_smooth(&g_pid_pitch,  t.ctl_att_smooth_input, t.ctl_att_smooth_p_term, t.ctl_att_smooth_output);

	pid_control_set_p_gain(&g_pid_yaw,  t.ctl_att_yaw_p);
	pid_control_set_d_gain(&g_pid_yaw,  t.ctl_att_yaw_d);
	pid_control_set_i_gain(&g_pid_yaw,  t.ctl_att_yaw_i, t.ctl_att_gain_time);
	pid_control_set_i_limit(&g_pid_yaw, t.ctl_att_yaw_i_limit);
	pid_control_set_p_limit(&g_pid_yaw, t.ctl_att_yaw_p_limit);
	pid_control_set_o_limit(&g_pid_yaw, t.ctl_att_yaw_o_limit);
	pid_control_set_smooth(&g_pid_yaw,  t.ctl_att_smooth_input, t.ctl_att_smooth_p_term, t.ctl_att_smooth_output);

	/* Outer position cascade gains */
	g_pos_xy_p           = t.ctl_pos_xy_p;
	g_pos_z_p            = t.ctl_pos_z_p;
	g_pos_veloc_xy_scale = t.ctl_pos_veloc_xy_scale;
	g_pos_veloc_z_scale  = t.ctl_pos_veloc_z_scale;
	g_pos_lpf_xy         = t.ctl_pos_lpf_xy;
	g_pos_lpf_z_raw      = t.ctl_pos_lpf_z;
	g_pos_angle_limit    = t.ctl_pos_angle_limit;
	g_rc_deadband        = t.ctl_pos_rc_deadband;

	g_rc_xy_scale  = t.rc_xy_scale;
	g_rc_z_scale   = t.rc_z_scale;
	g_rc_yaw_scale = t.rc_yaw_scale;
}

/* ============================================================
 * Setup
 * ============================================================ */

void state_control_setup(void) {
	pid_init_all();

	subscribe(STATE_UPDATE,            on_state_update);
	subscribe(ATT_CTL_SCHEDULER,       on_att_ctl);
	subscribe(PILOT_CTL_SCHEDULER,     on_pilot_ctl);
	subscribe(POSITION_TARGET_UPDATE,  on_position_target);
	subscribe(FLIGHT_STATE_UPDATE,     on_flight_state);
	subscribe(RC_STATE_UPDATE,         on_rc_state);
	subscribe(RC_MOVE_IN_UPDATE,       on_rc_in);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, on_optflow);
	subscribe(TUNING_READY,            on_tuning_ready);
}
