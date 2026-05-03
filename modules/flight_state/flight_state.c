#include "flight_state.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <vector3d.h>
#include <math.h>
#include <macro.h>
#include <messages.h>

/* Thresholds (units: mm for range, degrees for angles, loop iterations for time) */
static float g_disarm_angle  = 60.0f;   // Max tilt angle (deg) before emergency disarm
static float g_disarm_range  = 10.0f;   // Range (mm) below which drone is considered landed
static float g_allowed_landing_range = 500.0f;  // Min range (mm) to allow landing mode
static float g_took_off_range = 100.0f; // Range (mm) to confirm takeoff complete
#define DISARM_TIME_WHEN_LANDING 50      // Iterations at 100Hz (~500ms) to confirm landing

/* RC Input Constants */
#define RC_STATE_DISARMED 0
#define RC_STATE_ARMED 1
#define STICK_MIN -90                    // Min stick position (degrees)
#define STICK_MAX 90                     // Max stick position (degrees)
#define TAKEOFF_THROTTLE 5               // Throttle threshold to start takeoff (degrees)

static sensor_health_t g_sensor_health = {0};

static angle3d_t g_angular_state = {0, 0, 0};
static rc_state_ctl_t g_rc_state_ctl = {0};
static rc_state_ctl_t g_rc_state_ctl_prev = {0};
static rc_att_ctl_t g_rc_att_ctl = {0};
static state_t g_state = DISARMED;
static state_t g_state_prev = DISARMED;
static char g_gyro_calibrated = 0;
static char g_accel_calibrated = 0;
static char g_mag_calibrated = 0;
static double g_downward_range = 0;

static void state_control_update(uint8_t *data, size_t size) {
	if (size > sizeof(rc_state_ctl_t)) size = sizeof(rc_state_ctl_t);
	memcpy(&g_rc_state_ctl, data, size);
}

static void move_in_control_update(uint8_t *data, size_t size) {
	if (size > sizeof(rc_att_ctl_t)) size = sizeof(rc_att_ctl_t);
	memcpy(&g_rc_att_ctl, data, size);
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	
	optflow_data_t msg;
	memcpy(&msg, data, sizeof(optflow_data_t));
	if (msg.direction == OPTFLOW_DOWNWARD) {
		g_downward_range = msg.z;
	}
}

static void on_sensor_health_update(uint8_t *data, size_t size) {
	if (size < sizeof(sensor_health_t)) return;
	memcpy(&g_sensor_health, data, sizeof(sensor_health_t));
}

static void on_gyro_calibration_status(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_gyro_calibrated = (data[0] == 1);
}

static void on_accel_calibration_status(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_accel_calibrated = (data[0] == 1);
}

static void on_mag_calibration_status(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_mag_calibrated = (data[0] == 1);
}

static void on_state_update(uint8_t *data, size_t size) {
	if (g_rc_state_ctl.state == RC_STATE_DISARMED) {
		g_state = DISARMED;
	}

	if (g_rc_state_ctl.state == RC_STATE_ARMED && g_rc_state_ctl_prev.state == RC_STATE_DISARMED) {
		if (g_gyro_calibrated
				&& g_accel_calibrated
				//&& g_mag_calibrated
				&& g_sensor_health.optflow_down
				&& g_sensor_health.downward_range) {
			g_state = ARMED;
		}
	}

	if (g_state == ARMED) {
		char stick1_most_left 	= g_rc_att_ctl.yaw <= STICK_MIN;
		char stich1_most_bottom = g_rc_att_ctl.alt <= STICK_MIN;
		char stick2_most_right 	= g_rc_att_ctl.roll >= STICK_MAX;
		char stich2_most_bottom = g_rc_att_ctl.pitch <= STICK_MIN;
		if (stick1_most_left && stich1_most_bottom &&
				stick2_most_right && stich2_most_bottom) {
			g_state = READY;
		}
	}

	if (g_state == READY) {
		if (g_rc_att_ctl.alt > TAKEOFF_THROTTLE) {
			g_state = g_rc_state_ctl.state == 1 ? TAKING_OFF : TESTING;
		}
	}

	if (g_state == TAKING_OFF) {
		if (g_downward_range > g_took_off_range) {
			g_state = FLYING;
		}
	}

	if (g_state == FLYING) {
		if (g_downward_range < g_disarm_range && g_rc_att_ctl.alt == -90) {
			g_state = DISARMED;
		}

		if (g_rc_state_ctl.state == 2 && g_rc_state_ctl_prev.state != 2) {
			if (g_downward_range > g_allowed_landing_range) {
				g_state = LANDING;
			}
		}
	}

	if (g_state == TAKING_OFF || g_state == FLYING) {
		if (fabs(g_angular_state.roll) > g_disarm_angle
				|| fabs(g_angular_state.pitch) > g_disarm_angle) {
			g_state = DISARMED;
		}
	}

	if (g_state == LANDING) {
		static int landing_counter = DISARM_TIME_WHEN_LANDING;
		if (g_downward_range < g_disarm_range) {
			if (landing_counter < 1) {
				g_state = DISARMED;
			}

			landing_counter--;
		} else {
			landing_counter = DISARM_TIME_WHEN_LANDING;
		}

		if (g_rc_state_ctl.state != 2) {
			g_state = FLYING;
		}
	}

	memcpy(&g_rc_state_ctl_prev, &g_rc_state_ctl, sizeof(rc_state_ctl_t));

	if (g_state != g_state_prev) {
		publish(FLIGHT_STATE_UPDATE, (uint8_t*)&g_state, 1);
	}

	g_state_prev = g_state;
}

static void angular_state_update(uint8_t *data, size_t size) {
	if (size > sizeof(angle3d_t)) size = sizeof(angle3d_t);
	memcpy(&g_angular_state, data, size);
}

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t tp;
	memcpy(&tp, data, sizeof(tp));

	g_disarm_angle          = tp.disarm_angle;
	g_disarm_range          = tp.disarm_range;
	g_allowed_landing_range = tp.allowed_landing_range;
	g_took_off_range        = tp.took_off_range;
}

void flight_state_setup(void) {
	subscribe(CALIBRATION_GYRO_STATUS, on_gyro_calibration_status);
	subscribe(CALIBRATION_ACCEL_STATUS, on_accel_calibration_status);
	subscribe(CALIBRATION_MAG_STATUS, on_mag_calibration_status);
	subscribe(SENSOR_HEALTH_UPDATE, on_sensor_health_update);
	subscribe(RC_STATE_UPDATE, state_control_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(ANGULAR_STATE_UPDATE, angular_state_update);
	subscribe(PILOT_CTL_SCHEDULER, on_state_update);
	subscribe(TUNING_READY, on_tuning_ready);
}
