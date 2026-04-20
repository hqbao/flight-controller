/*
 * Quadcopter / Octacopter Motor Mixing
 *
 * Motor layout (top view, nose up, X-frame):
 *
 *   m1(FL,CW)    m2(FR,CCW)       m5(FL2,CCW)   m6(FR2,CW)
 *        \  ^  /                       \  ^  /
 *         \ | /                         \ | /
 *          X-X           or              X-X        (octocopter: 8 motors)
 *         /   \                         /   \
 *        /     \                       /     \
 *   m4(BL,CCW)   m3(BR,CW)       m8(BL2,CW)   m7(BR2,CCW)
 *
 * NED body frame: X=Forward, Y=Right, Z=Down
 * Roll  (about X): positive = right wing down. Left motors UP, right motors DOWN.
 * Pitch (about Y): positive = nose up. Front motors DOWN, rear motors UP.
 * Yaw   (about Z): positive = CW from above. Increase CCW motors, decrease CW motors.
 *
 * PID convention: error = (state - target), so:
 *   Positive target with zero state -> negative output -> signs below account for this.
 */

#include "quadcopter.h"
#include "mix_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <macro.h>
#include <messages.h>

#define MOTOR_TYPE 1 // 1: BRUSHLESS, 2: BRUSHED

#if MOTOR_TYPE == 1
static int g_min_speed = 150;
static int g_max_speed = 1800;
#elif MOTOR_TYPE == 2
static int g_min_speed = 80;
static int g_max_speed = 3000;
#endif

static int g_output_speed[8] = {0};
static state_t g_state = DISARMED;
static rc_att_ctl_t g_rc_att_ctl;
static uint8_t g_log_class = 0;
static uint8_t g_log_msg[32]; // 8 floats
static float g_thrust_p1 = 1.0f;
static float g_thrust_p2 = 0.0f;

/* Thrust linearization: normalize to [0,1], apply p1*t + p2*t^2, scale back.
 * Default p1=1, p2=0 is identity (linear passthrough). */
static double linearize_thrust(double cmd) {
	if (cmd <= g_min_speed) return g_min_speed;
	if (cmd >= g_max_speed) return g_max_speed;
	double range = g_max_speed - g_min_speed;
	double t = (cmd - g_min_speed) / range;  /* normalize to [0,1] */
	double out = g_thrust_p1 * t + g_thrust_p2 * t * t;
	if (out < 0.0) out = 0.0;
	if (out > 1.0) out = 1.0;
	return g_min_speed + out * range;
}

static void mix_motors(mix_control_input_t *input) {
	double base = g_min_speed + input->altitude;
	double roll  = input->roll;
	double pitch = input->pitch;
	double yaw   = input->yaw;

	/* Quad motors */
	double m1 = base - roll - pitch + yaw;   /* FL, CW  */
	double m2 = base + roll - pitch - yaw;   /* FR, CCW */
	double m3 = base + roll + pitch + yaw;   /* BR, CW  */
	double m4 = base - roll + pitch - yaw;   /* BL, CCW */

	/* Octo motors (coaxial or flat) */
	double m5 = base - roll - pitch - yaw;   /* FL2, CCW */
	double m6 = base + roll - pitch + yaw;   /* FR2, CW  */
	double m7 = base + roll + pitch - yaw;   /* BR2, CCW */
	double m8 = base - roll + pitch + yaw;   /* BL2, CW  */

	g_output_speed[0] = LIMIT((int)linearize_thrust(m1), g_min_speed, g_max_speed);
	g_output_speed[1] = LIMIT((int)linearize_thrust(m2), g_min_speed, g_max_speed);
	g_output_speed[2] = LIMIT((int)linearize_thrust(m3), g_min_speed, g_max_speed);
	g_output_speed[3] = LIMIT((int)linearize_thrust(m4), g_min_speed, g_max_speed);
	g_output_speed[4] = LIMIT((int)linearize_thrust(m5), g_min_speed, g_max_speed);
	g_output_speed[5] = LIMIT((int)linearize_thrust(m6), g_min_speed, g_max_speed);
	g_output_speed[6] = LIMIT((int)linearize_thrust(m7), g_min_speed, g_max_speed);
	g_output_speed[7] = LIMIT((int)linearize_thrust(m8), g_min_speed, g_max_speed);
}

static void mix_control_input_update(uint8_t *data, size_t size) {
	if (size < sizeof(mix_control_input_t)) return;
	mix_control_input_t *input = (mix_control_input_t *)data;
	mix_motors(input);
	publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output_speed, sizeof(int) * 8);
}

static void mix_control_loop(uint8_t *data, size_t size) {
	if (g_state == DISARMED || g_state == ARMED) {
		for (int i = 0; i < 8; i++) g_output_speed[i] = 0;
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output_speed, sizeof(int) * 8);
	}
	else if (g_state == READY) {
		for (int i = 0; i < 8; i++) g_output_speed[i] = g_min_speed;
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output_speed, sizeof(int) * 8);
	}
	else if (g_state == TESTING) {
		g_output_speed[0] = LIMIT(g_min_speed + g_rc_att_ctl.yaw / 90   * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[1] = LIMIT(g_min_speed + g_rc_att_ctl.alt / 90   * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[2] = LIMIT(g_min_speed + g_rc_att_ctl.roll / 90  * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[3] = LIMIT(g_min_speed + g_rc_att_ctl.pitch / 90 * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[4] = LIMIT(g_min_speed + g_rc_att_ctl.yaw / 90   * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[5] = LIMIT(g_min_speed + g_rc_att_ctl.alt / 90   * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[6] = LIMIT(g_min_speed + g_rc_att_ctl.roll / 90  * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output_speed[7] = LIMIT(g_min_speed + g_rc_att_ctl.pitch / 90 * (g_max_speed - g_min_speed), 0, g_max_speed);
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output_speed, sizeof(int) * 8);
	}
	/* TAKING_OFF / FLYING / LANDING are driven by MIX_CONTROL_UPDATE from attitude_control */
}

static void state_update(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_state = (state_t)data[0];
}

static void move_in_control_update(uint8_t *data, size_t size) {
	if (size > sizeof(rc_att_ctl_t)) size = sizeof(rc_att_ctl_t);
	memcpy(&g_rc_att_ctl, data, size);
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_log_class = (data[0] == LOG_CLASS_MIX_CONTROL) ? data[0] : 0;
}

static void loop_logger(uint8_t *data, size_t size) {
	if (g_log_class == 0) return;
	for (int i = 0; i < 8; i++) {
		float val = (float)g_output_speed[i];
		memcpy(&g_log_msg[i * 4], &val, sizeof(float));
	}
	publish(SEND_LOG, g_log_msg, sizeof(g_log_msg));
}

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t t;
	memcpy(&t, data, sizeof(tuning_params_t));
	g_min_speed = (int)t.motor_min;
	g_max_speed = (int)t.motor_max;
	g_thrust_p1 = t.thrust_p1;
	g_thrust_p2 = t.thrust_p2;
}

void quadcopter_setup(void) {
	speed_control_config_t cfg = {{
		PORT_DSHOT, PORT_DSHOT, PORT_DSHOT, PORT_DSHOT,
		PORT_DSHOT, PORT_DSHOT, PORT_DSHOT, PORT_DSHOT
	}};
	publish(SPEED_CONTROL_SETUP, (uint8_t *)&cfg, sizeof(cfg));

	subscribe(MIX_CONTROL_UPDATE, mix_control_input_update);
	subscribe(ATT_CTL_SCHEDULER, mix_control_loop);
	subscribe(FLIGHT_STATE_UPDATE, state_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_10HZ, loop_logger);
	subscribe(TUNING_READY, on_tuning_ready);
}
