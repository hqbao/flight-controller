#include "mix_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <macro.h>
#include <messages.h>

#define MOTOR_TYPE 1 // 1: BRUSHLESS, 2: BRUSHED

#if MOTOR_TYPE == 1
#define MIN_SPEED 150
#define MAX_SPEED 1800
#elif MOTOR_TYPE == 2
#define MIN_SPEED 80
#define MAX_SPEED 3000
#endif // MOTOR_TYPE

static int g_output_speed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static state_t g_state = DISARMED;
static rc_att_ctl_t g_rc_att_ctl;
static uint8_t g_log_class = 0;
static uint8_t g_log_msg[32]; // 8 floats

static void mix_motors(mix_control_input_t *input) {
	/*
	 * NED Motor Mixing — X-frame quadcopter
	 *
	 * Motor layout (top view, nose up):
	 *   m1(FL,CW)    m2(FR,CCW)
	 *        \  ^  /
	 *         \ | /
	 *          X-X
	 *         /   \
	 *        /     \
	 *   m4(BL,CCW)   m3(BR,CW)
	 *
	 * NED body frame: X=Forward, Y=Right, Z=Down
	 * Roll  (about X): positive = right wing down. Left motors UP, right motors DOWN.
	 * Pitch (about Y): positive = nose up. Front motors DOWN, rear motors UP.
	 * Yaw   (about Z): positive = CW from above. Increase CCW motors, decrease CW motors.
	 *
	 * PID convention: error = (state - target), so:
	 *   Positive target with zero state → negative output → signs below account for this.
	 */
	double base = MIN_SPEED + input->altitude;
	double roll  = input->roll;
	double pitch = input->pitch;
	double yaw   = input->yaw;

	double m1 = base - roll - pitch + yaw;
	double m2 = base + roll - pitch - yaw;
	double m3 = base + roll + pitch + yaw;
	double m4 = base - roll + pitch - yaw;

	double m5 = base - roll - pitch - yaw;
	double m6 = base + roll - pitch + yaw;
	double m7 = base + roll + pitch - yaw;
	double m8 = base - roll + pitch + yaw;

	g_output_speed[0] = LIMIT((int)m1, MIN_SPEED, MAX_SPEED);
	g_output_speed[1] = LIMIT((int)m2, MIN_SPEED, MAX_SPEED);
	g_output_speed[2] = LIMIT((int)m3, MIN_SPEED, MAX_SPEED);
	g_output_speed[3] = LIMIT((int)m4, MIN_SPEED, MAX_SPEED);

	g_output_speed[4] = LIMIT((int)m5, MIN_SPEED, MAX_SPEED);
	g_output_speed[5] = LIMIT((int)m6, MIN_SPEED, MAX_SPEED);
	g_output_speed[6] = LIMIT((int)m7, MIN_SPEED, MAX_SPEED);
	g_output_speed[7] = LIMIT((int)m8, MIN_SPEED, MAX_SPEED);
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
		for (int i = 0; i < 8; i++) g_output_speed[i] = MIN_SPEED;
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output_speed, sizeof(int) * 8);
	}
	else if (g_state == TESTING) {
		g_output_speed[0] = LIMIT(MIN_SPEED + g_rc_att_ctl.yaw / 90   * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[1] = LIMIT(MIN_SPEED + g_rc_att_ctl.alt / 90   * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[2] = LIMIT(MIN_SPEED + g_rc_att_ctl.roll / 90  * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[3] = LIMIT(MIN_SPEED + g_rc_att_ctl.pitch / 90 * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[4] = LIMIT(MIN_SPEED + g_rc_att_ctl.yaw / 90   * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[5] = LIMIT(MIN_SPEED + g_rc_att_ctl.alt / 90   * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[6] = LIMIT(MIN_SPEED + g_rc_att_ctl.roll / 90  * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
		g_output_speed[7] = LIMIT(MIN_SPEED + g_rc_att_ctl.pitch / 90 * (MAX_SPEED - MIN_SPEED), 0, MAX_SPEED);
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

void mix_control_setup(void) {
	publish(SPEED_CONTROL_SETUP, NULL, 0);

	subscribe(MIX_CONTROL_UPDATE, mix_control_input_update);
	subscribe(SCHEDULER_500HZ, mix_control_loop);
	subscribe(FLIGHT_STATE_UPDATE, state_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_10HZ, loop_logger);
}
