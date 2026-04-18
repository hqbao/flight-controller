/*
 * Bicopter Dual Tilt-Rotor Motor + Servo Mixing
 *
 * Layout (top view, nose up):
 *
 *   m1(L,CCW)              m2(R,CW)
 *   s1(L tilt)             s2(R tilt)
 *       |         ^         |
 *       |---------|---------|
 *                 |
 *
 * Two motors (DShot) + two tilt servos (PWM).
 * Motors provide thrust (altitude + roll).
 * Servos tilt motors to vector thrust (pitch + yaw).
 *
 * NED body frame: X=Forward, Y=Right, Z=Down
 * Roll  (about X): differential motor thrust (left up, right down)
 * Pitch (about Y): collective servo tilt (both tilt same direction)
 * Yaw   (about Z): differential servo tilt (opposite directions)
 *
 * Output array [8 ints]:
 *   [0] = left motor  (DShot value, MIN_SPEED..MAX_SPEED)  — Port1 TIM1 CH1
 *   [1] = right motor (DShot value, MIN_SPEED..MAX_SPEED)  — Port2 TIM1 CH2
 *   [2..3] = 0 (unused, TIM1 CH3-4 reserved for DShot)
 *   [4] = left servo  (PWM microseconds, SERVO_MIN..SERVO_MAX) — Port5 TIM2 CH1
 *   [5] = right servo (PWM microseconds, SERVO_MIN..SERVO_MAX) — Port6 TIM2 CH2
 *   [6..7] = 0 (unused)
 *
 * PID convention: error = (state - target), so:
 *   Positive target with zero state -> negative output -> signs below account for this.
 */

#include "bicopter.h"
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

static int g_servo_min    = 1000;
static int g_servo_max    = 2000;
static int g_servo_center = 1500;

static int g_output[8] = {0};
static state_t g_state = DISARMED;
static rc_att_ctl_t g_rc_att_ctl;
static uint8_t g_log_class = 0;
static uint8_t g_log_msg[32]; // 8 floats

static void mix_motors(mix_control_input_t *input) {
	double base  = g_min_speed + input->altitude;
	double roll  = input->roll;
	double pitch = input->pitch;
	double yaw   = input->yaw;

	/* Motors: differential thrust for roll control */
	double m1 = base - roll;   /* left motor  */
	double m2 = base + roll;   /* right motor */

	/* Servos: collective tilt for pitch, differential tilt for yaw.
	 * S2 is mirrored on the airframe, so negate pitch to get same physical tilt.
	 * Both share same yaw sign — mirrored mount makes them tilt opposite. */
	double s1 = g_servo_center + pitch - yaw;   /* left servo  */
	double s2 = g_servo_center - pitch - yaw;   /* right servo (mirrored mount) */

	g_output[0] = LIMIT((int)m1, g_min_speed, g_max_speed);
	g_output[1] = LIMIT((int)m2, g_min_speed, g_max_speed);
	g_output[2] = 0;
	g_output[3] = 0;
	g_output[4] = LIMIT((int)s1, g_servo_min, g_servo_max);
	g_output[5] = LIMIT((int)s2, g_servo_min, g_servo_max);
	g_output[6] = 0;
	g_output[7] = 0;
}

static void mix_control_input_update(uint8_t *data, size_t size) {
	if (size < sizeof(mix_control_input_t)) return;
	mix_control_input_t *input = (mix_control_input_t *)data;
	mix_motors(input);
	publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output, sizeof(int) * 8);
}

static void mix_control_loop(uint8_t *data, size_t size) {
	if (g_state == DISARMED || g_state == ARMED) {
		g_output[0] = 0;
		g_output[1] = 0;
		g_output[2] = 0;
		g_output[3] = 0;
		g_output[4] = g_servo_center;
		g_output[5] = g_servo_center;
		g_output[6] = 0;
		g_output[7] = 0;
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output, sizeof(int) * 8);
	}
	else if (g_state == READY) {
		g_output[0] = g_min_speed;
		g_output[1] = g_min_speed;
		g_output[2] = 0;
		g_output[3] = 0;
		g_output[4] = g_servo_center;
		g_output[5] = g_servo_center;
		g_output[6] = 0;
		g_output[7] = 0;
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output, sizeof(int) * 8);
	}
	else if (g_state == TESTING) {
		/* RC sticks: yaw->m1, alt->m2, roll->s1, pitch->s2 */
		g_output[0] = LIMIT(g_min_speed + g_rc_att_ctl.yaw / 90   * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output[1] = LIMIT(g_min_speed + g_rc_att_ctl.alt / 90   * (g_max_speed - g_min_speed), 0, g_max_speed);
		g_output[2] = 0;
		g_output[3] = 0;
		g_output[4] = LIMIT(g_servo_center + (int)(g_rc_att_ctl.roll / 90  * (g_servo_max - g_servo_center)), g_servo_min, g_servo_max);
		g_output[5] = LIMIT(g_servo_center + (int)(g_rc_att_ctl.pitch / 90 * (g_servo_max - g_servo_center)), g_servo_min, g_servo_max);
		g_output[6] = 0;
		g_output[7] = 0;
		publish(SPEED_CONTROL_UPDATE, (uint8_t *)g_output, sizeof(int) * 8);
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
		float val = (float)g_output[i];
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
	g_servo_min = (int)t.servo_min;
	g_servo_max = (int)t.servo_max;
	g_servo_center = (int)t.servo_center;
}

void bicopter_setup(void) {
	/* Motors on TIM1 (Port1-2), servos on TIM2 (Port5-6).
	 * DShot and servo PWM must be on separate timers — each timer's
	 * prescaler/period is shared across all 4 channels. */
	speed_control_config_t cfg = {{
		PORT_DSHOT, PORT_DSHOT, PORT_DISABLED, PORT_DISABLED,
		PORT_PWM, PORT_PWM, PORT_DISABLED, PORT_DISABLED
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
