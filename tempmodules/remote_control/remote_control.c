#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

#define PULSE_WIDTH_TO_DEGREE (90.0 / 490)

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float alt;
} rc_att_ctl_t;

typedef struct {
	uint8_t state;
	uint8_t mode;
} rc_state_ctl_t;

static uint8_t g_rc_data_raw[18] = {0};
static rc_att_ctl_t g_rc_att_ctl;
static rc_state_ctl_t g_rc_state_ctl;

static void on_internal_message(uint8_t *data, size_t size) {
	if (data[0] == 0x02) { // Command
		if (data[1] == 0x00) { // Move in
			memcpy(g_rc_data_raw, &data[4], 18);
			g_rc_att_ctl.roll 	= (float)(*(int*)&g_rc_data_raw[0]) * PULSE_WIDTH_TO_DEGREE;
			g_rc_att_ctl.pitch 	= (float)(*(int*)&g_rc_data_raw[4]) * PULSE_WIDTH_TO_DEGREE;
			g_rc_att_ctl.yaw 	= (float)(*(int*)&g_rc_data_raw[8]) * PULSE_WIDTH_TO_DEGREE;
			g_rc_att_ctl.alt 	= (float)(*(int*)&g_rc_data_raw[12]) * PULSE_WIDTH_TO_DEGREE;

			g_rc_state_ctl.state = g_rc_data_raw[16];
			g_rc_state_ctl.mode = g_rc_data_raw[17];
		}
	}
}

static void handle_internal_message(uint8_t *data, size_t size) {
	publish(COMMAND_SET_STATE, (uint8_t*)&g_rc_state_ctl, sizeof(rc_state_ctl_t));
	publish(COMMAND_SET_MOVE_IN, (uint8_t*)&g_rc_att_ctl, sizeof(rc_att_ctl_t));
}

void remote_control_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_internal_message);
	subscribe(SCHEDULER_25HZ, handle_internal_message);
}
