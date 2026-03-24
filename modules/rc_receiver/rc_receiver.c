#include "rc_receiver.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

#define PULSE_WIDTH_TO_DEGREE (90.0 / 490)
#define RC_CMD_ID 0x02
#define RC_SUB_CMD_MOVE 0x00
#define RC_DATA_SIZE 18
#define RC_DATA_OFFSET 4

static uint8_t g_rc_data_raw[RC_DATA_SIZE] = {0};
static rc_att_ctl_t g_rc_att_ctl;
static rc_state_ctl_t g_rc_state_ctl;
static uint8_t g_log_active = 0;
static uint32_t g_msg_count = 0;  /* DB_MESSAGE_UPDATE receive counter */

static void on_internal_message(uint8_t *data, size_t size) {
	if (data[0] == RC_CMD_ID) { // Command
		if (data[1] == RC_SUB_CMD_MOVE) { // Move in
			memcpy(g_rc_data_raw, &data[RC_DATA_OFFSET], RC_DATA_SIZE);
            
            int32_t roll, pitch, yaw, alt;
            memcpy(&roll, &g_rc_data_raw[0], sizeof(int32_t));
            memcpy(&pitch, &g_rc_data_raw[4], sizeof(int32_t));
            memcpy(&yaw, &g_rc_data_raw[8], sizeof(int32_t));
            memcpy(&alt, &g_rc_data_raw[12], sizeof(int32_t));

			g_rc_att_ctl.roll 	= (float)roll * PULSE_WIDTH_TO_DEGREE;
			g_rc_att_ctl.pitch 	= (float)pitch * PULSE_WIDTH_TO_DEGREE;
			g_rc_att_ctl.yaw 	= (float)yaw * PULSE_WIDTH_TO_DEGREE;
			g_rc_att_ctl.alt 	= (float)alt * PULSE_WIDTH_TO_DEGREE;

			g_rc_state_ctl.state = g_rc_data_raw[16];
			g_rc_state_ctl.mode = g_rc_data_raw[17];
			g_msg_count++;
		}
	}
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_log_active = (data[0] == LOG_CLASS_RC_RECEIVER) ? 1 : 0;
}

static void handle_internal_message(uint8_t *data, size_t size) {
	publish(RC_STATE_UPDATE, (uint8_t*)&g_rc_state_ctl, sizeof(rc_state_ctl_t));
	publish(RC_MOVE_IN_UPDATE, (uint8_t*)&g_rc_att_ctl, sizeof(rc_att_ctl_t));

	if (g_log_active) {
		/* 7 floats: roll, pitch, yaw, alt (deg), state, mode, msg_count */
		float buf[7];
		buf[0] = g_rc_att_ctl.roll;
		buf[1] = g_rc_att_ctl.pitch;
		buf[2] = g_rc_att_ctl.yaw;
		buf[3] = g_rc_att_ctl.alt;
		buf[4] = (float)g_rc_state_ctl.state;
		buf[5] = (float)g_rc_state_ctl.mode;
		buf[6] = (float)g_msg_count;
		publish(SEND_LOG, (uint8_t *)buf, sizeof(buf));
	}
}

void rc_receiver_setup(void) {
	subscribe(DB_MESSAGE_UPDATE, on_internal_message);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_25HZ, handle_internal_message);
}
