#include "remote_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

#define PULSE_WIDTH_TO_DEGREE (90.0 / 490)
#define RC_CMD_ID 0x02
#define RC_SUB_CMD_MOVE 0x00
#define RC_DATA_SIZE 18
#define RC_DATA_OFFSET 4

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

static uint8_t g_rc_data_raw[RC_DATA_SIZE] = {0};
static rc_att_ctl_t g_rc_att_ctl;
static rc_state_ctl_t g_rc_state_ctl;

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
		}
	}
}

static void handle_internal_message(uint8_t *data, size_t size) {
	publish(RC_STATE_UPDATE, (uint8_t*)&g_rc_state_ctl, sizeof(rc_state_ctl_t));
	publish(RC_MOVE_IN_UPDATE, (uint8_t*)&g_rc_att_ctl, sizeof(rc_att_ctl_t));
}

void remote_control_setup(void) {
	subscribe(DB_MESSAGE, on_internal_message);
	subscribe(SCHEDULER_25HZ, handle_internal_message);
}
