/**
 ******************************************************************************
 * @file           : platform_rc.c
 * @brief          : RC receiver (PPM) platform driver — STM32H7 HAL
 *
 * Decodes PPM pulse train from TIM16 input capture. Outputs a DB-protocol
 * style message with roll/pitch/yaw/throttle/state/mode via
 * platform_receive_db_message().
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>
#include <string.h>

/* --- Constants ----------------------------------------------------------- */

#define MIN_RC_IN_CAP  1000
#define AVR_RC_IN_CAP  1500
#define MAX_RC_IN_CAP  2000
#define RANGE_RC_IN_CAP (MAX_RC_IN_CAP - MIN_RC_IN_CAP)
#define SIGNIFICANT_CHANGE 20
#define MIN_THROTTLE ((int)(-RANGE_RC_IN_CAP / 2) + SIGNIFICANT_CHANGE)
#define MIN_YAW      ((int)(-RANGE_RC_IN_CAP / 2) + SIGNIFICANT_CHANGE)
#define MIN_PITCH    ((int)(-RANGE_RC_IN_CAP / 2) + SIGNIFICANT_CHANGE)
#define MAX_ROLL     ((int)( RANGE_RC_IN_CAP / 2) - SIGNIFICANT_CHANGE)

/* --- Static state -------------------------------------------------------- */

static int g_rc_params[16];
static int g_rc_param_idx = 0;
static int cap_value_prev = 0;
static uint8_t g_rc_db_msg_payload[22] = {0x02 /* Command */, 0x00 /* Set point */, 0, 0x0f};

/* --- HAL Callback -------------------------------------------------------- */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
		switch (htim->Channel) {
		case HAL_TIM_ACTIVE_CHANNEL_1: {
			int cap_value = HAL_TIM_ReadCapturedValue(&htim16, TIM_CHANNEL_1);
			int value = cap_value - cap_value_prev;
			cap_value_prev = cap_value;

			if (value < 0) value += 65536;

			if (g_rc_param_idx < 17) {
				g_rc_params[g_rc_param_idx] = value;
				g_rc_param_idx += 1;
			}

			if (value > MAX_RC_IN_CAP + MIN_RC_IN_CAP) {
				g_rc_param_idx = 0;

				int roll  = g_rc_params[3] - 1500;
				int pitch = g_rc_params[2] - 1500;
				int yaw   = g_rc_params[0] - 1500;
				int alt   = g_rc_params[1] - 1500;
				uint8_t state = g_rc_params[4] < 1250 ? 0 : (g_rc_params[4] > 1750 ? 2 : 1);
				uint8_t mode  = g_rc_params[5] < 1250 ? 0 : (g_rc_params[5] > 1750 ? 2 : 1);

				if (roll <= -10) roll = roll + 10;
				else if (roll >= 10) roll = roll - 10;
				else roll = 0;

				if (pitch <= -10) pitch = pitch + 10;
				else if (pitch >= 10) pitch = pitch - 10;
				else pitch = 0;

				if (yaw <= -10) yaw = yaw + 10;
				else if (yaw >= 10) yaw = yaw - 10;
				else yaw = 0;

				if (alt <= -10) alt = alt + 10;
				else if (alt >= 10) alt = alt - 10;
				else alt = 0;

				memcpy(&g_rc_db_msg_payload[4],  (uint8_t *)&roll,  sizeof(int));
				memcpy(&g_rc_db_msg_payload[8],  (uint8_t *)&pitch, sizeof(int));
				memcpy(&g_rc_db_msg_payload[12], (uint8_t *)&yaw,   sizeof(int));
				memcpy(&g_rc_db_msg_payload[16], (uint8_t *)&alt,   sizeof(int));
				g_rc_db_msg_payload[20] = state;
				g_rc_db_msg_payload[21] = mode;
				platform_receive_db_message(g_rc_db_msg_payload, 22);
			}
			break;
		}
		default:
			break;
		}
	}
}
