/**
 ******************************************************************************
 * @file           : platform_pwm.c
 * @brief          : PWM / DShot platform driver — STM32H7 HAL implementation
 ******************************************************************************
 */

#include "platform_hw.h"
#include "dshot.h"
#include "dshot_ex.h"
#include <platform.h>

/* --- Port mappings ------------------------------------------------------- */

static TIM_HandleTypeDef *g_pwm_timers[8] = {
	&htim1, &htim1, &htim1, &htim1,
	&htim2, &htim2, &htim2, &htim2
};

static uint32_t g_pwm_timer_channels[8] = {
	TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
	TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static TIM_TypeDef *g_pwm_time_bases[8] = {
	TIM1, TIM1, TIM1, TIM1,
	TIM2, TIM2, TIM2, TIM2
};

static dshot_t g_dshots[8];
static dshot_ex_t g_dshot_exs[4];

/* --- Internal helpers ---------------------------------------------------- */

static void set_pwm(TIM_TypeDef *timer_base, uint32_t channel, uint32_t duty) {
	switch (channel) {
	case TIM_CHANNEL_1: timer_base->CCR1 = duty; break;
	case TIM_CHANNEL_2: timer_base->CCR2 = duty; break;
	case TIM_CHANNEL_3: timer_base->CCR3 = duty; break;
	case TIM_CHANNEL_4: timer_base->CCR4 = duty; break;
	default: break;
	}
}

/* --- Platform API: PWM --------------------------------------------------- */

char platform_pwm_init(pwm_port_t port) {
	HAL_StatusTypeDef status = HAL_TIM_PWM_Start(g_pwm_timers[port],
			g_pwm_timer_channels[port]);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_pwm_send(pwm_port_t port, uint32_t data) {
	set_pwm(g_pwm_time_bases[port], g_pwm_timer_channels[port], data);
	return PLATFORM_OK;
}

/* --- Platform API: DShot ------------------------------------------------- */

char platform_dshot_init(dshot_port_t port) {
	switch (port) {
	case DSHOT_PORT1:
		dshot_init(&g_dshots[0], &htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1,
				(uint32_t)&((&htim1)->Instance->CCR1));
		break;
	case DSHOT_PORT2:
		dshot_init(&g_dshots[1], &htim1, TIM_CHANNEL_2, TIM_DMA_ID_CC2,
				(uint32_t)&((&htim1)->Instance->CCR2));
		break;
	case DSHOT_PORT3:
		dshot_init(&g_dshots[2], &htim1, TIM_CHANNEL_3, TIM_DMA_ID_CC3,
				(uint32_t)&((&htim1)->Instance->CCR3));
		break;
	case DSHOT_PORT4:
		dshot_init(&g_dshots[3], &htim1, TIM_CHANNEL_4, TIM_DMA_ID_CC4,
				(uint32_t)&((&htim1)->Instance->CCR4));
		break;
	case DSHOT_PORT5:
		dshot_init(&g_dshots[4], &htim2, TIM_CHANNEL_1, TIM_DMA_ID_CC1,
				(uint32_t)&((&htim2)->Instance->CCR1));
		break;
	case DSHOT_PORT6:
		dshot_init(&g_dshots[5], &htim2, TIM_CHANNEL_2, TIM_DMA_ID_CC2,
				(uint32_t)&((&htim2)->Instance->CCR2));
		break;
	case DSHOT_PORT7:
		dshot_init(&g_dshots[6], &htim2, TIM_CHANNEL_3, TIM_DMA_ID_CC3,
				(uint32_t)&((&htim2)->Instance->CCR3));
		break;
	case DSHOT_PORT8:
		dshot_init(&g_dshots[7], &htim2, TIM_CHANNEL_4, TIM_DMA_ID_CC4,
				(uint32_t)&((&htim2)->Instance->CCR4));
		break;
	default:
		break;
	}
	return PLATFORM_OK;
}

char platform_dshot_send(dshot_port_t port, uint16_t data) {
	dshot_write(&g_dshots[port], data);
	return PLATFORM_OK;
}

/* --- Platform API: DShot Extended ---------------------------------------- */

char platform_dshot_ex_init(dshot_ex_port_t port) {
	switch (port) {
	case DSHOT_EX_PORT1:
		dshot_ex_init(&g_dshot_exs[0], &htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1,
				(uint32_t)&((&htim1)->Instance->CCR1));
		break;
	case DSHOT_EX_PORT2:
		dshot_ex_init(&g_dshot_exs[1], &htim1, TIM_CHANNEL_2, TIM_DMA_ID_CC2,
				(uint32_t)&((&htim1)->Instance->CCR2));
		break;
	case DSHOT_EX_PORT3:
		dshot_ex_init(&g_dshot_exs[2], &htim1, TIM_CHANNEL_3, TIM_DMA_ID_CC3,
				(uint32_t)&((&htim1)->Instance->CCR3));
		break;
	case DSHOT_EX_PORT4:
		dshot_ex_init(&g_dshot_exs[3], &htim1, TIM_CHANNEL_4, TIM_DMA_ID_CC4,
				(uint32_t)&((&htim1)->Instance->CCR4));
		break;
	default:
		break;
	}
	return PLATFORM_OK;
}

char platform_dshot_ex_send(dshot_ex_port_t port, uint32_t data) {
	dshot_ex_write(&g_dshot_exs[port], data);
	return PLATFORM_OK;
}
