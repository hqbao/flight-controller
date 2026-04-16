/**
 ******************************************************************************
 * @file           : platform_pwm.c
 * @brief          : PWM / DShot platform driver — STM32H7 HAL implementation
 ******************************************************************************
 */

#include "platform_hw.h"
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

/* --- Timer constants for servo PWM --------------------------------------- */
/* 240 MHz timer clock / (PSC+1) = 1 MHz tick (1 µs resolution)
 * 1 MHz / (ARR+1) = PWM frequency
 * CCR 1000–2000 → 1000–2000 µs pulse width — standard servo range.
 *
 * NOTE: All 4 channels of a timer share one timebase. DShot and servo PWM
 * MUST NOT coexist on the same timer. */
#define SERVO_PWM_PRESCALER  239    /* 240 MHz / 240 = 1 MHz tick */

#define SERVO_PWM_50HZ      19999  /* 1 MHz / 20000 = 50 Hz  (analog servo) */
#define SERVO_PWM_250HZ     3999   /* 1 MHz / 4000  = 250 Hz (digital servo) */
#define SERVO_PWM_PERIOD    SERVO_PWM_50HZ

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
	/* Reconfigure timer for 50 Hz servo PWM (1 µs tick resolution) */
	__HAL_TIM_SET_PRESCALER(g_pwm_timers[port], SERVO_PWM_PRESCALER);
	__HAL_TIM_SET_AUTORELOAD(g_pwm_timers[port], SERVO_PWM_PERIOD);

	HAL_StatusTypeDef status = HAL_TIM_PWM_Start(g_pwm_timers[port],
			g_pwm_timer_channels[port]);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_pwm_send(pwm_port_t port, uint32_t data) {
	set_pwm(g_pwm_time_bases[port], g_pwm_timer_channels[port], data);
	return PLATFORM_OK;
}
