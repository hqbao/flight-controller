/**
 ******************************************************************************
 * @file           : platform_common.c
 * @brief          : Common platform utilities — LED, delay, time, console
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>

/* --- Platform API -------------------------------------------------------- */

void platform_toggle_led(char led) {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}

void platform_delay(uint32_t ms) {
	HAL_Delay(ms);
}

uint32_t platform_time_ms(void) {
	return HAL_GetTick();
}

void platform_console(const char *format, ...) {
	/* Not implemented on this board */
}
