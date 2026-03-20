/**
 ******************************************************************************
 * @file           : platform_common.c
 * @brief          : Common platform utilities — LED, delay, time, console
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>
#include <string.h>

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

void platform_reset(void) {
	NVIC_SystemReset();
}

void platform_get_chip_id(uint8_t *id_out) {
	uint32_t w0 = HAL_GetUIDw0();
	uint32_t w1 = HAL_GetUIDw1();
	memcpy(&id_out[0], &w0, 4);
	memcpy(&id_out[4], &w1, 4);
}
