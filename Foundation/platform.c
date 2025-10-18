#include "platform.h"
#include <pubsub.h>

static platform_t g_platform;

void platform_register_io_functions(
	i2c_write_read_dma_t i2c_write_read_dma,
	i2c_write_read_t i2c_write_read,
	i2c_read_t i2c_read,
	i2c_write_t i2c_write,
	uart_send_t uart_send,
	pwm_init_t pwm_init,
	pwm_send_t pwm_send,
	dshot_init_t dshot_init,
	dshot_send_t dshot_send,
	dshot_ex_init_t dshot_ex_init,
	dshot_ex_send_t dshot_ex_send) {
	g_platform.port.i2c_write_read_dma = i2c_write_read_dma;
	g_platform.port.i2c_write_read = i2c_write_read;
	g_platform.port.i2c_read = i2c_read;
	g_platform.port.i2c_write = i2c_write;
	g_platform.port.uart_send = uart_send;
	g_platform.port.pwm_init = pwm_init;
	g_platform.port.pwm_send = pwm_send;
	g_platform.port.dshot_init = dshot_init;
	g_platform.port.dshot_send = dshot_send;
	g_platform.port.dshot_ex_init = dshot_ex_init;
	g_platform.port.dshot_ex_send = dshot_ex_send;
}

void platform_register_toggle_led(toggle_led_t toggle_led) {
	g_platform.toggle_led = toggle_led;
}

void platform_register_time_ms(time_ms_t time_ms) {
	g_platform.time_ms = time_ms;
}

void platform_register_delay(delay_t delay) {
	g_platform.delay = delay;
}

void platform_register_storage(storage_read_t storage_read, storage_write_t storage_write) {
	g_platform.storage_read = storage_read;
	g_platform.storage_write = storage_write;
}

platform_t* get_platform(void) {
	return &g_platform;
}

void platform_scheduler_1hz(void) {
	publish(SCHEDULER_1HZ, NULL, 0);
}

void platform_scheduler_5hz(void) {
	publish(SCHEDULER_5HZ, NULL, 0);
}

void platform_scheduler_10hz(void) {
	publish(SCHEDULER_10HZ, NULL, 0);
}

void platform_scheduler_25hz(void) {
	publish(SCHEDULER_25HZ, NULL, 0);
}

void platform_scheduler_50hz(void) {
	publish(SCHEDULER_50HZ, NULL, 0);
}

void platform_scheduler_100hz(void) {
	publish(SCHEDULER_100HZ, NULL, 0);
}

void platform_scheduler_250hz(void) {
	publish(SCHEDULER_250HZ, NULL, 0);
}

void platform_scheduler_500hz(void) {
	publish(SCHEDULER_500HZ, NULL, 0);
}

void platform_scheduler_1khz(void) {
	publish(SCHEDULER_1KHZ, NULL, 0);
}

void platform_scheduler_2khz(void) {
	publish(SCHEDULER_2KHZ, NULL, 0);
}

void platform_scheduler_4khz(void) {
	publish(SCHEDULER_4KHZ, NULL, 0);
}

void platform_scheduler_8khz(void) {
	publish(SCHEDULER_8KHZ, NULL, 0);
}

void platform_receive_internal_message(uint8_t *data, uint16_t size) {
	publish(INTERNAL_MESSAGE, data, size);
}

void platform_receive_external_message(uint8_t *data, uint16_t size) {
	publish(EXTERNAL_MESSAGE, data, size);
}

void platform_on_fault_detected(uint8_t *data, uint16_t size) {
	publish(FAULT_DETECTION, data, size);
}

void platform_loop(void) {
	publish(LOOP, NULL, 0);
}
