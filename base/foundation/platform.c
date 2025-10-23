#include "platform.h"
#include <pubsub.h>

void platform_scheduler_1hz(void*) {
	publish(SCHEDULER_1HZ, NULL, 0);
}

void platform_scheduler_5hz(void*) {
	publish(SCHEDULER_5HZ, NULL, 0);
}

void platform_scheduler_10hz(void*) {
	publish(SCHEDULER_10HZ, NULL, 0);
}

void platform_scheduler_25hz(void*) {
	publish(SCHEDULER_25HZ, NULL, 0);
}

void platform_scheduler_50hz(void*) {
	publish(SCHEDULER_50HZ, NULL, 0);
}

void platform_scheduler_100hz(void*) {
	publish(SCHEDULER_100HZ, NULL, 0);
}

void platform_scheduler_250hz(void*) {
	publish(SCHEDULER_250HZ, NULL, 0);
}

void platform_scheduler_500hz(void*) {
	publish(SCHEDULER_500HZ, NULL, 0);
}

void platform_scheduler_1khz(void*) {
	publish(SCHEDULER_1KHZ, NULL, 0);
}

void platform_scheduler_2khz(void*) {
	publish(SCHEDULER_2KHZ, NULL, 0);
}

void platform_scheduler_4khz(void*) {
	publish(SCHEDULER_4KHZ, NULL, 0);
}

void platform_scheduler_8khz(void*) {
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

void platform_i2c_data_dma_callback(i2c_port__t port) {
	publish(I2C_CALLBACK_UPDATE, (uint8_t*)&port, 1);
}

void platform_spi_data_dma_callback(spi_port_t port) {
	publish(SPI_CALLBACK_UPDATE, (uint8_t*)&port, 1);
}

void platform_uart_data_dma_callback(uart_port_t port) {
	publish(UART_CALLBACK_UPDATE, (uint8_t*)&port, 1);
}
