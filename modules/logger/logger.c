#include "logger.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>
#include <macro.h>
#include <string.h>

#define LOGGER_PAYLOAD_SIZE 1024
#define LOGGER_OUTPUT_SIZE 128
#define LOGGER_HEADER_SIZE 6
#define LOGGER_CHECKSUM_SIZE 2

static uint8_t g_payload[LOGGER_PAYLOAD_SIZE] = {0};
static uint16_t g_payload_size = 0;
static uint8_t g_output_msg[LOGGER_OUTPUT_SIZE] = {'d', 'b', 0x00 /* ID */, 0x00 /* Class */};

static void data_update(uint8_t *data, size_t size) {
    if (size > LOGGER_PAYLOAD_SIZE) {
        size = LOGGER_PAYLOAD_SIZE;
    }
	g_payload_size = size;
	memcpy(g_payload, data, size);
}

static void logger_loop_25hz(uint8_t *data, size_t size) {
    if (g_payload_size > LOGGER_OUTPUT_SIZE - LOGGER_HEADER_SIZE - LOGGER_CHECKSUM_SIZE) {
        g_payload_size = LOGGER_OUTPUT_SIZE - LOGGER_HEADER_SIZE - LOGGER_CHECKSUM_SIZE;
    }

	int buf_idx = LOGGER_HEADER_SIZE;
	memcpy(&g_output_msg[buf_idx], &g_payload, g_payload_size); buf_idx += g_payload_size;

	uint16_t payload_size = buf_idx - LOGGER_HEADER_SIZE;
	memcpy(&g_output_msg[4], &payload_size, 2);
	
	// Calculate checksum: sum of ID + Class + length bytes + payload bytes
	uint16_t checksum = g_output_msg[2] + g_output_msg[3] + g_output_msg[4] + g_output_msg[5];
	for (int i = LOGGER_HEADER_SIZE; i < buf_idx; i++) {
		checksum += g_output_msg[i];
	}
	memcpy(&g_output_msg[buf_idx], &checksum, 2);

	platform_uart_send(UART_PORT1, g_output_msg, buf_idx + 2);
}

void logger_setup(void) {
	subscribe(MONITOR_DATA, data_update);
	subscribe(SCHEDULER_25HZ, logger_loop_25hz);
}

