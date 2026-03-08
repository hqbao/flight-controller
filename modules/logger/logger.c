#include "logger.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <string.h>

// Baud rate budget (UART_PORT1 = 9600 baud, 8N1 = 960 bytes/sec):
//   Frame = header(6) + payload(N) + checksum(2) = N + 8 bytes
//   At 25 Hz: max frame = 960/25 = 38 bytes -> max payload = 30 bytes (7 floats)
//   At 10 Hz: max frame = 960/10 = 96 bytes -> max payload = 88 bytes (22 floats)
//   Callers control send rate via their own scheduler subscriptions.

#define LOGGER_MAX_PAYLOAD 120
#define LOGGER_OUTPUT_SIZE 128
#define LOGGER_HEADER_SIZE 6
#define LOGGER_CHECKSUM_SIZE 2

static uint8_t g_log_class = LOG_CLASS_NONE;
static uint8_t g_output_msg[LOGGER_OUTPUT_SIZE] = {'d', 'b', 0x00 /* ID */, 0x00 /* Class */};

// Receive DB message from UART (sent by Python tools)
// DB_CMD_LOG_CLASS (0x03): payload[0] = log class to activate
static void on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;
	if (data[0] != DB_CMD_LOG_CLASS) return;

	g_log_class = data[4];
	g_output_msg[3] = g_log_class;
	publish(NOTIFY_LOG_CLASS, &g_log_class, 1);
}

static void send_log(uint8_t *data, size_t size) {
	if (size > LOGGER_MAX_PAYLOAD) {
		size = LOGGER_MAX_PAYLOAD;
	}

	int buf_idx = LOGGER_HEADER_SIZE;
	memcpy(&g_output_msg[buf_idx], data, size);
	buf_idx += size;

	uint16_t payload_size = (uint16_t)size;
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
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
	subscribe(SEND_LOG, send_log);
}

