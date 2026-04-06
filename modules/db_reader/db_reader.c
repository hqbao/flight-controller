#include "db_reader.h"
#include <pubsub.h>
#include <messages.h>
#include <string.h>

/*
 * DB/UBX frame parser module — converts raw UART bytes into protocol frames.
 *
 * RX path:  UART_RAW_RECEIVED → parse DB/UBX framing → publish DB_MESSAGE_UPDATE
 *           or UBX_MESSAGE_UPDATE
 *
 * DB wire format:
 *   [0]='d' [1]='b' [2]=CMD [3]=CLASS [4..5]=SIZE(LE) [6..]=PAYLOAD [N..N+1]=CHECKSUM
 *
 * UBX wire format:
 *   [0]=0xB5 [1]=0x62 [2]=CLASS [3]=ID [4..5]=SIZE(LE) [6..]=PAYLOAD [N..N+1]=CHECKSUM
 *
 * Replace this module with a MAVLink parser to switch protocols.
 */

#define MAX_BUFFER_SIZE UART_FRAME_MAX_SIZE
#define NUM_PORTS       4

typedef enum {
	PROTOCOL_NONE = 0,
	PROTOCOL_DB   = 1,
	PROTOCOL_UBX  = 2
} protocol_type_t;

typedef struct {
	uint8_t buffer[MAX_BUFFER_SIZE];
	uint8_t header[2];
	char stage;
	uint16_t payload_size;
	int buffer_idx;
	protocol_type_t protocol;
} parser_rx_t;

static parser_rx_t g_rx[NUM_PORTS] = {0};

static void handle_parsed_frame(parser_rx_t *rx) {
	if (rx->protocol == PROTOCOL_DB) {
		publish(DB_MESSAGE_UPDATE, rx->buffer, rx->payload_size + 6);
	} else if (rx->protocol == PROTOCOL_UBX) {
		publish(UBX_MESSAGE_UPDATE, rx->buffer, rx->payload_size + 6);
	}
}

static void parse_byte(parser_rx_t *rx, uint8_t byte) {
	if (rx->stage == 5) {
		if (rx->buffer_idx >= MAX_BUFFER_SIZE) {
			rx->stage = 0;
			rx->protocol = PROTOCOL_NONE;
			return;
		}
		rx->buffer[rx->buffer_idx] = byte;
		rx->buffer_idx++;
		if (rx->buffer_idx == (int)rx->payload_size + 6) {
			rx->stage = 0;
			handle_parsed_frame(rx);
			rx->protocol = PROTOCOL_NONE;
		}
	} else if (rx->stage == 0) {
		if (byte == 'd') {
			rx->header[0] = byte;
			rx->protocol = PROTOCOL_DB;
			rx->stage = 1;
		} else if (byte == 0xB5) {
			rx->header[0] = byte;
			rx->protocol = PROTOCOL_UBX;
			rx->stage = 1;
		}
	} else if (rx->stage == 1) {
		if ((rx->protocol == PROTOCOL_DB && byte == 'b') ||
		    (rx->protocol == PROTOCOL_UBX && byte == 0x62)) {
			rx->header[1] = byte;
			rx->buffer_idx = 0;
			rx->stage = 2;
		} else {
			rx->stage = 0;
			rx->protocol = PROTOCOL_NONE;
		}
	} else if (rx->stage == 2) {
		rx->buffer[rx->buffer_idx] = byte;
		rx->buffer_idx = 1;
		rx->stage = 3;
	} else if (rx->stage == 3) {
		rx->buffer[rx->buffer_idx] = byte;
		rx->buffer_idx = 2;
		rx->stage = 4;
	} else if (rx->stage == 4) {
		rx->buffer[rx->buffer_idx] = byte;
		rx->buffer_idx++;
		if (rx->buffer_idx == 4) {
			memcpy(&rx->payload_size, &rx->buffer[2], sizeof(uint16_t));
			if (rx->payload_size > MAX_BUFFER_SIZE - 6) {
				rx->stage = 0;
				rx->protocol = PROTOCOL_NONE;
			} else {
				rx->stage = 5;
			}
		}
	} else {
		rx->stage = 0;
		rx->protocol = PROTOCOL_NONE;
	}
}

static void on_raw_received(uint8_t *data, size_t size) {
	if (size < sizeof(uart_raw_t)) return;
	uart_raw_t *raw = (uart_raw_t *)data;
	if (raw->port >= NUM_PORTS) return;

	parser_rx_t *rx = &g_rx[raw->port];
	for (int i = 0; i < (int)raw->size; i++) {
		parse_byte(rx, raw->data[i]);
	}
}

void db_reader_setup(void) {
	subscribe(UART_RAW_RECEIVED, on_raw_received);
}
