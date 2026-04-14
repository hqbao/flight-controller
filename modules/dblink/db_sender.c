#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <messages.h>
#include <string.h>

/*
 * DB sender module — UART TX queue + DB protocol log framing.
 *
 * TX queue: buffers outgoing frames per port, sends one at a time.
 *   UART_RAW_SEND    — enqueue raw bytes for transmission
 *   UART_TX_COMPLETE — ISR notification that a port finished sending
 *
 * Logger: encodes SEND_LOG payloads into DB wire frames and enqueues them.
 *   DB_MESSAGE_UPDATE — receive commands from Python tools (log class, reset, chip ID)
 *   SEND_LOG          — encode payload into DB frame and enqueue for TX
 *   SCHEDULER_1HZ     — heartbeat when no other log class active
 */

/* --- TX Queue ------------------------------------------------------------ */

#define TX_QUEUE_SIZE  8
#define TX_BUF_SIZE    UART_FRAME_MAX_SIZE
#define NUM_PORTS      4

static struct {
	uint8_t buf[TX_QUEUE_SIZE][TX_BUF_SIZE];
	uint16_t len[TX_QUEUE_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail;
	volatile uint8_t busy;
} g_tx_q[NUM_PORTS] = {0};

static void tx_start_next(int port) {
	if (g_tx_q[port].head == g_tx_q[port].tail) {
		g_tx_q[port].busy = 0;
		return;
	}
	uint8_t slot = g_tx_q[port].tail;
	g_tx_q[port].busy = 1;
	platform_uart_send((uart_port_t)port,
	                    g_tx_q[port].buf[slot], g_tx_q[port].len[slot]);
}

static void tx_enqueue(int port, uint8_t *data, uint16_t size) {
	if (size > TX_BUF_SIZE) return;
	uint8_t next_head = (g_tx_q[port].head + 1) % TX_QUEUE_SIZE;
	if (next_head == g_tx_q[port].tail) return; /* Queue full — drop */

	uint8_t slot = g_tx_q[port].head;
	memcpy(g_tx_q[port].buf[slot], data, size);
	g_tx_q[port].len[slot] = size;
	g_tx_q[port].head = next_head;

	if (!g_tx_q[port].busy) {
		tx_start_next(port);
	}
}

static void on_raw_send(uint8_t *data, size_t size) {
	if (size < sizeof(uart_raw_t)) return;
	uart_raw_t *raw = (uart_raw_t *)data;
	if (raw->port >= NUM_PORTS) return;

	tx_enqueue(raw->port, raw->data, raw->size);
}

static void on_tx_complete(uint8_t *data, size_t size) {
	if (size < 1) return;
	uint8_t port = data[0];
	if (port >= NUM_PORTS) return;

	g_tx_q[port].tail = (g_tx_q[port].tail + 1) % TX_QUEUE_SIZE;
	tx_start_next(port);
}

/* --- Logger (DB frame encoding) ------------------------------------------ */

// Baud rate budget (UART_PORT1 = 19200 baud, 8N1 = 1920 bytes/sec):
//   Frame = header(6) + payload(N) + checksum(2) = N + 8 bytes
//   At 25 Hz: max frame = 960/25 = 38 bytes -> max payload = 30 bytes (7 floats)
//   At 10 Hz: max frame = 960/10 = 96 bytes -> max payload = 88 bytes (22 floats)

#define LOGGER_MAX_PAYLOAD 120
#define LOGGER_HEADER_SIZE 6

static uint8_t g_log_class = LOG_CLASS_HEART_BEAT;
static uint8_t g_output_msg[TX_BUF_SIZE] = {'d', 'b', 0x00 /* ID */, LOG_CLASS_HEART_BEAT /* Class */};
static uint32_t g_heartbeat_counter = 0;

// Receive DB message from UART (sent by Python tools)
static void on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;

	if (data[0] == DB_CMD_RESET) {
		platform_reset();
		return;
	}

	if (data[0] == DB_CMD_CHIP_ID) {
		uint8_t chip_id[8];
		platform_get_chip_id(chip_id);
		publish(SEND_LOG, chip_id, 8);
		return;
	}

	if (data[0] != DB_CMD_LOG_CLASS) return;

	g_log_class = data[4];
	g_output_msg[3] = g_log_class;
	publish(NOTIFY_LOG_CLASS, &g_log_class, 1);
}

static void loop_heartbeat(uint8_t *data, size_t size) {
	if (g_log_class != LOG_CLASS_HEART_BEAT) return;

	float counter = (float)(g_heartbeat_counter++);
	publish(SEND_LOG, (uint8_t*)&counter, sizeof(float));
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

	// Enqueue directly into TX queue (no extra pubsub hop)
	tx_enqueue(UART_PORT1, g_output_msg, (uint16_t)(buf_idx + 2));
}

/* --- Setup --------------------------------------------------------------- */

void db_sender_setup(void) {
	subscribe(UART_RAW_SEND, on_raw_send);
	subscribe(UART_TX_COMPLETE, on_tx_complete);
	subscribe(DB_MESSAGE_UPDATE, on_db_message);
	subscribe(SEND_LOG, send_log);
	subscribe(SCHEDULER_1HZ, loop_heartbeat);
}
