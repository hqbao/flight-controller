/**
 ******************************************************************************
 * @file           : platform_uart.c
 * @brief          : UART platform driver — STM32H7 HAL implementation
 *
 * Uses 32-byte DMA circular ring buffers with IDLE line detection.
 * HAL_UARTEx_ReceiveToIdle_DMA fires on UART IDLE (sender stops),
 * providing exact byte count — no fixed half/complete boundary delays.
 * Protocol parser handles both DB ('d','b') and UBX (0xB5,0x62) framing.
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>
#include <string.h>

/* --- Constants ----------------------------------------------------------- */

#define MAX_UART_BUFFER_SIZE 128
#define UART_DMA_BUF_SIZE    32

/* --- Types --------------------------------------------------------------- */

typedef enum {
	PROTOCOL_NONE = 0,
	PROTOCOL_DB   = 1,
	PROTOCOL_UBX  = 2
} protocol_type_t;

typedef struct {
	uint8_t byte;
	uint8_t _pad[3]; /* Align buffer[] to 4-byte boundary for safe type access */
	uint8_t buffer[MAX_UART_BUFFER_SIZE];
	uint8_t header[2];
	char stage;
	uint16_t payload_size;
	int buffer_idx;
	protocol_type_t protocol;
} uart_rx_t;

/* --- Static state -------------------------------------------------------- */

static UART_HandleTypeDef *uart_ports[4] = {&huart1, &huart2, &huart3, &huart4};

static uart_rx_t g_uart_rx1 = {0};
static uart_rx_t g_uart_rx2 = {0};
static uart_rx_t g_uart_rx3 = {0};
static uart_rx_t g_uart_rx4 = {0};

// DMA ring buffers — IDLE detection fires when sender stops transmitting.
static uint8_t g_uart_dma_buf[4][UART_DMA_BUF_SIZE];
static volatile uint16_t g_last_pos[4] = {0};
static uart_rx_t *g_uart_rx_map[4] = {&g_uart_rx1, &g_uart_rx2, &g_uart_rx3, &g_uart_rx4};

/* --- Protocol parser ----------------------------------------------------- */

static void handle_protocol_msg(uart_rx_t *msg) {
	if (msg->protocol == PROTOCOL_DB) {
		// DB protocol message: pass total buffer size (class + id + length + payload + checksum)
		platform_receive_db_message(msg->buffer, msg->payload_size + 6);
	} else if (msg->protocol == PROTOCOL_UBX) {
		// UBX protocol message: pass total buffer size (class + id + length + payload + checksum)
		platform_receive_ubx_message(msg->buffer, msg->payload_size + 6);
	}
}

static void read_uart_byte(uart_rx_t *g_uart_rx) {
	if (g_uart_rx->stage == 5) {
		// Guard against buffer overflow from corrupted payload_size
		if (g_uart_rx->buffer_idx >= MAX_UART_BUFFER_SIZE) {
			g_uart_rx->stage = 0;
			g_uart_rx->protocol = PROTOCOL_NONE;
			return;
		}
		g_uart_rx->buffer[g_uart_rx->buffer_idx] = g_uart_rx->byte;
		g_uart_rx->buffer_idx++;
		// Plus 2-byte class-id, 2-byte length and 2-byte checksum
		if (g_uart_rx->buffer_idx == (int) g_uart_rx->payload_size + 6) {
			g_uart_rx->stage = 0;
			handle_protocol_msg(g_uart_rx);
			g_uart_rx->protocol = PROTOCOL_NONE;
		}
	} else if (g_uart_rx->stage == 0) {
		// Detect protocol: 'd' for DB or 0xB5 for UBX
		if (g_uart_rx->byte == 'd') {
			g_uart_rx->header[0] = g_uart_rx->byte;
			g_uart_rx->protocol = PROTOCOL_DB;
			g_uart_rx->stage = 1;
		} else if (g_uart_rx->byte == 0xB5) {
			g_uart_rx->header[0] = g_uart_rx->byte;
			g_uart_rx->protocol = PROTOCOL_UBX;
			g_uart_rx->stage = 1;
		}
	} else if (g_uart_rx->stage == 1) {
		// Verify second header byte based on protocol
		if ((g_uart_rx->protocol == PROTOCOL_DB && g_uart_rx->byte == 'b') ||
		    (g_uart_rx->protocol == PROTOCOL_UBX && g_uart_rx->byte == 0x62)) {
			g_uart_rx->header[1] = g_uart_rx->byte;
			g_uart_rx->buffer_idx = 0;
			g_uart_rx->stage = 2;
		} else {
			g_uart_rx->stage = 0;
			g_uart_rx->protocol = PROTOCOL_NONE;
		}
	} else if (g_uart_rx->stage == 2) {
		g_uart_rx->buffer[g_uart_rx->buffer_idx] = g_uart_rx->byte;
		g_uart_rx->buffer_idx = 1;
		g_uart_rx->stage = 3;
	} else if (g_uart_rx->stage == 3) {
		g_uart_rx->buffer[g_uart_rx->buffer_idx] = g_uart_rx->byte;
		g_uart_rx->buffer_idx = 2;
		g_uart_rx->stage = 4;
	} else if (g_uart_rx->stage == 4) {
		g_uart_rx->buffer[g_uart_rx->buffer_idx] = g_uart_rx->byte;
		g_uart_rx->buffer_idx++;
		if (g_uart_rx->buffer_idx == 4) {
			// Both protocols have length at bytes 2-3 (little-endian)
			memcpy(&g_uart_rx->payload_size, &g_uart_rx->buffer[2], sizeof(uint16_t));
			// Reject corrupted length — total frame must fit in buffer
			if (g_uart_rx->payload_size > MAX_UART_BUFFER_SIZE - 6) {
				g_uart_rx->stage = 0;
				g_uart_rx->protocol = PROTOCOL_NONE;
			} else {
				g_uart_rx->stage = 5;
			}
		}
	} else {
		g_uart_rx->stage = 0;
		g_uart_rx->protocol = PROTOCOL_NONE;
	}
}

/* --- DMA ring buffer processing ------------------------------------------ */

// Process a range of bytes from a DMA ring buffer through the parser
static void process_dma_bytes(int uart_idx, int start, int end) {
	uart_rx_t *rx = g_uart_rx_map[uart_idx];
	for (int i = start; i < end; i++) {
		rx->byte = g_uart_dma_buf[uart_idx][i];
		read_uart_byte(rx);
	}
}

static int uart_instance_to_idx(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) return 0;
	if (huart->Instance == USART2) return 1;
	if (huart->Instance == USART3) return 2;
	if (huart->Instance == UART4)  return 3;
	return -1;
}

/* --- Platform API -------------------------------------------------------- */

char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t data_size) {
	HAL_StatusTypeDef status = HAL_UART_Transmit_IT(uart_ports[port], data, data_size);
	return status == HAL_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

void platform_hw_uart_start(void) {
	for (int i = 0; i < 4; i++) {
		g_last_pos[i] = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(uart_ports[i], g_uart_dma_buf[i], UART_DMA_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(uart_ports[i]->hdmarx, DMA_IT_HT);
	}
}

/* --- HAL Callbacks ------------------------------------------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	int idx = uart_instance_to_idx(huart);
	if (idx < 0) return;

	uint16_t start = g_last_pos[idx];
	if (Size > start) {
		process_dma_bytes(idx, start, Size);
	} else if (Size < start) {
		// Wrapped around
		process_dma_bytes(idx, start, UART_DMA_BUF_SIZE);
		process_dma_bytes(idx, 0, Size);
	}
	g_last_pos[idx] = Size % UART_DMA_BUF_SIZE;
}

/**
 * UART error callback — restarts DMA reception after any error.
 *
 * The STM32 HAL aborts DMA reception on ANY UART error (overrun, framing,
 * noise) when DMA mode is active.  Without this callback, a single glitch
 * permanently kills reception.  We clear the error and restart the DMA
 * circular transfer so data keeps flowing.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	int idx = uart_instance_to_idx(huart);
	if (idx >= 0) {
		// Clear all error flags
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
		                             UART_CLEAR_FEF  | UART_CLEAR_PEF);
		// Reset parser state so a partial frame doesn't corrupt the next one
		g_uart_rx_map[idx]->stage = 0;
		g_uart_rx_map[idx]->protocol = PROTOCOL_NONE;
		// Restart IDLE+DMA reception
		g_last_pos[idx] = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_dma_buf[idx], UART_DMA_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}
