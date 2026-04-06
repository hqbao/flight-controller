/**
 ******************************************************************************
 * @file           : platform_uart.c
 * @brief          : UART platform driver — STM32H7 HAL implementation
 *
 * Uses 32-byte DMA circular ring buffers with IDLE line detection.
 * HAL_UARTEx_ReceiveToIdle_DMA fires on UART IDLE (sender stops),
 * providing exact byte count — no fixed half/complete boundary delays.
 * Raw bytes are published via UART_RAW_RECEIVED for protocol-agnostic
 * consumption. Protocol parsing is handled by the db_reader module.
 ******************************************************************************
 */

#include "platform_hw.h"
#include <platform.h>
#include <pubsub.h>
#include <messages.h>
#include <string.h>

/* --- Constants ----------------------------------------------------------- */

#define UART_DMA_BUF_SIZE    32
#define NUM_PORTS            4

/* --- Static state -------------------------------------------------------- */

static UART_HandleTypeDef *uart_ports[NUM_PORTS] = {&huart1, &huart2, &huart3, &huart4};

// DMA ring buffers — IDLE detection fires when sender stops transmitting.
static uint8_t g_uart_dma_buf[NUM_PORTS][UART_DMA_BUF_SIZE];
static volatile uint16_t g_last_pos[NUM_PORTS] = {0};

/* --- Raw byte publishing ------------------------------------------------- */

static void publish_raw_bytes(int port_idx, int start, int count) {
	uart_raw_t raw;
	raw.port = (uint8_t)port_idx;
	raw.size = (uint16_t)count;
	memcpy(raw.data, &g_uart_dma_buf[port_idx][start], count);
	publish(UART_RAW_RECEIVED, (uint8_t *)&raw, sizeof(uart_raw_t));
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
	for (int i = 0; i < NUM_PORTS; i++) {
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
		publish_raw_bytes(idx, start, Size - start);
	} else if (Size < start) {
		// Wrapped around
		publish_raw_bytes(idx, start, UART_DMA_BUF_SIZE - start);
		if (Size > 0) {
			publish_raw_bytes(idx, 0, Size);
		}
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
		// Restart IDLE+DMA reception
		g_last_pos[idx] = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_dma_buf[idx], UART_DMA_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

/**
 * UART TX complete callback — notifies db_sender module to dequeue next frame.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	int idx = uart_instance_to_idx(huart);
	if (idx >= 0) {
		uint8_t port = (uint8_t)idx;
		publish(UART_TX_COMPLETE, &port, sizeof(port));
	}
}
