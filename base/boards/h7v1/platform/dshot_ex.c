#include "dshot_ex.h"
#include "platform_hw.h"
#include <platform.h>

// Design
// Max frequency = 4,000,000 / (34 * 20), approximate 5882 Hz
// 4,000,000 clock units per second
// 34 bit per frame
// 20 clock units for a bit

#define TIMER_CLOCK					240000000	// 240 MHz
#define MHZ_TO_HZ(x)				((x) * 1000000)
#define DSHOT_EX_FREQ				MHZ_TO_HZ(4) // 4 MHz
#define MOTOR_BIT_0            		0x00000007 // 7
#define MOTOR_BIT_1            		0x0000000E // 14
#define MOTOR_BITLENGTH        		20

static uint32_t dshot_prepare_packet(uint32_t value) {
	uint32_t packet = value;
	uint8_t csum = 0;
	csum ^= packet;
	csum ^= (packet >> 8);
	csum ^= (packet >> 16);
	csum ^= (packet >> 24);
	packet = (packet << 8) | csum;
	return packet;
}

static void dshot_prepare_dmabuffer(uint32_t* dmabuffer, uint32_t value) {
	uint32_t packet = dshot_prepare_packet(value);
	for (int i = 0; i < 32; i++) {
		dmabuffer[i] = (packet & (1 << i)) ? MOTOR_BIT_1 : MOTOR_BIT_0;
	}

	dmabuffer[32] = 0;
	dmabuffer[33] = 0;
}

void dshot_ex_init(dshot_ex_t *dshot_ex, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t cc, uint32_t ccr) {
	dshot_ex->timer = timer;
	dshot_ex->timer_channel = channel;
	dshot_ex->capture_compare = cc;
	dshot_ex->capture_compare_resigter = ccr;

	// Calculate pre-scaler by DSHOT type
	uint16_t prescaler = lrintf((float) TIMER_CLOCK / DSHOT_EX_FREQ + 0.01f) - 1;

	// Configure timer
	__HAL_TIM_SET_PRESCALER(dshot_ex->timer, prescaler);
	__HAL_TIM_SET_AUTORELOAD(dshot_ex->timer, MOTOR_BITLENGTH);

	// Start PWM timer
	HAL_TIM_PWM_Start_DMA(dshot_ex->timer,
			dshot_ex->timer_channel,
			dshot_ex->data_dmabuffer,
			DSHOT_EX_DMA_BUFFER_SIZE);
}

void dshot_ex_write(dshot_ex_t *dshot_ex, uint32_t data) {
	// Construct data
	dshot_prepare_dmabuffer(dshot_ex->data_dmabuffer, data);

	// Write data
	HAL_DMA_Start_IT(dshot_ex->timer->hdma[dshot_ex->capture_compare],
			(uint32_t)dshot_ex->data_dmabuffer,
			(uint32_t)dshot_ex->capture_compare_resigter,
			DSHOT_EX_DMA_BUFFER_SIZE);
}

/* --- Platform API -------------------------------------------------------- */

static dshot_ex_t g_dshot_exs[4];

char platform_dshot_ex_init(dshot_ex_port_t port) {
	switch (port) {
	case DSHOT_EX_PORT1:
		dshot_ex_init(&g_dshot_exs[0], &htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1,
				(uint32_t)&((&htim1)->Instance->CCR1));
		break;
	case DSHOT_EX_PORT2:
		dshot_ex_init(&g_dshot_exs[1], &htim1, TIM_CHANNEL_2, TIM_DMA_ID_CC2,
				(uint32_t)&((&htim1)->Instance->CCR2));
		break;
	case DSHOT_EX_PORT3:
		dshot_ex_init(&g_dshot_exs[2], &htim1, TIM_CHANNEL_3, TIM_DMA_ID_CC3,
				(uint32_t)&((&htim1)->Instance->CCR3));
		break;
	case DSHOT_EX_PORT4:
		dshot_ex_init(&g_dshot_exs[3], &htim1, TIM_CHANNEL_4, TIM_DMA_ID_CC4,
				(uint32_t)&((&htim1)->Instance->CCR4));
		break;
	default:
		break;
	}
	return PLATFORM_OK;
}

char platform_dshot_ex_send(dshot_ex_port_t port, uint32_t data) {
	dshot_ex_write(&g_dshot_exs[port], data);
	return PLATFORM_OK;
}
