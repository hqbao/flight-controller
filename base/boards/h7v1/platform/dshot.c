#include "dshot.h"
#include "platform_hw.h"
#include <platform.h>

// Design
// Max frequency = 4,000,000 / (34 * 20), approximate 5882 Hz
// 4,000,000 clock units per second
// 18 bit per frame
// 20 clock units for a bit

#define TIMER_CLOCK					240000000	// 240 MHz
#define MHZ_TO_HZ(x)				((x) * 1000000)
#define MOTOR_BIT_0            		7 // 7
#define MOTOR_BIT_1            		14 // 14
#define MOTOR_BITLENGTH        		20

static uint16_t dshot_prepare_packet(uint16_t value) {
	uint16_t packet;
	char dshot_telemetry = 0;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for (int i = 0; i < 3; i++) {
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

static void dshot_prepare_dmabuffer(uint32_t* dmabuffer, uint16_t value) {
	uint16_t packet;
	packet = dshot_prepare_packet(value);

	for (int i = 0; i < 16; i++) {
		dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	dmabuffer[16] = 0;
	dmabuffer[17] = 0;
}

void dshot_init(dshot_t *dshot, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t cc, uint32_t ccr) {
	dshot->timer = timer;
	dshot->timer_channel = channel;
	dshot->capture_compare = cc;
	dshot->capture_compare_resigter = ccr;

	// Calculate pre-scaler by DSHOT type
	uint16_t prescaler = lrintf((float) TIMER_CLOCK / DSHOT_FREQ + 0.01f) - 1;

	// Configure timer
	__HAL_TIM_SET_PRESCALER(dshot->timer, prescaler);
	__HAL_TIM_SET_AUTORELOAD(dshot->timer, MOTOR_BITLENGTH);

	// Start PWM timer
	HAL_TIM_PWM_Start_DMA(dshot->timer,
			dshot->timer_channel,
			dshot->data_dmabuffer,
			DSHOT_DMA_BUFFER_SIZE);
}

void dshot_write(dshot_t *dshot, uint16_t data) {
	// Construct data
	dshot_prepare_dmabuffer(dshot->data_dmabuffer, data);

	// Write data
	HAL_DMA_Start_IT(dshot->timer->hdma[dshot->capture_compare],
			(uint32_t)dshot->data_dmabuffer,
			(uint32_t)dshot->capture_compare_resigter,
			DSHOT_DMA_BUFFER_SIZE);
}

/* --- Platform API -------------------------------------------------------- */

static dshot_t g_dshots[8];

char platform_dshot_init(dshot_port_t port) {
	switch (port) {
	case DSHOT_PORT1:
		dshot_init(&g_dshots[0], &htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1,
				(uint32_t)&((&htim1)->Instance->CCR1));
		break;
	case DSHOT_PORT2:
		dshot_init(&g_dshots[1], &htim1, TIM_CHANNEL_2, TIM_DMA_ID_CC2,
				(uint32_t)&((&htim1)->Instance->CCR2));
		break;
	case DSHOT_PORT3:
		dshot_init(&g_dshots[2], &htim1, TIM_CHANNEL_3, TIM_DMA_ID_CC3,
				(uint32_t)&((&htim1)->Instance->CCR3));
		break;
	case DSHOT_PORT4:
		dshot_init(&g_dshots[3], &htim1, TIM_CHANNEL_4, TIM_DMA_ID_CC4,
				(uint32_t)&((&htim1)->Instance->CCR4));
		break;
	case DSHOT_PORT5:
		dshot_init(&g_dshots[4], &htim2, TIM_CHANNEL_1, TIM_DMA_ID_CC1,
				(uint32_t)&((&htim2)->Instance->CCR1));
		break;
	case DSHOT_PORT6:
		dshot_init(&g_dshots[5], &htim2, TIM_CHANNEL_2, TIM_DMA_ID_CC2,
				(uint32_t)&((&htim2)->Instance->CCR2));
		break;
	case DSHOT_PORT7:
		dshot_init(&g_dshots[6], &htim2, TIM_CHANNEL_3, TIM_DMA_ID_CC3,
				(uint32_t)&((&htim2)->Instance->CCR3));
		break;
	case DSHOT_PORT8:
		dshot_init(&g_dshots[7], &htim2, TIM_CHANNEL_4, TIM_DMA_ID_CC4,
				(uint32_t)&((&htim2)->Instance->CCR4));
		break;
	default:
		break;
	}
	return PLATFORM_OK;
}

char platform_dshot_send(dshot_port_t port, uint16_t data) {
	dshot_write(&g_dshots[port], data);
	return PLATFORM_OK;
}
