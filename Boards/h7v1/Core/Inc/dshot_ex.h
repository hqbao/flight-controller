#ifndef DSHOT_EX_H
#define DSHOT_EX_H

#include "main.h"

#define DSHOT_EX_FRAME_SIZE     	32
#define DSHOT_EX_DMA_BUFFER_SIZE   	34

typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t timer_channel;
	uint32_t data_dmabuffer[DSHOT_EX_DMA_BUFFER_SIZE];
	uint16_t capture_compare;
	__IO uint32_t capture_compare_resigter;
} dshot_ex_t;

void dshot_ex_init(dshot_ex_t *dshot_ex, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t cc, uint32_t ccr);
void dshot_ex_write(dshot_ex_t *dshot_ex, uint32_t motor_value);

#endif // __WIRED_DIGITS_TRANSFER__
