#ifndef DSHOT_H
#define DSHOT_H

#include "main.h"

#define DSHOT_FRAME_SIZE     		16
#define DSHOT_DMA_BUFFER_SIZE   	18

#define DSHOT600_HZ MHZ_TO_HZ(12)
#define DSHOT300_HZ MHZ_TO_HZ(6)
#define DSHOT150_HZ MHZ_TO_HZ(3)

#define DSHOT_FREQ DSHOT600_HZ

typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t timer_channel;
	uint32_t data_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
	uint16_t capture_compare;
	__IO uint32_t capture_compare_resigter;
} dshot_t;

typedef enum {
    DSHOT150,
    DSHOT300,
    DSHOT600
} dshot_type_e;

void dshot_init(dshot_t *dshot, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t cc, uint32_t ccr);
void dshot_write(dshot_t *dshot, uint16_t motor_value);

#endif // __WIRED_DIGITS_TRANSFER__
