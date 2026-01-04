#ifndef optflow_h
#define optflow_h
#include <stdint.h>

void optflow_init(uint16_t width, uint16_t height);
void optflow_calc(uint8_t *frame, float *dx, float *dy, float *clearity);

#endif /* optflow_h */
