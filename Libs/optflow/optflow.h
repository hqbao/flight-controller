#ifndef optflow_h
#define optflow_h
#include <stdint.h>

void optflow_init(uint16_t width, uint16_t height);
void optflow_calc(uint8_t *frame, float *dx, float *dy, float *clearity);
void get_roi(uint8_t *frame, int *x, int *y, int rw, int rh, int width, int height);
float get_clearity(uint8_t *frame, int width, int height);

#endif /* optflow_h */
