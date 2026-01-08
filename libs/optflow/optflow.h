#ifndef optflow_h
#define optflow_h
#include <stdint.h>

// Configuration (can be changed at runtime)
extern float optflow_camera_fov_degrees;
extern float optflow_reference_altitude_mm;

void optflow_init(uint16_t width, uint16_t height, int hybrid_mode);
void optflow_calc(uint8_t *frame, float *dx_mm, float *dy_mm, float *rotation, float *clearity, int *mode);

#endif /* optflow_h */
