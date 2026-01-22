#ifndef optflow_h
#define optflow_h
#include <stdint.h>

// Configuration (can be changed at runtime)
extern float optflow_camera_fov_degrees;

void optflow_init(uint16_t width, uint16_t height, int hybrid_mode);

/**
 * Compute optical flow between current and previous frame
 * 
 * @param frame     Current grayscale frame
 * @param dx_rad    Output: X angular displacement (radians/frame)
 * @param dy_rad    Output: Y angular displacement (radians/frame)
 * @param rotation  Output: Rotation around optical axis (radians/frame)
 * @param clarity   Output: Gradient strength (texture quality metric)
 * @param mode      Output: Algorithm mode used (1=dense)
 */
void optflow_calc(uint8_t *frame, float *dx_rad, float *dy_rad, float *rotation, float *clarity, int *mode);

void optflow_cleanup(void);

#endif /* optflow_h */
