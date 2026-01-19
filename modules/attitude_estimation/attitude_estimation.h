#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <vector3d.h>

typedef struct {
	vector3d_t body;
	vector3d_t earth;
} linear_accel_data_t;

void attitude_estimation_setup(void);

#endif
