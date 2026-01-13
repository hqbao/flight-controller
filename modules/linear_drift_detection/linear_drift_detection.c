#include "linear_drift_detection.h"
#include <pubsub.h>
#include <vector3d.h>
#include <string.h>

typedef enum {
	OPTFLOW_DOWNWARD = 0,
	OPTFLOW_UPWARD = 1,
} optflow_direction_t;

typedef struct {
    double dx;
    double dy;
    double z;
    optflow_direction_t direction; 
} optflow_data_t;

static double g_integrated_flow_x = 0;
static double g_integrated_flow_y = 0;
static float g_drift_time_x = 0;
static float g_drift_time_y = 0;
static double g_prev_deriv_x = 0;
static double g_prev_deriv_y = 0;
static vector3d_t g_drift_msg;

static void on_optflow_update(uint8_t *data, size_t size) {
    if (size < sizeof(optflow_data_t)) return;
    optflow_data_t *msg = (optflow_data_t*)data;

    double flow_dx = msg->dx;
    double flow_dy = msg->dy;

    if (msg->direction == OPTFLOW_UPWARD) {
        flow_dy = -flow_dy;
    }

    g_integrated_flow_x += flow_dx;
    g_integrated_flow_y += flow_dy;
}

static void loop_5hz(uint8_t *data, size_t size) {
	double deriv_x = g_integrated_flow_x;
	double deriv_y = g_integrated_flow_y;
	
	g_integrated_flow_x = 0;
	g_integrated_flow_y = 0;

	// Linear drift detection X
	if ((deriv_x > 0 && g_prev_deriv_x > 0) || (deriv_x < 0 && g_prev_deriv_x < 0)) {
		g_drift_time_x += 0.2f;
	} else {
		g_drift_time_x = 0.2f;
	}
	g_prev_deriv_x = deriv_x;

	// Linear drift detection Y
	if ((deriv_y > 0 && g_prev_deriv_y > 0) || (deriv_y < 0 && g_prev_deriv_y < 0)) {
		g_drift_time_y += 0.2f;
	} else {
		g_drift_time_y = 0.2f;
	}
	g_prev_deriv_y = deriv_y;

	g_drift_msg.x = (double)g_drift_time_x;
	g_drift_msg.y = (double)g_drift_time_y;
	g_drift_msg.z = 0;
	publish(LINEAR_DRIFT_DETECTION, (uint8_t*)&g_drift_msg, sizeof(vector3d_t));
}

void linear_drift_detection_setup(void) {
	subscribe(EXTERNAL_SENSOR_OPTFLOW, on_optflow_update);
	subscribe(SCHEDULER_5HZ, loop_5hz);
}
