#include "optflow.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <vector3d.h>

#define OPTFLOW_UNIT_SCALE 0.02

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

static optflow_data_t g_optflow_msg;

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
	if (data[0] == 0x01) { // Optical flow
		if (size < 20) return;

		int32_t raw_dx, raw_dy, raw_z, raw_clearity;
		memcpy(&raw_dx, &data[4], sizeof(int32_t));
		memcpy(&raw_dy, &data[8], sizeof(int32_t));
		memcpy(&raw_z, &data[12], sizeof(int32_t));
		memcpy(&raw_clearity, &data[16], sizeof(int32_t));
		
		float dx_mm = (float)raw_dx / 1000.0f;
		float dy_mm = (float)raw_dy / 1000.0f;
		float clearity = (float)raw_clearity / 10.0f;
		float texture_gain = 50.0f / (clearity < 5.0f ? 5.0f : clearity);

		double dx_scaled = LIMIT(dx_mm * texture_gain, -100.0, 100.0);
		double dy_scaled = LIMIT(dy_mm * texture_gain, -100.0, 100.0);

		g_optflow_msg.dx = dx_scaled * OPTFLOW_UNIT_SCALE;
		g_optflow_msg.dy = dy_scaled * OPTFLOW_UNIT_SCALE;
		g_optflow_msg.z = (double)raw_z;
		g_optflow_msg.direction = (optflow_direction_t)data[1];

		publish(EXTERNAL_SENSOR_OPTFLOW, (uint8_t*)&g_optflow_msg, sizeof(optflow_data_t));
	}
}

void optflow_setup(void) {
	subscribe(DB_MESSAGE_UPDATE, on_message_received);
	
	// Publish module initialized status
	module_initialized_t module_initialized;
	module_initialized.id = MODULE_ID_OPTFLOW;
	module_initialized.initialized = 1;
	publish(MODULE_INITIALIZED_UPDATE, (uint8_t*)&module_initialized, sizeof(module_initialized_t));
}
