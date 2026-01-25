#include "optflow.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <vector3d.h>
#include <messages.h>

static optflow_data_t g_optflow_msg;

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
	if (data[0] == 0x01) { // Optical flow
		if (size < 20) return;

		int32_t raw_dx, raw_dy, raw_z, raw_clarity;
		memcpy(&raw_dx, &data[4], sizeof(int32_t));
		memcpy(&raw_dy, &data[8], sizeof(int32_t));
		memcpy(&raw_z, &data[12], sizeof(int32_t));
		memcpy(&raw_clarity, &data[16], sizeof(int32_t));
		
		// Convert from scaled int (×100000) back to radians
		float dx_rad = (float)raw_dx / 100000.0f;
		float dy_rad = (float)raw_dy / 100000.0f;
		float clarity = (float)raw_clarity / 10.0f;

		// scale 50-100 to 0.2-1
		float texture_gain = clarity < 70.0 ? 0.5 : 1.0;

		g_optflow_msg.dx = dx_rad / texture_gain;
		g_optflow_msg.dy = dy_rad / texture_gain;
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
