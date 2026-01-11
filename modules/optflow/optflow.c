#include "optflow.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
	if (data[0] == 0x01) { // Optical flow
		publish(EXTERNAL_SENSOR_OPTFLOW, data, 16);
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
