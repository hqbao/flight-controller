#include "optflow.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
	if (data[0] == 0x01) { // External sensors
		if (data[1] == 0x00) { // Optical flow
			publish(EXTERNAL_SENSOR_OPTFLOW, &data[4], 12);
		}
	}
}

void optflow_setup(void) {
	subscribe(INTERNAL_MESSAGE, on_message_received);
}
