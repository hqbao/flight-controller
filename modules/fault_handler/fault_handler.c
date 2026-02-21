#include "fault_handler.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>

static uint64_t g_fault_count = 0;

static void handle_fault(uint8_t *data, size_t size) {
	g_fault_count++;
	platform_toggle_led(0);

	// Force state to DISARMED to stop all motors
	state_t disarmed = DISARMED;
	publish(STATE_DETECTION_UPDATE, (uint8_t*)&disarmed, 1);

	platform_console("FAULT #%llu detected - motors disarmed\n", g_fault_count);
}

void fault_handler_setup(void) {
	subscribe(FAULT_DETECTION, handle_fault);
}
