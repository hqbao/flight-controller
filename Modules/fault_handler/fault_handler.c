#include "fault_handler.h"
#include <pubsub.h>
#include <platform.h>

static uint64_t g_fault_count = 0;

static void handle_fault(uint8_t *data, size_t size) {
	g_fault_count++;
	platform_toggle_led(0);
}

void fault_handler_setup(void) {
	subscribe(FAULT_DETECTION, handle_fault);
}
