#include "fault_handler.h"
#include <pubsub.h>
#include <platform.h>
#include "main.h"

static void handle_fault(uint8_t *data, size_t size) {
	platform_toggle_led(0);
}

void fault_handler_setup(void) {
	subscribe(FAULT_DETECTION, handle_fault);
}
