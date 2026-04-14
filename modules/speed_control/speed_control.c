#include "speed_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <messages.h>

#define MAX_PORTS 8

static uint8_t g_protocol[MAX_PORTS] = {0}; // port_protocol_t per port
static int g_output[MAX_PORTS] = {0};

static void sc_setup(uint8_t *data, size_t size) {
	if (size < sizeof(speed_control_config_t)) return;
	speed_control_config_t *cfg = (speed_control_config_t *)data;
	memcpy(g_protocol, cfg->protocol, MAX_PORTS);

	for (int i = 0; i < MAX_PORTS; i++) {
		if (g_protocol[i] == PORT_DSHOT) {
			platform_dshot_init((dshot_port_t)i);
		} else if (g_protocol[i] == PORT_PWM) {
			platform_pwm_init((pwm_port_t)i);
		}
	}
}

static void sc_update(uint8_t *data, size_t size) {
	if (size > sizeof(g_output)) size = sizeof(g_output);
	memcpy(g_output, data, size);

	for (int i = 0; i < MAX_PORTS; i++) {
		if (g_protocol[i] == PORT_DSHOT) {
			platform_dshot_send((dshot_port_t)i, g_output[i]);
		} else if (g_protocol[i] == PORT_PWM) {
			platform_pwm_send((pwm_port_t)i, g_output[i]);
		}
	}
}

void speed_control_setup(void) {
	subscribe(SPEED_CONTROL_SETUP, sc_setup);
	subscribe(SPEED_CONTROL_UPDATE, sc_update);
}
