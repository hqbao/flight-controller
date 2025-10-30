#include "speed_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <macro.h>

#define SPEED_CONTROL_PROTOCOL 1 // 1: DSHOT, 2: PWM

static int g_output_speed[4] = {0, 0, 0, 0};

static void sc_setup(uint8_t *data, size_t size) {
#if SPEED_CONTROL_PROTOCOL == 1
	platform_dshot_init(DSHOT_PORT1);
	platform_dshot_init(DSHOT_PORT2);
	platform_dshot_init(DSHOT_PORT3);
	platform_dshot_init(DSHOT_PORT4);
#elif SPEED_CONTROL_PROTOCOL == 2
	platform_pwm_init(PWM_PORT1);
	platform_pwm_init(PWM_PORT2);
	platform_pwm_init(PWM_PORT3);
	platform_pwm_init(PWM_PORT4);
#endif
}

static void sc_update(uint8_t *data, size_t size) {
	memcpy(g_output_speed, data, size);
#if SPEED_CONTROL_PROTOCOL == 1
	platform_dshot_send(DSHOT_PORT1, g_output_speed[0]);
	platform_dshot_send(DSHOT_PORT2, g_output_speed[1]);
	platform_dshot_send(DSHOT_PORT3, g_output_speed[2]);
	platform_dshot_send(DSHOT_PORT4, g_output_speed[3]);
#elif SPEED_CONTROL_PROTOCOL == 2
	platform_pwm_send(PWM_PORT1, g_output_speed[0]);
	platform_pwm_send(PWM_PORT1, g_output_speed[1]);
	platform_pwm_send(PWM_PORT1, g_output_speed[2]);
	platform_pwm_send(PWM_PORT1, g_output_speed[3]);
#endif
}

void speed_control_setup(void) {
	subscribe(SPEED_CONTROL_SETUP, sc_setup);
	subscribe(SPEED_CONTROL_UPDATE, sc_update);
}
