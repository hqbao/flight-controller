#include "speed_control.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <macro.h>

#define SPEED_CONTROL_PROTOCOL 1 // 1: DSHOT, 2: PWM
#define MAX_MOTORS 8

static int g_output_speed[MAX_MOTORS] = {0};

static void sc_setup(uint8_t *data, size_t size) {
#if SPEED_CONTROL_PROTOCOL == 1
	platform_dshot_init(DSHOT_PORT1);
	platform_dshot_init(DSHOT_PORT2);
	platform_dshot_init(DSHOT_PORT3);
	platform_dshot_init(DSHOT_PORT4);
	platform_dshot_init(DSHOT_PORT5);
	platform_dshot_init(DSHOT_PORT6);
	platform_dshot_init(DSHOT_PORT7);
	platform_dshot_init(DSHOT_PORT8);
#elif SPEED_CONTROL_PROTOCOL == 2
	platform_pwm_init(PWM_PORT1);
	platform_pwm_init(PWM_PORT2);
	platform_pwm_init(PWM_PORT3);
	platform_pwm_init(PWM_PORT4);
	platform_pwm_init(PWM_PORT5);
	platform_pwm_init(PWM_PORT6);
	platform_pwm_init(PWM_PORT7);
	platform_pwm_init(PWM_PORT8);
#endif
}

static void sc_update(uint8_t *data, size_t size) {
    if (size > sizeof(g_output_speed)) {
        size = sizeof(g_output_speed);
    }
	memcpy(g_output_speed, data, size);
#if SPEED_CONTROL_PROTOCOL == 1
	platform_dshot_send(DSHOT_PORT1, g_output_speed[0]);
	platform_dshot_send(DSHOT_PORT2, g_output_speed[1]);
	platform_dshot_send(DSHOT_PORT3, g_output_speed[2]);
	platform_dshot_send(DSHOT_PORT4, g_output_speed[3]);
	platform_dshot_send(DSHOT_PORT5, g_output_speed[4]);
	platform_dshot_send(DSHOT_PORT6, g_output_speed[5]);
	platform_dshot_send(DSHOT_PORT7, g_output_speed[6]);
	platform_dshot_send(DSHOT_PORT8, g_output_speed[7]);
#elif SPEED_CONTROL_PROTOCOL == 2
	platform_pwm_send(PWM_PORT1, g_output_speed[0]);
	platform_pwm_send(PWM_PORT2, g_output_speed[1]);
	platform_pwm_send(PWM_PORT3, g_output_speed[2]);
	platform_pwm_send(PWM_PORT4, g_output_speed[3]);
	platform_pwm_send(PWM_PORT5, g_output_speed[4]);
	platform_pwm_send(PWM_PORT6, g_output_speed[5]);
	platform_pwm_send(PWM_PORT7, g_output_speed[6]);
	platform_pwm_send(PWM_PORT8, g_output_speed[7]);
#endif
}

void speed_control_setup(void) {
	subscribe(SPEED_CONTROL_SETUP, sc_setup);
	subscribe(SPEED_CONTROL_UPDATE, sc_update);
}
