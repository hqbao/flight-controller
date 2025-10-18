#include "air_pressure.h"
#include <pubsub.h>
#include <platform.h>
#include "dps310.h"
#include "main.h"

#define ALT_SAMPLES 100

static double g_air_pressure = 0;

static void air_pressure_init(void) {
	platform_delay(100);
	dps310_probe();
	platform_delay(100);
}

static void air_pressure_loop(uint8_t *data, size_t size) {
    static int g_alt_counter = -10;
    static float g_alt_off = 0;
    if (g_alt_counter > ALT_SAMPLES) {
    	g_air_pressure = 1000.0 * (get_altitude() - g_alt_off);
    	publish(SENSOR_AIR_PRESSURE, (uint8_t*)&g_air_pressure, sizeof(double));
    } else if (g_alt_counter == ALT_SAMPLES) {
        g_alt_off = g_alt_off / ALT_SAMPLES;
        g_alt_counter += 1;
    } else if (g_alt_counter >= 0) {
        g_alt_off += get_altitude();
        g_alt_counter += 1;
    } else {
        g_alt_counter += 1;
    }
}

void air_pressure_setup(void) {
	air_pressure_init();
	subscribe(LOOP, air_pressure_loop);
}
