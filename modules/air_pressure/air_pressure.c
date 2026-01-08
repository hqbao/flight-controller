#include "air_pressure.h"
#include <pubsub.h>
#include <platform.h>
#include "dps310.h"

#define ALT_SAMPLES 100
#define WARMUP_DELAY_MS 100
#define WARMUP_COUNTER_INIT -10
#define ALTITUDE_SCALE_FACTOR 1000.0

static double g_air_pressure = 0;

static void air_pressure_init(void) {
	platform_delay(WARMUP_DELAY_MS);
	dps310_probe();
	platform_delay(WARMUP_DELAY_MS);
}

static void air_pressure_loop(uint8_t *data, size_t size) {
    static int g_alt_counter = WARMUP_COUNTER_INIT;
    static float g_alt_off = 0;
    if (g_alt_counter > ALT_SAMPLES) {
    	g_air_pressure = ALTITUDE_SCALE_FACTOR * (get_altitude() - g_alt_off);
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
