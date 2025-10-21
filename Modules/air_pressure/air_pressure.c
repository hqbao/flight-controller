#include "air_pressure.h"
#include <pubsub.h>
#include <platform.h>
#include <math.h>
#include "dps310.h"

static dps310_handle_t dps310;
static double g_ap_alt_raw = 0;
static double g_ap_alt_raw_offset = 0;
static double g_ap_alt = 0;

static int g_counter_ignore = 100;
static int g_check_freq_counter = 0;
static int g_check_freq_period = 0;

static void air_pressure_init(void) {
    // Initialize DPS310
    if (!dps310_init(&dps310)) {  // Using 100Hz timer
    	publish(FAULT_DETECTION, NULL, 0);
    }

    // Start continuous measurement at 64Hz with 8x over-sampling
	if (!dps310_start_continuous(&dps310, DPS310_RATE_64HZ, DPS310_OVERSAMPLING_16)) {
		publish(FAULT_DETECTION, NULL, 0);
	}
}

static void loop_100hz(uint8_t *data, size_t size) {
	if (dps310.data.data_ready) {
		dps310.data.data_ready = false;
		g_check_freq_counter++;
		g_ap_alt_raw = dps310.data.pressure * 0.1;
		if (g_counter_ignore > 0) {
			g_counter_ignore--;
			g_ap_alt_raw_offset = g_ap_alt_raw;
		} else {
			g_ap_alt = g_ap_alt_raw - g_ap_alt_raw_offset;
		}
		publish(SENSOR_AIR_PRESSURE, (uint8_t*)&g_ap_alt, sizeof(double));
	}

	dps310_timer_callback(&dps310);
}

static void loop_1hz(uint8_t *data, size_t size) {
	g_check_freq_period = g_check_freq_counter;
	g_check_freq_counter = 0;
}

void air_pressure_setup(void) {
	air_pressure_init();
	subscribe(SCHEDULER_100HZ, loop_100hz);
	subscribe(SCHEDULER_1HZ, loop_1hz);
}
