#include "air_pressure.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include "dps310.h"

#define ALT_SAMPLES 100
#define WARMUP_DELAY_MS 100
#define WARMUP_COUNTER_INIT -10
#define ALTITUDE_SCALE_FACTOR 1000.0
#define LOOP_INTERVAL_MS 40  // 25 Hz

static double g_air_pressure = 0;

// Returns 0 on success, non-zero if DPS310 not found or I2C error.
static int16_t air_pressure_init(void) {
	platform_delay(WARMUP_DELAY_MS);

	int16_t ret = dps310_probe();
	if (ret != 0) return ret;

	platform_delay(WARMUP_DELAY_MS);

	// Start DPS310 continuous (background) mode.
	// The sensor now measures autonomously at configured rates
	// (pressure 32 Hz, temperature 4 Hz) and updates result registers
	// without any host-side trigger or polling.
	ret = dps310_start_continuous();
	if (ret != 0) return ret;

	platform_delay(WARMUP_DELAY_MS);  // let first measurements complete
	return 0;
}

// Runs from LOOP (main-thread context), rate-limited to ~25 Hz.
//
// MUST NOT run from TIM8 ISR (SCHEDULER_25HZ) because:
//   dps310_read_continuous() → HAL_I2C_Mem_Read (polling) uses
//   HAL_GetTick() for timeout.  TIM8 has priority 0, SysTick has
//   priority 15 — SysTick cannot preempt TIM8.  Any I2C bus error
//   causes HAL_GetTick() to never advance → timeout never fires →
//   permanent hang.
static void air_pressure_loop(uint8_t *data, size_t size) {
    static uint32_t last_ms = 0;
    uint32_t now = platform_time_ms();
    if ((now - last_ms) < LOOP_INTERVAL_MS) return;
    last_ms = now;

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
	if (air_pressure_init() != 0) return;  // DPS310 not available
	subscribe(LOOP, air_pressure_loop);
}
