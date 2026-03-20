#include "fault_handler.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <messages.h>

static uint64_t g_fault_count = 0;
static sensor_health_t g_sensor_health = {0};

static void handle_fault(uint8_t *data, size_t size) {
	g_fault_count++;
	platform_toggle_led(0);

	// Force state to DISARMED to stop all motors
	state_t disarmed = DISARMED;
	publish(FLIGHT_STATE_UPDATE, (uint8_t*)&disarmed, 1);

	platform_console("FAULT #%llu detected - motors disarmed\n", g_fault_count);
}

static void on_sensor_health_update(uint8_t *data, size_t size) {
	if (size < sizeof(sensor_health_t)) return;
	memcpy(&g_sensor_health, data, sizeof(sensor_health_t));
}

/*
 * LED status indicator (50 Hz)
 *
 * Flashes N times per second to indicate the highest-priority issue:
 *   5 flashes — IMU/compass unhealthy
 *   4 flashes — barometer unhealthy
 *   3 flashes — range finder unhealthy
 *   2 flashes — optical flow unhealthy
 *   1 flash   — GPS unhealthy
 *   0 (off)   — all systems nominal
 */
static void loop_50hz(uint8_t *data, size_t size) {
	uint8_t flash_count = 0;

	if (!g_sensor_health.gyro || !g_sensor_health.accel || !g_sensor_health.compass) {
		flash_count = 5;
	} else if (!g_sensor_health.baro) {
		flash_count = 4;
	} else if (!g_sensor_health.downward_range) {
		flash_count = 3;
	} else if (!g_sensor_health.optflow_down) {
		flash_count = 2;
	} else if (!g_sensor_health.gps) {
		flash_count = 1;
	}

	static uint16_t counter = 0;
	counter++;

	if (flash_count == 0) {
		return;
	}

	/* Flash N times in 500ms (25 cycles), then pause 500ms (25 cycles) */
	uint16_t total_cycle = 50;
	uint16_t flash_period = 25;

	if (counter <= flash_period) {
		uint16_t toggle_interval = flash_period / (2 * flash_count);
		if (toggle_interval < 1) toggle_interval = 1;

		if ((counter - 1) % toggle_interval == 0) {
			platform_toggle_led(0);
		}
	}

	if (counter >= total_cycle) {
		counter = 0;
	}
}

void fault_handler_setup(void) {
	subscribe(FAULT_DETECTION, handle_fault);
	subscribe(SENSOR_HEALTH_UPDATE, on_sensor_health_update);
	subscribe(SCHEDULER_50HZ, loop_50hz);
}
