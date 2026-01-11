#include "state_detector.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <vector3d.h>
#include <math.h>
#include <macro.h>

#define DISARM_RANGE_WHEN_LANDING 10
#define DISARM_TIME_WHEN_LANDING 50
#define DISARM_IF_EXCEEDED_ANGLE_RAGE 60
#define ALLOWED_LANDING_RANGE 500
#define TOOK_OFF_RANGE 100

#define RC_STATE_DISARMED 0
#define RC_STATE_ARMED 1
#define STICK_MIN -90
#define STICK_MAX 90
#define TAKEOFF_THROTTLE 5
#define OPTFLOW_TYPE_DOWNWARD 0
#define OPTFLOW_RANGE_OFFSET 12

typedef enum {
	DISARMED = 0,
	ARMED,
	READY,
	TAKING_OFF,
	FLYING,
	LANDING,
	TESTING,
} state_t;

typedef struct {
	uint8_t state;
	uint8_t mode;
} rc_state_ctl_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float alt;
} rc_att_ctl_t;

typedef struct {
	double roll;
	double pitch;
	double yaw;
} angle3d_t;

static uint8_t g_module_initialized[MODULE_ID_MAX] = {0};

typedef struct {
    double dx;
    double dy;
    double z;
} optflow_t;

static angle3d_t g_angular_state = {0, 0, 0};
static rc_state_ctl_t g_rc_state_ctl;
static rc_state_ctl_t g_rc_state_ctl_prev;
static rc_att_ctl_t g_rc_att_ctl;
static state_t g_state = DISARMED;
static state_t g_state_prev = DISARMED;
static char g_imu_calibrated = 0;
static double g_air_pressure = 0;
static double g_downward_range = 0;

static void state_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_state_ctl, data, size);
}

static void move_in_control_update(uint8_t *data, size_t size) {
	memcpy(&g_rc_att_ctl, data, size);
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (data[1] == OPTFLOW_TYPE_DOWNWARD) { // Downward
        int32_t range;
        memcpy(&range, &data[OPTFLOW_RANGE_OFFSET], sizeof(int32_t));
		g_downward_range = (double)range;
	}
}

static void air_pressure_update(uint8_t *data, size_t size) {
	if (size >= sizeof(double)) {
		memcpy(&g_air_pressure, data, sizeof(double));
	}
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 1) g_imu_calibrated = 1;
}

static void loop_100hz(uint8_t *data, size_t size) {
	if (g_rc_state_ctl.state == RC_STATE_DISARMED) {
		g_state = DISARMED;
	}

	if (g_rc_state_ctl.state == RC_STATE_ARMED && g_rc_state_ctl_prev.state == RC_STATE_DISARMED) {
		if (g_imu_calibrated == 1) {
			g_state = ARMED;
		}
	}

	if (g_state == ARMED) {
		char stick1_most_left 	= g_rc_att_ctl.yaw == STICK_MIN;
		char stich1_most_bottom = g_rc_att_ctl.alt == STICK_MIN;
		char stick2_most_right 	= g_rc_att_ctl.roll == STICK_MAX;
		char stich2_most_bottom = g_rc_att_ctl.pitch == STICK_MIN;
		if (stick1_most_left && stich1_most_bottom &&
				stick2_most_right && stich2_most_bottom) {
			g_state = READY;
		}
	}

	if (g_state == READY) {
		if (g_rc_att_ctl.alt > TAKEOFF_THROTTLE) {
			g_state = g_rc_state_ctl.state == 1 ? TAKING_OFF : TESTING;
		}
	}

	if (g_state == TAKING_OFF) {
		if (g_downward_range > TOOK_OFF_RANGE) {
			g_state = FLYING;
		}
	}

	if (g_state == FLYING) {
		if (g_downward_range < DISARM_RANGE_WHEN_LANDING && g_rc_att_ctl.alt == -90) {
			g_state = DISARMED;
		}

		if (g_rc_state_ctl.state == 2 && g_rc_state_ctl_prev.state != 2) {
			if (g_downward_range > ALLOWED_LANDING_RANGE) {
				g_state = LANDING;
			}
		}
	}

	if (g_state == TAKING_OFF || g_state == FLYING) {
		if (fabs(g_angular_state.roll) > DISARM_IF_EXCEEDED_ANGLE_RAGE
				|| fabs(g_angular_state.pitch) > DISARM_IF_EXCEEDED_ANGLE_RAGE) {
			g_state = DISARMED;
		}
	}

	if (g_state == LANDING) {
		static int landing_counter = DISARM_TIME_WHEN_LANDING;
		if (g_downward_range < DISARM_RANGE_WHEN_LANDING) {
			if (landing_counter < 1) {
				g_state = DISARMED;
			}

			landing_counter--;
		} else {
			landing_counter = DISARM_TIME_WHEN_LANDING;
		}

		if (g_rc_state_ctl.state != 2) {
			g_state = FLYING;
		}
	}

	memcpy(&g_rc_state_ctl_prev, &g_rc_state_ctl, sizeof(rc_state_ctl_t));

	if (g_state != g_state_prev) {
		publish(STATE_DETECTION_UPDATE, (uint8_t*)&g_state, 1);
	}

	g_state_prev = g_state;
}

static void module_initialized_update(uint8_t *data, size_t size) {
	if (size < sizeof(module_initialized_t)) {
		return;
	}
	
	module_initialized_t *module_initialized = (module_initialized_t *)data;
	
	if (module_initialized->id < MODULE_ID_MAX) {
		g_module_initialized[module_initialized->id] = module_initialized->initialized;
	}
}

static void angular_state_update(uint8_t *data, size_t size) {
	memcpy(&g_angular_state, data, size);
}

static void loop_1hz(uint8_t *data, size_t size) {
	// Check if all setup modules are ready, then trigger IMU calibration check
	uint8_t local_storage_initialized = g_module_initialized[MODULE_ID_LOCAL_STORAGE];
	uint8_t imu_initialized = g_module_initialized[MODULE_ID_IMU];
	uint8_t air_pressure_initialized = g_module_initialized[MODULE_ID_AIR_PRESSURE];
	
	static uint8_t calibration_triggered = 0;
	if (!calibration_triggered && local_storage_initialized && imu_initialized && air_pressure_initialized) {
		publish(SENSOR_CHECK_GYRO_CALIBRATION, NULL, 0);
		calibration_triggered = 1;
	}
}

static void loop_50hz(uint8_t *data, size_t size) {
	// Check modules in priority order and determine flash count
	uint8_t flash_count = 0;
	
	uint8_t optflow_initialized = g_module_initialized[MODULE_ID_OPTFLOW];
	uint8_t gps_initialized = g_module_initialized[MODULE_ID_GPS];
	
	// Check in priority order - highest priority first
	if (g_imu_calibrated == 0) {
		flash_count = 5;
	} else if (g_air_pressure == 0) {
		flash_count = 4;
	} else if (g_downward_range == 0) {
		flash_count = 3;
	} else if (!optflow_initialized) {
		flash_count = 2;
	} else if (!gps_initialized) {
		flash_count = 1;
	}
	
	static uint16_t counter = 0;
	counter++;
	
	if (flash_count == 0) {
		// All modules ready - no flashing
		return;
	} else {
		// Flash N times in 500ms (25 cycles), then pause 500ms (25 cycles)
		uint16_t total_cycle = 50; // 1 second cycle
		uint16_t flash_period = 25; // 500ms for flashing
		
		if (counter <= flash_period) {
			// Flashing phase: toggle every (25/(2*N)) cycles for N flashes
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
}

void state_detector_setup(void) {
	subscribe(MODULE_INITIALIZED_UPDATE, module_initialized_update);
	subscribe(SENSOR_IMU1_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	subscribe(RC_STATE_UPDATE, state_control_update);
	subscribe(RC_MOVE_IN_UPDATE, move_in_control_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(ANGULAR_STATE_UPDATE, angular_state_update);
	subscribe(SCHEDULER_100HZ, loop_100hz);
	subscribe(SCHEDULER_50HZ, loop_50hz);
	subscribe(SCHEDULER_1HZ, loop_1hz);
}
