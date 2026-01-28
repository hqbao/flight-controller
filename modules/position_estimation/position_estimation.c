/**
 * POSITION ESTIMATION MODULE
 * 
 * This module estimates the drone's 3D position (X, Y, Z) by fusing:
 * - IMU linear acceleration (Body-Frame with gravity removed)
 * - Optical flow for horizontal position (with gyro de-rotation)
 * - Barometer/Laser for altitude
 * 
 * ALGORITHM:
 * - Two-stage estimation with correction from optical flow
 * - EST0: Integrates IMU acceleration to estimate position
 * - EST1: Derives velocity from EST0, integrates again with higher correction gain
 * - Final output blends EST1 with position sensors
 * 
 * GYRO DE-ROTATION:
 * - Optical flow measures apparent motion = real motion + rotation effect
 * - Gyro integration between frames provides rotation component
 * - Subtracting gyro rotation gives pure translation motion
 * - This makes position hold immune to drone tilt changes
 * 
 * COORDINATE FRAMES:
 * - SENSOR_LINEAR_ACCEL comes in Body-Frame
 * - X/Y axes are swapped and negated to match navigation frame
 * - Z-axis uses Body-Frame (assumes mostly level flight)
 */
#include "position_estimation.h"
#include <pubsub.h>
#include <fusion6.h>
#include <vector3d.h>
#include <string.h>
#include <math.h>
#include <macro.h>
#include <messages.h>

/* Macro to enable/disable sending MONITOR_DATA via logger 
 * 0: Disable
 * 1: Enable (Sends Position and Velocity)
 */
#define ENABLE_POSITION_ESTIMATION_MONITOR_LOG 1

static double g_air_pressure_alt_raw = 0;
static double g_air_pressure_alt = 0;
static double g_alt = 0;
static double g_alt_prev = 0;
static struct {
    double dx;
    double dy;
    double z;
} g_optflow = {0, 0, 0};
static vector3d_t g_linear_accel = {0, 0, 0};
static vector3d_t g_pos_true = {0, 0, 0};
static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

static fusion6_t g_fusion_x;
static fusion6_t g_fusion_y;
static fusion6_t g_fusion_z;

#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG > 0
static uint8_t g_monitor_msg[24] = {0};
#endif

/* Tuning Parameters */

#define BARO_ALPHA_HIGH_ACCEL    0.05
#define BARO_ALPHA_LOW_ACCEL     0.005

#define ACCEL_Z_THRESHOLD       300.0
#define RANGE_SWITCH_TO_LASER_THRESHOLD 500.0
#define RANGE_SWITCH_TO_BARO_THRESHOLD 1000.0
#define OPTFLOW_SCALE 100.0
#define OPTFLOW_LIMIT_MAX 1.0
#define OPTFLOW_LIMIT_MIN 0.1
#define OPTFLOW_LIMIT_INCREMENT 0.0
#define OPTFLOW_LIMIT_DECREMENT 0.0
#define LOW_FREQ_THRESHOLD 5.0
#define DRIFT_TIME_THRESHOLD 2.0

typedef enum {
	ALT_SOURCE_LASER = 0,
	ALT_SOURCE_BARO = 1,
} alt_source_t;

static alt_source_t g_alt_source = ALT_SOURCE_LASER;
static double g_optflow_limit = OPTFLOW_LIMIT_MAX;
static double g_oscillation_freq = 0;
static double g_drift_time = 0;

static void linear_drift_update(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	vector3d_t *drift = (vector3d_t*)data;
	g_drift_time = drift->x > drift->y ? drift->x : drift->y;
	
	// Increase bound when drift is sustained (need more optical flow data)
	if (g_drift_time > DRIFT_TIME_THRESHOLD) {
		g_optflow_limit += OPTFLOW_LIMIT_INCREMENT;
		if (g_optflow_limit > OPTFLOW_LIMIT_MAX) {
			g_optflow_limit = OPTFLOW_LIMIT_MAX;
		}
	}
}

static void oscillation_freq_update(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	vector3d_t *freq = (vector3d_t*)data;
	g_oscillation_freq = freq->x < freq->y ? freq->x : freq->y;
	
	// Lower bound when high frequency oscillation detected (overshoot)
	if (g_oscillation_freq < LOW_FREQ_THRESHOLD) {
		g_optflow_limit -= OPTFLOW_LIMIT_DECREMENT;
		if (g_optflow_limit < OPTFLOW_LIMIT_MIN) {
			g_optflow_limit = OPTFLOW_LIMIT_MIN;
		}
	}
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t *msg = (optflow_data_t*)data;

	// Scale from radians to mm displacement
	double dx_mm = (float)msg->dx * OPTFLOW_SCALE;
	double dy_mm = (float)msg->dy * OPTFLOW_SCALE;

	if (msg->direction == OPTFLOW_DOWNWARD) {
		g_optflow.dx = LIMIT(dx_mm, -g_optflow_limit, g_optflow_limit);
		g_optflow.dy = LIMIT(dy_mm, -g_optflow_limit, g_optflow_limit);
	} else if (msg->direction == OPTFLOW_UPWARD) {
		g_optflow.dx = LIMIT(dx_mm, -g_optflow_limit, g_optflow_limit);
		g_optflow.dy = -LIMIT(dy_mm, -g_optflow_limit, g_optflow_limit);
	}
	
	if (msg->z > 0) g_optflow.z = msg->z;

	g_pos_true.x += g_optflow.dx;
	g_pos_true.y += g_optflow.dy;

	// Auto switch: use laser if range < 500, else use barometer if range > 1000
    alt_source_t new_alt_source = g_alt_source;
    if (g_optflow.z > 0 && g_optflow.z < RANGE_SWITCH_TO_LASER_THRESHOLD) {
        new_alt_source = ALT_SOURCE_LASER;
    } else if (g_optflow.z > RANGE_SWITCH_TO_BARO_THRESHOLD || g_optflow.z <= 0) {
        new_alt_source = ALT_SOURCE_BARO;
    }
	
    // Update state
    if (new_alt_source != g_alt_source) {
        g_alt_source = new_alt_source;
        if (g_alt_source == ALT_SOURCE_LASER) {
            g_alt_prev = g_optflow.z;
        } else if (g_alt_source == ALT_SOURCE_BARO) {
            g_alt_prev = g_air_pressure_alt;
        }
    }

	if (g_alt_source == ALT_SOURCE_LASER) {
		// Update altitude
		g_alt = g_optflow.z;
		double alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_pos_true.z += alt_d;
	}
}

static void air_pressure_update(uint8_t *data, size_t size) {
	if (size < sizeof(double)) return;
	g_air_pressure_alt_raw = *(double*)data;
	if (fabs(g_linear_accel.z) > ACCEL_Z_THRESHOLD) {
		g_air_pressure_alt += BARO_ALPHA_HIGH_ACCEL * (g_air_pressure_alt_raw - g_air_pressure_alt);
	} else {
		g_air_pressure_alt += BARO_ALPHA_LOW_ACCEL * (g_air_pressure_alt_raw - g_air_pressure_alt);
	}

	if (g_alt_source == ALT_SOURCE_BARO) {
		// Update altitude
		g_alt = g_air_pressure_alt;
		double alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_pos_true.z += alt_d;
	}
}

/**
 * LINEAR ACCEL UPDATE: Called at 500Hz
 * 
 * Receives linear acceleration (gravity removed) from attitude estimation module.
 * 
 * COORDINATE TRANSFORMATION:
 * - X/Y: Use Body-Frame (optical flow fusion requires body relative motion).
 *   Swapped/Negated to match navigation frame:
 *   - nav_x = -body_y
 *   - nav_y = -body_x
 * - Z: Use Earth-Frame (altitude estimation).
 */
static void linear_accel_update(uint8_t *data, size_t size) {
	if (size < sizeof(linear_accel_data_t)) return;
	linear_accel_data_t *la = (linear_accel_data_t*)data;
	g_linear_accel.x = -la->body.y * MAX_IMU_ACCEL;
	g_linear_accel.y = -la->body.x * MAX_IMU_ACCEL;
	g_linear_accel.z = la->earth.z * MAX_IMU_ACCEL;

	fusion6_update(&g_fusion_x, g_linear_accel.x, g_pos_true.x);
    fusion6_update(&g_fusion_y, g_linear_accel.y, g_pos_true.y);
    fusion6_update(&g_fusion_z, g_linear_accel.z, g_pos_true.z);

	g_pos_final.x = -g_fusion_x.pos_final;
	g_pos_final.y = -g_fusion_y.pos_final;
	g_pos_final.z = g_fusion_z.pos_final;

	g_linear_veloc_final.x = -g_fusion_x.veloc_final;
	g_linear_veloc_final.y = -g_fusion_y.veloc_final;
	g_linear_veloc_final.z = g_fusion_z.veloc_final;

	static uint8_t g_pos_est_msg[sizeof(vector3d_t) * 2] = {0};
	memcpy(g_pos_est_msg, &g_pos_final, sizeof(vector3d_t));
	memcpy(&g_pos_est_msg[sizeof(vector3d_t)], &g_linear_veloc_final, sizeof(vector3d_t));

	publish(POSITION_STATE_UPDATE, (uint8_t*)&g_pos_est_msg, sizeof(vector3d_t) * 2);
}

#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	float val[6] = {
		(float)g_pos_final.x, (float)g_pos_final.y, (float)g_pos_final.z,
		(float)g_linear_veloc_final.x, (float)g_linear_veloc_final.y, (float)g_linear_veloc_final.z
	};
	memcpy(g_monitor_msg, val, 24);
	publish(MONITOR_DATA, (uint8_t*)g_monitor_msg, 24);
}
#endif

void position_estimation_setup(void) {
	// XY Axis: Fast integration (0.05), standard correction
    fusion6_init(&g_fusion_x, (double)ACCEL_FREQ, 0.05, 0.5, 0.05, 20.0, 0.005);
    fusion6_init(&g_fusion_y, (double)ACCEL_FREQ, 0.05, 0.5, 0.05, 20.0, 0.005);
    
    // Z Axis: Stronger integration dominance (1.0)
    fusion6_init(&g_fusion_z, (double)ACCEL_FREQ, 1.0, 0.5, 1.0, 20.0, 0.005);

	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(OSCILLATION_FREQ_DETECTED, oscillation_freq_update);
	subscribe(LINEAR_DRIFT_DETECTION, linear_drift_update);
#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
