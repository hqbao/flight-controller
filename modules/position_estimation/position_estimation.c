/**
 * POSITION ESTIMATION MODULE
 * 
 * This module estimates the drone's 3D position (X, Y, Z) by fusing:
 * - IMU linear acceleration (Body-Frame with gravity removed)
 * - Optical flow for horizontal position (with gyro de-rotation)
 * - Barometer/Laser for altitude
 * 
 * ALGORITHM:
 * - Fusion 6 (Scalar Cascaded Complementary Filter) for X/Y/Z axes
 * 
 * KEY FEATURES:
 * - SI Units used throughout (Critical for Kalman Filter stability)
 * - 3-State Filter (Pos, Vel, AccelBias) estimates and removes IMU drift
 * 
 * SIMPLIFICATIONS:
 * - Gyro De-Rotation: Disabled (Valid for stable hover/cruise)
 * - Latency Compensation: Disabled (Valid for non-racing speeds)
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
 * 1: Mode 1 - Send Position & Velocity (6 floats, 24 bytes)
 * 2: Mode 2 - Send Optical Flow & Altitude (6 floats, 24 bytes)
 */
#define ENABLE_POSITION_ESTIMATION_MONITOR_LOG 1

/* SI Unit Constants */
#define GRAVITY_MSS 9.80665
#define IMU_RAW_G 16384.0

static double g_air_pressure_alt_raw = 0;
static double g_air_pressure_alt = 0;
static double g_alt = 0;
static double g_alt_prev = 0;
static double g_range_finder_alt = 0;
static vector3d_t g_linear_accel = {0, 0, 0};

static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

static fusion6_t g_fusion_x;
static fusion6_t g_fusion_y;
static fusion6_t g_fusion_z;

#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG > 0
static uint8_t g_monitor_msg[24] = {0};
#endif

/* Optical Flow Data Storage for Logging */
static float g_optflow_down_dx = 0.0f;
static float g_optflow_down_dy = 0.0f;
static float g_optflow_up_dx = 0.0f;
static float g_optflow_up_dy = 0.0f;

/* Tuning Parameters */

#define BARO_ALPHA_HIGH_ACCEL 0.05
#define BARO_ALPHA_LOW_ACCEL 0.005

// Thresholds in SI Units (m/s^2 and Meters)
#define ACCEL_Z_THRESHOLD 0.2
#define RANGE_SWITCH_TO_LASER_THRESHOLD 0.25
#define RANGE_SWITCH_TO_BARO_THRESHOLD 0.5

/* GPS Data Storage (Raw) */
static gps_position_t g_gps_raw_pos = {0};
static gps_velocity_t g_gps_raw_vel = {0};

typedef enum {
	ALT_SOURCE_LASER = 0,
	ALT_SOURCE_BARO = 1,
} alt_source_t;

static alt_source_t g_alt_source = ALT_SOURCE_LASER;

static void gps_position_update(uint8_t *data, size_t size) {
    if (size < sizeof(gps_position_t)) return;
    gps_position_t *pos = (gps_position_t*)data;
    // Store for debugging/fusion
    memcpy(&g_gps_raw_pos, pos, sizeof(gps_position_t));
}

static void gps_velocity_update(uint8_t *data, size_t size) {
    if (size < sizeof(gps_velocity_t)) return;
    gps_velocity_t *vel = (gps_velocity_t*)data;
    // Store for debugging/fusion
    memcpy(&g_gps_raw_vel, vel, sizeof(gps_velocity_t));
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t *msg = (optflow_data_t*)data;

	// Use generic 25Hz assumption (0.04s) as per legacy code
    double dt_flow = 0.04; 

	// 1. Update Altitude (Convert mm to Meters)
	if (msg->z > 0) g_range_finder_alt = msg->z / 1000.0;

	/**
	 * 2. Calculate SI Velocity (m/s) 
	 * 
	 * CRITICAL NOTE ON UNITS:
	 * msg->dx/dy are in Radians. 
	 * Velocity = AngularRate * Height = (dx/dt) * Height
	 * 
	 * SIMPLIFIED MODEL:
	 * 1. Gyro De-Rotation is disabled (assumes small angles).
	 * 2. Latency Compensation is disabled (assumes low speed).
	 * 3. Scaling is 1:1 (Radians -> m/s). Do NOT divide by 100.
	 */
	
	// Calculate base velocity components (Scalars), assuming height 1 meter
	double vel_x = msg->dx * 5.0;
	double vel_y = msg->dy * 5.0;

    double flow_vel_x = 0;
    double flow_vel_y = 0;

	// Apply direction and copy to final flow velocity
	if (msg->direction == OPTFLOW_DOWNWARD) {
		flow_vel_x = vel_x;
		flow_vel_y = vel_y;
		// Store raw values
		g_optflow_down_dx = (float)msg->dx;
		g_optflow_down_dy = (float)msg->dy;
	} else if (msg->direction == OPTFLOW_UPWARD) {
		flow_vel_x = vel_x;
		flow_vel_y = -vel_y;
		// Store raw values
		g_optflow_up_dx = (float)msg->dx;
		g_optflow_up_dy = (float)msg->dy;
	}

    fusion6_update(&g_fusion_x, flow_vel_x, dt_flow);
    fusion6_update(&g_fusion_y, flow_vel_y, dt_flow);

	if (g_alt_source == ALT_SOURCE_LASER) {
		// Update altitude
		g_alt = g_range_finder_alt;
		double alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		fusion6_update(&g_fusion_z, alt_d, 1.0);
	}
}

static void air_pressure_update(uint8_t *data, size_t size) {
	if (size < sizeof(double)) return;
	// Incoming is scaled by 1000 (mm). Convert to Meters.
	g_air_pressure_alt_raw = (*(double*)data) / 1000.0;
	
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
		fusion6_update(&g_fusion_z, alt_d, 1.0);
	}
}

static void check_altitude_source(uint8_t *data, size_t size) {
	// Auto switch: use laser if range < 0.25m, else use barometer if range > 0.5m
    alt_source_t new_alt_source = g_alt_source;
    if (g_range_finder_alt > 0 && g_range_finder_alt < RANGE_SWITCH_TO_LASER_THRESHOLD) {
        new_alt_source = ALT_SOURCE_LASER;
    } else if (g_range_finder_alt > RANGE_SWITCH_TO_BARO_THRESHOLD || g_range_finder_alt <= 0) {
        new_alt_source = ALT_SOURCE_BARO;
    }
	
    // Update state
    if (new_alt_source != g_alt_source) {
        g_alt_source = new_alt_source;
        if (g_alt_source == ALT_SOURCE_LASER) {
            g_alt_prev = g_range_finder_alt;
        } else if (g_alt_source == ALT_SOURCE_BARO) {
            g_alt_prev = g_air_pressure_alt;
        }
    }
}

/**
 * LINEAR ACCEL UPDATE: Called at 500Hz
 * 
 * Receives linear acceleration from attitude estimation module.
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
	
	// Convert to SI Units (m/s^2)
	// la->body is normalized (1.0 = 1G)
	g_linear_accel.x = -la->body.y * GRAVITY_MSS;
	g_linear_accel.y = -la->body.x * GRAVITY_MSS;
	g_linear_accel.z = la->earth.z * GRAVITY_MSS;

    // Predict state based on acceleration (500Hz -> 0.002s)
	fusion6_predict(&g_fusion_x, g_linear_accel.x, 1.0 / ACCEL_FREQ);
    fusion6_predict(&g_fusion_y, g_linear_accel.y, 1.0 / ACCEL_FREQ);
    fusion6_predict(&g_fusion_z, g_linear_accel.z, 1.0 / ACCEL_FREQ);

	g_pos_final.x = -g_fusion_x.pos_final;
	g_pos_final.y = -g_fusion_y.pos_final;
	g_pos_final.z = g_fusion_z.pos_final;

	g_linear_veloc_final.x = -g_fusion_x.veloc_final;
	g_linear_veloc_final.y = -g_fusion_y.veloc_final;
	g_linear_veloc_final.z = g_fusion_z.veloc_final;

    // Pack position and velocity into update buffer
	position_state_t state_update;
	state_update.position = g_pos_final;
	state_update.velocity = g_linear_veloc_final;

    publish(POSITION_STATE_UPDATE, (uint8_t*)&state_update, sizeof(state_update));
}

#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG > 0
static void loop_logger(uint8_t *data, size_t size) {
    float val[6];

#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG == 1
    // Mode 1: Position & Velocity
    val[0] = g_fusion_x.pos_final;
    val[1] = g_fusion_y.pos_final;
    val[2] = g_fusion_z.pos_final;
    val[3] = g_fusion_x.veloc_final;
    val[4] = g_fusion_y.veloc_final;
    val[5] = g_fusion_z.veloc_final;
#elif ENABLE_POSITION_ESTIMATION_MONITOR_LOG == 2
    // Mode 2: Optical Flow & Altitude (amplified 1000x for visibility)
    val[0] = g_optflow_down_dx;
    val[1] = g_optflow_down_dy;
    val[2] = g_optflow_up_dx;
    val[3] = g_optflow_up_dy;
    val[4] = (float)g_range_finder_alt;
    val[5] = (float)g_alt;
#endif

	memcpy(g_monitor_msg, val, 24);
	publish(MONITOR_DATA, (uint8_t*)g_monitor_msg, 24);
}
#endif

void position_estimation_setup(void) {
    fusion6_init(&g_fusion_x, 1.0, 0.5, 1.0, 20.0, 0.1);
    fusion6_init(&g_fusion_y, 1.0, 0.5, 1.0, 20.0, 0.1);
    fusion6_init(&g_fusion_z, 1.0, 0.5, 1.0, 20.0, 0.1);

	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_GPS, gps_position_update);
    subscribe(EXTERNAL_SENSOR_GPS_VELOC, gps_velocity_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SCHEDULER_10HZ, check_altitude_source);
#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG > 0
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
