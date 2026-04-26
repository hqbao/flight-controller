/**
 * POSITION ESTIMATION MODULE
 * 
 * This module estimates the drone's 3D position (X, Y, Z) by fusing:
 * - IMU linear acceleration (Body-Frame with gravity removed)
 * - Optical flow for horizontal velocity
 * - Barometer/Laser for altitude
 * 
 * ALGORITHM:
 * - Fusion 5 (Scalar Cascaded Complementary Filter) for X/Y/Z axes
 * 
 * KEY FEATURES:
 * - SI Units used throughout (Critical for filter stability)
 * - Cascaded dual-stage filter estimates position and velocity per axis
 * 
 * SIMPLIFICATIONS:
 * - Gyro De-Rotation: Disabled (Valid for stable hover/cruise)
 * - Latency Compensation: Disabled (Valid for non-racing speeds)
 * 
 * COORDINATE FRAMES:
 * - X/Y: Body-frame (X=Forward/Pitch, Y=Right/Roll)
 *   Matches optical flow sensor which also measures in body frame
 * - Z: Positive-UP altitude convention (laser/baro)
 *   Earth-frame Z-accel used (positive-up from fusion library)
 */
#include "position_estimation.h"
#include <pubsub.h>

#include <fusion5.h>
#include <fusion4.h>
#include <vector3d.h>
#include <string.h>
#include <math.h>
#include <macro.h>
#include <messages.h>

/* Select active filter for flight control: 0=Fusion5 (complementary), 1=Fusion4 (Kalman) */
#define USE_FUSION4 0

/* SI Unit Constants */
#define GRAVITY_MSS 9.80665

static double g_air_pressure_alt_raw = 0;
static double g_air_pressure_alt = 0;
static double g_alt = 0;
static double g_alt_prev = 0;
static double g_range_finder_alt = 0;
static vector3d_t g_linear_accel = {0, 0, 0};

static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

static fusion5_t g_fusion_x;
static fusion5_t g_fusion_y;
static fusion5_t g_fusion_z;

/* Fusion4 (Kalman) — parallel instances for comparison */
static fusion4_t g_fusion4_x;
static fusion4_t g_fusion4_y;
static fusion4_t g_fusion4_z;
static vector3d_t g_f4_pos = {0, 0, 0};
static vector3d_t g_f4_vel = {0, 0, 0};

static uint8_t g_monitor_msg[48] = {0};
static uint8_t g_log_class = LOG_CLASS_NONE;

/* Optical flow velocity gain (default — overridden by tuning) */
static double g_optflow_gain = 5.0;

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
#define RANGE_SWITCH_TO_LASER_THRESHOLD 1.0
#define RANGE_SWITCH_TO_BARO_THRESHOLD 1.2

/* GPS Data Storage (Raw) */
static gps_position_t g_gps_raw_pos = {0};
static gps_velocity_t g_gps_raw_vel = {0};

typedef enum {
	ALT_SOURCE_LASER = 0,
	ALT_SOURCE_BARO = 1,
} alt_source_t;

static alt_source_t g_alt_source = ALT_SOURCE_LASER;

/* Hybrid altitude: accumulated from alt_d deltas — source-agnostic like fusion5 */
static double g_hybrid_alt = 0;

static void gps_position_update(uint8_t *data, size_t size) {
    if (size < sizeof(gps_position_t)) return;
    memcpy(&g_gps_raw_pos, data, sizeof(gps_position_t));
}

static void gps_velocity_update(uint8_t *data, size_t size) {
    if (size < sizeof(gps_velocity_t)) return;
    memcpy(&g_gps_raw_vel, data, sizeof(gps_velocity_t));
}

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t msg;
	memcpy(&msg, data, sizeof(optflow_data_t));

	// Use generic 25Hz assumption (0.04s) as per legacy code
    double dt_flow = 0.04; 

	// 1. Update Altitude (Convert mm to Meters)
	if (msg.z > 0) g_range_finder_alt = msg.z / 1000.0;

	/**
	 * 2. Calculate SI Velocity (m/s) 
	 * 
	 * CRITICAL NOTE ON UNITS:
	 * msg.dx/dy are in Radians. 
	 * Velocity = AngularRate * Height = (dx/dt) * Height
	 * 
	 * SIMPLIFIED MODEL:
	 * 1. Gyro De-Rotation is disabled (assumes small angles).
	 * 2. Latency Compensation is disabled (assumes low speed).
	 * 3. Empirical gain of 5.0 applied to angular displacement.
	 */
	
	double vel_x = msg.dx * g_optflow_gain;
	double vel_y = msg.dy * g_optflow_gain;

    double flow_vel_x = 0;
    double flow_vel_y = 0;

	// Apply direction and copy to final flow velocity
	if (msg.direction == OPTFLOW_DOWNWARD) {
		flow_vel_x = vel_x;
		flow_vel_y = vel_y;
		// Store raw values
		g_optflow_down_dx = (float)msg.dx;
		g_optflow_down_dy = (float)msg.dy;
	} else if (msg.direction == OPTFLOW_UPWARD) {
		flow_vel_x = vel_x;
		flow_vel_y = -vel_y;
		// Store raw values
		g_optflow_up_dx = (float)msg.dx;
		g_optflow_up_dy = (float)msg.dy;
	}

    fusion5_update(&g_fusion_x, flow_vel_x, dt_flow);
    fusion5_update(&g_fusion_y, flow_vel_y, dt_flow);

    /* Fusion4: velocity update (same data) */
    fusion4_update(&g_fusion4_x, flow_vel_x);
    fusion4_update(&g_fusion4_y, flow_vel_y);

	if (g_alt_source == ALT_SOURCE_LASER) {
		// Update altitude
		g_alt = g_range_finder_alt;
		
        // Calculate delta (pseudo-velocity over 1 sec) to maintain continuity
        // when switching sources or large jumps occur
		double alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		fusion5_update(&g_fusion_z, alt_d, 1.0);

		/* Fusion4: use hybrid altitude accumulated from deltas */
		g_hybrid_alt += alt_d;
		fusion4_update_position(&g_fusion4_z, g_hybrid_alt, 0.05);
	}
}

static void air_pressure_update(uint8_t *data, size_t size) {
	if (size < sizeof(double)) return;
	// Incoming is scaled by 1000 (mm). Convert to Meters.
	memcpy(&g_air_pressure_alt_raw, data, sizeof(double));
	g_air_pressure_alt_raw /= 1000.0;
	
	if (fabs(g_linear_accel.z) > ACCEL_Z_THRESHOLD) {
		g_air_pressure_alt += BARO_ALPHA_HIGH_ACCEL * (g_air_pressure_alt_raw - g_air_pressure_alt);
	} else {
		g_air_pressure_alt += BARO_ALPHA_LOW_ACCEL * (g_air_pressure_alt_raw - g_air_pressure_alt);
	}

	if (g_alt_source == ALT_SOURCE_BARO) {
		// Update altitude
		g_alt = g_air_pressure_alt;

        // Calculate delta (pseudo-velocity over 1 sec) to maintain continuity
		double alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		fusion5_update(&g_fusion_z, alt_d, 1.0);

		/* Fusion4: use hybrid altitude accumulated from deltas */
		g_hybrid_alt += alt_d;
		fusion4_update_position(&g_fusion4_z, g_hybrid_alt, 0.1);
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
 * COORDINATE FRAMES:
 * - X/Y: Body-frame linear accel (X=Forward/Pitch, Y=Right/Roll)
 *   Matches optical flow sensor which also measures in body frame
 * - Z: Earth-frame linear accel (positive-up from fusion library)
 *   Used for altitude estimation with barometer/laser
 */
static void linear_accel_update(uint8_t *data, size_t size) {
	if (size < sizeof(linear_accel_data_t)) return;
	linear_accel_data_t la;
	memcpy(&la, data, sizeof(linear_accel_data_t));
	
	// X/Y: Body-frame (matches optical flow sensor frame)
	g_linear_accel.x = la.body.x * GRAVITY_MSS;
	g_linear_accel.y = la.body.y * GRAVITY_MSS;
	// Z: Earth-frame (positive-up from fusion library, for altitude)
	g_linear_accel.z = la.earth.z * GRAVITY_MSS;

	fusion5_predict(&g_fusion_x, g_linear_accel.x, 1.0 / ACCEL_FREQ);
    fusion5_predict(&g_fusion_y, g_linear_accel.y, 1.0 / ACCEL_FREQ);
    fusion5_predict(&g_fusion_z, g_linear_accel.z, 1.0 / ACCEL_FREQ);

	/* Fusion4: predict (same accel, same dt) */
	fusion4_predict(&g_fusion4_x, g_linear_accel.x, 1.0 / ACCEL_FREQ);
	fusion4_predict(&g_fusion4_y, g_linear_accel.y, 1.0 / ACCEL_FREQ);
	fusion4_predict(&g_fusion4_z, g_linear_accel.z, 1.0 / ACCEL_FREQ);

	/* Read both filters (both always run for comparison logging) */
	g_pos_final.x = g_fusion_x.pos_final;
	g_pos_final.y = g_fusion_y.pos_final;
	g_pos_final.z = g_fusion_z.pos_final;
	g_linear_veloc_final.x = g_fusion_x.veloc_final;
	g_linear_veloc_final.y = g_fusion_y.veloc_final;
	g_linear_veloc_final.z = g_fusion_z.veloc_final;

	g_f4_pos.x = fusion4_get_position(&g_fusion4_x);
	g_f4_pos.y = fusion4_get_position(&g_fusion4_y);
	g_f4_pos.z = fusion4_get_position(&g_fusion4_z);
	g_f4_vel.x = fusion4_get_velocity(&g_fusion4_x);
	g_f4_vel.y = fusion4_get_velocity(&g_fusion4_y);
	g_f4_vel.z = fusion4_get_velocity(&g_fusion4_z);

    // Pack position and velocity into update buffer
	position_state_t state_update;
#if USE_FUSION4
	state_update.position = g_f4_pos;
	state_update.velocity = g_f4_vel;
#else
	state_update.position = g_pos_final;
	state_update.velocity = g_linear_veloc_final;
#endif

    publish(POSITION_STATE_UPDATE, (uint8_t*)&state_update, sizeof(state_update));
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	if (data[0] == LOG_CLASS_POSITION || data[0] == LOG_CLASS_POSITION_OPTFLOW
		|| data[0] == LOG_CLASS_POSITION_COMPARE) {
		g_log_class = data[0];
	} else {
		g_log_class = LOG_CLASS_NONE;
	}
}

static void loop_logger(uint8_t *data, size_t size) {
	if (g_log_class == LOG_CLASS_NONE) return;

	if (g_log_class == LOG_CLASS_POSITION_OPTFLOW) {
		/* Optical Flow & Altitude + body-frame linear accel reference (8 floats, 32 bytes).
		 * Body accel X/Y is sent as an independent ground-truth-ish reference so the
		 * Python tool can integrate it and verify the sign of the optical flow
		 * matches the actual direction of motion (i.e. the module is mounted
		 * with the expected orientation). */
		float val[8];
		val[0] = g_optflow_down_dx;
		val[1] = g_optflow_down_dy;
		val[2] = g_optflow_up_dx;
		val[3] = g_optflow_up_dy;
		val[4] = (float)g_range_finder_alt;
		val[5] = (float)g_air_pressure_alt_raw;
		val[6] = (float)g_linear_accel.x; /* body-frame, m/s^2, X = forward */
		val[7] = (float)g_linear_accel.y; /* body-frame, m/s^2, Y = right   */
		memcpy(g_monitor_msg, val, 32);
		publish(SEND_LOG, (uint8_t*)g_monitor_msg, 32);
		return;
	}

	if (g_log_class == LOG_CLASS_POSITION_COMPARE) {
		/* Fusion5 vs Fusion4 comparison: 12 floats, 48 bytes */
		float val[12];
		/* Fusion5 (existing) */
		val[0] = g_pos_final.x;
		val[1] = g_pos_final.y;
		val[2] = g_pos_final.z;
		val[3] = g_linear_veloc_final.x;
		val[4] = g_linear_veloc_final.y;
		val[5] = g_linear_veloc_final.z;
		/* Fusion4 (new) */
		val[6]  = g_f4_pos.x;
		val[7]  = g_f4_pos.y;
		val[8]  = g_f4_pos.z;
		val[9]  = g_f4_vel.x;
		val[10] = g_f4_vel.y;
		val[11] = g_f4_vel.z;
		memcpy(g_monitor_msg, val, 48);
		publish(SEND_LOG, (uint8_t*)g_monitor_msg, 48);
		return;
	}

    float val[6];

	if (g_log_class == LOG_CLASS_POSITION) {
		// Position & Velocity (6 floats, 24 bytes)
		val[0] = g_pos_final.x;
		val[1] = g_pos_final.y;
		val[2] = g_pos_final.z;
		val[3] = g_linear_veloc_final.x;
		val[4] = g_linear_veloc_final.y;
		val[5] = g_linear_veloc_final.z;
	}

	memcpy(g_monitor_msg, val, 24);
	publish(SEND_LOG, (uint8_t*)g_monitor_msg, 24);
}

static void on_tuning_ready(uint8_t *data, size_t size) {
	if (size < sizeof(tuning_params_t)) return;
	tuning_params_t t;
	memcpy(&t, data, sizeof(tuning_params_t));

	g_optflow_gain = t.pe_optflow_gain;

	/* Update fusion5 params without resetting filter state */
	g_fusion_x.params.stage1_integ    = t.pe_xy_s1_integ;
	g_fusion_x.params.stage1_corr     = t.pe_xy_s1_corr;
	g_fusion_x.params.stage2_integ    = t.pe_xy_s2_integ;
	g_fusion_x.params.stage2_corr     = t.pe_xy_s2_corr;
	g_fusion_x.params.veloc_feedback  = t.pe_xy_v_fb;

	g_fusion_y.params.stage1_integ    = t.pe_xy_s1_integ;
	g_fusion_y.params.stage1_corr     = t.pe_xy_s1_corr;
	g_fusion_y.params.stage2_integ    = t.pe_xy_s2_integ;
	g_fusion_y.params.stage2_corr     = t.pe_xy_s2_corr;
	g_fusion_y.params.veloc_feedback  = t.pe_xy_v_fb;

	g_fusion_z.params.stage1_integ    = t.pe_z_s1_integ;
	g_fusion_z.params.stage1_corr     = t.pe_z_s1_corr;
	g_fusion_z.params.stage2_integ    = t.pe_z_s2_integ;
	g_fusion_z.params.stage2_corr     = t.pe_z_s2_corr;
	g_fusion_z.params.veloc_feedback  = t.pe_z_v_fb;
}

void position_estimation_setup(void) {
    fusion5_init(&g_fusion_x, 1.0, 1.25, 1.0, 10.0, 0.1);
    fusion5_init(&g_fusion_y, 1.0, 1.25, 1.0, 10.0, 0.1);
    fusion5_init(&g_fusion_z, 1.0, 0.5, 1.0, 10.0, 0.1);

    /* Fusion4: sigma_accel, sigma_vel, sigma_bias */
    fusion4_init(&g_fusion4_x, 0.5, 0.1, 0.01);
    fusion4_init(&g_fusion4_y, 0.5, 0.1, 0.01);
    fusion4_init(&g_fusion4_z, 0.2, 0.1, 0.001);

	subscribe(LINEAR_ACCEL_UPDATE, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_GPS, gps_position_update);
    subscribe(EXTERNAL_SENSOR_GPS_VELOC, gps_velocity_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(SCHEDULER_10HZ, check_altitude_source);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_25HZ, loop_logger);
	subscribe(TUNING_READY, on_tuning_ready);
}
