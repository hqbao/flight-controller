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
 * - SENSOR_LINEAR_ACCEL comes in Body-Frame (from Fusion2, Fusion1 doesn't provide it)
 * - X/Y axes are swapped and negated to match navigation frame
 * - Z-axis uses Body-Frame (assumes mostly level flight)
 * 
 * NOTE: This module currently relies on Fusion2 (EKF) for linear acceleration.
 *       For Fusion1 (Mahony), linear acceleration with gravity removed is not published.
 *       When using Fusion1, position estimation will only use optical flow and altitude sensors.
 */
#include "position_estimation.h"
#include <pubsub.h>
#include <vector3d.h>
#include <string.h>
#include <math.h>
#include <macro.h>

/* Structure for SENSOR_LINEAR_ACCEL data
 * Copied from attitude_estimation.h to avoid cross-module include
 */
typedef struct {
	vector3d_t body;
	vector3d_t earth;
} linear_accel_data_t;


/* Macro to enable/disable sending MONITOR_DATA via logger 
 * 0: Disable
 * 1: Position
 * 2: Velocity
 */
#define ENABLE_POSITION_ESTIMATION_MONITOR_LOG 0

#define ACCEL_FREQ 500
#define GYRO_FREQ 1000
#define MAX_IMU_ACCEL 16384
#define DEG2RAD 0.01745329251

typedef enum {
	OPTFLOW_DOWNWARD = 0,
	OPTFLOW_UPWARD = 1,
} optflow_direction_t;

typedef struct {
    double dx;      // Linear displacement X (mm)
    double dy;      // Linear displacement Y (mm)
    double z;       // Range finder altitude (mm)
    optflow_direction_t direction; 
} optflow_data_t;

static double g_air_pressure_alt_raw = 0;
static double g_air_pressure_alt = 0;
static double g_alt = 0;
static double g_alt_prev = 0;
static double g_alt_d = 0;
static struct {
    double dx;
    double dy;
    double z;
} g_optflow = {0, 0, 0};
static vector3d_t g_linear_accel = {0, 0, 0};
static vector3d_t g_linear_veloc0 = {0, 0, 0};
static vector3d_t g_linear_veloc1 = {0, 0, 0};
static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_est0 = {0, 0, 0};
static vector3d_t g_pos_est0_prev = {0, 0, 0};
static vector3d_t g_pos_true = {0, 0, 0};
static vector3d_t g_pos_est1 = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};
static optflow_data_t g_optflow_up = {0, 0, 0, 0};
static optflow_data_t g_optflow_down = {0, 0, 0, 0};

/* Gyro integration for de-rotation (accumulated in degrees) */
static vector3d_t g_gyro_integrated = {0, 0, 0};
static float g_derotated_rad_x = 0;
static float g_derotated_rad_y = 0;
static float g_gyro_corr_x = 0;
static float g_gyro_corr_y = 0;

/* Tuning Parameters */
#define POS_XY_EST0_INTEGRATION_GAIN     0.05
#define POS_Z_EST0_INTEGRATION_GAIN      1.0
#define POS_XY_EST0_TRUE_CORRECTION_GAIN 0.5
#define POS_Z_EST0_TRUE_CORRECTION_GAIN  0.5
#define POS_XY_EST1_INTEGRATION_GAIN 0.05
#define POS_Z_EST1_INTEGRATION_GAIN  1.0
#define POS_XY_EST1_TRUE_CORRECTION_GAIN 10.0
#define POS_Z_EST1_TRUE_CORRECTION_GAIN  10.0
#define ALT_DERIVATIVE_SCALE    100.0
#define BARO_ALPHA_HIGH_ACCEL   0.05
#define BARO_ALPHA_LOW_ACCEL    0.005
#define OUTPUT_POS_SCALE        1.0
#define OUTPUT_VELOC_SCALE      1.0
#define ACCEL_Z_THRESHOLD       300.0
#define RANGE_SWITCH_TO_LASER_THRESHOLD 500.0
#define RANGE_SWITCH_TO_BARO_THRESHOLD 1000.0
#define VELOC_XY_CORRECTION_GAIN 0.005
#define VELOC_Z_CORRECTION_GAIN 0.005
#define OPTFLOW_LIMIT_MAX 2.0
#define OPTFLOW_LIMIT_MIN 0.2
#define OPTFLOW_LIMIT_INCREMENT 0.05
#define OPTFLOW_LIMIT_DECREMENT 0.05
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

/* Gyro update: Called at 1kHz - integrate gyro for de-rotation */
static void gyro_update(uint8_t *data, size_t size) {
	if (size < 12) return;  // Expect 3 floats (gx, gy, gz)
	float *gyro = (float*)data;  // Data is float[3]: gx, gy, gz in deg/s
	
	// Integrate gyro (accumulate rotation since last optical flow update) in degrees
	g_gyro_integrated.x += gyro[0] / GYRO_FREQ;
	g_gyro_integrated.y -= gyro[1] / GYRO_FREQ;
}

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

	// Apply gyro de-rotation
	// Optical flow comes in radians, gyro accumulated in degrees
	// Camera X axis maps to drone pitch (gyro Y)
	// Camera Y axis maps to drone roll (gyro X)
	float gyro_rad_x = (float)g_gyro_integrated.y * DEG2RAD;
	float gyro_rad_y = (float)g_gyro_integrated.x * DEG2RAD;
	
	// Clamp gyro correction to prevent over-correction
	// Limit to ±|flow| to avoid sign flips
	g_gyro_corr_x = LIMIT(gyro_rad_x, -fabs((float)msg->dx), fabs((float)msg->dx));
	g_gyro_corr_y = LIMIT(gyro_rad_y, -fabs((float)msg->dy), fabs((float)msg->dy));
	
	// De-rotation: For downward camera, subtract gyro
	// For upward camera, add gyro (camera rotated 180°)
	if (msg->direction == OPTFLOW_DOWNWARD) {
		g_derotated_rad_x = (float)msg->dx - g_gyro_corr_x;
		g_derotated_rad_y = (float)msg->dy - g_gyro_corr_y;
	} else {  // OPTFLOW_UPWARD
		g_derotated_rad_x = (float)msg->dx + g_gyro_corr_x;
		g_derotated_rad_y = -(float)msg->dy + g_gyro_corr_y;
	}
	
	// Reset gyro integration for next optical flow frame
	g_gyro_integrated.x = 0;
	g_gyro_integrated.y = 0;
	
	// Scale from radians to mm displacement
	double dx_mm = g_derotated_rad_x * 200.0;
	double dy_mm = g_derotated_rad_y * 200.0;

	if (msg->direction == OPTFLOW_DOWNWARD) {
		memcpy(&g_optflow_down, msg, sizeof(optflow_data_t));
		g_optflow.dx = LIMIT(dx_mm, -2.0, 2.0);
		g_optflow.dy = LIMIT(dy_mm, -2.0, 2.0);
	} else if (msg->direction == OPTFLOW_UPWARD) {
		memcpy(&g_optflow_up, msg, sizeof(optflow_data_t));
		g_optflow.dx = LIMIT(dx_mm, -2.0, 2.0);
		g_optflow.dy = LIMIT(dy_mm, -2.0, 2.0);
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
		g_alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_pos_true.z += g_alt_d;
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
		g_alt_d = g_alt - g_alt_prev;
		g_alt_prev = g_alt;
		g_pos_true.z += g_alt_d;
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

	g_linear_veloc0.x += 1.0 / ACCEL_FREQ * g_linear_accel.x;
	g_linear_veloc0.y += 1.0 / ACCEL_FREQ * g_linear_accel.y;
	g_linear_veloc0.z += 1.0 / ACCEL_FREQ * g_linear_accel.z;

	g_pos_est0.x += POS_XY_EST0_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc0.x;
	g_pos_est0.y += POS_XY_EST0_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc0.y;
	g_pos_est0.z += POS_Z_EST0_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc0.z;
	
	g_pos_est0.x += POS_XY_EST0_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.x - g_pos_est0.x);
	g_pos_est0.y += POS_XY_EST0_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.y - g_pos_est0.y);
	g_pos_est0.z += POS_Z_EST0_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.z - g_pos_est0.z);

	g_linear_veloc1.x = (g_pos_est0.x - g_pos_est0_prev.x) * ACCEL_FREQ;
	g_linear_veloc1.y = (g_pos_est0.y - g_pos_est0_prev.y) * ACCEL_FREQ;
	g_linear_veloc1.z = (g_pos_est0.z - g_pos_est0_prev.z) * ACCEL_FREQ;
	g_pos_est0_prev.x = g_pos_est0.x;
	g_pos_est0_prev.y = g_pos_est0.y;
	g_pos_est0_prev.z = g_pos_est0.z;
	
	g_linear_veloc0.x += VELOC_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_linear_veloc1.x - g_linear_veloc0.x);
	g_linear_veloc0.y += VELOC_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_linear_veloc1.y - g_linear_veloc0.y);
	g_linear_veloc0.z += VELOC_Z_CORRECTION_GAIN / ACCEL_FREQ * (g_linear_veloc1.z - g_linear_veloc0.z);

	g_pos_est1.x += POS_XY_EST1_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc1.x;
	g_pos_est1.y += POS_XY_EST1_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc1.y;
	g_pos_est1.z += POS_Z_EST1_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc1.z;

	g_pos_est1.x += POS_XY_EST1_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.x - g_pos_est1.x);
	g_pos_est1.y += POS_XY_EST1_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.y - g_pos_est1.y);
	g_pos_est1.z += POS_Z_EST1_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.z - g_pos_est1.z);

	vector3d_scale(&g_pos_final, &g_pos_est1, OUTPUT_POS_SCALE);
	g_pos_final.x = -g_pos_final.x;
	g_pos_final.y = -g_pos_final.y;

	vector3d_scale(&g_linear_veloc_final, &g_linear_veloc1, OUTPUT_VELOC_SCALE);
	g_linear_veloc_final.x = -g_linear_veloc_final.x;
	g_linear_veloc_final.y = -g_linear_veloc_final.y;

	static uint8_t g_pos_est_msg[sizeof(vector3d_t) * 2] = {0};
	memcpy(g_pos_est_msg, &g_pos_final, sizeof(vector3d_t));
	memcpy(&g_pos_est_msg[sizeof(vector3d_t)], &g_linear_veloc_final, sizeof(vector3d_t));

	publish(POSITION_STATE_UPDATE, (uint8_t*)&g_pos_est_msg, sizeof(vector3d_t) * 2);
}

#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG
static void loop_logger(uint8_t *data, size_t size) {
	static uint8_t g_msg[12] = {0};
#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG == 1
	float val[3] = {(float)g_pos_est1.x, (float)g_pos_est1.y, (float)g_pos_est1.z};
#elif ENABLE_POSITION_ESTIMATION_MONITOR_LOG == 2
	float val[3] = {(float)g_linear_veloc1.x, (float)g_linear_veloc1.y, (float)g_linear_veloc1.z};
#elif ENABLE_POSITION_ESTIMATION_MONITOR_LOG == 3
	float val[3] = {(float)g_pos_true.z, (float)g_pos_est1.z, (float)g_linear_veloc1.z};
#endif
	memcpy(g_msg, val, 12);
	publish(MONITOR_DATA, (uint8_t*)g_msg, 12);
}
#endif

void position_estimation_setup(void) {
	subscribe(SENSOR_IMU1_GYRO_UPDATE, gyro_update);
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
	subscribe(OSCILLATION_FREQ_DETECTED, oscillation_freq_update);
	subscribe(LINEAR_DRIFT_DETECTION, linear_drift_update);
#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
