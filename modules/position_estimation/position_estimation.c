#include "position_estimation.h"
#include <pubsub.h>
#include <vector3d.h>
#include <string.h>
#include <math.h>
#include <macro.h>

/* Macro to enable/disable sending MONITOR_DATA via logger 
 * 0: Disable
 * 1: Position
 * 2: Velocity
 */
#define ENABLE_POSITION_ESTIMATION_MONITOR_LOG 0

#define ACCEL_FREQ 500
#define MAX_IMU_ACCEL 16384

typedef enum {
	OPTFLOW_DOWNWARD = 0,
	OPTFLOW_UPWARD = 1,
} optflow_direction_t;

typedef struct {
    double dx;
    double dy;
    double z;
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
static vector3d_t g_linear_veloc = {0, 0, 0};
static vector3d_t g_linear_veloc_est = {0, 0, 0};
static vector3d_t g_linear_veloc_final = {0, 0, 0};
static vector3d_t g_pos_est2 = {0, 0, 0};
static vector3d_t g_pos_est2_prev = {0, 0, 0};
static vector3d_t g_pos_true = {0, 0, 0};
static vector3d_t g_pos_est1 = {0, 0, 0};
static vector3d_t g_pos_final = {0, 0, 0};

/* Tuning Parameters */
#define POS_XY_CORRECTION_GAIN      20.0
#define POS_Z_CORRECTION_GAIN       20.0
#define POS_XY_INTEGRATION_GAIN     0.05
#define POS_Z_INTEGRATION_GAIN      1.0
#define POS_XY_TRUE_CORRECTION_GAIN 1.0
#define POS_Z_TRUE_CORRECTION_GAIN  1.0
#define POS_XY_EST1_INTEGRATION_GAIN 0.0
#define POS_Z_EST1_INTEGRATION_GAIN  1.0
#define ALT_DERIVATIVE_SCALE    100.0
#define BARO_ALPHA_HIGH_ACCEL   0.05
#define BARO_ALPHA_LOW_ACCEL    0.005
#define OUTPUT_POS_SCALE        1.0
#define OUTPUT_VELOC_SCALE      1.0
#define ACCEL_Z_THRESHOLD       300.0
#define RANGE_SWITCH_TO_LASER_THRESHOLD 500.0
#define RANGE_SWITCH_TO_BARO_THRESHOLD 1000.0
#define VELOC_XY_CORRECTION_GAIN 0.01
#define VELOC_Z_CORRECTION_GAIN 0.01

typedef enum {
	ALT_SOURCE_LASER = 0,
	ALT_SOURCE_BARO = 1,
} alt_source_t;

static alt_source_t g_alt_source = ALT_SOURCE_LASER;

static void optflow_sensor_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t *msg = (optflow_data_t*)data;

	if (msg->direction == OPTFLOW_DOWNWARD) {
		g_optflow.dx = msg->dx;
		g_optflow.dy = msg->dy;
	} else if (msg->direction == OPTFLOW_UPWARD) {
		g_optflow.dx = msg->dx;
		g_optflow.dy = -msg->dy;
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

static void linear_accel_update(uint8_t *data, size_t size) {
	vector3d_t v = *(vector3d_t*)data;
	g_linear_accel.x = -v.y * MAX_IMU_ACCEL;
	g_linear_accel.y = -v.x * MAX_IMU_ACCEL;
	g_linear_accel.z = v.z * MAX_IMU_ACCEL;

	g_linear_veloc.x += 1.0 / ACCEL_FREQ * g_linear_accel.x;
	g_linear_veloc.y += 1.0 / ACCEL_FREQ * g_linear_accel.y;
	g_linear_veloc.z += 1.0 / ACCEL_FREQ * g_linear_accel.z;

	g_linear_veloc.x += VELOC_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_optflow.dx - g_linear_veloc.x);
	g_linear_veloc.y += VELOC_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_optflow.dy - g_linear_veloc.y);
	g_linear_veloc.z += VELOC_Z_CORRECTION_GAIN / ACCEL_FREQ * (g_alt_d - g_linear_veloc.z);

	g_pos_est2.x += POS_XY_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc.x;
	g_pos_est2.y += POS_XY_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc.y;
	g_pos_est2.z += POS_Z_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc.z;
	
	g_pos_est2.x += POS_XY_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.x - g_pos_est2.x);
	g_pos_est2.y += POS_XY_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.y - g_pos_est2.y);
	g_pos_est2.z += POS_Z_TRUE_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.z - g_pos_est2.z);

	g_linear_veloc_est.x = (g_pos_est2.x - g_pos_est2_prev.x) * ACCEL_FREQ;
	g_linear_veloc_est.y = (g_pos_est2.y - g_pos_est2_prev.y) * ACCEL_FREQ;
	g_linear_veloc_est.z = (g_pos_est2.z - g_pos_est2_prev.z) * ACCEL_FREQ;
	g_pos_est2_prev.x = g_pos_est2.x;
	g_pos_est2_prev.y = g_pos_est2.y;
	g_pos_est2_prev.z = g_pos_est2.z;

	g_pos_est1.x += POS_XY_EST1_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc_est.x;
	g_pos_est1.y += POS_XY_EST1_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc_est.y;
	g_pos_est1.z += POS_Z_EST1_INTEGRATION_GAIN / ACCEL_FREQ * g_linear_veloc_est.z;

	g_pos_est1.x += POS_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.x - g_pos_est1.x);
	g_pos_est1.y += POS_XY_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.y - g_pos_est1.y);
	g_pos_est1.z += POS_Z_CORRECTION_GAIN / ACCEL_FREQ * (g_pos_true.z - g_pos_est1.z);

	vector3d_scale(&g_pos_final, &g_pos_est1, OUTPUT_POS_SCALE);
	g_pos_final.x = -g_pos_final.x;
	g_pos_final.y = -g_pos_final.y;

	vector3d_scale(&g_linear_veloc_final, &g_linear_veloc_est, OUTPUT_VELOC_SCALE);
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
	float val[3] = {(float)g_linear_veloc_est.x, (float)g_linear_veloc_est.y, (float)g_linear_veloc_est.z};
#endif
	memcpy(g_msg, val, 12);
	publish(MONITOR_DATA, (uint8_t*)g_msg, 12);
}
#endif

void position_estimation_setup(void) {
	subscribe(SENSOR_LINEAR_ACCEL, linear_accel_update);
	subscribe(SENSOR_AIR_PRESSURE, air_pressure_update);
	subscribe(EXTERNAL_SENSOR_OPTFLOW, optflow_sensor_update);
#if ENABLE_POSITION_ESTIMATION_MONITOR_LOG
	subscribe(SCHEDULER_25HZ, loop_logger);
#endif
}
