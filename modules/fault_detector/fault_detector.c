#include "fault_detector.h"
#include <pubsub.h>
#include <string.h>
#include <math.h>
#include <messages.h>

/*
 * --- SENSOR FAULT DETECTOR ---
 *
 * Monitors all sensors for health using a sliding window of recent values.
 * The 25 Hz loop proactively pushes the latest value every tick. If a sensor
 * disconnects, the stale value fills the window and triggers stuck detection.
 * A sensor is "stuck" (dead) if all samples in window are identical
 * (max - min < epsilon).
 *
 * Checked at 1 Hz. Publishes sensor_health_t bitmask on SENSOR_HEALTH_UPDATE
 * so that flight_state can gate arming accordingly.
 *
 * Sensors monitored:
 *   - Gyro (SENSOR_IMU1_GYRO_UPDATE)       — 3-axis float, stuck check
 *   - Accel (SENSOR_IMU1_ACCEL_UPDATE)      — 3-axis float, stuck check
 *   - Compass (SENSOR_COMPASS)              — 3-axis float, stuck check
 *   - Air pressure (SENSOR_AIR_PRESSURE)    — 1-axis double, stuck check
 *   - Optflow down (EXTERNAL_SENSOR_OPTFLOW, dir=DOWN) — clarity, stuck check
 *   - Optflow up (EXTERNAL_SENSOR_OPTFLOW, dir=UP)     — clarity, stuck check
 *   - Downward range (EXTERNAL_SENSOR_OPTFLOW, z>0)    — range, stuck check
 *   - GPS (EXTERNAL_SENSOR_GPS_QUALITY)     — num_sv, stuck check
 */

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */

#define WINDOW_SIZE       8    /* samples in ring buffer */
#define STUCK_EPSILON_F   1e-6f
#define STUCK_EPSILON_D   1e-9

/* -------------------------------------------------------------------------
 * Ring buffer for stuck detection (float sensors)
 * ---------------------------------------------------------------------- */

typedef struct {
	float buf[WINDOW_SIZE][3];  /* ring buffer: each entry is 3-axis */
	uint8_t idx;                /* next write position */
	uint8_t count;              /* samples written (saturates at WINDOW_SIZE) */
} sensor_window_3f_t;

static void window_3f_push(sensor_window_3f_t *w, const float v[3]) {
	w->buf[w->idx][0] = v[0];
	w->buf[w->idx][1] = v[1];
	w->buf[w->idx][2] = v[2];
	w->idx = (w->idx + 1) % WINDOW_SIZE;
	if (w->count < WINDOW_SIZE) w->count++;
}

/* Returns 1 if all samples in window are identical (sensor stuck) */
static uint8_t window_3f_is_stuck(const sensor_window_3f_t *w) {
	if (w->count < WINDOW_SIZE) return 0;  /* not enough data yet */
	for (int axis = 0; axis < 3; axis++) {
		float vmin = w->buf[0][axis];
		float vmax = w->buf[0][axis];
		for (uint8_t i = 1; i < WINDOW_SIZE; i++) {
			if (w->buf[i][axis] < vmin) vmin = w->buf[i][axis];
			if (w->buf[i][axis] > vmax) vmax = w->buf[i][axis];
		}
		if ((vmax - vmin) > STUCK_EPSILON_F) return 0;
	}
	return 1;
}

/* -------------------------------------------------------------------------
 * Ring buffer for stuck detection (double, 1-axis — barometer)
 * ---------------------------------------------------------------------- */

typedef struct {
	double buf[WINDOW_SIZE];
	uint8_t idx;
	uint8_t count;
} sensor_window_1d_t;

static void window_1d_push(sensor_window_1d_t *w, double v) {
	w->buf[w->idx] = v;
	w->idx = (w->idx + 1) % WINDOW_SIZE;
	if (w->count < WINDOW_SIZE) w->count++;
}

static uint8_t window_1d_is_stuck(const sensor_window_1d_t *w) {
	if (w->count < WINDOW_SIZE) return 0;
	double vmin = w->buf[0];
	double vmax = w->buf[0];
	for (uint8_t i = 1; i < WINDOW_SIZE; i++) {
		if (w->buf[i] < vmin) vmin = w->buf[i];
		if (w->buf[i] > vmax) vmax = w->buf[i];
	}
	return (vmax - vmin) < STUCK_EPSILON_D;
}

/* -------------------------------------------------------------------------
 * State
 * ---------------------------------------------------------------------- */

/* Ring buffers for high-rate sensors (sampled at 25 Hz into window) */
static sensor_window_3f_t g_gyro_window;
static sensor_window_3f_t g_accel_window;
static sensor_window_3f_t g_compass_window;
static sensor_window_1d_t g_baro_window;
static sensor_window_1d_t g_optflow_down_window; /* clarity */
static sensor_window_1d_t g_optflow_up_window;   /* clarity */
static sensor_window_1d_t g_downward_range_window; /* range mm */
static sensor_window_1d_t g_gps_window;            /* num_sv */

/* Latest sensor values (written by callbacks, pushed at 25 Hz) */
static float g_latest_gyro[3] = {0};
static float g_latest_accel[3] = {0};
static float g_latest_compass[3] = {0};
static double g_latest_baro = 0;
static double g_latest_optflow_down_clarity = 0;
static double g_latest_optflow_up_clarity = 0;
static double g_latest_downward_range = 0;
static double g_latest_gps_num_sv = 0;


/* Published health state */
static sensor_health_t g_health = {0};

/* -------------------------------------------------------------------------
 * Sensor data callbacks (store latest value)
 * ---------------------------------------------------------------------- */

static void on_gyro_update(uint8_t *data, size_t size) {
	if (size < 12) return;
	memcpy(g_latest_gyro, data, 12);
}

static void on_accel_update(uint8_t *data, size_t size) {
	if (size < 12) return;
	memcpy(g_latest_accel, data, 12);
}

static void on_compass_update(uint8_t *data, size_t size) {
	if (size < sizeof(vector3d_t)) return;
	vector3d_t v;
	memcpy(&v, data, sizeof(vector3d_t));
	g_latest_compass[0] = (float)v.x;
	g_latest_compass[1] = (float)v.y;
	g_latest_compass[2] = (float)v.z;
}

static void on_baro_update(uint8_t *data, size_t size) {
	if (size < sizeof(double)) return;
	memcpy(&g_latest_baro, data, sizeof(double));
}

static void on_optflow_update(uint8_t *data, size_t size) {
	if (size < sizeof(optflow_data_t)) return;
	optflow_data_t msg;
	memcpy(&msg, data, sizeof(optflow_data_t));
	if (msg.direction == OPTFLOW_DOWNWARD) {
		g_latest_optflow_down_clarity = msg.clarity;
		if (msg.z > 0) g_latest_downward_range = msg.z;
	} else {
		g_latest_optflow_up_clarity = msg.clarity;
	}
}

static void on_gps_quality_update(uint8_t *data, size_t size) {
	if (size < sizeof(gps_quality_t)) return;
	gps_quality_t q;
	memcpy(&q, data, sizeof(gps_quality_t));
	g_latest_gps_num_sv = (double)q.num_sv;
}

/* -------------------------------------------------------------------------
 * 25 Hz: sample latest sensor values into ring buffers
 * ---------------------------------------------------------------------- */

static void loop_25hz(uint8_t *data, size_t size) {
	window_3f_push(&g_gyro_window, g_latest_gyro);
	window_3f_push(&g_accel_window, g_latest_accel);
	window_3f_push(&g_compass_window, g_latest_compass);
	window_1d_push(&g_baro_window, g_latest_baro);
	window_1d_push(&g_optflow_down_window, g_latest_optflow_down_clarity);
	window_1d_push(&g_optflow_up_window, g_latest_optflow_up_clarity);
	window_1d_push(&g_downward_range_window, g_latest_downward_range);
	window_1d_push(&g_gps_window, g_latest_gps_num_sv);
}

/* -------------------------------------------------------------------------
 * 1 Hz: evaluate health and publish
 * ---------------------------------------------------------------------- */

static void loop_1hz(uint8_t *data, size_t size) {
	sensor_health_t h = {0};

	/* All sensors: healthy = window shows variation (not stuck) */
	h.gyro           = !window_3f_is_stuck(&g_gyro_window);
	h.accel          = !window_3f_is_stuck(&g_accel_window);
	h.compass        = !window_3f_is_stuck(&g_compass_window);
	h.baro           = !window_1d_is_stuck(&g_baro_window);
	h.optflow_down   = !window_1d_is_stuck(&g_optflow_down_window);
	h.optflow_up     = !window_1d_is_stuck(&g_optflow_up_window);
	h.downward_range = !window_1d_is_stuck(&g_downward_range_window);
	h.gps            = !window_1d_is_stuck(&g_gps_window);

	g_health = h;
	publish(SENSOR_HEALTH_UPDATE, (uint8_t *)&h, sizeof(sensor_health_t));
}

/* -------------------------------------------------------------------------
 * Request handler: any module can ask for current health state
 * ---------------------------------------------------------------------- */

static void on_health_request(uint8_t *data, size_t size) {
	publish(SENSOR_HEALTH_UPDATE, (uint8_t *)&g_health, sizeof(sensor_health_t));
}

/* -------------------------------------------------------------------------
 * Setup
 * ---------------------------------------------------------------------- */

void fault_detector_setup(void) {
	memset(&g_gyro_window, 0, sizeof(g_gyro_window));
	memset(&g_accel_window, 0, sizeof(g_accel_window));
	memset(&g_compass_window, 0, sizeof(g_compass_window));
	memset(&g_baro_window, 0, sizeof(g_baro_window));
	memset(&g_optflow_down_window, 0, sizeof(g_optflow_down_window));
	memset(&g_optflow_up_window, 0, sizeof(g_optflow_up_window));
	memset(&g_downward_range_window, 0, sizeof(g_downward_range_window));
	memset(&g_gps_window, 0, sizeof(g_gps_window));

	/* On-board sensor data */
	subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
	subscribe(SENSOR_IMU1_ACCEL_UPDATE, on_accel_update);
	subscribe(SENSOR_COMPASS, on_compass_update);
	subscribe(SENSOR_AIR_PRESSURE, on_baro_update);

	/* External sensor data */
	subscribe(EXTERNAL_SENSOR_OPTFLOW, on_optflow_update);
	subscribe(EXTERNAL_SENSOR_GPS_QUALITY, on_gps_quality_update);

	/* Periodic checks */
	subscribe(SCHEDULER_25HZ, loop_25hz);
	subscribe(SCHEDULER_1HZ, loop_1hz);

	/* Re-publish on request */
	subscribe(SENSOR_HEALTH_REQUEST, on_health_request);
}
