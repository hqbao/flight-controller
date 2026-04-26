#include "flight_telemetry.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <messages.h>

/*
 * Flight Telemetry Module
 *
 * Centralizes key flight data into a single 66-byte frame
 * sent via SEND_LOG at 10 Hz.
 *
 * Frame layout (66 bytes):
 *   Offset  Size  Field
 *   0       12    Attitude: roll, pitch, yaw (3 × float, degrees)
 *   12      12    Position: x, y, z (3 × float, meters)
 *   24      12    Velocity: vx, vy, vz (3 × float, m/s)
 *   36      16    Motors: m1–m8 (8 × int16_t, speed units)
 *   52      12    PID outputs: roll, pitch, yaw (3 × float)
 *   64      1     Flight state (uint8_t)
 *   65      1     Sensor health bitmask (uint8_t)
 *
 * Health bitmask bits:
 *   0=gyro, 1=accel, 2=compass, 3=baro,
 *   4=downward_range, 5=optflow_down, 6=optflow_up, 7=gps
 *
 * Bandwidth: 66 + 8 overhead = 74 bytes/frame × 10 Hz = 740 bytes/s
 * (within 3840 bytes/s at 38400 baud)
 */

#define TELEMETRY_FRAME_SIZE 66

// --- Cached state from subscriptions ---
static float g_att_roll, g_att_pitch, g_att_yaw;
static float g_pos_x, g_pos_y, g_pos_z;
static float g_vel_x, g_vel_y, g_vel_z;
static int g_motors[8];
static float g_pid_roll, g_pid_pitch, g_pid_yaw;
static uint8_t g_state;
static sensor_health_t g_health;
static uint8_t g_log_class;

// --- Callbacks ---

static void on_angular_state(uint8_t *data, size_t size) {
	if (size < sizeof(angle3d_t)) return;
	angle3d_t *a = (angle3d_t *)data;
	g_att_roll  = (float)a->roll;
	g_att_pitch = (float)a->pitch;
	g_att_yaw   = (float)a->yaw;
}

static void on_position_state(uint8_t *data, size_t size) {
	if (size < sizeof(position_state_t)) return;
	position_state_t *p = (position_state_t *)data;
	g_pos_x = (float)p->position.x;
	g_pos_y = (float)p->position.y;
	g_pos_z = (float)p->position.z;
	g_vel_x = (float)p->velocity.x;
	g_vel_y = (float)p->velocity.y;
	g_vel_z = (float)p->velocity.z;
}

static void on_speed_control(uint8_t *data, size_t size) {
	if (size < 8 * sizeof(int)) return;
	memcpy(g_motors, data, 8 * sizeof(int));
}

static void on_mix_control(uint8_t *data, size_t size) {
	if (size < sizeof(mix_control_input_t)) return;
	mix_control_input_t *m = (mix_control_input_t *)data;
	g_pid_roll  = (float)m->roll;
	g_pid_pitch = (float)m->pitch;
	g_pid_yaw   = (float)m->yaw;
}

static void on_flight_state(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_state = data[0];
}

static void on_sensor_health(uint8_t *data, size_t size) {
	if (size < sizeof(sensor_health_t)) return;
	memcpy(&g_health, data, sizeof(sensor_health_t));
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_log_class = data[0];
}

static void loop_10hz(uint8_t *data, size_t size) {
	if (g_log_class != LOG_CLASS_FLIGHT_TELEMETRY) return;

	uint8_t buf[TELEMETRY_FRAME_SIZE];
	size_t offset = 0;

	// Attitude (12 bytes)
	memcpy(buf + offset, &g_att_roll, 4);  offset += 4;
	memcpy(buf + offset, &g_att_pitch, 4); offset += 4;
	memcpy(buf + offset, &g_att_yaw, 4);   offset += 4;

	// Position (12 bytes)
	memcpy(buf + offset, &g_pos_x, 4); offset += 4;
	memcpy(buf + offset, &g_pos_y, 4); offset += 4;
	memcpy(buf + offset, &g_pos_z, 4); offset += 4;

	// Velocity (12 bytes)
	memcpy(buf + offset, &g_vel_x, 4); offset += 4;
	memcpy(buf + offset, &g_vel_y, 4); offset += 4;
	memcpy(buf + offset, &g_vel_z, 4); offset += 4;

	// Motors (16 bytes — 8 × int16_t)
	for (int i = 0; i < 8; i++) {
		int16_t m = (int16_t)g_motors[i];
		memcpy(buf + offset, &m, 2);
		offset += 2;
	}

	// PID outputs (12 bytes)
	memcpy(buf + offset, &g_pid_roll, 4);  offset += 4;
	memcpy(buf + offset, &g_pid_pitch, 4); offset += 4;
	memcpy(buf + offset, &g_pid_yaw, 4);   offset += 4;

	// Flight state (1 byte)
	buf[offset++] = g_state;

	// Sensor health bitmask (1 byte)
	uint8_t h = 0;
	h |= (g_health.gyro          ? 1 : 0) << 0;
	h |= (g_health.accel         ? 1 : 0) << 1;
	h |= (g_health.compass       ? 1 : 0) << 2;
	h |= (g_health.baro          ? 1 : 0) << 3;
	h |= (g_health.downward_range ? 1 : 0) << 4;
	h |= (g_health.optflow_down  ? 1 : 0) << 5;
	h |= (g_health.optflow_up    ? 1 : 0) << 6;
	h |= (g_health.gps           ? 1 : 0) << 7;
	buf[offset++] = h;

	publish(SEND_LOG, buf, TELEMETRY_FRAME_SIZE);
}

void flight_telemetry_setup(void) {
	subscribe(ANGULAR_STATE_UPDATE, on_angular_state);
	subscribe(POSITION_STATE_UPDATE, on_position_state);
	subscribe(SPEED_CONTROL_UPDATE, on_speed_control);
	subscribe(MIX_CONTROL_UPDATE, on_mix_control);
	subscribe(FLIGHT_STATE_UPDATE, on_flight_state);
	subscribe(SENSOR_HEALTH_UPDATE, on_sensor_health);
	subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
	subscribe(SCHEDULER_10HZ, loop_10hz);
}
