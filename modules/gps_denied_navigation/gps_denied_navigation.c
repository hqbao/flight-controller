#include "gps_denied_navigation.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>

#define WAYPOINT_THRESHOLD 20.0  // cm (local coordinate system)
#define MAX_WAYPOINTS 10

// Waypoints in local coordinate frame (cm)
static vector3d_t g_waypoints[MAX_WAYPOINTS] = {0};
static int g_num_waypoints = 0;
static int g_current_waypoint_index = 0;
static uint8_t g_mission_active = 0;

static vector3d_t g_current_position = {0, 0, 0};
static vector3d_t g_current_velocity = {0, 0, 0};
static vector3d_t g_target_position = {0, 0, 0};

static double calculate_distance(vector3d_t *p1, vector3d_t *p2) {
	double dx = p2->x - p1->x;
	double dy = p2->y - p1->y;
	return sqrt(dx * dx + dy * dy);
}

static void position_state_update(uint8_t *data, size_t size) {
	// Receive current position from position_estimation
	memcpy(&g_current_position, data, sizeof(vector3d_t));
	memcpy(&g_current_velocity, &data[sizeof(vector3d_t)], sizeof(vector3d_t));
}

static void navigation_update(uint8_t *data, size_t size) {
	if (!g_mission_active || g_num_waypoints == 0) {
		// If no mission active, hold current position
		vector3d_set(&g_target_position, &g_current_position);
		publish(POSITION_TARGET_UPDATE, (uint8_t*)&g_target_position, sizeof(vector3d_t));
		return;
	}
	
	// Get current waypoint target
	vector3d_set(&g_target_position, &g_waypoints[g_current_waypoint_index]);
	
	// Check if we've reached current waypoint
	double distance = calculate_distance(&g_current_position, &g_target_position);
	
	if (distance < WAYPOINT_THRESHOLD) {
		// Move to next waypoint
		g_current_waypoint_index++;
		
		if (g_current_waypoint_index >= g_num_waypoints) {
			// Mission complete - return to start
			g_current_waypoint_index = 0;
			g_mission_active = 0;
		}
	}
	
	// Publish target position
	publish(POSITION_TARGET_UPDATE, (uint8_t*)&g_target_position, sizeof(vector3d_t));
}

void gps_denied_navigation_setup(void) {
	// TODO: Load waypoints from storage or receive via external command
	// Example: Create a simple square pattern
	g_num_waypoints = 5;
	
	// Waypoint 0: Origin (home)
	vector3d_init(&g_waypoints[0], 0, 0, 0);
	
	// Waypoint 1: Move forward 100cm
	vector3d_init(&g_waypoints[1], 100, 0, 0);
	
	// Waypoint 2: Move right 100cm
	vector3d_init(&g_waypoints[2], 100, 100, 0);
	
	// Waypoint 3: Move back 100cm
	vector3d_init(&g_waypoints[3], 0, 100, 0);
	
	// Waypoint 4: Return home
	vector3d_init(&g_waypoints[4], 0, 0, 0);
	
	g_current_waypoint_index = 0;
	g_mission_active = 0;
	
	subscribe(POSITION_STATE_UPDATE, position_state_update);
	subscribe(SCHEDULER_10HZ, navigation_update);
}
