#include "gps_navigation.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <math.h>
#include <vector3d.h>
#include <messages.h>

#define WAYPOINT_THRESHOLD 5.0  // meters
#define MAX_WAYPOINTS 10

typedef struct {
	double lat;  // latitude in degrees
	double lon;  // longitude in degrees
	double alt;  // altitude in meters
} gps_waypoint_t;

static gps_waypoint_t g_waypoints[MAX_WAYPOINTS] = {0};
static int g_num_waypoints = 0;
static int g_current_waypoint_index = 0;
static uint8_t g_mission_active = 0;
static gps_quality_t g_gps_quality = {0};

static gps_position_t g_gps_position = {0};

static vector3d_t g_current_position = {0, 0, 0};
static vector3d_t g_target_position = {0, 0, 0};

// Convert GPS coordinates to local NED (North-East-Down) frame
static void gps_to_local(double lat, double lon, double alt, vector3d_t *local) {
	// Earth radius in meters
	const double R = 6371000.0;
	
	// Convert to radians
	double lat_rad = lat * M_PI / 180.0;
	double lon_rad = lon * M_PI / 180.0;
	double home_lat_rad = g_waypoints[0].lat * M_PI / 180.0;
	double home_lon_rad = g_waypoints[0].lon * M_PI / 180.0;
	
	// Calculate local position (simplified flat-earth approximation)
	local->x = R * (lat_rad - home_lat_rad);  // North
	local->y = R * (lon_rad - home_lon_rad) * cos(home_lat_rad);  // East
	local->z = alt - g_waypoints[0].alt;  // Down (negative up)
}

static double calculate_distance(vector3d_t *p1, vector3d_t *p2) {
	double dx = p2->x - p1->x;
	double dy = p2->y - p1->y;
	return sqrt(dx * dx + dy * dy);
}

static void gps_update(uint8_t *data, size_t size) {
	memcpy(&g_gps_position, data, sizeof(gps_position_t));
	
	// Convert GPS position to local coordinates
	double lat = (double)g_gps_position.lat / 1e7;
	double lon = (double)g_gps_position.lon / 1e7;
	double alt = (double)g_gps_position.alt / 1000.0;
	
	gps_to_local(lat, lon, alt, &g_current_position);
}

static void gps_quality_update(uint8_t *data, size_t size) {
	if (size >= sizeof(gps_quality_t)) {
		memcpy(&g_gps_quality, data, sizeof(gps_quality_t));
		
		// If GPS becomes unreliable during mission, abort
		if (!g_gps_quality.reliable && g_mission_active) {
			g_mission_active = 0;
			g_current_waypoint_index = 0;
		}
	}
}

static void navigation_update(uint8_t *data, size_t size) {
	// Check GPS quality and mission state
	if (!g_mission_active || g_num_waypoints == 0 || !g_gps_quality.reliable) {
		return;
	}
	
	// Convert current waypoint to local coordinates
	gps_to_local(
		g_waypoints[g_current_waypoint_index].lat,
		g_waypoints[g_current_waypoint_index].lon,
		g_waypoints[g_current_waypoint_index].alt,
		&g_target_position
	);
	
	// Check if we've reached current waypoint
	double distance = calculate_distance(&g_current_position, &g_target_position);
	
	if (distance < WAYPOINT_THRESHOLD) {
		// Move to next waypoint
		g_current_waypoint_index++;
		
		if (g_current_waypoint_index >= g_num_waypoints) {
			// Mission complete - return to home
			g_current_waypoint_index = 0;
			g_mission_active = 0;
		}
	}
	
	// Publish target position
	publish(POSITION_TARGET_UPDATE, (uint8_t*)&g_target_position, sizeof(vector3d_t));
}

void gps_navigation_setup(void) {
	// TODO: Load waypoints from storage or receive via external command
	// For now, initialize with example waypoints
	g_num_waypoints = 0;
	g_current_waypoint_index = 0;
	g_mission_active = 0;
	
	subscribe(EXTERNAL_SENSOR_GPS, gps_update);
	subscribe(EXTERNAL_SENSOR_GPS_QUALITY, gps_quality_update);
	subscribe(SCHEDULER_10HZ, navigation_update);
}
