#include "gps.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <string.h>
#include <stdint.h>

// UBX message IDs
#define UBX_CLASS_NAV 0x01
#define UBX_NAV_PVT 0x07

// GPS position data structure (from NAV-PVT)
typedef struct {
	int32_t lon;        // Longitude (deg * 1e7)
	int32_t lat;        // Latitude (deg * 1e7)
	int32_t alt;        // Height above mean sea level (mm)
} gps_position_t;

// GPS velocity data structure (from NAV-PVT)
typedef struct {
	int32_t velN;       // North velocity (mm/s)
	int32_t velE;       // East velocity (mm/s)
	int32_t velD;       // Down velocity (mm/s)
} gps_velocity_t;

// GPS quality data structure
typedef struct {
	uint8_t fix_type;   // 0=no fix, 2=2D, 3=3D
	uint8_t num_sv;     // Number of satellites
	uint32_t h_acc;     // Horizontal accuracy (mm)
	uint8_t reliable;   // 1=reliable, 0=not reliable
} gps_quality_t;

static gps_position_t g_gps_position;
static gps_velocity_t g_gps_velocity;
static gps_quality_t g_gps_quality = {0};

static void parse_nav_pvt(uint8_t *payload, uint16_t length) {
	// Parse NAV-PVT payload (92 bytes minimum)
	if (length < 92) return;
	
	// Extract GPS quality indicators
	g_gps_quality.num_sv = payload[11];     // Number of satellites
	g_gps_quality.fix_type = payload[21];   // Fix type
	memcpy(&g_gps_quality.h_acc, &payload[40], 4);  // Horizontal accuracy (mm)
	
	// Determine GPS reliability
	// Reliable when: 3D fix, >= 6 satellites, horizontal accuracy < 5m
	g_gps_quality.reliable = (g_gps_quality.fix_type == 3 && 
	                          g_gps_quality.num_sv >= 6 && 
	                          g_gps_quality.h_acc < 5000) ? 1 : 0;
	
	// Extract position (bytes 24-39)
	memcpy(&g_gps_position.lon, &payload[24], 4);  // Longitude (deg * 1e7)
	memcpy(&g_gps_position.lat, &payload[28], 4);  // Latitude (deg * 1e7)
	memcpy(&g_gps_position.alt, &payload[36], 4);  // Height MSL (mm)
	
	// Extract velocity (bytes 48-59)
	memcpy(&g_gps_velocity.velN, &payload[48], 4); // North velocity (mm/s)
	memcpy(&g_gps_velocity.velE, &payload[52], 4); // East velocity (mm/s)
	memcpy(&g_gps_velocity.velD, &payload[56], 4); // Down velocity (mm/s)
	
	// Publish GPS quality (reliability status)
	publish(EXTERNAL_SENSOR_GPS_QUALITY, (uint8_t*)&g_gps_quality, sizeof(gps_quality_t));
	
	// Publish GPS position data
	publish(EXTERNAL_SENSOR_GPS, (uint8_t*)&g_gps_position, sizeof(gps_position_t));
	
	// Publish GPS velocity data
	publish(EXTERNAL_SENSOR_GPS_VELOC, (uint8_t*)&g_gps_velocity, sizeof(gps_velocity_t));
}

static void on_ubx_message_received(uint8_t *data, size_t size) {
	// UBX message format: class | id | length (2 bytes) | payload | checksum (2 bytes)
	if (size < 4) return;
	
	uint8_t msg_class = data[0];
	uint8_t msg_id = data[1];
	uint16_t length = (data[3] << 8) | data[2];  // Little-endian
	
	// Verify we have enough data
	if (size < (uint32_t)(4 + length + 2)) return;
	
	uint8_t *payload = &data[4];
	
	// Parse NAV class messages
	if (msg_class == UBX_CLASS_NAV) {
		if (msg_id == UBX_NAV_PVT) {
			parse_nav_pvt(payload, length);
		}
	}
}

void gps_setup(void) {
	memset(&g_gps_position, 0, sizeof(gps_position_t));
	memset(&g_gps_velocity, 0, sizeof(gps_velocity_t));
	
	// Subscribe to UBX messages from UART
	subscribe(UBX_MESSAGE_UPDATE, on_ubx_message_received);
	
	// Publish module initialized status
	module_initialized_t initialized = {.id = MODULE_ID_GPS, .initialized = 1};
	publish(MODULE_INITIALIZED_UPDATE, (uint8_t*)&initialized, sizeof(module_initialized_t));
}

