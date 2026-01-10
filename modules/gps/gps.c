#include "gps.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <stdint.h>

// UBX message IDs
#define UBX_CLASS_NAV 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_VELNED 0x12

// GPS position data structure (from NAV-POSLLH)
typedef struct {
	uint32_t iTOW;      // GPS time of week (ms)
	int32_t lon;        // Longitude (deg * 1e7)
	int32_t lat;        // Latitude (deg * 1e7)
	int32_t height;     // Height above ellipsoid (mm)
	int32_t hMSL;       // Height above mean sea level (mm)
	uint32_t hAcc;      // Horizontal accuracy estimate (mm)
	uint32_t vAcc;      // Vertical accuracy estimate (mm)
} gps_position_t;

// GPS velocity data structure (from NAV-VELNED)
typedef struct {
	uint32_t iTOW;      // GPS time of week (ms)
	int32_t velN;       // North velocity (cm/s)
	int32_t velE;       // East velocity (cm/s)
	int32_t velD;       // Down velocity (cm/s)
	uint32_t speed;     // 3D speed (cm/s)
	uint32_t gSpeed;    // Ground speed (cm/s)
	int32_t heading;    // Heading of motion (deg * 1e5)
	uint32_t sAcc;      // Speed accuracy estimate (cm/s)
	uint32_t cAcc;      // Course/heading accuracy (deg * 1e5)
} gps_velocity_t;

static gps_position_t g_gps_position;
static gps_velocity_t g_gps_velocity;

static void parse_nav_posllh(uint8_t *payload) {
	// Parse NAV-POSLLH payload (28 bytes)
	memcpy(&g_gps_position.iTOW, &payload[0], 4);
	memcpy(&g_gps_position.lon, &payload[4], 4);
	memcpy(&g_gps_position.lat, &payload[8], 4);
	memcpy(&g_gps_position.height, &payload[12], 4);
	memcpy(&g_gps_position.hMSL, &payload[16], 4);
	memcpy(&g_gps_position.hAcc, &payload[20], 4);
	memcpy(&g_gps_position.vAcc, &payload[24], 4);
	
	// Publish GPS position data
	publish(EXTERNAL_SENSOR_GPS, (uint8_t*)&g_gps_position, sizeof(gps_position_t));
}

static void parse_nav_velned(uint8_t *payload) {
	// Parse NAV-VELNED payload (36 bytes)
	memcpy(&g_gps_velocity.iTOW, &payload[0], 4);
	memcpy(&g_gps_velocity.velN, &payload[4], 4);
	memcpy(&g_gps_velocity.velE, &payload[8], 4);
	memcpy(&g_gps_velocity.velD, &payload[12], 4);
	memcpy(&g_gps_velocity.speed, &payload[16], 4);
	memcpy(&g_gps_velocity.gSpeed, &payload[20], 4);
	memcpy(&g_gps_velocity.heading, &payload[24], 4);
	memcpy(&g_gps_velocity.sAcc, &payload[28], 4);
	memcpy(&g_gps_velocity.cAcc, &payload[32], 4);
	
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
		switch (msg_id) {
		case UBX_NAV_POSLLH:
			if (length == 28) {
				parse_nav_posllh(payload);
			}
			break;
		case UBX_NAV_VELNED:
			if (length == 36) {
				parse_nav_velned(payload);
			}
			break;
		default:
			break;
		}
	}
}

void gps_setup(void) {
	memset(&g_gps_position, 0, sizeof(gps_position_t));
	memset(&g_gps_velocity, 0, sizeof(gps_velocity_t));
	
	// Subscribe to UBX messages from UART
	subscribe(UBX_MESSAGE_UPDATE, on_ubx_message_received);
}

