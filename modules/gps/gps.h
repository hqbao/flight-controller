#ifndef GPS_H
#define GPS_H

#include <stdint.h>

// UBX message IDs
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

void gps_setup(void);

#endif
