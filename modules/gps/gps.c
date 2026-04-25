#include "gps.h"
#include <pubsub.h>
#include <platform.h>
#include <macro.h>
#include <string.h>
#include <stdint.h>
#include <messages.h>

/*
 * GPS module — parses UBX NAV-PVT frames published by db_reader on
 * UBX_MESSAGE_UPDATE (any UART port: data from F9P UART1 or UART2 both flow
 * through the same DB reader and arrive here independent of which STM32 USART
 * is wired up).
 *
 * Outputs:
 *   EXTERNAL_SENSOR_GPS         — gps_position_t  (lat/lon/alt)
 *   EXTERNAL_SENSOR_GPS_VELOC   — gps_velocity_t  (NED mm/s)
 *   EXTERNAL_SENSOR_GPS_QUALITY — gps_quality_t   (fix_type, num_sv,
 *                                  h_acc, v_acc, p_dop, flags, reliable)
 *
 * Logger:
 *   LOG_CLASS_GPS → emits packed gps_log_t (48 bytes) on every NAV-PVT
 *   (typically 10 Hz from a F9P configured by tools/gps_config_f9p.py).
 */

#define UBX_CLASS_NAV   0x01
#define UBX_NAV_PVT     0x07
#define UBX_NAV_PVT_LEN 92

static gps_position_t g_gps_position;
static gps_velocity_t g_gps_velocity;
static gps_quality_t  g_gps_quality;
static gps_log_t      g_gps_log;
static uint8_t        g_active_log_class = 0;

static void parse_nav_pvt(uint8_t *payload, uint16_t length) {
	if (length < UBX_NAV_PVT_LEN) return;

	/* --- quality --- */
	g_gps_quality.fix_type = payload[20];
	g_gps_quality.flags    = payload[21];
	g_gps_quality.num_sv   = payload[23];
	memcpy(&g_gps_quality.h_acc, &payload[40], 4);  /* mm */
	memcpy(&g_gps_quality.v_acc, &payload[44], 4);  /* mm */
	memcpy(&g_gps_quality.p_dop, &payload[76], 2);  /* 0.01 */

	/* gnssFixOK bit must be set, plus 3D fix and reasonable accuracy */
	g_gps_quality.reliable = ((g_gps_quality.flags & 0x01) &&
	                          g_gps_quality.fix_type == 3 &&
	                          g_gps_quality.num_sv >= 6 &&
	                          g_gps_quality.h_acc < 5000) ? 1 : 0;

	/* --- position (deg×1e7, mm above MSL) --- */
	memcpy(&g_gps_position.lon, &payload[24], 4);
	memcpy(&g_gps_position.lat, &payload[28], 4);
	memcpy(&g_gps_position.alt, &payload[36], 4);

	/* --- velocity (NED, mm/s) --- */
	memcpy(&g_gps_velocity.velN, &payload[48], 4);
	memcpy(&g_gps_velocity.velE, &payload[52], 4);
	memcpy(&g_gps_velocity.velD, &payload[56], 4);

	publish(EXTERNAL_SENSOR_GPS_QUALITY, (uint8_t*)&g_gps_quality,  sizeof(gps_quality_t));
	publish(EXTERNAL_SENSOR_GPS,         (uint8_t*)&g_gps_position, sizeof(gps_position_t));
	publish(EXTERNAL_SENSOR_GPS_VELOC,   (uint8_t*)&g_gps_velocity, sizeof(gps_velocity_t));

	/* --- logger (only when LOG_CLASS_GPS active) --- */
	if (g_active_log_class != LOG_CLASS_GPS) return;

	int32_t g_speed_mm_s;       /* ground speed (mm/s) */
	int32_t head_mot_1e5_deg;   /* heading of motion (deg × 1e-5) */
	memcpy(&g_speed_mm_s,     &payload[60], 4);
	memcpy(&head_mot_1e5_deg, &payload[64], 4);

	g_gps_log.lat_deg      = (float)((double)g_gps_position.lat * 1e-7);
	g_gps_log.lon_deg      = (float)((double)g_gps_position.lon * 1e-7);
	g_gps_log.alt_msl_m    = (float)g_gps_position.alt * 0.001f;
	g_gps_log.vel_n_mps    = (float)g_gps_velocity.velN * 0.001f;
	g_gps_log.vel_e_mps    = (float)g_gps_velocity.velE * 0.001f;
	g_gps_log.vel_d_mps    = (float)g_gps_velocity.velD * 0.001f;
	g_gps_log.g_speed_mps  = (float)g_speed_mm_s * 0.001f;
	g_gps_log.head_mot_deg = (float)head_mot_1e5_deg * 1e-5f;
	g_gps_log.h_acc_m      = (float)g_gps_quality.h_acc * 0.001f;
	g_gps_log.v_acc_m      = (float)g_gps_quality.v_acc * 0.001f;
	g_gps_log.p_dop        = g_gps_quality.p_dop;
	g_gps_log.num_sv       = g_gps_quality.num_sv;
	g_gps_log.fix_type     = g_gps_quality.fix_type;
	g_gps_log.flags        = g_gps_quality.flags;
	g_gps_log.reliable     = g_gps_quality.reliable;
	g_gps_log._pad         = 0;

	publish(SEND_LOG, (uint8_t*)&g_gps_log, sizeof(gps_log_t));
}

static void on_ubx_message_received(uint8_t *data, size_t size) {
	/* db_reader hands us [class, id, len_lo, len_hi, payload..., ck_a, ck_b]
	 * (the 0xB5 0x62 sync bytes are stripped). */
	if (size < 6) return;

	uint8_t  msg_class = data[0];
	uint8_t  msg_id    = data[1];
	uint16_t length;
	memcpy(&length, &data[2], 2);

	if (size < (size_t)(4 + length + 2)) return;

	if (msg_class == UBX_CLASS_NAV && msg_id == UBX_NAV_PVT) {
		parse_nav_pvt(&data[4], length);
	}
}

static void on_notify_log_class(uint8_t *data, size_t size) {
	if (size < 1) return;
	g_active_log_class = (data[0] == LOG_CLASS_GPS) ? LOG_CLASS_GPS : 0;
}

void gps_setup(void) {
	memset(&g_gps_position, 0, sizeof(gps_position_t));
	memset(&g_gps_velocity, 0, sizeof(gps_velocity_t));
	memset(&g_gps_quality,  0, sizeof(gps_quality_t));
	memset(&g_gps_log,      0, sizeof(gps_log_t));

	subscribe(UBX_MESSAGE_UPDATE, on_ubx_message_received);
	subscribe(NOTIFY_LOG_CLASS,   on_notify_log_class);
}
