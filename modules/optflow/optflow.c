#include "optflow.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <vector3d.h>
#include <messages.h>

static optflow_data_t g_optflow_msg;

/* Inter-arrival timing per direction. The wire format does NOT carry dt,
 * and the previous hardcode of 40 ms (25 Hz) was wrong whenever the
 * actual camera/dblink rate differed — the divisor in optflow_to_rad_per_s
 * scales velocity linearly, so an under-stated dt amplifies the reported
 * v_body by (true_dt / hardcoded_dt). We instead time-stamp messages on
 * arrival and report the true gap. First sample of each direction has no
 * prior reference and is published with dt_us=0 (state_estimation.c
 * already drops samples with dt_us==0). */
static uint32_t g_last_us[2] = { 0, 0 };

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
	if (data[0] == 0x01) { // Optical flow
		if (size < 22) return;

        // Verify checksum
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 0; i < size - 2; i++) {
            ck_a = ck_a + data[i];
            ck_b = ck_b + ck_a;
        }
        if (ck_a != data[size - 2] || ck_b != data[size - 1]) {
            return;
        }

		int32_t raw_dx, raw_dy, raw_z, raw_clarity;
		memcpy(&raw_dx, &data[4], sizeof(int32_t));
		memcpy(&raw_dy, &data[8], sizeof(int32_t));
		memcpy(&raw_z, &data[12], sizeof(int32_t));
		memcpy(&raw_clarity, &data[16], sizeof(int32_t));
		
		// Convert from scaled int (×100000) back to radians
		float dx_rad = (float)raw_dx / 100000.0f;
		float dy_rad = (float)raw_dy / 100000.0f;
		float clarity = (float)raw_clarity / 10.0f;

		optflow_direction_t dir = (optflow_direction_t)data[1];
		int dir_idx = (dir == OPTFLOW_UPWARD) ? 1 : 0;

		/* Measure true inter-arrival from previous frame in this direction.
		 *
		 * Caveat: this is the UART-arrival gap on the FC, not the camera's
		 * frame-capture interval. If two frames arrive back-to-back because
		 * the ESP32 batched them or the UART RX FIFO drained in one go, the
		 * gap collapses to a few hundred μs. Velocity is range × dx_rad /
		 * dt, so a 100× under-stated dt produces a 100× velocity spike.
		 *
		 * The camera nominally runs ~5..30 Hz (33..200 ms). Anything outside
		 * a generous [10 ms, 500 ms] band is treated as untrustworthy and
		 * we publish dt_us=0 so state_estimation drops the frame instead of
		 * fusing a huge garbage velocity. The ZUPT branch in
		 * state_estimation also requires dt_us != 0 (via the guard at the
		 * top of on_optflow), so a gated frame neither fuses nor ZUPTs. */
		uint32_t now_us = (uint32_t)platform_time_us();
		uint32_t dt_us = 0;
		if (g_last_us[dir_idx] != 0) {
			uint32_t gap = now_us - g_last_us[dir_idx];   /* wraps at 32-bit, OK */
			if (gap >= 10000u && gap <= 500000u) {
				dt_us = gap;
			}
		}
		g_last_us[dir_idx] = now_us;

		g_optflow_msg.dx = dx_rad;
		g_optflow_msg.dy = dy_rad;
		g_optflow_msg.z = (double)raw_z;
		g_optflow_msg.clarity = (double)clarity;
		g_optflow_msg.dt_us = dt_us;
		g_optflow_msg.direction = dir;

		publish(EXTERNAL_SENSOR_OPTFLOW, (uint8_t*)&g_optflow_msg, sizeof(optflow_data_t));
	}
}

void optflow_setup(void) {
	subscribe(DB_MESSAGE_UPDATE, on_message_received);
}
