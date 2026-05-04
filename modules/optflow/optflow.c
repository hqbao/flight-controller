#include "optflow.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>
#include <macro.h>
#include <vector3d.h>
#include <messages.h>

static optflow_data_t g_optflow_msg;

/* Per-direction record of the most recent camera ms-counter, used to derive
 * the true frame interval. The wire now carries a uint32 free-running ms
 * counter sampled by the camera at packing time, so the FC just diffs
 * consecutive values (32-bit unsigned subtraction wraps cleanly every
 * ~49.7 days). The previous arrival-gap heuristic was unreliable: UART
 * batching on the ESP32 or RX FIFO drain bursts on the FC could collapse
 * the measured gap to a few hundred μs, producing a 100× velocity spike
 * that the state_estimation outlier gate would silently drop — exactly the
 * "sparse green v_meas" symptom in optflow_view.
 *
 * g_have_last[dir] is 0 until the first counter has been recorded; the
 * first sample of each direction is published with dt_us=0 so
 * state_estimation skips it (no prior reference to diff against). */
static uint32_t g_last_t_ms[2] = { 0, 0 };
static uint8_t  g_have_last[2] = { 0, 0 };

static void on_message_received(uint8_t *data, size_t size) {
	//platform_toggle_led(0);
	if (data[0] == 0x01) { // Optical flow
		if (size < 26) return;

        // Verify checksum
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 0; i < size - 2; i++) {
            ck_a = ck_a + data[i];
            ck_b = ck_b + ck_a;
        }
        if (ck_a != data[size - 2] || ck_b != data[size - 1]) {
            return;
        }

		int32_t  raw_dx, raw_dy, raw_z, raw_clarity;
		uint32_t raw_t_ms;
		memcpy(&raw_dx,      &data[4],  sizeof(int32_t));
		memcpy(&raw_dy,      &data[8],  sizeof(int32_t));
		memcpy(&raw_z,       &data[12], sizeof(int32_t));
		memcpy(&raw_clarity, &data[16], sizeof(int32_t));
		memcpy(&raw_t_ms,    &data[20], sizeof(uint32_t));

		// Convert from scaled int (×100000) back to radians
		float dx_rad = (float)raw_dx / 100000.0f;
		float dy_rad = (float)raw_dy / 100000.0f;
		float clarity = (float)raw_clarity / 10.0f;

		optflow_direction_t dir = (optflow_direction_t)data[1];
		int dir_idx = (dir == OPTFLOW_UPWARD) ? 1 : 0;

		/* Derive the true frame interval from the camera-stamped ms counter.
		 *
		 * Sanity band [10 ms, 500 ms] catches the same failure modes as the
		 * old arrival-gap path (frozen counter, repeated frame, multi-second
		 * stalls): anything outside is published with dt_us=0 so
		 * state_estimation drops the frame instead of fusing a garbage
		 * velocity. The ZUPT branch in state_estimation also requires
		 * dt_us != 0 (via the guard at the top of on_optflow), so a gated
		 * frame neither fuses nor ZUPTs. */
		uint32_t dt_us = 0;
		if (g_have_last[dir_idx]) {
			uint32_t dt_ms = raw_t_ms - g_last_t_ms[dir_idx];   /* wraps at 32-bit, OK */
			if (dt_ms >= 10u && dt_ms <= 500u) {
				dt_us = dt_ms * 1000u;
			}
		}
		g_last_t_ms[dir_idx] = raw_t_ms;
		g_have_last[dir_idx] = 1;

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
