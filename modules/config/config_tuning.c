#include "config_internal.h"
#include <math.h>

static tuning_params_t g_tuning = {
	// Attitude PID (state_control inner loop)
	.ctl_att_roll_p = 4.0f, .ctl_att_roll_i = 1.0f, .ctl_att_roll_d = 2.0f, .ctl_att_roll_i_limit = 5.0f,
	.ctl_att_roll_p_limit = 1000000.0f, .ctl_att_roll_o_limit = 1000000.0f,
	.ctl_att_pitch_p = 4.0f, .ctl_att_pitch_i = 1.0f, .ctl_att_pitch_d = 2.0f, .ctl_att_pitch_i_limit = 5.0f,
	.ctl_att_pitch_p_limit = 1000000.0f, .ctl_att_pitch_o_limit = 1000000.0f,
	.ctl_att_yaw_p = 10.0f, .ctl_att_yaw_i = 1.0f, .ctl_att_yaw_d = 5.0f, .ctl_att_yaw_i_limit = 5.0f,
	.ctl_att_yaw_p_limit = 1000000.0f, .ctl_att_yaw_o_limit = 1000000.0f,
	.ctl_att_smooth_input = 1.0f, .ctl_att_smooth_p_term = 1.0f, .ctl_att_smooth_output = 1.0f, .ctl_att_gain_time = 1.0f,
	// Position Control (state_control outer loop)
	.ctl_pos_xy_p = 100.0f, .ctl_pos_z_p = 2000.0f,
	.ctl_pos_veloc_xy_scale = 50.0f, .ctl_pos_veloc_z_scale = 2000.0f,
	.ctl_pos_lpf_xy = 1.0f, .ctl_pos_lpf_z = 5.0f,
	.ctl_pos_angle_limit = 30.0f, .ctl_pos_rc_deadband = 0.1f,
	// Motor/Servo
	.motor_min = 150.0f, .motor_max = 1800.0f,
	.servo_min = 1000.0f, .servo_max = 2000.0f, .servo_center = 1500.0f,
	// Estimator process noise PSDs (Q diagonal)
	.est_q_accel = 0.5f, .est_q_gyro = 0.01f,
	.est_q_ba = 0.01f, .est_q_bg = 0.001f, .est_q_bbaro = 0.01f,
	// Estimator measurement noise (R diagonal)
	.est_r_accel = 1.0f, .est_r_mag_yaw = 0.1f, .est_r_baro = 1.0f,
	.est_r_lidar = 0.05f, .est_r_optflow = 0.1f,
	.est_r_gps_pos_h = 1.0f, .est_r_gps_pos_v = 2.0f, .est_r_gps_vel = 0.3f,
	// Estimator gating + lever arms
	.est_chi2_pos = 11.34f, .est_chi2_vel = 11.34f,
	.est_accel_g_tol = 1.5f,
	.est_gps_lever_x = 0.0f, .est_gps_lever_y = 0.0f, .est_gps_lever_z = 0.0f,
	// FFT/Notch
	.notch_q = 3.0f, .notch_min_hz = 50.0f,
	.fft_freq_alpha = 0.15f,
	// Flight State
	.disarm_angle = 60.0f, .disarm_range = 10.0f,
	.allowed_landing_range = 500.0f, .took_off_range = 100.0f,
	// Servo Bias
	.servo_bias_1 = 0.0f, .servo_bias_2 = 0.0f,
	.servo_bias_3 = 0.0f, .servo_bias_4 = 0.0f,
	// Thrust Linearization
	.thrust_p1 = 1.0f, .thrust_p2 = 0.0f,
	// RC Scale
	.rc_xy_scale = 0.01f, .rc_z_scale = 0.04f, .rc_yaw_scale = -0.5f,
	// Sensor latency compensation (ms)
	.est_latency_baro_ms = 30.0f, .est_latency_lidar_ms = 30.0f,
	.est_latency_optflow_ms = 50.0f, .est_latency_gps_ms = 100.0f,
	// Sensor health timeouts (ms)
	.est_timeout_gps_ms = 2000.0f, .est_timeout_optflow_ms = 200.0f,
	.est_timeout_lidar_ms = 200.0f, .est_timeout_mag_ms = 500.0f,
	.est_timeout_baro_ms = 500.0f,
	// Filter divergence + position-loop fade
	.est_p_runaway_pos_m2 = 100.0f,
	.ctl_position_loop_fade_s = 3.0f,
};
static uint8_t g_tuning_loaded = 0;
static uint8_t g_tuning_count = 0;

void config_tuning_on_result(param_storage_t *p) {
	if (p->id < PARAM_ID_TUNING_FIRST || p->id > PARAM_ID_TUNING_LAST) return;

	/* Reject invalid values from flash — keep compiled default. */
	if (isnan(p->value) || isinf(p->value)) {
		g_tuning_count++;
		if (g_tuning_count >= PARAM_ID_TUNING_COUNT) g_tuning_loaded = 1;
		return;
	}

	uint16_t offset = (p->id - PARAM_ID_TUNING_FIRST) * sizeof(float);
	memcpy((uint8_t *)&g_tuning + offset, &p->value, sizeof(float));
	g_tuning_count++;
	if (g_tuning_count >= PARAM_ID_TUNING_COUNT) {
		g_tuning_loaded = 1;
	}
}

void config_tuning_on_save(param_storage_t *p) {
	if (p->id < PARAM_ID_TUNING_FIRST || p->id > PARAM_ID_TUNING_LAST) return;
	uint16_t offset = (p->id - PARAM_ID_TUNING_FIRST) * sizeof(float);
	memcpy((uint8_t *)&g_tuning + offset, &p->value, sizeof(float));
	g_tuning_loaded = 1;
	publish(TUNING_READY, (uint8_t *)&g_tuning, sizeof(tuning_params_t));
}

void config_tuning_publish(void) {
	if (g_tuning_loaded) {
		publish(TUNING_READY,
			(uint8_t *)&g_tuning, sizeof(tuning_params_t));
	}
}

void config_tuning_request_load(void) {
	if (!g_tuning_loaded) {
		config_request_params(PARAM_ID_TUNING_FIRST, PARAM_ID_TUNING_LAST);
	}
}

void config_tuning_on_db_message(uint8_t *data, size_t size) {
	if (size < 5) return;
	if (data[0] != DB_CMD_TUNING) return;

	if (size < 4 + 8) return;

	param_storage_t param;
	memcpy(&param, &data[4], sizeof(param_storage_t));

	if (param.id < PARAM_ID_TUNING_FIRST || param.id > PARAM_ID_TUNING_LAST) return;

	publish(LOCAL_STORAGE_SAVE, (uint8_t *)&param, sizeof(param_storage_t));
}
