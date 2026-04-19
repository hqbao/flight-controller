#include "config_internal.h"
#include <math.h>

static tuning_params_t g_tuning = {
	// Attitude PID
	.att_roll_p = 4.0f, .att_roll_i = 1.0f, .att_roll_d = 2.0f, .att_roll_i_limit = 5.0f,
	.att_pitch_p = 4.0f, .att_pitch_i = 1.0f, .att_pitch_d = 2.0f, .att_pitch_i_limit = 5.0f,
	.att_yaw_p = 10.0f, .att_yaw_i = 1.0f, .att_yaw_d = 5.0f, .att_yaw_i_limit = 5.0f,
	.att_smooth_input = 1.0f, .att_smooth_p_term = 1.0f, .att_smooth_output = 1.0f, .att_gain_time = 1.0f,
	// Position Control
	.pos_xy_p = 50.0f, .pos_z_p = 2000.0f,
	.pos_veloc_xy_scale = 50.0f, .pos_veloc_z_scale = 2000.0f,
	.pos_lpf_xy = 1.0f, .pos_lpf_z = 5.0f,
	.pos_angle_limit = 30.0f, .pos_rc_deadband = 0.1f,
	// Motor/Servo
	.motor_min = 150.0f, .motor_max = 1800.0f,
	.servo_min = 1000.0f, .servo_max = 2000.0f, .servo_center = 1500.0f,
	// Attitude Estimation
	.att_mahony_kp = 0.1f, .att_mahony_ki = 0.001f,
	.att_f3_beta = 0.001f, .att_f3_zeta = 0.0001f,
	.att_accel_smooth = 4.0f, .att_lin_acc_decay = 0.5f,
	.att_lin_accel_min = 0.5f, .att_lin_accel_max = 2.0f,
	// Position Estimation
	.pe_xy_s1_integ = 1.0f, .pe_xy_s1_corr = 1.25f,
	.pe_xy_s2_integ = 1.0f, .pe_xy_s2_corr = 10.0f, .pe_xy_v_fb = 0.1f,
	.pe_z_s1_integ = 1.0f, .pe_z_s1_corr = 0.5f,
	.pe_z_s2_integ = 1.0f, .pe_z_s2_corr = 10.0f, .pe_z_v_fb = 0.1f,
	.pe_optflow_gain = 5.0f,
	// FFT/Notch
	.notch_q = 3.0f, .notch_min_hz = 50.0f,
	.fft_peak_snr = 5.0f, .fft_freq_alpha = 0.15f,
	// Flight State
	.disarm_angle = 60.0f, .disarm_range = 10.0f,
	.allowed_landing_range = 500.0f, .took_off_range = 100.0f,
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
