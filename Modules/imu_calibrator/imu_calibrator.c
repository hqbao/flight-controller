#include "imu_calibrator.h"
#include <pubsub.h>
#include <platform.h>
#include <vector3d.h>

#define POSITION_MIN 	10000
#define POSITION_CENTER 15000
#define POSITION_MAX 	20000

static vector3d_t g_attitude = {0, 0, 1};
static char g_imu_available = 0;

static void attitude_vector_update(uint8_t *data, size_t size) {
	g_attitude = *(vector3d_t*)data;
}

static void imu_calibrator_loop(uint8_t *data, size_t size) {
	int x = g_attitude.x * 5000;
	int y = g_attitude.y * 5000;

	platform_pwm_send(PWM_PORT1, POSITION_CENTER + x);
	platform_pwm_send(PWM_PORT2, POSITION_CENTER + y);
}

static void on_imu_calibration_result(uint8_t *data, size_t size) {
	if (data[0] == 1) g_imu_available = 1;
	else publish(SENSOR_IMU1_CALIBRATE_GYRO, NULL, 0);
}

static void imu_calibrator_loop_25hz(uint8_t *data, size_t size) {
	if (g_imu_available) {
		platform_toggle_led(0);
	}
}

void imu_calibrator_setup(void) {
	platform_pwm_init(PWM_PORT1);
	platform_pwm_init(PWM_PORT2);

	subscribe(SENSOR_ATTITUDE_VECTOR, attitude_vector_update);
	subscribe(SCHEDULER_1KHZ, imu_calibrator_loop);
	subscribe(SCHEDULER_25HZ, imu_calibrator_loop_25hz);
	subscribe(SENSOR_IMU1_GYRO_CALIBRATION_UPDATE, on_imu_calibration_result);
	publish(SENSOR_IMU1_CALIBRATE_GYRO, NULL, 0);
}

