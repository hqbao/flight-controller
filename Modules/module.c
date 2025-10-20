#include <imu/imu.h>
#include <attitude_fusion/attitude_fusion.h>
#include <air_pressure/air_pressure.h>
#include <attitude_control/attitude_control.h>
#include <remote_control/remote_control.h>
#include <imu_calibrator/imu_calibrator.h>
#include <nav_fusion/nav_fusion.h>
#include <nav_control/nav_control.h>
#include <optflow/optflow.h>
#include <state_detector/state_detector.h>
#include <speed_control/speed_control.h>
#include <logger/logger.h>
#include <test/test.h>

void platform_setup(void) {
	//test_setup();
	imu_setup();
	speed_control_setup();
	attitude_fusion_setup();
	air_pressure_setup();
	attitude_control_setup();
	nav_fusion_setup();
	nav_control_setup();
	state_detector_setup();
	optflow_setup();
	remote_control_setup();
	logger_setup();
}
