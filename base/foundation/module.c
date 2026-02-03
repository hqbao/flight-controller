#include <fault_handler/fault_handler.h>
#include <local_storage/local_storage.h>
#include <imu/imu.h>
#include <attitude_estimation/attitude_estimation.h>
#include <air_pressure/air_pressure.h>
#include <compass/compass.h>
#include <attitude_control/attitude_control.h>
#include <rc_receiver/rc_receiver.h>
#include <position_estimation/position_estimation.h>
#include <position_control/position_control.h>
#include <gps_navigation/gps_navigation.h>
#include <gps_denied_navigation/gps_denied_navigation.h>
#include <optflow/optflow.h>
#include <gps/gps.h>
#include <state_detector/state_detector.h>
#include <speed_control/speed_control.h>
#include <logger/logger.h>
#include <oscillation_detection/oscillation_detection.h>
#include <linear_drift_detection/linear_drift_detection.h>
#include <noise_meas/noise_meas.h>

void platform_setup(void) {
	state_detector_setup();
	local_storage_setup();
	fault_handler_setup();
	speed_control_setup();
	imu_setup();
	air_pressure_setup();
	//compass_setup();
	attitude_estimation_setup();
	attitude_control_setup();
	position_estimation_setup();
	oscillation_detection_setup();
	linear_drift_detection_setup();
	position_control_setup();
	optflow_setup();
	//gps_setup();
	//gps_navigation_setup();
	//gps_denied_navigation_setup();
	rc_receiver_setup();
	logger_setup();
	noise_meas_setup();
}
