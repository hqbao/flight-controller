#include <fault_handler/fault_handler.h>
#include <fault_detector/fault_detector.h>
#include <local_storage/local_storage.h>
#include <imu/imu.h>
#include <calibration/calibration.h>
#include <attitude_estimation/attitude_estimation.h>
#include <air_pressure/air_pressure.h>
#include <compass/compass.h>
#include <attitude_control/attitude_control.h>
#include <mix_control/mix_control.h>
#include <rc_receiver/rc_receiver.h>
#include <position_estimation/position_estimation.h>
#include <position_control/position_control.h>
#include <gps_navigation/gps_navigation.h>
#include <gps_denied_navigation/gps_denied_navigation.h>
#include <optflow/optflow.h>
#include <gps/gps.h>
#include <flight_state/flight_state.h>
#include <flight_telemetry/flight_telemetry.h>
#include <speed_control/speed_control.h>
#include <config/config.h>
#include <fft/fft.h>
#include <notch_filter/notch_filter.h>
#include <troubleshoot/troubleshoot.h>
#include <dblink/dblink.h>

void platform_setup(void) {
	dblink_setup();
	flight_state_setup();
	local_storage_setup();
	calibration_setup();
	config_setup();
	fault_handler_setup();
	speed_control_setup();
	imu_setup();
	fft_setup();
	notch_filter_setup();
	air_pressure_setup();
	compass_setup();
	fault_detector_setup();
	attitude_estimation_setup();
	attitude_control_setup();
	mix_control_setup();
	position_estimation_setup();
	position_control_setup();
	optflow_setup();
	//gps_setup();
	//gps_navigation_setup();
	//gps_denied_navigation_setup();
	rc_receiver_setup();
	flight_telemetry_setup();
	troubleshoot_setup();
}
