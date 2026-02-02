#include <rc_receiver/rc_receiver.h>
#include <attitude_control/attitude_control.h>
#include <attitude_estimation/attitude_estimation.h>
#include <local_storage/local_storage.h>
#include <logger/logger.h>
#include <fault_handler/fault_handler.h>
#include <oscillation_detection/oscillation_detection.h>
#include <linear_drift_detection/linear_drift_detection.h>
#include <position_estimation/position_estimation.h>
#include <state_detector/state_detector.h>

/**
 * SIMULATION MODULE INITIALIZATION
 * 
 * Only initializes pure logic modules. 
 * Hardware drivers (IMU, Mag, Baro) are skipped because
 * their data is injected directly by main.c from Gazebo.
 */
void modules_setup(void) {
    // 0. System Services
    local_storage_setup();
    fault_handler_setup();
    logger_setup();

    // 1. Input Processing
    rc_receiver_setup();

    // 2. State Estimation (Sensor Fusion)
    attitude_estimation_setup();
    position_estimation_setup();
    
    // 3. Control Loops (PID)
    attitude_control_setup();

    // 4. Safety Logic
    oscillation_detection_setup();
    linear_drift_detection_setup();
    state_detector_setup();
}

