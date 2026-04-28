#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

/**
 * STATE_ESTIMATION MODULE
 *
 * Unified ESKF (fusion6) navigation estimator. Replaces the legacy
 * attitude_estimation + position_estimation modules. Publishes nav_state_t
 * on STATE_UPDATE @ 25 Hz.
 *
 * Subscribes:
 *   SENSOR_IMU1_GYRO_FILTERED_UPDATE (1 kHz, post-notch, deg/s float[3])
 *   SENSOR_IMU1_ACCEL_UPDATE         (500 Hz, raw LSB float[3])
 *   SENSOR_COMPASS                   (vector3d_t, body-frame unit vector)
 *   SENSOR_AIR_PRESSURE              (double, mm)
 *   EXTERNAL_SENSOR_OPTFLOW          (optflow_data_t)
 *   EXTERNAL_SENSOR_GPS              (gps_position_t)
 *   EXTERNAL_SENSOR_GPS_VELOC        (gps_velocity_t)
 *   TUNING_READY                     (tuning_params_t — latency/timeout/etc.)
 *   SCHEDULER_25HZ                   (health check + publish snapshot)
 *
 * Publishes:
 *   STATE_UPDATE                     (nav_state_t @ 25 Hz)
 *
 * All sensor unit/axis conversions go through sensor_unit.h.
 */
void state_estimation_setup(void);

#endif /* STATE_ESTIMATION_H */
