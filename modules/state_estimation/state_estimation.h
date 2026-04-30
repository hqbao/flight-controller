#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

/**
 * STATE_ESTIMATION MODULE
 *
 * Unified ESKF (fusion6) navigation estimator. Replaces the legacy
 * attitude_estimation + position_estimation modules. Publishes nav_state_t
 * on STATE_UPDATE @ 500 Hz after accel correction.
 *
 * Subscribes:
 *   SENSOR_IMU1_GYRO_FILTERED_UPDATE (1 kHz, post-notch, deg/s float[3])
 *   SENSOR_IMU1_ACCEL_UPDATE         (500 Hz, raw LSB float[3])
 *   SENSOR_COMPASS                   (vector3d_t, calibrated sensor-frame unit vector)
 *   NOTIFY_LOG_CLASS                 (uint8_t log class selector)
 *   SCHEDULER_50HZ                   (attitude diagnostic log)
 *   SCHEDULER_25HZ                   (mag-fusion diagnostic log)
 *
 * Publishes:
 *   STATE_UPDATE                     (nav_state_t @ 500 Hz)
 *   SEND_LOG                         (attitude/mag diagnostic payloads)
 *
 * All sensor unit/axis conversions go through sensor_unit.h.
 */
void state_estimation_setup(void);

#endif /* STATE_ESTIMATION_H */
