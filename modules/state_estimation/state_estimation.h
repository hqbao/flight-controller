#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

/**
 * STATE_ESTIMATION MODULE
 *
 * Unified ESKF (fusion6) navigation estimator. Replaces the legacy
 * attitude_estimation + position_estimation modules. Publishes nav_state_t
 * on STATE_UPDATE at gyro rate (1 kHz) right after each predict step.
 *
 * Subscribes:
 *   SENSOR_IMU1_GYRO_FILTERED_UPDATE (1 kHz, post-notch, deg/s float[3])
 *   SENSOR_IMU1_ACCEL_UPDATE         (500 Hz, raw LSB float[3])
 *   SENSOR_COMPASS                   (~25 Hz, vector3d_t calibrated unit vec;
 *                                     drives 1-D yaw pseudo-measurement)
 *   EXTERNAL_SENSOR_OPTFLOW          (~25 Hz, optflow_data_t; DOWN cam only,
 *                                     drives v_body_xy + lidar p.z)
 *   SENSOR_AIR_PRESSURE              (~25 Hz, double mm above startup)
 *   EXTERNAL_SENSOR_GPS_QUALITY      (~5 Hz, gps_quality_t; gates pos+vel)
 *   EXTERNAL_SENSOR_GPS              (~5 Hz, gps_position_t; LLA \u2192 NED)
 *   EXTERNAL_SENSOR_GPS_VELOC        (~5 Hz, gps_velocity_t; mm/s \u2192 NED m/s)
 *   NOTIFY_LOG_CLASS                 (uint8_t log class selector)
 *   SCHEDULER_50HZ                   (attitude diagnostic log)
 *   SCHEDULER_25HZ                   (mag/baro/optflow + ESKF P/F/K/H logs)
 *   SCHEDULER_5HZ                    (gps_fusion log)
 *
 * Publishes:
 *   STATE_UPDATE                     (nav_state_t @ ~1 kHz)
 *   SEND_LOG                         (per-class diagnostic payloads)
 *
 * All sensor unit/axis conversions go through sensor_unit.h.
 */
void state_estimation_setup(void);

#endif /* STATE_ESTIMATION_H */
