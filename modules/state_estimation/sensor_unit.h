/**
 * sensor_unit.h — Pure conversion helpers from raw sensor messages into the
 * canonical units / frames consumed by fusion6.
 *
 * SCOPE: This header is the single source of truth for sensor unit + axis
 * conventions in flight-controller. Every sensor PubSub callback in
 * state_estimation.c must route its data through one of these helpers; no
 * conversion math may live anywhere else. Host tests in robotkit-tests/
 * test_sensor_unit.c lock these contracts down.
 *
 * All helpers are `static inline` and have NO PubSub dependency, so they can
 * be called from both the live module and from cffi-driven host tests.
 *
 * --- Coordinate frame conventions (NED) ---
 *   Body  frame: +X forward (nose), +Y right, +Z down.
 *   Earth frame: +X North, +Y East, +Z down.
 *   Specific force at rest level: accel reads (0, 0, −g)  (table pushes up).
 *
 * --- Sensor → body axis maps (PCB v1) ---
 *   ICM-42688P (sensor X=Right, Y=Forward, Z=Down on PCB):
 *     body_gx = -raw_gy;  body_gy = -raw_gx;  body_gz = -raw_gz;
 *     body_ax = -raw_ay;  body_ay = -raw_ax;  body_az = -raw_az;
 *   BMM350 (sensor X=Right, Y=Forward, Z=Down on PCB):
 *     body_x  =  sensor_y; body_y = -sensor_x; body_z =  sensor_z;
 *
 * --- Per-sensor input contract table ---
 *   accel       m/s²   body NED   gravity-incl. specific force   at-rest (0,0,-g)
 *   gyro        rad/s  body NED   right-hand rule                at-rest (0,0,0)
 *   baro_alt    m      pos-up     rises when going up            0 at boot
 *   lidar_range m      body Z dn  always positive when valid     n/a
 *   optflow     rad/s  body XY    per optflow_direction_t        (0,0) static
 *   mag         unit   sensor     calibrated/normalized; mapped to body here n/a
 *   gps_pos     m      NED, local origin, (0,0,0) at first fix   n/a
 *   gps_vel     m/s    NED absolute                              (0,0,0) rest
 */
#ifndef SENSOR_UNIT_H
#define SENSOR_UNIT_H

#include <stdint.h>
#include <math.h>

/* These typedefs are pulled in via <messages.h> when used in flight-controller.
 * For host tests we redefine them via a tiny shim header before including this
 * file (see robotkit-tests/test_sensor_unit.c). */
#ifndef SENSOR_UNIT_HOST_TEST
#include <messages.h>   /* optflow_data_t, optflow_direction_t, gps_position_t,
                           gps_velocity_t */
#endif

/* WGS-84 mean earth radius, meters. Adequate for equirectangular projection
 * over local distances (<10 km error <1%). */
#define SENSOR_UNIT_EARTH_R_M  6371000.0
#define SENSOR_UNIT_DEG2RAD    (3.14159265358979323846 / 180.0)

/* ============================================================
 * Barometer
 * ============================================================ */

/** Convert raw baro reading (mm above MSL or relative reference) to meters. */
static inline double baro_mm_to_m(int32_t mm) {
    return (double)mm * 1e-3;
}

/* ============================================================
 * Range finder (lidar / ToF, downward-facing)
 * ============================================================ */

/** Validity gate for the downward range finder.
 *  Returns 1 if the reading is plausible (in range, positive), else 0. */
static inline int lidar_valid_mm(int32_t mm, int32_t min_mm, int32_t max_mm) {
    if (mm <= 0) return 0;
    if (mm < min_mm) return 0;
    if (mm > max_mm) return 0;
    return 1;
}

/** Convert a validated range reading (mm) to meters. */
static inline double lidar_mm_to_m(int32_t mm) {
    return (double)mm * 1e-3;
}

/* ============================================================
 * Optical flow
 * ============================================================ */

/** Convert raw optical-flow angular displacement (radians over dt_us μs)
 *  into angular rate (rad/s) in body frame. Sign flipped for upward-facing
 *  cameras so the published rate is always relative to the surface the body
 *  is moving over (downward = ground, upward = ceiling).
 *  Returns 0 if dt_us is zero (caller should reject the sample). */
static inline double optflow_to_rad_per_s(double dx_or_dy_rad,
                                          uint32_t dt_us,
                                          int upward) {
    if (dt_us == 0) return 0.0;
    double rate = dx_or_dy_rad * 1e6 / (double)dt_us;
    return upward ? -rate : rate;
}

/** Adaptive R scale for optical flow based on reported quality score (0..100+).
 *  Returns multiplicative factor on baseline R:
 *    quality >= q_good : 1.0  (trust as-is)
 *    quality <= q_min  : INFINITY (reject — caller should mask the update)
 *    else              : linearly interpolated 1.0 → 4.0 across the range. */
static inline double optflow_quality_to_R_scale(double quality,
                                                double q_min,
                                                double q_good) {
    if (quality <= q_min) return INFINITY;
    if (quality >= q_good) return 1.0;
    double t = (q_good - quality) / (q_good - q_min);   /* 0 at good, 1 at min */
    return 1.0 + 3.0 * t;
}

/* ============================================================
 * IMU axis mapping (ICM-42688P, PCB v1)
 *
 * Sensor frame on PCB: X = Right, Y = Forward, Z = Down.
 * Body frame (NED): X = Forward (nose), Y = Right, Z = Down.
 * Mapping (with sign correction for sensor mount orientation):
 *   body_x = -raw_y;  body_y = -raw_x;  body_z = -raw_z;
 * ============================================================ */

static inline void imu_axis_map_gyro(double raw_x, double raw_y, double raw_z,
                                     double out_body[3]) {
    out_body[0] = -raw_y;
    out_body[1] = -raw_x;
    out_body[2] = -raw_z;
}

static inline void imu_axis_map_accel(double raw_x, double raw_y, double raw_z,
                                      double out_body[3]) {
    out_body[0] = -raw_y;
    out_body[1] = -raw_x;
    out_body[2] = -raw_z;
}

/* ============================================================
 * Magnetometer axis mapping (BMM350, PCB v1)
 *   body_x =  sensor_y; body_y = -sensor_x; body_z = sensor_z;
 * ============================================================ */

static inline void mag_axis_map(double sensor_x, double sensor_y, double sensor_z,
                                double out_body[3]) {
    out_body[0] =  sensor_y;
    out_body[1] = -sensor_x;
    out_body[2] =  sensor_z;
}

/* ============================================================
 * GPS
 *
 * gps_position_t carries deg×1e7 (lat, lon) and mm (alt above MSL).
 * Convert into NED meters relative to a lazy-initialized local origin.
 * Caller owns the origin state — pass origin_set=0 on first call to capture.
 * ============================================================ */

/** Lazy-init helper: capture (or update) the local NED origin.
 *  On the first call where *origin_set == 0, copies (lat,lon,alt) into the
 *  origin slots and sets *origin_set = 1. Subsequent calls are no-ops. */
static inline void gps_origin_capture(int *origin_set,
                                      int32_t *lat0_e7, int32_t *lon0_e7,
                                      int32_t *alt0_mm,
                                      int32_t lat_e7, int32_t lon_e7,
                                      int32_t alt_mm) {
    if (*origin_set) return;
    *lat0_e7 = lat_e7;
    *lon0_e7 = lon_e7;
    *alt0_mm = alt_mm;
    *origin_set = 1;
}

/** Equirectangular projection of (lat,lon,alt) onto NED relative to origin.
 *  Accurate to <1 % up to ~10 km from origin; adequate for drone navigation.
 *  out_ned: [N, E, D] in meters; D = -(alt_above_origin). */
static inline void gps_lla_to_ned(int32_t lat_e7, int32_t lon_e7, int32_t alt_mm,
                                  int32_t lat0_e7, int32_t lon0_e7, int32_t alt0_mm,
                                  double out_ned[3]) {
    double dlat = (double)(lat_e7 - lat0_e7) * 1e-7 * SENSOR_UNIT_DEG2RAD;
    double dlon = (double)(lon_e7 - lon0_e7) * 1e-7 * SENSOR_UNIT_DEG2RAD;
    double lat0 = (double)lat0_e7 * 1e-7 * SENSOR_UNIT_DEG2RAD;
    out_ned[0] = dlat * SENSOR_UNIT_EARTH_R_M;                   /* North */
    out_ned[1] = dlon * SENSOR_UNIT_EARTH_R_M * cos(lat0);       /* East  */
    out_ned[2] = -((double)(alt_mm - alt0_mm) * 1e-3);           /* Down  */
}

/** Convert GPS velocity from mm/s components to m/s NED. Sign preserved. */
static inline void gps_vel_mm_per_s_to_m_per_s(int32_t vN_mmps, int32_t vE_mmps,
                                               int32_t vD_mmps,
                                               double out_ned[3]) {
    out_ned[0] = (double)vN_mmps * 1e-3;
    out_ned[1] = (double)vE_mmps * 1e-3;
    out_ned[2] = (double)vD_mmps * 1e-3;
}

#endif /* SENSOR_UNIT_H */
