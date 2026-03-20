#include "calibration.h"
#include "calibration_gyro.h"
#include "calibration_accel.h"
#include "calibration_mag.h"
#include <pubsub.h>
#include <messages.h>
#include <stdint.h>

/*
 * --- SENSOR CALIBRATION MODULE (Coordinator) ---
 *
 * Top-level coordinator that initializes all calibration sub-modules.
 * Each sub-module is self-contained:
 *   - Loads calibration from flash at boot
 *   - Saves calibration to flash when received from host
 *   - Delivers calibration to its sensor module when that module reports ready
 *   - Responds to CALIBRATION_*_REQUEST by re-publishing calibration
 *
 * Sub-modules:
 *   - calibration_gyro.c  — Gyro bias (OTA upload + flash)
 *   - calibration_accel.c — Accel bias & scale matrix (OTA upload + flash)
 *   - calibration_mag.c   — Compass hard/soft iron correction (OTA upload + flash)
 *
 * All three sensors use the same linear correction model:
 *   V_cal = S * (V_raw - B)
 */

void calibration_setup(void) {
	calibration_gyro_setup();
	calibration_accel_setup();
	calibration_mag_setup();
}
