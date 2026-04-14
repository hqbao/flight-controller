#ifndef MIX_CONTROL_H
#define MIX_CONTROL_H

/*
 * Aircraft type selection — change this to select the mixer.
 *
 * AIRCRAFT_QUADCOPTER (1): X-frame quad/octocopter, 8 motors (DShot)
 * AIRCRAFT_BICOPTER   (2): Dual tilt-rotor, 2 motors (DShot) + 2 servos (PWM)
 *
 * Override at build time: -DAIRCRAFT_TYPE=AIRCRAFT_BICOPTER
 */
#define AIRCRAFT_QUADCOPTER 1
#define AIRCRAFT_BICOPTER   2

#ifndef AIRCRAFT_TYPE
#define AIRCRAFT_TYPE AIRCRAFT_QUADCOPTER
#endif

void mix_control_setup(void);

#endif
