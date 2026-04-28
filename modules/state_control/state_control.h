#ifndef STATE_CONTROL_H
#define STATE_CONTROL_H

/*
 * STATE_CONTROL — cascaded position→attitude→mix controller.
 *
 * Replaces the deleted attitude_control + position_control modules. Reads the
 * unified state estimate on STATE_UPDATE (nav_state_t) and produces motor
 * mix commands on MIX_CONTROL_UPDATE (mix_control_input_t).
 *
 * Loop layout:
 *   STATE_UPDATE        @ 1 kHz from state_estimation → cache nav_state_t
 *   ATT_CTL_SCHEDULER   @ 500 Hz → attitude inner PID, publishes MIX_CONTROL_UPDATE
 *   PILOT_CTL_SCHEDULER @ 100 Hz → RC stick handling + outer position cascade
 *
 * Subscriptions:
 *   STATE_UPDATE             — nav_state_t snapshot (pose / twist)
 *   ATT_CTL_SCHEDULER        — inner attitude PID tick
 *   PILOT_CTL_SCHEDULER      — outer position cascade + RC manual control
 *   POSITION_TARGET_UPDATE   — vector3d_t external target (gps_navigation)
 *   FLIGHT_STATE_UPDATE      — state_t armed/flying machine
 *   RC_STATE_UPDATE          — rc_state_ctl_t (mode change)
 *   RC_MOVE_IN_UPDATE        — rc_att_ctl_t (stick deflection)
 *   EXTERNAL_SENSOR_OPTFLOW  — landing-range piggy-back
 *   TUNING_READY             — ctl_att_* / ctl_pos_* live-mirror
 *
 * Publishes:
 *   MIX_CONTROL_UPDATE       — mix_control_input_t (roll/pitch/yaw + altitude)
 */

void state_control_setup(void);

#endif // STATE_CONTROL_H
