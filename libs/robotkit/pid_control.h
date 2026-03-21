#ifndef PID_CONTROL_H
#define PID_CONTROL_H

/**
 * PID controller with three-stage smoothing and output limiting.
 * Call order: pid_control_update(pid, value, target, dt) — value first, then target.
 */
typedef struct {
  double value;          // Smoothed input value
  double value_prev;     // Previous value (for derivative)
  double p_value;        // Double-smoothed value for P-term

  double error;          // Current error (p_value - target)
  double derivative;     // Rate of change of value
  double integral;       // Accumulated error

  double smooth1;        // Input smoothing factor (1.0 = no smoothing)
  double smooth2;        // P-value smoothing factor
  double smooth3;        // Output smoothing factor

  double p_gain;         // Proportional gain
  double i_gain;         // Integral gain
  double i_gain_accum;   // Integral accumulation rate
  double d_gain;         // Derivative gain

  double p_limit;        // P-term absolute limit
  double i_limit;        // I-term absolute limit
  double o_limit;        // Output absolute limit

  char halt_i;           // If 1, freeze integral accumulation

  double p_term;         // Current P contribution
  double i_term;         // Current I contribution
  double d_term;         // Current D contribution

  double output;         // Final PID output
} pid_control_t;

/** Initialize PID controller with default values */
void pid_control_init(pid_control_t *pid_control);

/** Reset PID state and set initial value */
void pid_control_reset(pid_control_t *pid_control, double init_value);

/** Set smoothing factors: input, P-value, output (1.0 = no smoothing) */
void pid_control_set_smooth(pid_control_t *pid_control, double smooth1, double smooth2, double smooth3);

/** Set proportional gain */
void pid_control_set_p_gain(pid_control_t *pid_control, double p_gain);

/** Set integral gain and accumulation rate */
void pid_control_set_i_gain(pid_control_t *pid_control, double i_gain, double i_gain_accum);

/** Set derivative gain */
void pid_control_set_d_gain(pid_control_t *pid_control, double d_gain);

/** Set P-term absolute limit */
void pid_control_set_p_limit(pid_control_t *pid_control, double abs_value);

/** Set I-term absolute limit */
void pid_control_set_i_limit(pid_control_t *pid_control, double abs_value);

/** Set output absolute limit */
void pid_control_set_o_limit(pid_control_t *pid_control, double abs_value);

/** Freeze/unfreeze integral accumulation */
void pid_control_halt_i(pid_control_t *pid_control, char halt_i);

/** Compute PID output: pid_control_update(pid, value, target, dt) */
double pid_control_update(pid_control_t *pid_control, double value, double target, double dt);

#endif
