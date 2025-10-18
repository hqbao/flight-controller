#ifndef PID_CONTROL_H
#define PID_CONTROL_H

typedef struct {
  double value;
  double value_prev;
  double p_value;

  double error;
  double derivative;
  double integral;

  double smooth1;
  double smooth2;
  double smooth3;

  double p_gain;
  double i_gain;
  double i_gain_accum;
  double d_gain;

  double p_limit;
  double i_limit;
  double o_limit;

  char halt_i;

  double p_term;
  double i_term;
  double d_term;

  double output;
} pid_control_t;

void pid_control_init(pid_control_t *pid_control);
void pid_control_reset(pid_control_t *pid_control, double init_value);
void pid_control_set_smooth(pid_control_t *pid_control, double smooth1, double smooth2, double smooth3);
void pid_control_set_p_gain(pid_control_t *pid_control, double p_gain);
void pid_control_set_i_gain(pid_control_t *pid_control, double i_gain, double i_gain_accum);
void pid_control_set_d_gain(pid_control_t *pid_control, double d_gain);
void pid_control_set_p_limit(pid_control_t *pid_control, double abs_value);
void pid_control_set_i_limit(pid_control_t *pid_control, double abs_value);
void pid_control_set_o_limit(pid_control_t *pid_control, double abs_value);
void pid_control_halt_i(pid_control_t *pid_control, char halt_i);
double pid_control_update(pid_control_t *pid_control, double value, double target, double dt);

#endif
