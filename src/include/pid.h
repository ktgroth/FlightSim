
#ifndef PID_H
#define PID_H

typedef struct {
    double kp, ki, kd;
    double integral, prev_error;
    double output_min, output_max;
    double integral_limit;
} pid_t;

void pid_init(pid_t *pid, double kp, double ki, double kd,
              double out_min, double out_max,
              double int_limit);

double pid_update(pid_t *pid, double target, double current, double dt);

#endif
