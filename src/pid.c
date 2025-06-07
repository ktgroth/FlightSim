
#include "include/pid.h"

void pid_init(pid_t *pid, double kp, double ki, double kd,
              double out_min, double out_max,
              double int_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_limit = int_limit;
}

double pid_update(pid_t *pid, double target, double current, double dt) {
    double error = target - current;
    double P = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    double I = pid->ki * pid->integral;

    double derivative = (error - pid->prev_error) / dt;
    double D = pid->kd * derivative;

    pid->prev_error = error;

    double output = P + I + D;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}
