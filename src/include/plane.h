
#ifndef PLANE_H
#define PLANE_H

#include <stdint.h>

typedef struct {
    double altitude;
    double velocity;
    double thrust;
    double drag;
    double gravity;
} plane_t;

void update_physics(plane_t *plane, double dt);

void update_controls(plane_t *plane, char input, double kp, double ki, double kd, uint8_t *autopilot_enabled);
void apply_autopilot(plane_t *plane, double target_altitude, double dt);

#endif
