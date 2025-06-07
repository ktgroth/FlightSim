
#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

#include "include/plane.h"
#include "include/pid.h"

static double thrust_min = -800., thrust_max = 1000.;

static pid_t autopilot_pid;

void update_physics(plane_t *plane, double dt) {
    double net_force = plane->thrust - plane->drag - plane->gravity;
    plane->velocity += net_force * dt;
    plane->altitude += plane->velocity * dt;

    if (plane->altitude < 0) {
        plane->altitude = 0;
        plane->velocity = 0;
    }
}

void update_controls(plane_t *plane, char input, double kp, double ki, double kd, uint8_t *autopilot) {
    if (input == 'w') {
        *autopilot = 0;
        plane->thrust = plane->thrust + 1.;
    } else if (input == 's') {
        *autopilot = 0;
        plane->thrust = plane->thrust - 1.;
    } else if (input == 'p') {
        *autopilot = !*autopilot;
        pid_init(&autopilot_pid, kp, ki, kd, thrust_min, thrust_max, 5000.);
    }

    if (plane->thrust < thrust_min) plane->thrust = thrust_min;
    if (plane->thrust > thrust_max) plane->thrust = thrust_max;
}

void apply_autopilot(plane_t *plane, double target_altitude, double dt) {
    plane->thrust = pid_update(&autopilot_pid, target_altitude, plane->altitude, dt);
    if (plane->thrust < thrust_min) plane->thrust = thrust_min;
    if (plane->thrust > thrust_max) plane->thrust = thrust_max;
}
