
#include <stdio.h>
#include <stdlib.h>

#include "include/plane.h"

void test(plane_t *plane) {
    uint8_t autopilot = 0;
    double dt = 0.001;
    double t = 0;
    
    double kp, ki, kd;
    if (scanf("%lf %lf %lf", &kp, &ki, &kd) != 3)
        exit(1);
    else
        fflush(stdin);

    FILE *output_file = fopen("stats", "w");

    update_controls(plane, 'p', kp, ki, kd, &autopilot);
    fprintf(output_file, "%lf %lf %lf\n",
            plane->altitude, plane->velocity, plane->thrust);
            
    while (t < 100) {
        apply_autopilot(plane, 10000, dt);
        update_physics(plane, dt);

        fprintf(output_file, "%lf %lf %lf\n",
                plane->altitude, plane->velocity, plane->thrust);
        t += dt;
    }

    fclose(output_file);
}
