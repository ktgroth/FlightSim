
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "include/plane.h"
#include "include/test.h"

struct termios orig_termios;

void set_input_mode();
void reset_input_mode();

void sim(plane_t *plane) {
    uint8_t autopilot = 0;
    double dt = 0.001;
    
    double kp, ki, kd;
    // min test 1.2282 0.2210 3.1814
    printf("Enter (kp) (ki) (kd): ");
    if (scanf("%lf %lf %lf", &kp, &ki, &kd) != 3)
        exit(1);
    else
        fflush(stdin);
    set_input_mode();

    system("clear");
    printf("\n\n\n\n");
    while (1) {
        char input = 0;
        if (read(STDIN_FILENO, &input, 1) > 0) {
            if (input == 'e')
                exit(0);
            update_controls(plane, input, kp, ki, kd, &autopilot);
        }
        
        if (autopilot)
            apply_autopilot(plane, 10000, dt);

        update_physics(plane, dt);
        
        printf("\033[4A");
        printf("\rAltitude: %.3f m | Velocity: %.3f m/h | Thrust: %.3f N         \n",
                  plane->altitude,     plane->velocity,        plane->thrust);
        printf("\rControls: W = increase thrust, S = decrease thrust             \n");
        printf("\r          P = toggle auto-pilot [%s]                           \n",
                            autopilot ? "ON ": "OFF");
        printf("\r          E = Exit                                             \n");
        
        usleep(10000);
    }
}

int main(int argc, char *argv[]) {
    plane_t plane = { 0.0, 0.0, 0.0, 0.2, 9.8 };
    if (argc > 1 && !strcmp(argv[1], "test"))
        test(&plane);
    else
        sim(&plane);

    return 0;
}

void set_input_mode() {
    struct termios tattr;
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(reset_input_mode);

    tattr = orig_termios;
    tattr.c_lflag &= !(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tattr);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void reset_input_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}
