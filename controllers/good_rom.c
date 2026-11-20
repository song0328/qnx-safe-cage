/* good_rom.c - reference controller: passive range-of-motion exercise.
 * Smoothly moves the elbow 20 -> 60 -> 20 degrees every 4 s (PD + stiffness feed-forward). */
#include <math.h>
#include "ctrl_api.h"
static const double PI = 3.14159265358979;
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    double ph = 2 * PI * o->t / 4.0;
    double target = 40.0 - 20.0 * cos(ph);                 /* deg */
    double target_v = 20.0 * (2 * PI / 4.0) * sin(ph);     /* deg/s */
    double ff = 1.5 * (target - 20.0) * PI / 180.0;        /* hold against tissue stiffness */
    return ff + 0.05 * (target - o->angle_deg) + 0.004 * (target_v - o->velocity_dps);
}
