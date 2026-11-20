/* bad_loop.c - fault: a "wait until the arm settles" loop that never exits (after 3 s). */
#include "ctrl_api.h"
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    volatile double a = o->angle_deg;
    if (o->t > 3.0) while (a > -1000.0) a += 0.0;           /* never true -> spins forever */
    return 0.05 * (40.0 - a);
}
