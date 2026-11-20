/* bad_torque.c - fault: "make it faster" - huge gains, aims past the range of motion. */
#include "ctrl_api.h"
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    double target = (o->t < 3.0) ? 130.0 : 0.0;            /* outside the 0..90 deg range */
    return 2.0 * (target - o->angle_deg) - 0.01 * o->velocity_dps;
}
