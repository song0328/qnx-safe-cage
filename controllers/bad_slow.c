/* bad_slow.c - fault: does heavy work every step (busy-waits ~3 ms) - always late. */
#include <time.h>
#include "ctrl_api.h"
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    struct timespec a, b; clock_gettime(CLOCK_MONOTONIC, &a);
    do clock_gettime(CLOCK_MONOTONIC, &b); while ((b.tv_sec - a.tv_sec) * 1e9 + (b.tv_nsec - a.tv_nsec) < 3e6);
    return 0.05 * (40.0 - o->angle_deg);
}
