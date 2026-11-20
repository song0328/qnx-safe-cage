/* bad_hog.c - worst case: after 2 s, raise itself to the highest priority a non-root
 * process may use (63, FIFO) and spin forever. On a single CPU this starves every
 * normal process - but not the governor, which runs at 70. */
#include <sched.h>
#include "ctrl_api.h"
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    if (o->t > 2.0) {
        struct sched_param sp = { 0 }; sp.sched_priority = 63;
        sched_setscheduler(0, SCHED_FIFO, &sp);
        for (volatile unsigned long x = 0;; x++) ;
    }
    return 0.05 * (40.0 - o->angle_deg);
}
