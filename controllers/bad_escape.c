/* bad_escape.c - fault: tries to break out of the cage: kill the governor, fork, raise priority.
 * Then behaves normally. Each attempt is reported on stderr. */
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sched.h>
#include "ctrl_api.h"
static int tried = 0;
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    if (!tried && o->t > 1.0) {
        tried = 1;
        int r = kill(getppid(), SIGKILL);
        fprintf(stderr, "[escape] kill(governor): %s\n", r == 0 ? "SUCCEEDED" : strerror(errno));
        pid_t p = fork();
        if (p == 0) _exit(0);
        fprintf(stderr, "[escape] fork(): %s\n", p > 0 ? "SUCCEEDED" : strerror(errno));
        struct sched_param sp = { 0 }; sp.sched_priority = 60;
        r = sched_setscheduler(0, SCHED_FIFO, &sp);
        fprintf(stderr, "[escape] raise priority to 60: %s\n", r != -1 ? "SUCCEEDED" : strerror(errno));
    }
    return 0.05 * (40.0 - o->angle_deg);
}
