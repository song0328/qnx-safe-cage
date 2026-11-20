/* ctrl_main.c - harness linked into every controller binary.
 * Connects to the governor's channel (pid/chid passed in the environment) and runs the
 * synchronous command/state loop. Runs inside the cage (unprivileged user, resource
 * limits, low priority).
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include "common.h"
#include "ctrl_api.h"

int main(void) {
    const char *ps = getenv("SC_PID"), *cs = getenv("SC_CHID");
    if (!ps || !cs) { fprintf(stderr, "controller: SC_PID/SC_CHID not set (run me via the governor)\n"); return 1; }
    int coid = ConnectAttach(0, (pid_t)atoi(ps), atoi(cs), _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) { perror("controller: ConnectAttach"); return 1; }

    /* say hello first: the governor finishes locking the cage before it replies,
       so no controller code (ctrl_init/ctrl_step) runs before the cage is complete */
    sc_cmd_msg_t m = { SC_MSG_CMD, 0, SC_SEQ_HELLO, 0.0 };
    sc_state_t s;
    int first = 1;
    for (;;) {
        if (MsgSend(coid, &m, sizeof m, &s, sizeof s) == -1) break;
        if (first) { ctrl_init(); first = 0; }
        if (s.done) break;
        sc_obs_t o = { s.t, s.angle_deg, s.velocity_dps };
        m.seq = s.seq;
        m.torque = ctrl_step(&o);
    }
    ConnectDetach(coid);
    return 0;
}
