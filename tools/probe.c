/* probe.c - diagnostic controller: prints the cage it finds itself in, then holds still. */
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <stdlib.h>
#include <sys/resource.h>
#include "ctrl_api.h"
static void lim(const char *n, int r) { struct rlimit l; getrlimit(r, &l); fprintf(stderr, "[probe] %s cur=%lld max=%lld\n", n, (long long)l.rlim_cur, (long long)l.rlim_max); }
void ctrl_init(void) {
    fprintf(stderr, "[probe] uid=%d euid=%d prio=%d\n", (int)getuid(), (int)geteuid(), ({ struct sched_param sp; sched_getparam(0, &sp); sp.sched_priority; }));
    lim("AS", RLIMIT_AS); lim("DATA", RLIMIT_DATA); lim("CPU", RLIMIT_CPU); lim("NPROC", RLIMIT_NPROC); lim("NOFILE", RLIMIT_NOFILE);
    size_t got = 0; for (int i = 0; i < 40; i++) { char *p = malloc(4 << 20); if (!p) break; got += 4; }
    fprintf(stderr, "[probe] malloc'd %zu MB in 4 MB chunks (untouched)\n", got);
}
double ctrl_step(const sc_obs_t *o) { (void)o; return 0.0; }
