/* bad_mem.c - fault: keeps a log of every step in buffers that grow forever and
 * never checks malloc (1 MB per step, kept in a global list). */
#include <stdlib.h>
#include <string.h>
#include "ctrl_api.h"
static char *volatile history[100000];
static int n = 0;
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    char *log = malloc(1024 * 1024);
    memset(log, (int)o->angle_deg, 1024 * 1024);            /* no NULL check */
    history[n++ % 100000] = log;
    return 0.05 * (40.0 - o->angle_deg);
}
