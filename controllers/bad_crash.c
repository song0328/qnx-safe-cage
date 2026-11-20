/* bad_crash.c - fault: uses a lookup table pointer that was never set (after 3 s). */
#include <stddef.h>
#include "ctrl_api.h"
static double *gain_table = NULL;
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    if (o->t > 3.0) return gain_table[(int)o->angle_deg] * (40.0 - o->angle_deg);
    return 0.05 * (40.0 - o->angle_deg);
}
