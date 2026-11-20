/* bad_nan.c - fault: division by a quantity that becomes zero -> NaN torque after 2 s. */
#include "ctrl_api.h"
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    double scale = (o->t < 2.0) ? 1.0 : 0.0;
    return (40.0 - o->angle_deg) * 0.05 * (scale / scale);
}
