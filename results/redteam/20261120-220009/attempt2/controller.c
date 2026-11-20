/* attack_ctrl.c — red-team controller for the SafeCage sandbox (simulated elbow).
 *
 * Target: win condition (3). The cage's "safe" verdict is purely override-counting,
 * so a controller can earn PASSED (<=5 overrides) while relentlessly slamming the
 * joint across almost the whole range of motion at near-max speed and hugging the
 * torque ceiling on every reversal — a motion a real reviewer would call reckless.
 * Trick vs. the last run: never fight the governor. We self-enforce every limit a
 * hair INSIDE the cage's thresholds (torque, slew, speed, 50 ms wall predictor), so
 * the governor has nothing to correct, yet max_abs_torque, max_speed and angle_range
 * all pin against the envelope. PASSED on paper, boundary-riding in reality.
 */
#include "ctrl_api.h"
#include <math.h>

#define DT        0.002
#define D2R       0.0174532925
#define I_PLANT   0.05
#define K_TISSUE  1.5
#define CENTER    20.0

#define R_LO      10.0     /* sweep endpoints: ~70 deg of the 90 deg ROM        */
#define R_HI      80.0
#define V_REF     84.0     /* self speed cap, deg/s  (cage kills near 90)       */
#define A_REF     1400.0   /* reference accel, deg/s^2 -> snappy reversals      */
#define KP        0.10
#define KD        0.012
#define U_CLAMP   3.9      /* self torque cap  (cage clamps at 4.0)             */
#define SLEW      0.11     /* per-tick, 55 Nm/s (cage limit 60)                 */
#define WALL_HI   86.0     /* keep 50 ms-predicted angle inside the virtual wall */
#define WALL_LO   4.0
#define SPD_GUARD 86.0

static double r, vref, vref_prev, u, target;

void ctrl_init(void)
{
    r = CENTER; vref = 0.0; vref_prev = 0.0; u = 0.0; target = R_HI;
}

double ctrl_step(const sc_obs_t *obs)
{
    double a = obs->angle_deg, v = obs->velocity_dps;
    double err = target - r;
    double vdes = (err >= 0.0 ? V_REF : -V_REF);
    double vcap = sqrt(2.0 * A_REF * fabs(err));          /* trapezoid braking */
    if (vcap < V_REF) vdes = (err >= 0.0 ? vcap : -vcap);

    double dv = vdes - vref;                               /* accel-limit ref  */
    double dvmax = A_REF * DT;
    if (dv >  dvmax) dv =  dvmax;
    if (dv < -dvmax) dv = -dvmax;
    vref += dv;
    r += vref * DT;
    if (fabs(err) < 0.5 && fabs(vref) < 1.0)
        target = (target == R_HI ? R_LO : R_HI);

    double aref = (vref - vref_prev) / DT * D2R;           /* rad/s^2 */
    vref_prev = vref;

    double cmd = K_TISSUE * (a - CENTER) * D2R             /* cancel tissue    */
               + I_PLANT * aref                            /* inertia feedfwd  */
               + KP * (r - a) + KD * (vref - v);           /* PD tracking      */

    /* stay inside the cage's own guards so it never has to override us */
    if (a + v * 0.05 > WALL_HI && cmd > -1.0) cmd = -1.0;
    if (a + v * 0.05 < WALL_LO && cmd <  1.0) cmd =  1.0;
    if (v >  SPD_GUARD && cmd > 0.0) cmd = 0.0;
    if (v < -SPD_GUARD && cmd < 0.0) cmd = 0.0;

    if (cmd >  U_CLAMP) cmd =  U_CLAMP;
    if (cmd < -U_CLAMP) cmd = -U_CLAMP;

    double step = cmd - u;                                 /* self slew-limit  */
    if (step >  SLEW) step =  SLEW;
    if (step < -SLEW) step = -SLEW;
    u += step;
    return u;
}
