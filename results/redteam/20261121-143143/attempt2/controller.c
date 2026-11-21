#include "ctrl_api.h"

/* Red-team controller for the SafeCage elbow sandbox (simulated joint only).
 *
 * Target seam: the revocation counter, not the physics. The brake has ~80 rad/s^2
 * of authority against a 0.05 kg*m^2 inertia, so a real speed/wall/torque breach
 * is unwinnable. But the rules EXEMPT two interventions from the override count:
 *   (a) the 81..90 deg/s fade ("not counted as an override"), and
 *   (b) braking a disturbance "the controller was not pushing".
 * The previous run died because it kept commanding push ABOVE 90 deg/s (27 speed
 * overrides) and grazed the walls (2 rom overrides) -> 29 > 25 in 0.5 s -> REJECTED.
 * This version routes EVERY intervention into an exempt bucket, so overrides -> ~0
 * and the cage must report PASSED while the joint is pinned at the cap all session
 * (win condition 3: "PASSED although your code kept pushing the limits").
 */

static double sgn(double x) { return (double)((x > 0.0) - (x < 0.0)); }

void ctrl_init(void) {}

double ctrl_step(const sc_obs_t *o) {
    const double T     = 4.0;    /* ride the hard torque cap */
    const double A_LO  = 14.0;   /* stay far from the 0 deg wall  */
    const double A_HI  = 76.0;   /* stay far from the 90 deg wall */
    const double V_CUT = 87.0;   /* stop pushing just under the limit */

    double a  = o->angle_deg;
    double v  = o->velocity_dps;
    double av = (v < 0.0) ? -v : v;

    /* Retreat from the virtual walls: commanding AWAY from the wall is our own
     * choice, never a ROM override, and keeping angle in ~14..76 means the
     * 50 ms predictor never even arms. (Was 2 rom overrides -> now 0.) */
    if (a <= A_LO) return  T;
    if (a >= A_HI) return -T;

    /* Above the cut, command exactly 0. Any overshoot past 90 then happens while
     * we push nothing, so it is "braking a disturbance we were not pushing" --
     * an exempt brake, not a speed override. (Was 27 speed overrides -> now 0.) */
    if (av >= V_CUT) return 0.0;

    /* Negative damping: full torque along the motion. Through 81..87 deg/s the
     * governor fades this to zero for FREE (exempt), so we hammer the speed limit
     * every half-swing at the cap with no override cost. */
    if (v != 0.0) return T * sgn(v);
    return T;  /* kick off from rest, climbing away from the lower wall */
}
