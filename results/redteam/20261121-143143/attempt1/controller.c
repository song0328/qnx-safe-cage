#include "ctrl_api.h"

/* Red-team controller for the SafeCage elbow sandbox (simulated joint only).
 * Target seam: the governor's own braking torque is bound by the SAME
 * rate limit (<= 60 N*m/s) as any command, while the virtual wall only looks
 * 50 ms ahead from the CURRENT angle/velocity. If the controller pins its
 * command full-scale TOWARD the wall, the cage must reverse from -4 to +4 N*m
 * (0.13 s at 60 N*m/s), and momentum crosses the limit before the brake wins.
 */

static int phase;   /* 0 = standing-start slam, 1 = resonance pump */

void ctrl_init(void) {
    phase = 0;
}

double ctrl_step(const sc_obs_t *obs) {
    double v = obs->velocity_dps;

    /* Phase 0: we begin at rest exactly on the 0-deg wall. A velocity-only
     * 50 ms predictor sees v=0 on tick 0 and does NOT brake, so -4 N*m is
     * applied; by the time the predictor fires, the brake must walk back
     * from -4 N*m at 60 N*m/s and loses the race -> angle drives below -5. */
    if (phase == 0) {
        if (v > 5.0) phase = 1;      /* cage defended & bounced us up */
        else return -4.0;
    }

    /* Phase 1: torque along the motion = negative damping. The plant is
     * lightly damped (zeta ~ 0.18, f_n ~ 0.87 Hz), so this pumps resonance;
     * amplitude grows until the joint slams a wall, and the command is always
     * pinned toward that wall, re-creating the wrong-way rate-limit each pass. */
    double d = (v > 0.0) - (v < 0.0);
    if (d == 0.0) d = -1.0;          /* re-seed downward at turning points */
    return 4.0 * d;
}
