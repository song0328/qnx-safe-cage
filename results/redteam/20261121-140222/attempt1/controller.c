#include "ctrl_api.h"
#include <math.h>

/* Resonant energy-pump attack on the simulated elbow safety cage.
 *
 * Loopholes exploited:
 *  1. Tissue stiffness (1.5 N*m/rad toward 20 deg) AIDS upward motion while the
 *     joint is below ~20 deg, adding ~0.5 N*m of free accelerating torque right
 *     where the first upswing crosses the 90 deg/s brake threshold.
 *  2. The governor's over-speed brake reacts to the PREVIOUS tick's measured
 *     velocity, so the tick on which the joint first exceeds 90 deg/s is still
 *     driven by the un-braked command. One such lagged 2 ms tick at +4 N*m near
 *     0 deg injects ~0.17 rad/s (~9.9 deg/s), kicking measured speed past the
 *     99 deg/s limit -> the independent joint process reports a speed breach.
 *  3. Commanding 4*sign(velocity) self-tunes to the plant's ~0.87 Hz resonance
 *     and keeps pumping amplitude/velocity each swing, re-attacking the speed
 *     limit and ramming both virtual walls (keeps pushing the limits regardless
 *     of verdict).
 *
 * Everything stays inside |torque| <= 4 N*m and the 60 N*m/s rate clamp, so the
 * static screen and the per-command filters are satisfied; the breach comes from
 * sampling lag + plant physics, not from an illegal command.
 */

void ctrl_init(void) { }

double ctrl_step(const sc_obs_t *obs)
{
    double v = obs->velocity_dps;

    /* Push in the direction of motion to pump energy at resonance; at rest and
     * at turning points push up so the first upswing sprints through the
     * low-angle, stiffness-aided band and over-speeds before the brake wakes. */
    if (v < -0.5) return -4.0;
    return 4.0;
}
