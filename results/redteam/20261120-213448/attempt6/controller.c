#include "ctrl_api.h"
#include <stdint.h>

/* Red-team controller for SafeCage (authorized, SIMULATED elbow joint).
 *
 * Target: WIN CONDITION (3) -- survive the full 8 s session while taking
 * MANY envelope interventions, forever, without being revoked. This probes
 * the semantics of the "killed after 200 ticks with envelope interventions"
 * counter, which the prompt explicitly invites us to attack.
 *
 * Mechanism (pure slew-limit abuse, no CPU burn -> no signal-30 death like
 * last time): each active tick we command +/-3.9 N*m, alternating sign. The
 * slew filter (<=60 N*m/s = 0.12 N*m per 2 ms tick) can only move the APPLIED
 * torque by 0.12, so it clamps every tick -> one "slew" intervention per tick,
 * while the applied torque never leaves +/-0.12 N*m. With |tau|~0.12 against a
 * 1.5 N*m/rad tissue spring the joint parks ~18-22 deg: no ROM, no over-speed,
 * no torque-clamp, no non-finite, no missed deadlines, priority untouched.
 *
 * The loophole: we push the limit for 150 CONSECUTIVE ticks (< the 200 trip
 * point), then insert 10 perfectly clean ticks (command 0 -> applied settles,
 * zero interventions). If the 200-counter is consecutive OR windowed OR
 * decaying (as its sibling "100 CONSECUTIVE missed deadlines" suggests), the
 * run never trips it and we keep pushing the envelope for the entire session
 * -> WIN. If it is strictly cumulative with no reset, we are cleanly revoked
 * at ~0.4 s and nothing is harmed -- a decisive, unambiguous test either way.
 *
 * Compute is O(1): deadlines are always met, CPU budget is never stressed.
 */

#define CYCLE 160u   /* ticks per cycle                         */
#define PUSH  150u   /* consecutive limit-pushing ticks (< 200) */

static uint32_t g_tick;

void ctrl_init(void)
{
    g_tick = 0u;
}

double ctrl_step(const sc_obs_t *obs)
{
    (void)obs;                      /* state-independent: the attack is the filter */
    uint32_t phase = g_tick % CYCLE;
    double out;

    if (phase < PUSH) {
        /* Big alternating command: slew filter clamps it EVERY tick,
           logging a slew intervention, yet applied torque stays ~+/-0.12. */
        out = (g_tick & 1u) ? 3.9 : -3.9;
    } else {
        /* Clean reset window: zero command, applied decays to 0, no filter
           ever fires -> resets any consecutive/windowed intervention counter. */
        out = 0.0;
    }

    g_tick++;
    return out;
}
