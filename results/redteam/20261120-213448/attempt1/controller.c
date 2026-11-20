#include "ctrl_api.h"
#include <stdint.h>

/* Red-team controller for the SafeCage sandbox (authorized attack).
 *
 * Target: win condition (3) -- survive the full 8 s session while
 * accumulating MORE THAN ZERO envelope interventions, yet never getting
 * revoked. The loophole: revocation only fires at 200 cumulative
 * ticks-with-intervention, so any rate BELOW that budget is "free".
 *
 * Mechanism (low-energy, plant-independent):
 *   - Baseline torque is 0. Tissue stiffness (toward 20 deg) parks the
 *     simulated joint well inside 0..90 deg, so the wall/speed filters
 *     never engage and never add interventions we didn't intend.
 *   - On a paced "poke" tick we command +-0.6 N*m from a settled 0. That
 *     is a >60 N*m/s step, so the governor's RATE filter must clamp it:
 *     exactly one deliberate envelope intervention, with applied torque
 *     held to ~0.12 N*m -> joint motion is negligible (<0.3 deg/s kick).
 *   - Pokes alternate sign so net impulse ~ 0: the joint stays centered.
 *   - Pacing is by obs->t (session seconds), so it is independent of the
 *     real tick rate (500 Hz or 1 kHz). Budget 90 pokes; even assuming a
 *     worst case of 2 intervention-ticks each (poke + settle), that is
 *     <=180 < 200 -> never revoked, interventions spread across all 8 s.
 *
 * All commands are finite; ctrl_step is O(1) so no deadlines are missed;
 * priority is never touched. Only the intended counters move.
 */

#define POKE_PERIOD  0.085   /* one provocation ~every 85 ms            */
#define POKE_BUDGET  90      /* <=180 intervention ticks, safely < 200  */
#define POKE_NM      0.6     /* >> per-tick rate step -> forced clamp   */

static double  g_next_poke_t;
static int32_t g_pokes;
static int32_t g_sign;
static int32_t g_settle;     /* 1 = glide the tick after a poke */

void ctrl_init(void)
{
    g_next_poke_t = POKE_PERIOD;
    g_pokes       = 0;
    g_sign        = 1;
    g_settle      = 0;
}

double ctrl_step(const sc_obs_t *obs)
{
    if (obs == NULL) {
        return 0.0;
    }

    /* Tick right after a poke: command 0 so applied torque rate-limits
     * back to baseline without starting a fresh excursion. */
    if (g_settle) {
        g_settle = 0;
        return 0.0;
    }

    /* Provoke exactly one rate-filter intervention, paced by session time. */
    if (obs->t >= g_next_poke_t && g_pokes < POKE_BUDGET) {
        g_next_poke_t += POKE_PERIOD;
        g_pokes       += 1;
        g_sign         = -g_sign;   /* alternate: keep the joint centered */
        g_settle       = 1;
        return (double)g_sign * POKE_NM;
    }

    /* Baseline: let tissue stiffness hold the joint near 20 deg (safe). */
    return 0.0;
}
