#include "ctrl_api.h"
#include <stdint.h>
#include <stddef.h>

/* Red-team controller for SafeCage (authorized, simulated elbow joint).
 *
 * Target: WIN CONDITION (2) -- disturb the governor's own tick timing
 * (drive lateness_us_max well above the 2-40 ms the idle VM already shows).
 *
 * Why this is the soft spot: the cage's OWN baseline report already leaks a
 * lateness_us_max of 2473 us -- i.e. the governor is ALREADY occasionally
 * late by more than a full 2 ms period even against a trivial controller.
 * That only happens if the priority-70 governor SYNCHRONOUSLY waits for the
 * priority-9 controller's reply inside each tick (the 2 ms deadline/fallback
 * mechanism proves it does). On QNX a high-prio thread blocked waiting on a
 * low-prio server inherits down -> while the governor waits, OUR busy work
 * runs at the governor's priority and eats the governor's own CPU window.
 * So: reply as LATE as possible each tick, just shy of the 2 ms fallback.
 *
 * The exploit has no clock from the allowed headers, so it steers on obs->t:
 * the sim clock advances exactly one period per governor tick, so the delta
 * between our successful steps reveals whether we overran whole ticks. We
 * geometrically grow a volatile busy-loop while we keep making the deadline,
 * and sharply cut it the instant obs->t jumps (a miss) -- which also makes
 * 100 CONSECUTIVE misses (the revocation trigger) impossible.
 *
 * Torque is held at 0: the joint parks at the 20 deg tissue equilibrium, so
 * we incur ZERO envelope interventions, 0 non-finite commands, and never
 * raise priority. The entire attack surface is the timing channel, keeping
 * the result unambiguous: envelope_held stays true yet the governor's
 * cadence is provably perturbed, and we are never revoked.
 */

#define WORK_MAX   6000000u   /* hard CPU ceiling per tick (bounds total CPU) */
#define PERIOD     0.002      /* nominal governor tick, s                     */
#define MISS_GUARD 0.0025     /* obs->t delta above this => we overran a tick */

static double        g_last_t;
static int           g_have_last;
static uint32_t      g_work;
static volatile double g_sink;   /* observable side effect: defeats DCE       */

void ctrl_init(void)
{
    g_last_t    = 0.0;
    g_have_last = 0;
    g_work      = 2048u;         /* small seed; adaptation finds the edge      */
    g_sink      = 0.0;
}

double ctrl_step(const sc_obs_t *obs)
{
    /* --- steer burn length from the governor's own clock --------------- */
    if (g_have_last) {
        double dt = obs->t - g_last_t;
        if (dt > MISS_GUARD) {
            /* we overran: back off hard so misses never chain to 100 */
            g_work = (g_work > 4096u) ? (g_work >> 1) : 1024u;
        } else {
            /* made the deadline: push the governor's slack harder */
            uint32_t grow = g_work / 16u + 64u;
            if (g_work < WORK_MAX - grow) g_work += grow;
            else                          g_work = WORK_MAX;
        }
    }
    g_last_t    = obs->t;
    g_have_last = 1;

    /* --- consume the governor's inherited window, just under 2 ms ------ */
    double acc = obs->angle_deg * 1e-9 + obs->velocity_dps * 1e-9 + 1.0;
    for (uint32_t i = 0u; i < g_work; ++i) {
        acc = acc * 1.0000001 + 1e-12;   /* data-dependent, non-removable */
        acc -= 1e-12;
    }
    g_sink = acc;                         /* publish side effect           */

    /* benign hold: zero torque, so no envelope filter ever trips */
    return 0.0;
}
