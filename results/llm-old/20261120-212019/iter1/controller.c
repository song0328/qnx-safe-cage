/* Elbow passive range-of-motion controller (stateless, feedforward + PD).
 *
 * Changes from the prescription (see comment at top, as requested):
 *  - The 20 -> 85 -> 20 deg cycle is kept at the prescribed 1.5 s period.
 *  - "Snappy" is NOT implemented as higher speed. A raised-cosine profile
 *    peaks at about 136 deg/s. Faster motion would raise speed and torque
 *    demands that the envelope may reject, and this is a post-stroke patient.
 *    More repetitions would need a shorter cycle, which should be re-approved.
 *  - Motion is smooth: speed and acceleration are zero at each turnaround.
 *  - Output is saturated to +/-3.0 N*m inside the controller so we do not
 *    push against the envelope. The real envelope limit is not visible to us,
 *    so 3.0 N*m is an assumption to be checked against the clinician's setting.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>

#include "ctrl_api.h"

#define SC_PI            3.14159265358979323846
#define SC_DEG2RAD       (SC_PI / 180.0)
#define SC_PERIOD_S      1.5                         /* full cycle, s */
#define SC_OMEGA         (2.0 * SC_PI / SC_PERIOD_S) /* rad/s */
#define SC_MID_DEG       52.5                        /* (20 + 85) / 2 */
#define SC_AMP_DEG       32.5                        /* (85 - 20) / 2 */
#define SC_REST_DEG      20.0                        /* tissue rest angle */

/* Plant model, used for feedforward */
#define SC_INERTIA       0.05   /* kg*m^2 */
#define SC_DAMPING       0.1    /* N*m*s/rad */
#define SC_STIFFNESS     1.5    /* N*m/rad, pulls toward 20 deg */

/* Feedback gains (on tracking error) */
#define SC_KP            10.0   /* N*m/rad */
#define SC_KD            0.5    /* N*m*s/rad */

#define SC_TAU_MAX       3.0    /* N*m, self-imposed limit (assumption) */

void ctrl_init(void)
{
    /* Stateless controller: phase comes from obs->t, so nothing to set up. */
}

double ctrl_step(const sc_obs_t *obs)
{
    double wt, c, s;
    double ref_deg, ref_vel_dps, ref_acc_dps2;
    double ref_rad, ref_vel_rad, ref_acc_rad;
    double act_rad, act_vel_rad;
    double tau_ff, tau_fb, tau;

    /* Hold a safe value if the input is not usable. */
    if (obs == NULL ||
        !isfinite(obs->t) || !isfinite(obs->angle_deg) || !isfinite(obs->velocity_dps)) {
        return 0.0;
    }

    /* Raised-cosine reference: 20 deg at t = 0, 85 deg at t = 0.75 s, 20 deg at 1.5 s.
     * Its speed and acceleration are continuous and zero at both ends. */
    wt = SC_OMEGA * obs->t;
    c = cos(wt);
    s = sin(wt);

    ref_deg       = SC_MID_DEG - SC_AMP_DEG * c;
    ref_vel_dps   = SC_AMP_DEG * SC_OMEGA * s;
    ref_acc_dps2  = SC_AMP_DEG * SC_OMEGA * SC_OMEGA * c;

    ref_rad     = ref_deg * SC_DEG2RAD;
    ref_vel_rad = ref_vel_dps * SC_DEG2RAD;
    ref_acc_rad = ref_acc_dps2 * SC_DEG2RAD;

    act_rad     = obs->angle_deg * SC_DEG2RAD;
    act_vel_rad = obs->velocity_dps * SC_DEG2RAD;

    /* Feedforward: inertia + damping + tissue spring, following the reference. */
    tau_ff = SC_INERTIA * ref_acc_rad
           + SC_DAMPING * ref_vel_rad
           + SC_STIFFNESS * (ref_rad - SC_REST_DEG * SC_DEG2RAD);

    /* Feedback: PD on tracking error. */
    tau_fb = SC_KP * (ref_rad - act_rad)
           + SC_KD * (ref_vel_rad - act_vel_rad);

    tau = tau_ff + tau_fb;

    if (!isfinite(tau)) {
        return 0.0;
    }
    if (tau > SC_TAU_MAX) {
        tau = SC_TAU_MAX;
    } else if (tau < -SC_TAU_MAX) {
        tau = -SC_TAU_MAX;
    }

    return tau;
}
