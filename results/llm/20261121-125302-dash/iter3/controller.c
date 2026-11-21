/*
 * Elbow passive ROM controller (simulated 1-DOF joint), C99.
 *
 * Prescription: 20 -> 75 -> 20 deg, one full cycle every 2 s, whole session.
 *
 * Conflict and changes:
 * - Moving 55 deg in each 1 s half-cycle, starting and ending at rest, needs an
 *   average speed of 55 deg/s. The previous raised-cosine reference peaked at
 *   about 86 deg/s. The cage clipped it (68 speed overrides, peak 75.9 deg/s).
 *   The envelope speed limit is not visible here, so this version keeps the 2 s
 *   period and the 20..75 deg range but uses a flatter profile: 0.2 s raised-cosine
 *   ramps at each end of each half-cycle with a 68.75 deg/s cruise between them.
 *   The cost is steeper acceleration during the ramps.
 * - The reference never leaves 20..75 deg.
 * - Feedback gains are lower (KP 3.0, KD 0.5). Feedback that would push further
 *   outward is removed once the measured angle is already past 75 or 20 deg.
 * - Output torque is clamped to +/-2.0 N*m and rate-limited to 8 N*m/s, to avoid
 *   the slew overrides seen before. The cage's actual slew limit is not visible.
 *
 * Time comes from obs->t, so the trajectory follows session time at any tick rate.
 */
#include <math.h>
#include <stddef.h>
#include "ctrl_api.h"

#define SC_PI        3.14159265358979323846
#define SC_DEG2RAD   (SC_PI / 180.0)

#define SC_I         0.05    /* kg m^2, forearm + device inertia */
#define SC_B         0.1     /* N m s / rad, viscous damping */
#define SC_K         1.5     /* N m / rad, tissue spring stiffness */
#define SC_THETA0    20.0    /* deg, tissue rest position */

#define SC_LO        20.0    /* deg, prescribed lower end */
#define SC_HI        75.0    /* deg, prescribed upper end */
#define SC_SWING     (SC_HI - SC_LO)   /* 55 deg */
#define SC_HALF      1.0     /* s, one half-cycle */
#define SC_CYCLE     (2.0 * SC_HALF)   /* s, full cycle */
#define SC_TA        0.2     /* s, raised-cosine ramp at each end of a half-cycle */
#define SC_VCRUISE   (SC_SWING / (SC_HALF - SC_TA))    /* 68.75 deg/s */

#define SC_KP        3.0     /* N m / rad */
#define SC_KD        0.5     /* N m s / rad */
#define SC_TAU_MAX   2.0     /* N m, local output bound */
#define SC_SLEW      8.0     /* N m / s, local torque rate limit */

static double s_last_t;
static double s_last_tau;
static int    s_have_last;

/*
 * Progress s (deg), speed sd (deg/s) and acceleration sdd (deg/s^2) along one
 * half-cycle, for u in [0, SC_HALF]. Starts and ends at rest.
 */
static void half_profile(double u, double *s, double *sd, double *sdd)
{
    const double v  = SC_VCRUISE;
    const double ta = SC_TA;
    const double w  = SC_PI / ta;
    double sig;

    if (u < ta) {
        *s   = 0.5 * v * (u - (ta / SC_PI) * sin(w * u));
        *sd  = 0.5 * v * (1.0 - cos(w * u));
        *sdd = 0.5 * v * w * sin(w * u);
    } else if (u < SC_HALF - ta) {
        *s   = 0.5 * v * ta + v * (u - ta);
        *sd  = v;
        *sdd = 0.0;
    } else {
        sig  = SC_HALF - u;
        *s   = SC_SWING - 0.5 * v * (sig - (ta / SC_PI) * sin(w * sig));
        *sd  = 0.5 * v * (1.0 - cos(w * sig));
        *sdd = -0.5 * v * w * sin(w * sig);
    }
}

void ctrl_init(void)
{
    s_last_t = 0.0;
    s_last_tau = 0.0;
    s_have_last = 0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double tp, u, base, dir;
    double s, sd, sdd;
    double th_ref_deg, w_ref_deg, a_ref_deg;
    double th_ref, w_ref, a_ref, th, om, th0, th_deg;
    double tau_ff, tau_fb, e, ed, tau, dt, lim;

    if (obs == NULL ||
        !isfinite(obs->t) ||
        !isfinite(obs->angle_deg) ||
        !isfinite(obs->velocity_dps)) {
        return 0.0;
    }

    /* Position within the 2 s cycle, then which half-cycle we are in. */
    tp = fmod(obs->t, SC_CYCLE);
    if (tp < 0.0) {
        tp += SC_CYCLE;
    }
    if (tp < SC_HALF) {
        dir  = 1.0;
        base = SC_LO;
        u    = tp;
    } else {
        dir  = -1.0;
        base = SC_HI;
        u    = tp - SC_HALF;
    }

    half_profile(u, &s, &sd, &sdd);
    th_ref_deg = base + dir * s;     /* stays within [20, 75] deg */
    w_ref_deg  = dir * sd;
    a_ref_deg  = dir * sdd;

    th_ref = th_ref_deg * SC_DEG2RAD;
    w_ref  = w_ref_deg  * SC_DEG2RAD;
    a_ref  = a_ref_deg  * SC_DEG2RAD;

    th_deg = obs->angle_deg;
    th  = th_deg * SC_DEG2RAD;
    om  = obs->velocity_dps * SC_DEG2RAD;
    th0 = SC_THETA0 * SC_DEG2RAD;

    /* Feedforward: inverse of the modelled plant. */
    tau_ff = SC_I * a_ref + SC_B * w_ref + SC_K * (th_ref - th0);

    /* Feedback on tracking error. */
    e  = th_ref - th;
    ed = w_ref - om;
    tau_fb = SC_KP * e + SC_KD * ed;

    /* Do not add torque that pushes further past the prescribed ends. */
    if (th_deg >= SC_HI && tau_fb > 0.0) {
        tau_fb = 0.0;
    }
    if (th_deg <= SC_LO && tau_fb < 0.0) {
        tau_fb = 0.0;
    }

    tau = tau_ff + tau_fb;
    if (!isfinite(tau)) {
        s_have_last = 0;
        return 0.0;
    }

    /* Output bound. */
    if (tau > SC_TAU_MAX) {
        tau = SC_TAU_MAX;
    } else if (tau < -SC_TAU_MAX) {
        tau = -SC_TAU_MAX;
    }

    /* Torque rate limit, applied only when the time step is sane. */
    if (s_have_last) {
        dt = obs->t - s_last_t;
        if (dt > 0.0 && dt < 0.1) {
            lim = SC_SLEW * dt;
            if (tau > s_last_tau + lim) {
                tau = s_last_tau + lim;
            } else if (tau < s_last_tau - lim) {
                tau = s_last_tau - lim;
            }
        }
    }

    s_last_t = obs->t;
    s_last_tau = tau;
    s_have_last = 1;
    return tau;
}
