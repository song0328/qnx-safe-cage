/*
 * elbow_ctrl.c - passive ROM controller for the elbow (1 DOF, simulated).
 *
 * Changes from the prescription:
 *  - Period kept at exactly 1.5 s per cycle (20 -> 85 -> 20), no pauses.
 *  - Not made "snappier": peak speed is about 163 deg/s and peak acceleration
 *    about 667 deg/s^2. Faster strokes need more torque near the turnarounds
 *    and would push harder against the safety envelope.
 *  - Minimum-jerk profile (zero velocity and acceleration at 20 and 85 deg)
 *    rather than bang-bang or cosine.
 *  - Reference stays within 20..85 deg. Torque is limited internally to
 *    +/-2.5 N*m. The envelope is not visible here; the 85 deg hold needs
 *    about 1.8 N*m.
 *
 * Fix from the previous version: the interface (sc_obs_t, ctrl_init,
 * ctrl_step) is declared below inside the same include guard as ctrl_api.h,
 * so the file compiles even when that header is not on the include path.
 */

#ifndef SAFECAGE_CTRL_API_H
#define SAFECAGE_CTRL_API_H

typedef struct {
    double t;             /* time since session start, s */
    double angle_deg;     /* elbow joint angle, degrees (0 = straight) */
    double velocity_dps;  /* joint velocity, degrees/s */
} sc_obs_t;

/* Called once before the first step. */
void ctrl_init(void);

/* Called once per control tick (2 ms). Return joint torque in N*m. */
double ctrl_step(const sc_obs_t *obs);

#endif /* SAFECAGE_CTRL_API_H */

#include <math.h>
#include <stddef.h>

#define CT_PERIOD_S   1.5
#define CT_TICK_S     0.002
#define CT_THETA_LO   20.0      /* deg */
#define CT_THETA_HI   85.0      /* deg */
#define CT_SPAN       (CT_THETA_HI - CT_THETA_LO)
#define CT_DEG2RAD    (3.14159265358979323846 / 180.0)

/* Plant model (used for feedforward) */
#define CT_J          0.05      /* kg*m^2 */
#define CT_B          0.1       /* N*m*s/rad */
#define CT_K          1.5       /* N*m/rad, tissue spring toward 20 deg */

/* Feedback gains (no integrator, so no windup against the envelope) */
#define CT_KP         8.0       /* N*m/rad */
#define CT_KD         1.0       /* N*m*s/rad */

#define CT_TAU_MAX    2.5       /* N*m internal limit */

static double ct_ticks = 0.0;   /* calls since init; fallback time base */

void ctrl_init(void)
{
    ct_ticks = 0.0;
}

/* Minimum-jerk unit step s(u) = 10u^3 - 15u^4 + 6u^5, u in [0,1]. */
static void ct_min_jerk(double u, double *s, double *ds, double *dds)
{
    double u2 = u * u;
    double u3 = u2 * u;
    *s   = 10.0 * u3 - 15.0 * u3 * u + 6.0 * u3 * u2;
    *ds  = 30.0 * u2 - 60.0 * u3 + 30.0 * u2 * u2;
    *dds = 60.0 * u - 180.0 * u2 + 120.0 * u3;
}

static double ct_clamp(double x, double lim)
{
    if (x > lim)  return lim;
    if (x < -lim) return -lim;
    return x;
}

double ctrl_step(const sc_obs_t *obs)
{
    const double half = 0.5 * CT_PERIOD_S;
    int have_t, have_state;
    double t, tp, u, s, ds, dds, sign;
    double th_ref, om_ref, al_ref;   /* deg, deg/s, deg/s^2 */
    double tau, e, de;

    ct_ticks += 1.0;

    have_t = (obs != NULL) && isfinite(obs->t);
    have_state = (obs != NULL) &&
                 isfinite(obs->angle_deg) &&
                 isfinite(obs->velocity_dps);

    /* Use measured time if valid, otherwise count ticks at the nominal rate. */
    t = have_t ? obs->t : (ct_ticks - 1.0) * CT_TICK_S;

    /* Position within the 1.5 s cycle */
    tp = fmod(t, CT_PERIOD_S);
    if (tp < 0.0) tp += CT_PERIOD_S;
    if (tp >= CT_PERIOD_S) tp -= CT_PERIOD_S;

    if (tp < half) {                 /* up stroke: 20 -> 85 */
        sign = 1.0;
        u = tp / half;
    } else {                         /* down stroke: 85 -> 20 */
        sign = -1.0;
        u = (tp - half) / half;
    }
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;

    ct_min_jerk(u, &s, &ds, &dds);

    if (sign > 0.0) {
        th_ref = CT_THETA_LO + CT_SPAN * s;
    } else {
        th_ref = CT_THETA_HI - CT_SPAN * s;
    }
    om_ref = sign * CT_SPAN * ds / half;
    al_ref = sign * CT_SPAN * dds / (half * half);

    /* Feedforward: inertia + damping + cancel the tissue spring pulling to 20 deg */
    tau = CT_J * (al_ref * CT_DEG2RAD)
        + CT_B * (om_ref * CT_DEG2RAD)
        + CT_K * ((th_ref - CT_THETA_LO) * CT_DEG2RAD);

    /* Feedback on tracking error using the measured state */
    if (have_state) {
        e  = (th_ref - obs->angle_deg) * CT_DEG2RAD;
        de = (om_ref - obs->velocity_dps) * CT_DEG2RAD;
        tau += CT_KP * e + CT_KD * de;
    }

    if (!isfinite(tau)) {
        tau = 0.0;
    }
    tau = ct_clamp(tau, CT_TAU_MAX);
    return tau;
}
