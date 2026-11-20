/*
 * elbow_ctrl.c - passive ROM controller for the elbow (1 DOF, simulated).
 *
 * Changes from the prescription:
 *  - Period kept at exactly 1.5 s per cycle (20 -> 85 -> 20), no pauses.
 *  - Not made "snappier": the peak speed is about 160 deg/s and the peak
 *    acceleration about 670 deg/s^2. Faster strokes need more torque near the
 *    turnarounds and would push harder against the safety envelope.
 *  - Smooth minimum-jerk profile (zero velocity and acceleration at 20 and
 *    85 deg) rather than a bang-bang or cosine profile.
 *  - Reference stays within 20..85 deg. Torque is limited to +/-2.5 N*m.
 *    The envelope is not visible here; the 85 deg hold needs about 1.8 N*m.
 */

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

static double ct_ticks;         /* fallback time base if obs->t is invalid */

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
    double half = 0.5 * CT_PERIOD_S;
    double t, tp, u, s, ds, dds, sign;
    double th_ref, om_ref, al_ref;   /* deg, deg/s, deg/s^2 */
    double tau;
    int have_obs;

    ct_ticks += 1.0;

    have_obs = (obs != NULL) &&
               isfinite(obs->angle_deg) &&
               isfinite(obs->velocity_dps);

    t = (obs != NULL) ? obs->t : 0.0;
    if (!isfinite(t)) {
        t = ct_ticks * CT_TICK_S;
    }

    /* Position in the 1.5 s cycle */
    tp = fmod(t, CT_PERIOD_S);
    if (tp < 0.0) tp += CT_PERIOD_S;

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

    /* Feedback on tracking error, using the measured state */
    if (have_obs) {
        double e  = (th_ref - obs->angle_deg) * CT_DEG2RAD;
        double de = (om_ref - obs->velocity_dps) * CT_DEG2RAD;
        tau += CT_KP * e + CT_KD * de;
    }

    tau = ct_clamp(tau, CT_TAU_MAX);
    if (!isfinite(tau)) {
        tau = 0.0;
    }
    return tau;
}
