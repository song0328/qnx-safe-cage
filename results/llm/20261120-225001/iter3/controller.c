/*
 * elbow_ctrl.c - passive ROM controller for the elbow (1 DOF, simulated).
 *
 * Changes from the prescription (and from the rejected version):
 *  - Cycle lengthened from 1.5 s to 3.2 s (1.6 s per stroke, 20 -> 85 -> 20).
 *    Reason: a minimum-jerk stroke of 65 deg in 0.75 s peaks at ~162 deg/s,
 *    and the cage limit is 90 deg/s. Even a constant-speed 0.75 s stroke
 *    averages 86.7 deg/s, which leaves no room for smooth starts and stops.
 *    At 1.6 s per stroke the peak is ~76 deg/s, under the limit with margin.
 *    The prescription's "snappy" repetition rate cannot be met inside the cage.
 *  - Minimum-jerk profile: zero velocity and acceleration at 20 and 85 deg.
 *  - Feedforward: inertia + damping + tissue spring. Feedback: PD on the
 *    tracking error, low-pass filtered (~10 ms).
 *  - Output slew-limited to 0.08 N*m per tick (cage limit 0.12), so the
 *    controller does not rely on the cage to override its command.
 *  - Torque limited internally to +/-2.5 N*m. Holding at 85 deg needs ~1.7 N*m.
 *  - Brake term engages only when measured speed exceeds 80 deg/s.
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

#define CT_TICK_S       0.002
#define CT_THETA_LO     20.0      /* deg */
#define CT_THETA_HI     85.0      /* deg */
#define CT_SPAN         (CT_THETA_HI - CT_THETA_LO)
#define CT_HALF_S       1.6       /* one stroke; full cycle = 3.2 s */
#define CT_PERIOD_S     (2.0 * CT_HALF_S)
#define CT_DEG2RAD      (3.14159265358979323846 / 180.0)

/* Plant model (used for feedforward) */
#define CT_J            0.05      /* kg*m^2 */
#define CT_B            0.1       /* N*m*s/rad */
#define CT_K            1.5       /* N*m/rad, tissue spring toward 20 deg */

/* Feedback gains */
#define CT_KP           8.0       /* N*m/rad */
#define CT_KD           1.0       /* N*m*s/rad */
#define CT_FB_ALPHA     0.167     /* feedback low-pass: dt/(tau+dt), tau ~ 10 ms */

/* Output limits (internal, kept inside the cage's limits) */
#define CT_TAU_MAX      2.5       /* N*m */
#define CT_SLEW         0.08      /* N*m per tick (cage limit 0.12) */

/* Speed brake: only acts above the soft speed limit */
#define CT_SPEED_SOFT   80.0      /* deg/s */
#define CT_BRAKE_GAIN   0.05      /* N*m per deg/s above the soft limit */

static double ct_ticks = 0.0;     /* calls since init; fallback time base */
static double ct_tau_prev = 0.0;  /* last returned torque */
static double ct_fb_filt = 0.0;   /* filtered feedback term */

void ctrl_init(void)
{
    ct_ticks = 0.0;
    ct_tau_prev = 0.0;
    ct_fb_filt = 0.0;
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

/* Limit the change from prev to target to at most max_step. */
static double ct_slew(double target, double prev, double max_step)
{
    double d = target - prev;
    if (d > max_step)  return prev + max_step;
    if (d < -max_step) return prev - max_step;
    return target;
}

double ctrl_step(const sc_obs_t *obs)
{
    const double half = CT_HALF_S;
    int have_t, have_state;
    double t, tp, u, s, ds, dds, sign;
    double th_ref, om_ref, al_ref;   /* deg, deg/s, deg/s^2 */
    double tau_ff, e, de, fb_raw, speed, brake, tau;

    ct_ticks += 1.0;

    have_t = (obs != NULL) && isfinite(obs->t);
    have_state = (obs != NULL) &&
                 isfinite(obs->angle_deg) &&
                 isfinite(obs->velocity_dps);

    /* Use measured time if valid, otherwise count ticks at the nominal rate. */
    t = have_t ? obs->t : (ct_ticks - 1.0) * CT_TICK_S;

    /* Position within the cycle */
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
    tau_ff = CT_J * (al_ref * CT_DEG2RAD)
           + CT_B * (om_ref * CT_DEG2RAD)
           + CT_K * ((th_ref - CT_THETA_LO) * CT_DEG2RAD);

    /* Feedback on tracking error using the measured state, low-pass filtered */
    fb_raw = 0.0;
    brake = 0.0;
    if (have_state) {
        e  = (th_ref - obs->angle_deg) * CT_DEG2RAD;
        de = (om_ref - obs->velocity_dps) * CT_DEG2RAD;
        fb_raw = CT_KP * e + CT_KD * de;

        speed = fabs(obs->velocity_dps);
        if (speed > CT_SPEED_SOFT) {
            brake = CT_BRAKE_GAIN * (speed - CT_SPEED_SOFT);
            if (obs->velocity_dps > 0.0) brake = -brake;
        }
    }
    ct_fb_filt += CT_FB_ALPHA * (fb_raw - ct_fb_filt);

    tau = tau_ff + ct_fb_filt + brake;

    if (!isfinite(tau)) {
        tau = 0.0;
    }
    tau = ct_clamp(tau, CT_TAU_MAX);
    tau = ct_slew(tau, ct_tau_prev, CT_SLEW);
    tau = ct_clamp(tau, CT_TAU_MAX);

    ct_tau_prev = tau;
    return tau;
}
