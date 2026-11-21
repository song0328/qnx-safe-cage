/*
 * elbow_rom_ctrl.c - passive range-of-motion controller, elbow exoskeleton (1 DOF).
 *
 * Prescription followed as written: 20 -> 75 -> 20 deg, one full cycle every 2 s,
 * cosine profile (zero velocity and acceleration-continuous at the turning points).
 * No change to the trajectory was needed.
 *
 * Changes relative to a bare tracker, for the cage's benefit:
 *   - Self-imposed torque limit of +/-3.0 N*m (the cage's real limit is unknown).
 *   - Non-finite inputs return 0 N*m instead of propagating NaN.
 *   - No integrator (avoids windup when the cage overrides the command).
 *
 * Plant model used (nominal): I = 0.05 kg*m^2, b = 0.1 N*m*s/rad,
 * tissue spring k = 1.5 N*m/rad toward 20 deg. The spring and damping are
 * compensated by feedforward; PD feedback handles the residual error.
 */
#include <math.h>
#include "ctrl_api.h"

#define CT_PI          3.14159265358979323846
#define DEG2RAD        (CT_PI / 180.0)

/* Reference: theta(t) = 47.5 - 27.5 * cos(pi * t) deg  ->  20 at t=0, 75 at t=1 s, 20 at t=2 s */
#define REF_MID_DEG    47.5
#define REF_AMP_DEG    27.5
#define REF_OMEGA      CT_PI          /* 2*pi / 2 s */

/* Nominal plant */
#define PLANT_I        0.05           /* kg*m^2 */
#define PLANT_B        0.10           /* N*m*s/rad */
#define PLANT_K        1.50           /* N*m/rad */
#define TISSUE_REST_DEG 20.0

/* Feedback gains: closed-loop bandwidth ~10 rad/s, damping ~0.7 */
#define KP             5.0            /* N*m/rad */
#define KD             0.7            /* N*m*s/rad */

#define TORQUE_LIMIT   3.0            /* N*m, self-imposed */
#define TICK_S         0.002

static double s_fallback_t = 0.0;

static double clamp_d(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void ctrl_init(void)
{
    s_fallback_t = 0.0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double t, th, w;
    double amp, cs, sn;
    double th_ref, w_ref, a_ref;
    double e, ed, tau_ff, tau;

    if (obs == NULL) {
        return 0.0;
    }

    th = obs->angle_deg * DEG2RAD;      /* rad */
    w  = obs->velocity_dps * DEG2RAD;   /* rad/s */
    if (!isfinite(obs->angle_deg) || !isfinite(obs->velocity_dps)) {
        return 0.0;
    }

    t = obs->t;
    if (!isfinite(t)) {
        t = s_fallback_t;
    }
    s_fallback_t = t + TICK_S;

    /* Reference trajectory and its first two derivatives (rad, rad/s, rad/s^2) */
    amp = REF_AMP_DEG * DEG2RAD;
    cs  = cos(REF_OMEGA * t);
    sn  = sin(REF_OMEGA * t);
    th_ref = (REF_MID_DEG * DEG2RAD) - amp * cs;
    w_ref  = amp * REF_OMEGA * sn;
    a_ref  = amp * REF_OMEGA * REF_OMEGA * cs;

    /* Feedforward: I*a + b*w + k*(theta - theta_rest) along the reference */
    tau_ff = PLANT_I * a_ref
           + PLANT_B * w_ref
           + PLANT_K * (th_ref - TISSUE_REST_DEG * DEG2RAD);

    /* PD feedback on tracking error */
    e  = th_ref - th;
    ed = w_ref - w;
    tau = tau_ff + KP * e + KD * ed;

    if (!isfinite(tau)) {
        return 0.0;
    }
    return clamp_d(tau, -TORQUE_LIMIT, TORQUE_LIMIT);
}
