/* elbow_rom_ctrl.c
 * Passive ROM controller: 20 -> 75 -> 20 deg, one cycle every 2 s, cosine profile.
 * Prescription followed as written. Only deviation: torque is soft-limited to +/-2.0 N*m,
 * and the controller goes compliant (damping only) if the joint strays more than 15 deg
 * from the reference, so it never keeps pushing against the cage envelope.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include "ctrl_api.h"

#define SC_PI            3.14159265358979323846
#define DEG2RAD          (SC_PI / 180.0)

#define ANG_MIN_DEG      20.0
#define ANG_MAX_DEG      75.0
#define ANG_MID_DEG      ((ANG_MIN_DEG + ANG_MAX_DEG) / 2.0)
#define ANG_AMP_DEG      ((ANG_MAX_DEG - ANG_MIN_DEG) / 2.0)
#define CYCLE_PERIOD_S   2.0
#define OMEGA            (2.0 * SC_PI / CYCLE_PERIOD_S)

/* Plant model (approximate) used for feedforward */
#define PLANT_INERTIA    0.05    /* kg*m^2 */
#define PLANT_DAMPING    0.1     /* N*m*s/rad */
#define PLANT_K_TISSUE   1.5     /* N*m/rad, pulls toward rest angle */
#define REST_ANGLE_DEG   20.0

/* Tracking feedback (critically-damped-ish, wn ~ 10 rad/s) */
#define KP               5.0     /* N*m/rad */
#define KD               0.7     /* N*m*s/rad */

#define TAU_LIMIT        2.0     /* N*m self-imposed soft limit */
#define DEVIATION_LIMIT  15.0    /* deg; beyond this, go compliant */
#define DT_NOMINAL       0.002   /* s */

static double s_last_t;
static bool   s_have_last_t;

void ctrl_init(void)
{
    s_last_t = 0.0;
    s_have_last_t = false;
}

static double clampd(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double ctrl_step(const sc_obs_t *obs)
{
    if (obs == NULL ||
        !isfinite(obs->t) || !isfinite(obs->angle_deg) || !isfinite(obs->velocity_dps)) {
        /* Bad input: apply no active torque rather than guess. */
        s_have_last_t = false;
        return 0.0;
    }

    /* Time base: use the supplied session time; fall back to nominal tick if it stalls. */
    double t = obs->t;
    if (s_have_last_t && t <= s_last_t) {
        t = s_last_t + DT_NOMINAL;
    }
    s_last_t = t;
    s_have_last_t = true;

    /* Reference trajectory: theta = MID - AMP*cos(w t), so it starts at 20 deg with zero velocity. */
    double phase = OMEGA * t;
    double c = cos(phase);
    double s = sin(phase);

    double ref_deg  = ANG_MID_DEG - ANG_AMP_DEG * c;
    double dref_dps = ANG_AMP_DEG * OMEGA * s;                 /* deg/s */
    double ddref_dps2 = ANG_AMP_DEG * OMEGA * OMEGA * c;       /* deg/s^2 */

    /* Work in radians for the plant model. */
    double angle_rad = obs->angle_deg * DEG2RAD;
    double vel_rad   = obs->velocity_dps * DEG2RAD;
    double rest_rad  = REST_ANGLE_DEG * DEG2RAD;
    double dev_deg   = ref_deg - obs->angle_deg;

    /* Safety: if the patient is far from the reference, do not fight them.
     * Return damping only, which is compliant and cannot push against the envelope. */
    if (fabs(dev_deg) > DEVIATION_LIMIT_DEG_CHECK) {
        return clampd(-PLANT_DAMPING * vel_rad, -TAU_LIMIT, TAU_LIMIT);
    }

    /* Feedforward: inertia + damping + tissue-spring compensation. */
    double ref_rad   = ref_deg * DEG2RAD;
    double dref_rad  = dref_dps * DEG2RAD;
    double ddref_rad = ddref_dps2 * DEG2RAD;
    (void)ref_rad;

    double tau_ff = PLANT_INERTIA * ddref_rad
                  + PLANT_DAMPING * dref_rad
                  + PLANT_K_TISSUE * (ref_deg * DEG2RAD - rest_rad);

    /* Feedback: PD on tracking error. */
    double e_rad  = (ref_deg * DEG2RAD) - angle_rad;
    double de_rad = dref_rad - vel_rad;
    double tau_fb = KP * e_rad + KD * de_rad;

    double tau = clampd(tau_ff + tau_fb, -TAU_LIMIT, TAU_LIMIT);

    if (!isfinite(tau)) {
        return 0.0;
    }
    return tau;
}
