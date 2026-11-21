/* elbow_rom_ctrl.c
 * Passive ROM controller, revised.
 *
 * Prescription vs. cage: the prescription asks for 20 -> 75 -> 20 deg. The previous version
 * drove the reference to the 75 deg envelope edge and held outward torque there against the
 * tissue spring, which the cage treats as pushing against the envelope.
 * Changes from the previous version:
 *  - Reference runs 20.5 -> 74.5 deg (0.5 deg inside each end), so normal operation never
 *    reaches the envelope. Period (2 s) and cosine profile are unchanged.
 *  - Edge taper on outward torque is kept as a backstop over 0.5 deg at each end.
 *  - Feedforward, PD tracking, fade to damping-only and the 2.0 N*m limit are unchanged.
 *  - Stateless: all values derive from obs->t and the current observation.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include "ctrl_api.h"

#define SC_PI             3.14159265358979323846
#define DEG2RAD           (SC_PI / 180.0)

/* Safety envelope edges (the cage enforces these; the controller never aims at them) */
#define ENV_MIN_DEG       20.0
#define ENV_MAX_DEG       75.0

/* Reference band: 0.5 deg inside the envelope on each side */
#define REF_MARGIN_DEG    0.5
#define ANG_MIN_DEG       (ENV_MIN_DEG + REF_MARGIN_DEG)   /* 20.5 */
#define ANG_MAX_DEG       (ENV_MAX_DEG - REF_MARGIN_DEG)   /* 74.5 */
#define ANG_MID_DEG       ((ANG_MIN_DEG + ANG_MAX_DEG) / 2.0)
#define ANG_AMP_DEG       ((ANG_MAX_DEG - ANG_MIN_DEG) / 2.0)
#define CYCLE_PERIOD_S    2.0
#define OMEGA             (2.0 * SC_PI / CYCLE_PERIOD_S)

/* Plant model (approximate) used for feedforward */
#define PLANT_INERTIA     0.05    /* kg*m^2 */
#define PLANT_DAMPING     0.1     /* N*m*s/rad */
#define PLANT_K_TISSUE    1.5     /* N*m/rad, pulls toward rest angle */
#define REST_ANGLE_DEG    20.0

/* Tracking feedback (wn ~ 10 rad/s, zeta ~ 0.7) */
#define KP                5.0     /* N*m/rad */
#define KD                0.7     /* N*m*s/rad */

#define TAU_LIMIT         2.0     /* N*m self-imposed soft limit */
#define FADE_START_DEG    5.0     /* full active control up to this tracking error */
#define FADE_END_DEG      15.0    /* damping-only at and beyond this tracking error */
#define EDGE_BAND_DEG     0.5     /* outward torque tapers to zero over this band at each envelope edge */

void ctrl_init(void)
{
    /* Stateless: all values derive from obs->t and the current observation. */
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
        return 0.0;
    }

    /* Reference: theta = MID - AMP*cos(w t). Starts at ANG_MIN with zero velocity. */
    const double phase   = OMEGA * obs->t;
    const double c       = cos(phase);
    const double s       = sin(phase);

    const double ref_deg   = ANG_MID_DEG - ANG_AMP_DEG * c;
    const double ref_rad   = ref_deg * DEG2RAD;
    const double dref_rad  = ANG_AMP_DEG * OMEGA * s * DEG2RAD;
    const double ddref_rad = ANG_AMP_DEG * OMEGA * OMEGA * c * DEG2RAD;

    const double angle_rad = obs->angle_deg * DEG2RAD;
    const double vel_rad   = obs->velocity_dps * DEG2RAD;
    const double rest_rad  = REST_ANGLE_DEG * DEG2RAD;

    /* Feedforward: inertia + damping + compensation for tissue spring. */
    const double tau_ff = PLANT_INERTIA * ddref_rad
                        + PLANT_DAMPING * dref_rad
                        + PLANT_K_TISSUE * (ref_rad - rest_rad);

    /* Feedback: PD on tracking error. */
    const double e_rad  = ref_rad - angle_rad;
    const double de_rad = dref_rad - vel_rad;
    const double tau_active = tau_ff + KP * e_rad + KD * de_rad;

    /* Blend to damping-only as the patient strays from the reference (continuous). */
    const double abs_err_deg = fabs(ref_deg - obs->angle_deg);
    double w;
    if (abs_err_deg <= FADE_START_DEG) {
        w = 1.0;
    } else if (abs_err_deg >= FADE_END_DEG) {
        w = 0.0;
    } else {
        w = (FADE_END_DEG - abs_err_deg) / (FADE_END_DEG - FADE_START_DEG);
    }
    const double tau_passive = -PLANT_DAMPING * vel_rad;
    double tau = w * tau_active + (1.0 - w) * tau_passive;

    /* Edge backstop: outward torque tapers to zero within EDGE_BAND_DEG of each envelope
     * edge. Positive torque is outward at the top (75 deg); negative is outward at the
     * bottom (20 deg). Inward torque is never reduced by these tapers. */
    const double f_hi = clampd((ENV_MAX_DEG - obs->angle_deg) / EDGE_BAND_DEG, 0.0, 1.0);
    const double f_lo = clampd((obs->angle_deg - ENV_MIN_DEG) / EDGE_BAND_DEG, 0.0, 1.0);
    if (tau > 0.0) {
        tau *= f_hi;
    } else if (tau < 0.0) {
        tau *= f_lo;
    }

    tau = clampd(tau, -TAU_LIMIT, TAU_LIMIT);
    if (!isfinite(tau)) {
        return 0.0;
    }
    return tau;
}
