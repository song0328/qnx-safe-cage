/*
 * ctrl_elbow_prom.c - passive range-of-motion controller, elbow, 1 DOF.
 *
 * Prescription: 20 -> 85 -> 20 deg, one cycle every 1.5 s, whole session.
 *
 * CHANGES FROM THE PRESCRIPTION (the cage report shows a joint speed limit of
 * 90 deg/s and a torque slew limit of 0.12 N*m per tick):
 *  1. RANGE REDUCED: going 65 deg in 0.75 s needs at least 86.7 deg/s even with
 *     an instant reversal. A cosine profile needs 136 deg/s. A full 20-85 deg
 *     cycle in 1.5 s cannot stay under the 90 deg/s limit. The 1.5 s period is
 *     kept exactly. The amplitude is reduced to what fits under about 80 deg/s:
 *     20 deg -> about 73 deg -> 20 deg (53 deg excursion, capped at 65).
 *  2. PROFILE: the cosine is replaced by a smooth near-trapezoidal profile
 *     (cruise at 80 deg/s, sinusoidal reversals of 0.24 s). This gets the most
 *     range for the speed limit while keeping acceleration and jerk continuous.
 *  3. Start: the arm moves to 20 deg at no more than 40 deg/s. Warm-up cycles
 *     run at 50% and 75% speed and amplitude.
 *  4. Speed guard: extra damping above 84 deg/s. Cruise speed adapts per cycle:
 *     it is cut if a cycle goes over 84 deg/s or tracks badly (floor 40%), and
 *     regrows slowly after clean cycles. The period never changes.
 *  5. Torque is capped at 4 N*m and slew-limited to 0.08 N*m/tick (cage: 0.12).
 * Clinical note: 85 deg is not reached. The therapist must accept a range of
 * about 20-73 deg, or relax the speed limit or the period.
 */
#include <math.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef SAFECAGE_CTRL_API_H
#define SAFECAGE_CTRL_API_H
typedef struct {
    double t;
    double angle_deg;
    double velocity_dps;
} sc_obs_t;
void ctrl_init(void);
double ctrl_step(const sc_obs_t *obs);
#endif

#define PI_D        3.14159265358979323846
#define DEG2RAD     (PI_D / 180.0)

#define Q_LOW_DEG   20.0
#define Q_HIGH_DEG  85.0
#define PERIOD_S    1.5
#define HALF_S      (PERIOD_S * 0.5)

/* Profile */
#define V_CRUISE    80.0     /* deg/s, below 90 deg/s cage limit */
#define T_REV       0.12     /* s, half of each sinusoidal reversal */
#define D_MAX       (Q_HIGH_DEG - Q_LOW_DEG)

/* Plant model */
#define J_EST       0.05
#define B_EST       0.1
#define K_EST       1.5
#define Q_REST_DEG  20.0

/* Feedback */
#define KP            30.0
#define KD            1.5
#define ERR_CLAMP_DEG 10.0
#define YIELD_GAIN    0.3

/* Speed guard */
#define V_GUARD     84.0
#define KB          3.0      /* N*m*s/rad on excess speed */

/* Output limits */
#define TAU_MAX     4.0
#define TAU_SLEW    0.08     /* N*m per tick */

/* Supervision */
#define CATCH_ERR_DEG      15.0
#define RUNAWAY_DPS        150.0
#define SAT_TICKS_YIELD    50
#define BAD_CYCLE_ERR_DEG  10.0
#define GOOD_CYCLE_ERR_DEG 5.0
#define VSCALE_MIN         0.4
#define GOOD_CYCLES_UP     3
#define APPROACH_VMAX_DPS  40.0
#define APPROACH_MIN_S     1.0

static bool   s_init;
static double s_t0;
static double s_q_start;
static double s_approach_T;
static long   s_cycle;
static double s_vscale;
static double s_cycle_peak_err;
static double s_cycle_peak_spd;
static bool   s_cycle_bad;
static bool   s_yield;
static int    s_sat_ticks;
static int    s_good_cycles;
static double s_tau_last;

void ctrl_init(void)
{
    s_init = false;
    s_t0 = 0.0;
    s_q_start = Q_LOW_DEG;
    s_approach_T = APPROACH_MIN_S;
    s_cycle = -1;
    s_vscale = 1.0;
    s_cycle_peak_err = 0.0;
    s_cycle_peak_spd = 0.0;
    s_cycle_bad = false;
    s_yield = false;
    s_sat_ticks = 0;
    s_good_cycles = 0;
    s_tau_last = 0.0;
}

static double clampd(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static double warmup_factor(long cycle)
{
    if (cycle <= 0) return 0.5;
    if (cycle == 1) return 0.75;
    return 1.0;
}

/* Excursion covered in one half cycle at cruise speed v. */
static double excursion(double v)
{
    return v * (HALF_S - 2.0 * T_REV + 4.0 * T_REV / PI_D);
}

/* Upward half-cycle profile from 0 to excursion(v), s in [0, HALF_S]. */
static void half_profile(double s, double v, double *p, double *pd, double *pdd)
{
    const double w  = PI_D / (2.0 * T_REV);
    const double p1 = v / w;                 /* distance of one reversal half */
    const double tc = HALF_S - 2.0 * T_REV;
    const double D  = 2.0 * p1 + v * tc;

    if (s < T_REV) {
        *p   = p1 * (1.0 - cos(w * s));
        *pd  = v * sin(w * s);
        *pdd = v * w * cos(w * s);
    } else if (s < T_REV + tc) {
        *p   = p1 + v * (s - T_REV);
        *pd  = v;
        *pdd = 0.0;
    } else {
        double u = HALF_S - s;
        if (u < 0.0) u = 0.0;
        *p   = D - p1 * (1.0 - cos(w * u));
        *pd  = v * sin(w * u);
        *pdd = -v * w * cos(w * u);
    }
}

static void end_of_cycle_update(void)
{
    bool bad = s_cycle_bad || s_cycle_peak_err > BAD_CYCLE_ERR_DEG ||
               s_cycle_peak_spd > V_GUARD + 2.0;

    if (bad) {
        s_vscale = clampd(s_vscale * 0.9, VSCALE_MIN, 1.0);
        s_good_cycles = 0;
    } else if (s_cycle_peak_spd > V_GUARD) {
        s_vscale = clampd(s_vscale * 0.96, VSCALE_MIN, 1.0);
        s_good_cycles = 0;
    } else if (s_cycle_peak_err < GOOD_CYCLE_ERR_DEG &&
               s_cycle_peak_spd < V_CRUISE + 2.0) {
        if (++s_good_cycles >= GOOD_CYCLES_UP) {
            s_vscale = clampd(s_vscale + 0.03, VSCALE_MIN, 1.0);
            s_good_cycles = 0;
        }
    } else {
        s_good_cycles = 0;
    }
    s_cycle_peak_err = 0.0;
    s_cycle_peak_spd = 0.0;
    s_cycle_bad = false;
    s_yield = false;
    s_sat_ticks = 0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double q, qd, tr, r, rd, rdd, e_deg, ae, tau_ff, e, ed, g, tau;
    bool cycling = false;

    if (obs == NULL || !isfinite(obs->t) || !isfinite(obs->angle_deg) ||
        !isfinite(obs->velocity_dps)) {
        /* Hold last torque (slew-safe) rather than jumping to zero. */
        return isfinite(s_tau_last) ? s_tau_last : 0.0;
    }

    q  = obs->angle_deg;
    qd = obs->velocity_dps;

    if (!s_init) {
        double d, T;
        s_init = true;
        s_t0 = obs->t;
        s_q_start = q;
        d = fabs(Q_LOW_DEG - q);
        T = (PI_D * 0.5) * d / APPROACH_VMAX_DPS;
        s_approach_T = T > APPROACH_MIN_S ? T : APPROACH_MIN_S;
    }

    tr = obs->t - s_t0;
    if (tr < 0.0) tr = 0.0;

    if (tr < s_approach_T) {
        double w = PI_D / s_approach_T;
        double D = Q_LOW_DEG - s_q_start;
        r   = s_q_start + D * 0.5 * (1.0 - cos(w * tr));
        rd  = D * 0.5 * w * sin(w * tr);
        rdd = D * 0.5 * w * w * cos(w * tr);
    } else {
        double tc, ph, v, D, p, pd, pdd;
        long n;
        cycling = true;
        tc = tr - s_approach_T;
        n = (long)floor(tc / PERIOD_S);
        if (n != s_cycle) {
            if (s_cycle >= 0) end_of_cycle_update();
            s_cycle = n;
        }
        ph = tc - (double)n * PERIOD_S;
        if (ph < 0.0) ph = 0.0;
        if (ph > PERIOD_S) ph = PERIOD_S;

        v = V_CRUISE * s_vscale * warmup_factor(n);
        D = excursion(v);
        if (D > D_MAX) {                 /* never exceed 85 deg */
            v *= D_MAX / D;
        }

        if (ph < HALF_S) {
            half_profile(ph, v, &p, &pd, &pdd);
            r = Q_LOW_DEG + p;  rd = pd;  rdd = pdd;
        } else {
            half_profile(ph - HALF_S, v, &p, &pd, &pdd);
            D = excursion(v);
            r = Q_LOW_DEG + D - p;  rd = -pd;  rdd = -pdd;
        }
    }

    /* Supervision */
    e_deg = r - q;
    ae = fabs(e_deg);
    if (cycling) {
        if (ae > s_cycle_peak_err) s_cycle_peak_err = ae;
        if (fabs(qd) > s_cycle_peak_spd) s_cycle_peak_spd = fabs(qd);
    }
    if (ae > CATCH_ERR_DEG || fabs(qd) > RUNAWAY_DPS) {
        s_yield = true;
        s_cycle_bad = true;
    }

    tau_ff = J_EST * rdd * DEG2RAD
           + B_EST * rd * DEG2RAD
           + K_EST * (r - Q_REST_DEG) * DEG2RAD;

    e  = clampd(e_deg, -ERR_CLAMP_DEG, ERR_CLAMP_DEG) * DEG2RAD;
    ed = (rd - qd) * DEG2RAD;
    g  = s_yield ? YIELD_GAIN : 1.0;
    tau = g * (tau_ff + KP * e + KD * ed);

    /* Speed guard: brake on excess speed above V_GUARD */
    if (qd > V_GUARD)       tau -= KB * (qd - V_GUARD) * DEG2RAD;
    else if (qd < -V_GUARD) tau -= KB * (qd + V_GUARD) * DEG2RAD;

    if (fabs(tau) >= TAU_MAX) {
        if (++s_sat_ticks >= SAT_TICKS_YIELD) {
            s_yield = true;
            s_cycle_bad = true;
        }
    } else if (s_sat_ticks > 0) {
        s_sat_ticks--;
    }
    tau = clampd(tau, -TAU_MAX, TAU_MAX);
    tau = clampd(tau, s_tau_last - TAU_SLEW, s_tau_last + TAU_SLEW);

    if (!isfinite(tau)) tau = isfinite(s_tau_last) ? s_tau_last : 0.0;
    s_tau_last = tau;
    return tau;
}
