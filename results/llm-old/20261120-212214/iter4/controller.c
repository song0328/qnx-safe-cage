/*
 * ctrl_elbow_prom.c - passive range-of-motion controller, elbow, 1 DOF.
 *
 * Prescription: 20 -> 85 -> 20 deg, one cycle every 1.5 s, whole session.
 *
 * CHANGES FROM THE PRESCRIPTION (earlier cage reports showed a joint speed
 * limit of about 90 deg/s and a torque slew limit of 0.12 N*m per tick):
 *  1. RANGE REDUCED: moving 65 deg in 0.75 s needs at least 86.7 deg/s even
 *     with an instant reversal, and a cosine profile needs 136 deg/s. A full
 *     20-85 deg cycle in 1.5 s cannot stay under the speed limit. The 1.5 s
 *     period is kept exactly. The range is reduced to about 20 -> 76 -> 20 deg
 *     (cruise speed 84 deg/s). The 85 deg cap is still enforced.
 *  2. PROFILE: near-trapezoidal (cruise speed, sinusoidal reversals of
 *     0.22 s). Acceleration is continuous.
 *  3. Start: a min-jerk move to 20 deg at no more than 40 deg/s. Warm-up
 *     cycles run at 60% and 85%.
 *  4. Speed guard above 86 deg/s, and per-cycle speed adaptation. The period
 *     never changes.
 *  5. Torque is capped at 4 N*m and slew-limited to 0.09 N*m per tick.
 *  6. TIMING: the previous version missed one 2 ms deadline. The profile is
 *     now a table built in ctrl_init. ctrl_step does only arithmetic: no libm
 *     calls and no loops. If a tick gap is seen (missed tick), the slew limit
 *     restarts from 0 N*m, because the cage applied zero torque on the miss.
 * Clinical note: 85 deg is not reached. The therapist must accept a range of
 * about 20-76 deg, or relax the speed limit or the period.
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
#define D_MAX       (Q_HIGH_DEG - Q_LOW_DEG)

/* Profile */
#define V_CRUISE    84.0
#define T_REV       0.11
#define TAB_N       375                 /* 2 ms resolution over a half cycle */
#define TAB_DS      (HALF_S / (double)TAB_N)

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
#define V_GUARD     86.0
#define V_BAD       88.0
#define KB          3.0

/* Output limits */
#define TAU_MAX     4.0
#define TAU_SLEW    0.09

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
#define GAP_FACTOR         1.8

/* Normalised (v = 1 deg/s) upward half-cycle profile */
static double s_tp[TAB_N + 1];
static double s_tv[TAB_N + 1];
static double s_ta[TAB_N + 1];
static double s_D1;

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
static bool   s_have_t;
static double s_t_last;
static double s_dt_nom;

static double absd(double x) { return x < 0.0 ? -x : x; }

static double clampd(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void build_table(void)
{
    const double w  = PI_D / (2.0 * T_REV);
    const double p1 = 1.0 / w;
    const double tc = HALF_S - 2.0 * T_REV;
    const double D  = 2.0 * p1 + tc;
    int i;
    for (i = 0; i <= TAB_N; ++i) {
        double s = (double)i * TAB_DS;
        if (s < T_REV) {
            s_tp[i] = p1 * (1.0 - cos(w * s));
            s_tv[i] = sin(w * s);
            s_ta[i] = w * cos(w * s);
        } else if (s < T_REV + tc) {
            s_tp[i] = p1 + (s - T_REV);
            s_tv[i] = 1.0;
            s_ta[i] = 0.0;
        } else {
            double u = HALF_S - s;
            if (u < 0.0) u = 0.0;
            s_tp[i] = D - p1 * (1.0 - cos(w * u));
            s_tv[i] = sin(w * u);
            s_ta[i] = -w * cos(w * u);
        }
    }
    s_D1 = D;
}

static void lookup(double s, double *p, double *pd, double *pdd)
{
    double x = s / TAB_DS, f;
    int i;
    if (!(x > 0.0)) x = 0.0;
    if (x > (double)TAB_N) x = (double)TAB_N;
    i = (int)x;
    if (i >= TAB_N) i = TAB_N - 1;
    f = x - (double)i;
    *p   = s_tp[i] + f * (s_tp[i + 1] - s_tp[i]);
    *pd  = s_tv[i] + f * (s_tv[i + 1] - s_tv[i]);
    *pdd = s_ta[i] + f * (s_ta[i + 1] - s_ta[i]);
}

void ctrl_init(void)
{
    build_table();
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
    s_have_t = false;
    s_t_last = 0.0;
    s_dt_nom = 0.002;
}

static double warmup_factor(long cycle)
{
    if (cycle <= 0) return 0.6;
    if (cycle == 1) return 0.85;
    return 1.0;
}

static void end_of_cycle_update(void)
{
    bool bad = s_cycle_bad || s_cycle_peak_err > BAD_CYCLE_ERR_DEG ||
               s_cycle_peak_spd > V_BAD;

    if (bad) {
        s_vscale = clampd(s_vscale * 0.9, VSCALE_MIN, 1.0);
        s_good_cycles = 0;
    } else if (s_cycle_peak_spd > V_GUARD) {
        s_vscale = clampd(s_vscale * 0.97, VSCALE_MIN, 1.0);
        s_good_cycles = 0;
    } else if (s_cycle_peak_err < GOOD_CYCLE_ERR_DEG &&
               s_cycle_peak_spd < V_CRUISE + 1.5) {
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
        return isfinite(s_tau_last) ? s_tau_last : 0.0;
    }

    q  = obs->angle_deg;
    qd = obs->velocity_dps;

    /* Tick-gap detection: after a missed tick the cage applied 0 N*m. */
    if (s_have_t) {
        double dt = obs->t - s_t_last;
        if (dt > 1e-5 && dt < s_dt_nom) s_dt_nom = dt;
        if (dt > GAP_FACTOR * s_dt_nom) s_tau_last = 0.0;
    }
    s_have_t = true;
    s_t_last = obs->t;

    if (!s_init) {
        double d, T;
        s_init = true;
        s_t0 = obs->t;
        s_q_start = q;
        d = absd(Q_LOW_DEG - q);
        T = 1.875 * d / APPROACH_VMAX_DPS;      /* min-jerk peak speed */
        s_approach_T = T > APPROACH_MIN_S ? T : APPROACH_MIN_S;
    }

    tr = obs->t - s_t0;
    if (tr < 0.0) tr = 0.0;

    if (tr < s_approach_T) {
        double T = s_approach_T, D = Q_LOW_DEG - s_q_start;
        double x = tr / T, x2 = x * x, x3 = x2 * x;
        r   = s_q_start + D * (10.0 * x3 - 15.0 * x3 * x + 6.0 * x3 * x2);
        rd  = D / T * (30.0 * x2 - 60.0 * x3 + 30.0 * x3 * x);
        rdd = D / (T * T) * (60.0 * x - 180.0 * x2 + 120.0 * x3);
    } else {
        double tc, ph, sc, p, pd, pdd;
        long n;
        cycling = true;
        tc = tr - s_approach_T;
        n = (long)(tc / PERIOD_S);
        if (n < 0) n = 0;
        if (n != s_cycle) {
            if (s_cycle >= 0) end_of_cycle_update();
            s_cycle = n;
        }
        ph = tc - (double)n * PERIOD_S;
        ph = clampd(ph, 0.0, PERIOD_S);

        sc = V_CRUISE * s_vscale * warmup_factor(n);
        if (sc * s_D1 > D_MAX) sc = D_MAX / s_D1;   /* never exceed 85 deg */

        if (ph < HALF_S) {
            lookup(ph, &p, &pd, &pdd);
            r = Q_LOW_DEG + sc * p;  rd = sc * pd;  rdd = sc * pdd;
        } else {
            lookup(ph - HALF_S, &p, &pd, &pdd);
            r = Q_LOW_DEG + sc * (s_D1 - p);  rd = -sc * pd;  rdd = -sc * pdd;
        }
    }

    e_deg = r - q;
    ae = absd(e_deg);
    if (cycling) {
        if (ae > s_cycle_peak_err) s_cycle_peak_err = ae;
        if (absd(qd) > s_cycle_peak_spd) s_cycle_peak_spd = absd(qd);
    }
    if (ae > CATCH_ERR_DEG || absd(qd) > RUNAWAY_DPS) {
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

    if (qd > V_GUARD)       tau -= KB * (qd - V_GUARD) * DEG2RAD;
    else if (qd < -V_GUARD) tau -= KB * (qd + V_GUARD) * DEG2RAD;

    if (absd(tau) >= TAU_MAX) {
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
