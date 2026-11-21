/*
 * ctrl.c - Passive range-of-motion controller for a 1-DOF elbow exoskeleton.
 *
 * Prescription: 20 -> 75 -> 20 deg, smoothly, one cycle every 2 s, whole session.
 *
 * CHANGES FROM THE PRESCRIPTION, AND WHY:
 *  - Period is about 3.8 s instead of 2 s. A 2 s cycle over 55 deg needs peak speeds above
 *    80 deg/s. The cage overrode speeds near 70 deg/s. The reference now peaks at 42 deg/s,
 *    using a trapezoidal velocity profile with smooth cosine ramps. The 20-75 deg range is kept.
 *  - The last run's speed overrides (13 ticks) were probably what set off its slew overrides
 *    (275 ticks). The speed went over the limit for three reasons: the range shrank in the
 *    middle of a stroke (that added speed nobody planned for), the feedback was stiff, and the
 *    torque rate limit added lag. The fixes:
 *      * The range changes only at the 20 deg turnaround, between cycles.
 *      * If speed or tracking error gets too high, or torque saturates, the reference slows
 *        down along its own path (time scaling). It no longer jumps.
 *      * The feedback is softer. Tissue stiffness and bias are adapted slowly, so the
 *        controller reaches 75 deg with feedforward instead of with large errors.
 *      * My own torque rate cap is now 2.5 N*m/s (it was 4). The trajectory needs about 1.1 N*m/s.
 *  - Start-up: the arm moves slowly (<=25 deg/s) to 20 deg first. The first cycles use 50% and
 *    then 80% of the range. If cycles are bad (overspeed, large error or saturation), the range
 *    shrinks; it recovers by 5% per clean cycle. Bad sensor readings ramp the torque to zero,
 *    then the controller restarts gently from the current position.
 */
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "ctrl_api.h"

#define PI_D        3.14159265358979323846
#define DEG2RAD     (PI_D / 180.0)

/* Prescribed range */
#define Q_REST_DEG  20.0
#define Q_SPAN_DEG  55.0

/* Speed-capped trajectory */
#define V_PEAK_DPS  42.0
#define T_ACC       0.6
#define T_HALF      (Q_SPAN_DEG / V_PEAK_DPS + T_ACC)   /* ~1.91 s */
#define T_CYCLE     (2.0 * T_HALF)                      /* ~3.82 s */

/* Plant model (nominal) */
#define J_MODEL     0.05
#define B_MODEL     0.1
#define K_MODEL     1.5

/* Feedback */
#define KP          6.0
#define KD          0.6

/* Slow adaptation of stiffness and bias */
#define GAM_K       2.0
#define GAM_B       0.5
#define SIG_LEAK    0.05
#define KHAT_MIN    0.8
#define KHAT_MAX    2.5
#define BHAT_MAX    0.4

/* Filters / output */
#define VEL_FILT_TAU 0.010
#define TAU_FILT_TAU 0.015
#define TAU_MAX      2.5
#define DTAU_MAX     2.5      /* N*m/s */
#define DT_NOM       0.002

/* Start-up blend */
#define BLEND_V_DPS  25.0
#define BLEND_T_MIN  1.0

/* Time-scaling governor */
#define V_GUARD_DPS  50.0
#define E_SLOW_DEG   10.0
#define KAPPA_SLOW   0.6
#define KAPPA_DOWN   0.8      /* 1/s */
#define KAPPA_UP     0.2      /* 1/s */

/* Per-cycle range adaptation */
#define V_CYC_HI     50.0
#define V_CYC_OK     47.0
#define E_CYC_HI     8.0
#define E_CYC_OK     5.0
#define G_MIN        0.3
#define G_DOWN_MUL   0.85
#define G_UP_STEP    0.05

/* Plausibility */
#define Q_MIN_PLAUS  -10.0
#define Q_MAX_PLAUS  150.0
#define V_MAX_PLAUS  500.0

enum { MODE_BLEND = 0, MODE_OSC = 1 };

static bool   s_need_restart, s_have_t;
static double s_tprev;
static int    s_mode;
static double s_tb0, s_e0_deg, s_tblend;
static double s_phi, s_kappa, s_kdot;
static bool   s_slow_req;
static int    s_cycle;
static double s_g, s_A;
static double s_cyc_vmax, s_cyc_emax, s_cyc_kmin;
static bool   s_cyc_sat;
static double s_khat, s_bhat;
static double s_vf, s_tau_f, s_tau_prev;

static double clampd(double x, double lo, double hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void smooth5(double t, double T, double *s, double *sd, double *sdd)
{
    if (t <= 0.0) { *s = 0.0; *sd = 0.0; *sdd = 0.0; return; }
    if (t >= T)   { *s = 1.0; *sd = 0.0; *sdd = 0.0; return; }
    double u = t / T, u2 = u * u;
    *s   = u2 * u * (10.0 - 15.0 * u + 6.0 * u2);
    *sd  = 30.0 * u2 * (1.0 - u) * (1.0 - u) / T;
    *sdd = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T);
}

static void ramp_part(double tau, double V, double *s, double *sd, double *sdd)
{
    double w = PI_D / T_ACC;
    *s   = 0.5 * V * (tau - sin(w * tau) / w);
    *sd  = 0.5 * V * (1.0 - cos(w * tau));
    *sdd = 0.5 * V * w * sin(w * tau);
}

/* Normalized 0->1 move over T_HALF; zero velocity and acceleration at both ends. */
static void move01(double tau, double *s, double *sd, double *sdd)
{
    double V = 1.0 / (T_HALF - T_ACC);
    tau = clampd(tau, 0.0, T_HALF);
    if (tau < T_ACC) {
        ramp_part(tau, V, s, sd, sdd);
    } else if (tau <= T_HALF - T_ACC) {
        *s = 0.5 * V * T_ACC + V * (tau - T_ACC);
        *sd = V;
        *sdd = 0.0;
    } else {
        double a, ad, add;
        ramp_part(T_HALF - tau, V, &a, &ad, &add);
        *s = 1.0 - a;
        *sd = ad;
        *sdd = -add;
    }
}

static double ramp_frac(int cycle)
{
    if (cycle <= 0) return 0.5;
    if (cycle == 1) return 0.8;
    return 1.0;
}

static void reset_cycle_stats(void)
{
    s_cyc_vmax = 0.0;
    s_cyc_emax = 0.0;
    s_cyc_kmin = 1.0;
    s_cyc_sat = false;
}

static void start_blend(double t, double q_deg, double v_dps)
{
    s_mode = MODE_BLEND;
    s_tb0 = t;
    s_e0_deg = clampd(q_deg - Q_REST_DEG, -60.0, 60.0);
    s_tblend = fmax(BLEND_T_MIN, 1.875 * fabs(s_e0_deg) / BLEND_V_DPS);
    s_vf = clampd(v_dps, -V_MAX_PLAUS, V_MAX_PLAUS);
    s_phi = 0.0;
    s_kappa = 1.0;
    s_kdot = 0.0;
    s_slow_req = false;
    s_cycle = 0;
    reset_cycle_stats();
}

static double output(double tau_cmd, double dt)
{
    if (!isfinite(tau_cmd)) tau_cmd = 0.0;
    tau_cmd = clampd(tau_cmd, -TAU_MAX, TAU_MAX);
    s_tau_f += (tau_cmd - s_tau_f) * clampd(dt / TAU_FILT_TAU, 0.0, 1.0);
    if (!isfinite(s_tau_f)) s_tau_f = 0.0;
    double dmax = DTAU_MAX * dt;
    double tau = clampd(s_tau_f, s_tau_prev - dmax, s_tau_prev + dmax);
    tau = clampd(tau, -TAU_MAX, TAU_MAX);
    if (!isfinite(tau)) tau = 0.0;
    s_tau_prev = tau;
    return tau;
}

void ctrl_init(void)
{
    s_need_restart = true;
    s_have_t = false;
    s_tprev = 0.0;
    s_mode = MODE_BLEND;
    s_tb0 = 0.0;
    s_e0_deg = 0.0;
    s_tblend = BLEND_T_MIN;
    s_phi = 0.0;
    s_kappa = 1.0;
    s_kdot = 0.0;
    s_slow_req = false;
    s_cycle = 0;
    s_g = 1.0;
    s_A = Q_SPAN_DEG * ramp_frac(0);
    reset_cycle_stats();
    s_khat = K_MODEL;
    s_bhat = 0.0;
    s_vf = 0.0;
    s_tau_f = 0.0;
    s_tau_prev = 0.0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double dt = DT_NOM;

    if (obs == NULL || !isfinite(obs->t) || !isfinite(obs->angle_deg) ||
        !isfinite(obs->velocity_dps)) {
        s_need_restart = true;
        return output(0.0, dt);
    }

    double t = obs->t;
    double q_deg = obs->angle_deg;
    double v_dps = obs->velocity_dps;

    if (s_have_t) {
        double d = t - s_tprev;
        if (isfinite(d) && d > 0.0) dt = clampd(d, 0.0005, 0.01);
    }
    s_tprev = t;
    s_have_t = true;

    if (q_deg < Q_MIN_PLAUS || q_deg > Q_MAX_PLAUS || fabs(v_dps) > V_MAX_PLAUS) {
        s_need_restart = true;
        return output(0.0, dt);
    }

    if (s_need_restart) {
        start_blend(t, q_deg, v_dps);
        s_need_restart = false;
    }

    s_vf += (v_dps - s_vf) * clampd(dt / VEL_FILT_TAU, 0.0, 1.0);

    double qref, qdref, qddref;

    if (s_mode == MODE_BLEND) {
        double tr = t - s_tb0;
        if (!isfinite(tr) || tr < 0.0) tr = 0.0;
        if (tr >= s_tblend) {
            s_mode = MODE_OSC;
            s_phi = 0.0;
            s_cycle = 0;
            s_kappa = 1.0;
            s_kdot = 0.0;
            s_A = Q_SPAN_DEG * fmin(s_g, ramp_frac(0));
            reset_cycle_stats();
        }
    }

    if (s_mode == MODE_BLEND) {
        double tr = clampd(t - s_tb0, 0.0, s_tblend);
        double b, bd, bdd;
        smooth5(tr, s_tblend, &b, &bd, &bdd);
        qref   = Q_REST_DEG + s_e0_deg * (1.0 - b);
        qdref  = -s_e0_deg * bd;
        qddref = -s_e0_deg * bdd;
    } else {
        /* Time-scaling governor: slow down along the path instead of jumping. */
        double ktgt = s_slow_req ? KAPPA_SLOW : 1.0;
        double kprev = s_kappa;
        double rate = (ktgt < s_kappa) ? KAPPA_DOWN : KAPPA_UP;
        s_kappa += clampd(ktgt - s_kappa, -rate * dt, rate * dt);
        s_kappa = clampd(s_kappa, KAPPA_SLOW, 1.0);
        s_kdot = (s_kappa - kprev) / dt;
        if (s_kappa < s_cyc_kmin) s_cyc_kmin = s_kappa;

        s_phi += s_kappa * dt;
        if (s_phi >= T_CYCLE) {
            /* Turnaround at 20 deg: the only place the range is allowed to change. */
            s_phi -= T_CYCLE;
            if (s_phi < 0.0 || s_phi > T_CYCLE) s_phi = 0.0;
            bool bad  = (s_cyc_vmax > V_CYC_HI) || (s_cyc_emax > E_CYC_HI) || s_cyc_sat;
            bool good = (s_cyc_vmax < V_CYC_OK) && (s_cyc_emax < E_CYC_OK) &&
                        !s_cyc_sat && (s_cyc_kmin > 0.95);
            if (bad)       s_g = fmax(G_MIN, s_g * G_DOWN_MUL);
            else if (good) s_g = fmin(1.0, s_g + G_UP_STEP);
            if (s_cycle < 1000000) s_cycle++;
            s_A = Q_SPAN_DEG * fmin(s_g, ramp_frac(s_cycle));
            reset_cycle_stats();
        }

        double s, sd, sdd;
        if (s_phi < T_HALF) {
            move01(s_phi, &s, &sd, &sdd);
        } else {
            move01(s_phi - T_HALF, &s, &sd, &sdd);
            s = 1.0 - s; sd = -sd; sdd = -sdd;
        }
        qref   = Q_REST_DEG + s_A * s;
        qdref  = s_A * s_kappa * sd;
        qddref = s_A * (s_kappa * s_kappa * sdd + s_kdot * sd);
    }

    double qr   = qref * DEG2RAD;
    double qdr  = qdref * DEG2RAD;
    double qddr = qddref * DEG2RAD;
    double q    = q_deg * DEG2RAD;
    double v    = s_vf * DEG2RAD;
    double x    = qr - Q_REST_DEG * DEG2RAD;
    double e    = qr - q;

    double tau_ff = J_MODEL * qddr + B_MODEL * qdr + s_khat * x + s_bhat;
    double tau_fb = KP * e + KD * (qdr - v);
    double tau = tau_ff + tau_fb;
    if (!isfinite(tau)) tau = 0.0;

    bool sat = fabs(tau) > 0.9 * TAU_MAX;
    double e_deg = fabs(qref - q_deg);
    double vabs = fabs(s_vf);

    /* Slow, bounded, leaky adaptation (only while cycling and not saturated). */
    if (s_mode == MODE_OSC && !sat) {
        s_khat += dt * (GAM_K * e * x - SIG_LEAK * (s_khat - K_MODEL));
        s_bhat += dt * (GAM_B * e - SIG_LEAK * s_bhat);
        s_khat = clampd(s_khat, KHAT_MIN, KHAT_MAX);
        s_bhat = clampd(s_bhat, -BHAT_MAX, BHAT_MAX);
        if (!isfinite(s_khat)) s_khat = K_MODEL;
        if (!isfinite(s_bhat)) s_bhat = 0.0;
    }

    /* Governor request with hysteresis. */
    if (vabs > V_GUARD_DPS || e_deg > E_SLOW_DEG || sat) {
        s_slow_req = true;
    } else if (vabs < V_GUARD_DPS - 6.0 && e_deg < E_SLOW_DEG - 4.0) {
        s_slow_req = false;
    }

    if (s_mode == MODE_OSC) {
        if (vabs > s_cyc_vmax) s_cyc_vmax = vabs;
        if (e_deg > s_cyc_emax) s_cyc_emax = e_deg;
        if (sat) s_cyc_sat = true;
    }

    return output(tau, dt);
}
