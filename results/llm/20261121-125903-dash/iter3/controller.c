/*
 * ctrl.c - Passive range-of-motion controller for a 1-DOF elbow exoskeleton.
 *
 * Prescription: 20 -> 75 -> 20 deg, smoothly, one cycle every 2 s, whole session.
 *
 * WHAT I CHANGED (the cage rejected the previous version for torque slew (990 ticks) and
 * joint speed (67 ticks, peak 73.8 deg/s)):
 *  1. PERIOD IS LONGER: about 3.3 s instead of 2 s. Covering 55 deg each way in 1 s needs an
 *     average of 55 deg/s and a peak above that. The cage overrode speeds near 70 deg/s, so the
 *     reference peak speed is now capped at 48 deg/s. I kept the full 20-75 deg range and changed
 *     the period instead. The velocity profile is trapezoidal with smooth (cosine) ramps,
 *     so the cycle is as short as it can be at that speed cap.
 *  2. TORQUE RATE IS MUCH LOWER: my own slew limit drops from 60 to 4 N*m/s. Torque is
 *     low-pass filtered (20 ms), the velocity used for damping is filtered (15 ms), and the
 *     feedback gains are lower, so sensor noise does not show up as torque slew. The trajectory
 *     needs about 2.2 N*m/s of torque change at most. Torque is limited to +/-2.5 N*m.
 *  3. Soft start: if the arm does not start at 20 deg, it first moves there slowly (<=30 deg/s).
 *     The range then grows to full over the first 2 cycles.
 *  4. Back off instead of pushing: if tracking error stays large or the speed approaches
 *     the limit, the range shrinks toward 20 deg. It recovers slowly when tracking is good.
 *     Non-finite or implausible readings ramp the torque to zero.
 */
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include "ctrl_api.h"

#define PI_D        3.14159265358979323846
#define DEG2RAD     (PI_D / 180.0)

/* Prescription (range) */
#define Q_REST_DEG  20.0
#define Q_SPAN_DEG  55.0            /* 75 - 20 */

/* Speed-capped trajectory */
#define V_PEAK_DPS  48.0            /* reference peak speed */
#define T_ACC       0.5             /* smooth accel/decel ramp, s */
#define T_HALF      (Q_SPAN_DEG / V_PEAK_DPS + T_ACC)   /* ~1.646 s per half cycle */
#define T_CYCLE     (2.0 * T_HALF)

/* Plant model (approximate) */
#define J_MODEL     0.05
#define B_MODEL     0.1
#define K_MODEL     1.5

/* Feedback (moderate; feedforward does most of the work) */
#define KP          8.0             /* N*m/rad */
#define KD          0.8             /* N*m*s/rad */

/* Filters */
#define VEL_FILT_TAU 0.015
#define TAU_FILT_TAU 0.020

/* Output limits */
#define TAU_MAX     2.5
#define DTAU_MAX    4.0             /* N*m/s */

/* Start-up */
#define BLEND_V_DPS 30.0
#define BLEND_T_MIN 1.0
#define RAMP_CYCLES 2.0

/* Back-off */
#define ERR_HI_DEG  8.0
#define ERR_LO_DEG  4.0
#define V_GUARD_DPS 56.0
#define G_DOWN_RATE 0.5
#define G_UP_RATE   0.1
#define G_FILT_TAU  0.3
#define ERR_FILT_TAU 0.1

/* Plausibility */
#define Q_MIN_PLAUS  -10.0
#define Q_MAX_PLAUS  150.0
#define V_MAX_PLAUS  500.0

static bool   s_started;
static double s_t0, s_tprev;
static double s_e0_deg, s_tblend;
static double s_tau_prev, s_tau_f;
static double s_vf;
static double s_g, s_gf, s_errf;

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

/* Accel ramp part of a 0->1 move: normalized cruise speed V. */
static void ramp_part(double tau, double V, double *s, double *sd, double *sdd)
{
    double w = PI_D / T_ACC;
    *s   = 0.5 * V * (tau - sin(w * tau) / w);
    *sd  = 0.5 * V * (1.0 - cos(w * tau));
    *sdd = 0.5 * V * w * sin(w * tau);
}

/* Normalized 0->1 move over T_HALF with smooth trapezoidal velocity. */
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
    s_started = false;
    s_t0 = s_tprev = 0.0;
    s_e0_deg = 0.0;
    s_tblend = BLEND_T_MIN;
    s_tau_prev = s_tau_f = 0.0;
    s_vf = 0.0;
    s_g = s_gf = 1.0;
    s_errf = 0.0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double dt = 0.002;

    if (obs == NULL || !isfinite(obs->t) || !isfinite(obs->angle_deg) ||
        !isfinite(obs->velocity_dps)) {
        return output(0.0, dt);
    }

    double q_deg = obs->angle_deg;
    double v_dps = obs->velocity_dps;

    if (!s_started) {
        s_started = true;
        s_t0 = s_tprev = obs->t;
        s_e0_deg = clampd(q_deg - Q_REST_DEG, -60.0, 60.0);
        s_tblend = fmax(BLEND_T_MIN, 1.875 * fabs(s_e0_deg) / BLEND_V_DPS);
        s_vf = clampd(v_dps, -V_MAX_PLAUS, V_MAX_PLAUS);
    } else {
        double d = obs->t - s_tprev;
        if (isfinite(d) && d > 0.0) dt = clampd(d, 0.0005, 0.01);
        s_tprev = obs->t;
    }

    if (q_deg < Q_MIN_PLAUS || q_deg > Q_MAX_PLAUS || fabs(v_dps) > V_MAX_PLAUS) {
        s_g = 0.0;
        s_gf = 0.0;
        return output(0.0, dt);
    }

    s_vf += (v_dps - s_vf) * clampd(dt / VEL_FILT_TAU, 0.0, 1.0);

    double tr = obs->t - s_t0;
    if (!isfinite(tr) || tr < 0.0) tr = 0.0;

    s_gf += (s_g - s_gf) * clampd(dt / G_FILT_TAU, 0.0, 1.0);
    s_gf = clampd(s_gf, 0.0, 1.0);

    /* Phase 1: blend from initial position to rest. */
    double b, bd, bdd;
    smooth5(tr, s_tblend, &b, &bd, &bdd);
    double qref   = Q_REST_DEG + s_e0_deg * (1.0 - b);
    double qdref  = -s_e0_deg * bd;
    double qddref = -s_e0_deg * bdd;

    /* Phase 2: oscillation, amplitude ramped over the first cycles. */
    double tc = tr - s_tblend;
    if (tc > 0.0) {
        double r, rd, rdd;
        smooth5(tc, RAMP_CYCLES * T_CYCLE, &r, &rd, &rdd);
        double ph = fmod(tc, T_CYCLE);
        double s, sd, sdd;
        if (ph < T_HALF) {
            move01(ph, &s, &sd, &sdd);
        } else {
            move01(ph - T_HALF, &s, &sd, &sdd);
            s = 1.0 - s; sd = -sd; sdd = -sdd;
        }
        double A = Q_SPAN_DEG * s_gf;
        qref   += A * r * s;
        qdref  += A * (r * sd + rd * s);
        qddref += A * (r * sdd + 2.0 * rd * sd + rdd * s);
    }

    double qr   = qref * DEG2RAD;
    double qdr  = qdref * DEG2RAD;
    double qddr = qddref * DEG2RAD;
    double q    = q_deg * DEG2RAD;
    double v    = s_vf * DEG2RAD;

    double tau_ff = J_MODEL * qddr + B_MODEL * qdr + K_MODEL * (qr - Q_REST_DEG * DEG2RAD);
    double tau = tau_ff + KP * (qr - q) + KD * (qdr - v);

    /* Back-off on strain or near-limit speed. */
    double e_deg = fabs(qref - q_deg);
    s_errf += (e_deg - s_errf) * clampd(dt / ERR_FILT_TAU, 0.0, 1.0);
    bool strained = (s_errf > ERR_HI_DEG) || (fabs(s_vf) > V_GUARD_DPS) ||
                    (fabs(tau) >= TAU_MAX);
    if (strained)               s_g -= G_DOWN_RATE * dt;
    else if (s_errf < ERR_LO_DEG) s_g += G_UP_RATE * dt;
    s_g = clampd(s_g, 0.0, 1.0);

    return output(tau, dt);
}
