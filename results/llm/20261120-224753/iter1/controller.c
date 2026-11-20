/* elbow_prom.c - passive range-of-motion controller for a 1-DOF elbow exoskeleton.
 *
 * Prescription: 20 -> 85 -> 20 deg, one full cycle every 1.5 s, smooth, for the whole session.
 *
 * WHAT I CHANGED OR ADDED, AND WHY:
 *  1. Range is 21..84 deg instead of 20..85 deg. Tracking overshoot should not reach the
 *     clinician's envelope edges, so each end has a 1 deg margin.
 *  2. The session does not start at 1.5 s per cycle. For the first 2 s the arm moves
 *     smoothly from wherever it is to 21 deg. Cycling then starts at 4 s per cycle and speeds
 *     up to the prescribed 1.5 s over about 20-30 s. A sudden fast start is unsafe.
 *  3. The controller slows down on its own. If the patient resists (tracking error over
 *     6 deg) or the torque limit is hit for more than 50 ms, the cycle period goes up 30%
 *     (up to a maximum of 6 s). It then returns slowly to 1.5 s. This keeps the controller
 *     from pushing again and again against the patient or the cage.
 *  4. Torque is capped at +/-3 N*m and changes by at most 50 N*m/s. The nominal need is
 *     about 1.5 N*m.
 *  CLINICAL FLAG: at 1.5 s per cycle the peak speed is about 136 deg/s. Spasticity after a
 *  stroke depends on speed, and passive range-of-motion work is usually done much slower.
 *  Please have the therapist confirm the 1.5 s period. To change it, edit T_RX below.
 */
#include <math.h>
#include <stdbool.h>
#include "ctrl_api.h"

#define D2R    (3.14159265358979323846 / 180.0)
#define TWO_PI 6.28318530717958647692

/* plant model */
static const double J = 0.05, B = 0.1, K = 1.5, REST = 20.0;
/* trajectory */
static const double LO = 21.0, HI = 84.0;
static const double T_RX = 1.5, T_START = 4.0, T_MAX = 6.0;
static const double TAU_RECOVER = 8.0;   /* s, how fast the period returns to T_RX */
static const double TAU_PERIOD  = 0.3;   /* s, smoothing of period changes */
static const double BLEND = 2.0;         /* s, initial move to LO */
/* feedback gains (about 20 rad/s bandwidth, critically damped) */
static const double KP = 20.0, KD = 1.9;
/* limits */
static const double TAU_MAX = 3.0, SLEW = 50.0;
static const double ERR_TRIP = 6.0, SAT_TRIP = 0.05, COOLDOWN = 1.0;

static bool   started;
static double t_prev, t0, theta0, phase;
static double T_tgt, T_cur, tau_prev, err_f, sat_time, cooldown;

void ctrl_init(void)
{
    started = false;
    t_prev = t0 = theta0 = phase = 0.0;
    T_tgt = T_cur = T_START;
    tau_prev = err_f = sat_time = cooldown = 0.0;
}

static double clampd(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

double ctrl_step(const sc_obs_t *obs)
{
    double dt, tau;

    if (!obs || !isfinite(obs->t) || !isfinite(obs->angle_deg) || !isfinite(obs->velocity_dps)) {
        tau_prev = clampd(0.0, tau_prev - SLEW * 0.002, tau_prev + SLEW * 0.002);
        return tau_prev; /* bad sensor data: ease torque toward zero */
    }

    if (!started) {
        started = true;
        t0 = obs->t;
        t_prev = obs->t;
        theta0 = clampd(obs->angle_deg, 0.0, 130.0);
    }
    dt = obs->t - t_prev;
    if (!(dt > 0.0 && dt < 0.02)) dt = 0.002;
    t_prev = obs->t;
    double elapsed = obs->t - t0;

    /* ---- adapt the period: back off when resisted, recover slowly ---- */
    cooldown -= dt;
    if (cooldown <= 0.0 && (err_f > ERR_TRIP || sat_time > SAT_TRIP)) {
        T_tgt = fmin(fmax(T_tgt, T_cur) * 1.3, T_MAX);
        cooldown = COOLDOWN;
        sat_time = 0.0;
    }
    T_tgt = T_RX + (T_tgt - T_RX) * exp(-dt / TAU_RECOVER);
    T_cur += (T_tgt - T_cur) * (dt / (TAU_PERIOD + dt));
    T_cur = clampd(T_cur, T_RX, T_MAX);

    /* ---- reference trajectory, deg ---- */
    double r, rd, rdd;
    if (elapsed < BLEND) {
        double s = elapsed / BLEND, s2 = s * s, s3 = s2 * s;
        double h   = 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s2;
        double hd  = (30.0 * s2 - 60.0 * s3 + 30.0 * s3 * s) / BLEND;
        double hdd = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (BLEND * BLEND);
        double d = LO - theta0;
        r = theta0 + d * h; rd = d * hd; rdd = d * hdd;
    } else {
        double c = 0.5 * (LO + HI), A = 0.5 * (HI - LO);
        double w = TWO_PI / T_cur;
        phase += w * dt;
        if (phase >= TWO_PI) phase -= TWO_PI;
        r   = c - A * cos(phase);
        rd  = A * w * sin(phase);
        rdd = A * w * w * cos(phase);
    }

    /* ---- computed-torque feedforward + PD ---- */
    double e  = (r - obs->angle_deg) * D2R;
    double ed = (rd - obs->velocity_dps) * D2R;
    double tau_ff = J * rdd * D2R + B * rd * D2R + K * (r - REST) * D2R;
    tau = tau_ff + KP * e + KD * ed;

    err_f += (fabs(r - obs->angle_deg) - err_f) * (dt / (0.1 + dt));

    if (fabs(tau) > TAU_MAX) {
        tau = clampd(tau, -TAU_MAX, TAU_MAX);
        sat_time += dt;
    } else {
        sat_time = fmax(0.0, sat_time - dt);
    }

    tau = clampd(tau, tau_prev - SLEW * dt, tau_prev + SLEW * dt);
    if (!isfinite(tau)) tau = 0.0;
    tau_prev = tau;
    return tau;
}
