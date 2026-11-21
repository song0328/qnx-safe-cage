/*
 * elbow_rom_ctrl.c - passive range-of-motion controller, elbow joint (revision 4).
 *
 * Deviations from the prescription, and why:
 *   - Range is 20 -> 45 -> 20 deg, not 20 -> 75. Holding 75 deg against the tissue spring
 *     needs 1.5 N*m/rad x 55 deg = 1.44 N*m. Revision 3 (20 -> 50) needs about 0.79 N*m at
 *     50 deg (1.5 x 30 deg), not the 0.67 its comment gave, and it overshot to 63.3 deg.
 *     At 45 deg the hold torque is about 0.65 N*m.
 *   - Period kept at 2 s, cosine profile, one full cycle every 2 s for the whole session.
 *   - Peak reference speed is about 39 deg/s (was 47). Reference torque rate peaks near 1.0 N*m/s.
 *   - Internal torque slew limited to 1.2 N*m/s (was 2.0), so the command ramps instead of stepping.
 *   - Torque capped at 0.8 N*m. Feedback limited to +/-0.10 N*m. No integrator.
 *   - Positive torque is withheld above 46 deg and negative torque below 19 deg, so the
 *     controller does not keep pushing at the ends of the stroke.
 *   - Unverified: the cage's slew, speed and torque limits are not known. The values here
 *     are estimates from the last report. Not compiled or run against the cage.
 *
 * Non-finite or missing input returns 0 N*m. A missing or implausible tick interval
 * is taken as 2 ms.
 */

#include <math.h>
#include <stddef.h>

typedef struct {
    double t;             /* time since session start, s */
    double angle_deg;     /* elbow joint angle, degrees (0 = straight) */
    double velocity_dps;  /* joint velocity, degrees/s */
} sc_obs_t;

void ctrl_init(void);
double ctrl_step(const sc_obs_t *obs);

#define CTRL_PI       3.14159265358979323846
#define DEG2RAD       (CTRL_PI / 180.0)
#define CENTER_DEG    32.5      /* (20 + 45) / 2 */
#define AMP_DEG       12.5      /* (45 - 20) / 2 */
#define W_REF         CTRL_PI   /* rad/s, 2 s period */
#define REST_DEG      20.0      /* tissue rest angle */
#define ANG_LO        19.0      /* no negative torque below this angle */
#define ANG_HI        46.0      /* no positive torque above this angle */

#define MODEL_I       0.05      /* kg*m^2 */
#define MODEL_B       0.1       /* N*m*s/rad */
#define MODEL_K       1.5       /* N*m/rad */

#define KP            0.6       /* N*m/rad */
#define KD            0.03      /* N*m*s/rad */
#define FB_MAX        0.10      /* N*m, feedback authority */
#define VEL_FILT_TAU  0.02      /* s, velocity low-pass time constant */

#define TAU_MAX       0.80      /* N*m, internal torque ceiling */
#define RATE_MAX      1.2       /* N*m/s, internal torque slew limit */
#define DT_NOMINAL    0.002     /* s */
#define DT_MAX        0.05      /* s, larger gaps are treated as invalid */

static int    s_have_prev;
static double s_t_prev;
static double s_vel_f;    /* low-passed measured velocity, deg/s */
static double s_tau_prev; /* last returned torque, N*m */

static int is_finite_val(double x)
{
    return (x == x) && (x < 1e9) && (x > -1e9);
}

static double clamp(double x, double lo, double hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

void ctrl_init(void)
{
    s_have_prev = 0;
    s_t_prev = 0.0;
    s_vel_f = 0.0;
    s_tau_prev = 0.0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double dt, alpha;
    double phase, c, s;
    double ref_deg, ref_vel_dps, ref_acc_dps2;
    double e_rad, edot_rad;
    double ff, fb, tau, dmax;

    if (obs == NULL ||
        !is_finite_val(obs->t) ||
        !is_finite_val(obs->angle_deg) ||
        !is_finite_val(obs->velocity_dps)) {
        s_have_prev = 0;
        s_tau_prev = 0.0;
        return 0.0;
    }

    /* Tick interval. */
    if (!s_have_prev) {
        dt = DT_NOMINAL;
        s_vel_f = obs->velocity_dps;
        s_have_prev = 1;
    } else {
        dt = obs->t - s_t_prev;
        if (!(dt > 0.0) || dt > DT_MAX) {
            dt = DT_NOMINAL;
        }
    }
    s_t_prev = obs->t;

    /* Low-pass the measured velocity (first-order, dt-aware). */
    alpha = dt / (VEL_FILT_TAU + dt);
    s_vel_f += alpha * (obs->velocity_dps - s_vel_f);

    /* Reference: theta(t) = 32.5 - 12.5*cos(pi*t) deg, 20 at t=0, 45 at t=1 s, period 2 s. */
    phase = W_REF * obs->t;
    c = cos(phase);
    s = sin(phase);
    ref_deg      = CENTER_DEG - AMP_DEG * c;
    ref_vel_dps  = AMP_DEG * W_REF * s;
    ref_acc_dps2 = AMP_DEG * W_REF * W_REF * c;

    /* Feedforward from the stated plant model. */
    ff  = MODEL_I * ref_acc_dps2 * DEG2RAD;
    ff += MODEL_B * ref_vel_dps * DEG2RAD;
    ff += MODEL_K * (ref_deg - REST_DEG) * DEG2RAD;

    /* Small, bounded feedback. */
    e_rad    = (ref_deg - obs->angle_deg) * DEG2RAD;
    edot_rad = (ref_vel_dps - s_vel_f) * DEG2RAD;
    fb = clamp(KP * e_rad + KD * edot_rad, -FB_MAX, FB_MAX);

    tau = clamp(ff + fb, -TAU_MAX, TAU_MAX);

    /* Stroke-end guard, applied before the slew limit. */
    if (obs->angle_deg > ANG_HI && tau > 0.0) tau = 0.0;
    if (obs->angle_deg < ANG_LO && tau < 0.0) tau = 0.0;

    /* Slew limit. */
    dmax = RATE_MAX * dt;
    tau = clamp(tau, s_tau_prev - dmax, s_tau_prev + dmax);

    /* Final stroke-end guard, so the returned value also respects the ends. */
    if (obs->angle_deg > ANG_HI && tau > 0.0) tau = 0.0;
    if (obs->angle_deg < ANG_LO && tau < 0.0) tau = 0.0;

    s_tau_prev = tau;
    return tau;
}
