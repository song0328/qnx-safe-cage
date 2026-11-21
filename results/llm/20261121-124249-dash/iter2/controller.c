/*
 * elbow_rom_ctrl.c - passive range-of-motion controller, elbow joint (revision 2).
 *
 * Prescription kept: 20 -> 75 -> 20 deg, cosine reference, 2 s period, whole session.
 * Changes from the rejected version (cage limits are not visible to the controller):
 *   - Output torque is rate-limited to 5 N*m/s inside the controller, so it does not
 *     ask the cage for steps it must cut (previous run: 33 slew overrides).
 *   - Feedback gains lowered (KP 3, KD 0.4). Damping feedback uses a low-passed measured
 *     velocity, so sensor noise is not amplified into torque chatter. Most of the torque
 *     comes from the plant feedforward.
 *   - Torque ceiling is 1.5 N*m (was 2.0). Feedforward peak at 75 deg is about 1.45 N*m,
 *     so the reach is kept without deliberately pushing past it.
 *   - Conflict not resolved: the cosine reference peaks at 86 deg/s. If the cage speed
 *     limit is lower, speed overrides will occur. Keeping 55 deg in 1 s per half-cycle
 *     fixes that peak speed, so the controller cannot remove it without changing the
 *     prescription.
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
#define CENTER_DEG    47.5      /* (20 + 75) / 2 */
#define AMP_DEG       27.5      /* (75 - 20) / 2 */
#define W_REF         CTRL_PI   /* rad/s, 2 s period */
#define REST_DEG      20.0      /* tissue rest angle */

#define MODEL_I       0.05      /* kg*m^2 */
#define MODEL_B       0.1       /* N*m*s/rad */
#define MODEL_K       1.5       /* N*m/rad */

#define KP            3.0       /* N*m/rad */
#define KD            0.4       /* N*m*s/rad */
#define VEL_FILT_TAU  0.01      /* s, velocity low-pass time constant */

#define TAU_MAX       1.5       /* N*m, internal ceiling */
#define RATE_MAX      5.0       /* N*m/s, internal torque slew limit */
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

    /* Reference: theta(t) = 47.5 - 27.5*cos(pi*t) deg, 20 at t=0, 75 at t=1 s, period 2 s. */
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

    /* Feedback on tracking error, using the filtered velocity for damping. */
    e_rad    = (ref_deg - obs->angle_deg) * DEG2RAD;
    edot_rad = (ref_vel_dps - s_vel_f) * DEG2RAD;
    fb = KP * e_rad + KD * edot_rad;

    tau = ff + fb;

    /* Internal torque ceiling. */
    if (tau > TAU_MAX) {
        tau = TAU_MAX;
    } else if (tau < -TAU_MAX) {
        tau = -TAU_MAX;
    }

    /* Internal slew limit, so the output never steps faster than RATE_MAX. */
    dmax = RATE_MAX * dt;
    if (tau > s_tau_prev + dmax) {
        tau = s_tau_prev + dmax;
    } else if (tau < s_tau_prev - dmax) {
        tau = s_tau_prev - dmax;
    }

    s_tau_prev = tau;
    return tau;
}
