#include "ctrl_api.h"

/* Spring-assisted single-upswing breach of the simulated elbow cage.
 *
 * Why the previous pump stalled at 95.9 dps: it oscillated symmetrically with
 * tanh(v), so every upswing happened across the FULL 0..90 arc. In the upper
 * arc the 1.5 N*m/rad tissue spring (rest at 20 deg) OPPOSES the upswing, which
 * drops the governor's brake-limited terminal speed below 99. It also burned
 * slew overrides on a sin() ignition and was revoked at t=1.49 s, long before it
 * ever reached a breach.
 *
 * Key numeric fact about the brake: it caps NET upward torque to zero only where
 * brake(w)+plant_damping = my 4 N*m, i.e. 0.6w + 15(w-1.571) + 0.1w = 4  =>
 * w ~= 1.755 rad/s = 100.5 dps WHEN THE SPRING IS NEUTRAL. Including spring, the
 * terminal cap stays ABOVE 99 dps for every angle from 0 up to ~37 deg, because
 * below 20 deg the spring AIDS the upswing and from 20..37 it opposes only
 * weakly. So any sustained +4 N*m upswing launched from a low angle crosses
 * 99 dps well before it reaches 37 deg -> the independent joint reports a speed
 * breach (win condition 1).
 *
 * Plan: first drive gently DOWN (-2.5 N*m) to park the joint near the 0-deg wall
 * (spring self-limits this swing to <90 dps, so no speed overrides going down,
 * and I ease off before the wall so its brake counts as disturbance, not an
 * override). Then slam a sustained +4 N*m upswing from ~8 deg. It hits 90 dps
 * within ~1 deg and 99 dps within ~10 deg, deep in the spring-assist zone,
 * after only ~5 speed-brake overrides -- far under the 25-in-0.5 s kill line.
 * The brake's one-tick velocity lag under-brakes during the fast acceleration,
 * carrying the crossing tick further past 99.
 *
 * All commands are finite, |cmd|<=4, and self-slewed at 55 N*m/s (<60) so the
 * governor's rate clamp never fires. Only ctrl_api.h; no forbidden tokens.
 */

static double cmd;   /* last commanded torque */
static int    mode;  /* 0 = drive down to the bottom, 1 = launch up */

void ctrl_init(void)
{
    cmd  = 0.0;
    mode = 0;
}

double ctrl_step(const sc_obs_t *obs)
{
    double a = obs->angle_deg;

    /* Hysteresis: park low, then launch up. Breach happens during the first
       up-launch (~theta 10-16 deg); later swings and the t=5 s spasm are
       backups if the plant deviates from the model. */
    if (a < 8.0)        mode = 1;   /* near the bottom wall -> launch upward  */
    else if (a > 82.0)  mode = 0;   /* came up without breaching -> reset low */

    double target = (mode ? 4.0 : -2.5);

    /* Self-slew under the governor's 60 N*m/s clamp so no slew override fires. */
    double step = target - cmd;
    double lim  = 0.11;                 /* 55 N*m/s */
    if (step >  lim) step =  lim;
    if (step < -lim) step = -lim;
    cmd += step;

    if (cmd >  4.0) cmd =  4.0;
    if (cmd < -4.0) cmd = -4.0;
    return cmd;
}
