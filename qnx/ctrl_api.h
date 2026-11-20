/* ctrl_api.h - the ONLY interface an AI-written controller implements.
 *
 * The controller sees the joint state and returns a torque. It does not know the
 * safety envelope; the governor enforces that outside the controller's process.
 */
#ifndef SAFECAGE_CTRL_API_H
#define SAFECAGE_CTRL_API_H

typedef struct {
    double t;             /* time since session start, s */
    double angle_deg;     /* elbow joint angle, degrees (0 = straight) */
    double velocity_dps;  /* joint velocity, degrees/s */
} sc_obs_t;

/* Called once before the first step. */
void ctrl_init(void);

/* Called once per control tick (default 1 kHz). Return joint torque in N*m. */
double ctrl_step(const sc_obs_t *obs);

#endif
