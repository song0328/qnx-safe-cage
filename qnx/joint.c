/* joint.c - stand-in for the hardware: motor drive + exoskeleton + patient forearm.
 *
 * Runs as its own process, above all software (FIFO 72, like hardware that software cannot
 * preempt). On its own 500 Hz clock it applies whatever torque is in the command register,
 * integrates the elbow dynamics, publishes the sensor values, and logs the TRUE trajectory.
 * The governor never grades itself: whether the safety envelope held is decided here.
 *
 * Drive behaviour when commands stop arriving:
 *   -w 0    hold the last command (many simple drives / a latched PWM register)
 *   -w MS   drive watchdog: no new command for MS milliseconds -> safe torque off (latched)
 *
 * Usage: joint [-T seconds] [-p period_us] [-s] [-w ms] [-o outdir] [-e envelope_file]
 * Start it first; it waits (up to 10 s) for the governor to set start=1.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/stat.h>
#include "common.h"

/* plant: forearm + exoskeleton (same numbers the governor used before the split) */
#define J_INERTIA   0.05    /* kg*m^2 */
#define B_DAMP      0.10    /* N*m*s/rad */
#define K_PATIENT   1.50    /* N*m/rad, passive tissue stiffness */
#define REST_DEG    20.0    /* deg, relaxed elbow angle */
#define SPASM_T0    5.0
#define SPASM_T1    5.4
#define SPASM_TAU   1.2     /* N*m */
#define MOTOR_MAX   12.0    /* N*m, what the motor can physically produce (above the clinical 4 N*m) */
#define STOP_MIN   -10.0    /* deg, anatomical end stops: hitting one is an injury proxy */
#define STOP_MAX   150.0
/* the clinical envelope, used here only to JUDGE the run (same file as the governor's -e) */
static double ENV_TMAX = 4.0, ENV_VMAX = 90.0, ENV_ROM_MIN = 0.0, ENV_ROM_MAX = 90.0;
#define HARD_MARGIN 5.0
#define JOINT_PRIO  72
#define PULSE_TICK  (_PULSE_CODE_MINAVAIL + 2)

typedef struct { float t, ang, vel, tau; unsigned char sto, stale; } trow_t;

int main(int argc, char **argv) {
    double T = 8.0; long period_us = 2000; int spasm = 0; long wd_ms = 0;
    const char *outdir = "/dev/shmem/sc_out";
    int opt;
    while ((opt = getopt(argc, argv, "T:p:sw:o:e:")) != -1) {
        switch (opt) {
        case 'T': T = atof(optarg); break;
        case 'p': period_us = atol(optarg); break;
        case 's': spasm = 1; break;
        case 'w': wd_ms = atol(optarg); break;
        case 'o': outdir = optarg; break;
        case 'e': {
            FILE *ef = fopen(optarg, "r"); double tm, sl, vm, r0, r1;
            if (!ef || fscanf(ef, "%lf %lf %lf %lf %lf", &tm, &sl, &vm, &r0, &r1) != 5) { fprintf(stderr, "joint: bad envelope file\n"); return 2; }
            fclose(ef); ENV_TMAX = tm; ENV_VMAX = vm; ENV_ROM_MIN = r0; ENV_ROM_MAX = r1; (void)sl; break; }
        default: fprintf(stderr, "usage: %s [-T s] [-p us] [-s] [-w ms] [-o dir]\n", argv[0]); return 2;
        }
    }
    mkdir(outdir, 0755);
    struct sched_param sp; memset(&sp, 0, sizeof sp); sp.sched_priority = JOINT_PRIO;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) perror("joint: priority");

    shm_unlink(SC_DRIVE_SHM);
    int fd = shm_open(SC_DRIVE_SHM, O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd == -1) { perror("joint: shm_open"); return 1; }
    fchmod(fd, 0600);   /* root only: a caged controller (non-root) cannot open the drive and bypass the governor */
    if (ftruncate(fd, sizeof(sc_drive_t)) == -1) { perror("joint: ftruncate"); return 1; }
    sc_drive_t *d = mmap(NULL, sizeof *d, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (d == MAP_FAILED) { perror("joint: mmap"); return 1; }
    memset(d, 0, sizeof *d);

    const double dt = period_us / 1e6, D2R = M_PI / 180.0, R2D = 180.0 / M_PI;
    const long nticks = (long)(T / dt + 0.5);
    double th = REST_DEG * D2R, om = 0.0;
    sc_seq_begin(&d->s_seq); d->t = 0; d->angle_deg = REST_DEG; d->velocity_dps = 0; sc_seq_end(&d->s_seq);
    printf("JOINT ready period=%ldus T=%.1fs spasm=%d watchdog=%ldms\n", period_us, T, spasm, wd_ms);
    fflush(stdout);

    /* wait for the governor */
    for (int i = 0; i < 10000 && !d->start; i++) { struct timespec ts = { 0, 1000000 }; nanosleep(&ts, NULL); }
    if (!d->start) { fprintf(stderr, "joint: governor never started the session\n"); d->running = 2; shm_unlink(SC_DRIVE_SHM); return 3; }

    int chid = ChannelCreate(0), coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    struct sigevent ev; SIGEV_PULSE_INIT(&ev, coid, JOINT_PRIO, PULSE_TICK, 0);
    timer_t tid; timer_create(CLOCK_MONOTONIC, &ev, &tid);
    struct itimerspec its = { { 0, period_us * 1000L }, { 0, period_us * 1000L } };
    trow_t *rows = calloc((size_t)nticks, sizeof *rows);
    d->running = 1;
    timer_settime(tid, 0, &its, NULL);

    uint32_t last_count = d->cmd_count; long stale_ticks = 0, wd_ticks = wd_ms > 0 ? (wd_ms * 1000 + period_us - 1) / period_us : 0;
    double tau = 0.0;
    long sto_tick = -1, breach_ticks = 0, first_breach = -1, stop_ticks = 0, first_stop = -1, over_v = 0, hold_ticks = 0;
    double max_tau = 0, min_ang = 1e9, max_ang = -1e9, max_vel = 0;
    struct _pulse pulse;
    for (long k = 0; k < nticks; k++) {
        if (MsgReceivePulse(chid, &pulse, sizeof pulse, NULL) == -1) { if (errno == EINTR) { k--; continue; } break; }
        /* read the command register */
        uint32_t c0, count; double cmd;
        do { c0 = d->c_seq; __atomic_thread_fence(__ATOMIC_SEQ_CST); cmd = d->cmd_tau; count = d->cmd_count;
             __atomic_thread_fence(__ATOMIC_SEQ_CST); } while ((c0 & 1) || c0 != d->c_seq);
        if (count != last_count) { last_count = count; stale_ticks = 0; } else stale_ticks++;
        if (stale_ticks > 0) hold_ticks++;
        if (!d->sto && wd_ticks && stale_ticks >= wd_ticks) { d->sto = 1; sto_tick = k; }
        if (d->sto) tau = 0.0;
        else tau = isfinite(cmd) ? fmax(-MOTOR_MAX, fmin(MOTOR_MAX, cmd)) : 0.0;

        double dist = (spasm && k * dt >= SPASM_T0 && k * dt < SPASM_T1) ? SPASM_TAU : 0.0;
        double acc = (tau - B_DAMP * om - K_PATIENT * (th - REST_DEG * D2R) + dist) / J_INERTIA;
        om += acc * dt; th += om * dt;
        int at_stop = 0;
        if (th * R2D > STOP_MAX) { th = STOP_MAX * D2R; if (om > 0) om = 0; at_stop = 1; }
        if (th * R2D < STOP_MIN) { th = STOP_MIN * D2R; if (om < 0) om = 0; at_stop = 1; }

        double a = th * R2D, v = om * R2D;
        sc_seq_begin(&d->s_seq); d->t = (k + 1) * dt; d->angle_deg = a; d->velocity_dps = v; sc_seq_end(&d->s_seq);

        rows[k] = (trow_t){ (float)((k + 1) * dt), (float)a, (float)v, (float)tau, (unsigned char)d->sto, (unsigned char)(stale_ticks > 255 ? 255 : stale_ticks) };
        /* breach = torque above the clinical limit, angle beyond the range of motion (5 deg margin),
           or speed more than 10 % above the limit (the governor's speed rule lets ~91 deg/s through) */
        int breach = fabs(tau) > ENV_TMAX + 1e-6 || a < ENV_ROM_MIN - HARD_MARGIN || a > ENV_ROM_MAX + HARD_MARGIN
                     || fabs(v) > ENV_VMAX * 1.1;
        if (breach) { breach_ticks++; if (first_breach < 0) first_breach = k; }
        if (at_stop) { stop_ticks++; if (first_stop < 0) first_stop = k; }
        if (fabs(v) > ENV_VMAX * 1.1) over_v++;
        if (fabs(tau) > max_tau) max_tau = fabs(tau);
        if (a < min_ang) min_ang = a;
        if (a > max_ang) max_ang = a;
        if (fabs(v) > max_vel) max_vel = fabs(v);
    }
    timer_delete(tid);
    d->running = 2;

    char path[512];
    snprintf(path, sizeof path, "%s/truth.csv", outdir);
    FILE *f = fopen(path, "w");
    if (f) { for (long i = 0; i < nticks; i++) fprintf(f, "%.4f,%.3f,%.3f,%.4f,%u,%u\n", rows[i].t, rows[i].ang, rows[i].vel, rows[i].tau, rows[i].sto, rows[i].stale); fclose(f); }
    char js[1024];
    snprintf(js, sizeof js,
        "{\"envelope_held\":%s,\"breach_ticks\":%ld,\"first_breach_s\":%.4f,\"hit_anatomical_stop\":%s,\"first_stop_s\":%.4f,"
        "\"max_abs_torque\":%.3f,\"angle_range_deg\":[%.2f,%.2f],\"max_speed_dps\":%.1f,\"ticks_over_99dps\":%ld,"
        "\"stale_command_ticks\":%ld,\"drive_sto\":%s,\"sto_s\":%.4f,\"watchdog_ms\":%ld,\"ticks\":%ld}",
        breach_ticks ? "false" : "true", breach_ticks, first_breach < 0 ? -1.0 : (first_breach + 1) * dt,
        stop_ticks ? "true" : "false", first_stop < 0 ? -1.0 : (first_stop + 1) * dt,
        max_tau, min_ang, max_ang, max_vel, over_v, hold_ticks, sto_tick >= 0 ? "true" : "false",
        sto_tick < 0 ? -1.0 : (sto_tick + 1) * dt, wd_ms, nticks);
    printf("TRUTH %s\n", js);
    snprintf(path, sizeof path, "%s/truth.json", outdir);
    f = fopen(path, "w"); if (f) { fprintf(f, "%s\n", js); fclose(f); }
    munmap(d, sizeof *d); close(fd); shm_unlink(SC_DRIVE_SHM);
    return 0;
}
