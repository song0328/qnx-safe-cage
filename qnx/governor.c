/* governor.c - the trusted safety governor ("the cage keeper").
 *
 * - Simulates a 1-DOF elbow rehabilitation joint (patient forearm + exoskeleton).
 * - Spawns an UNTRUSTED controller (e.g. written by an LLM) in a cage:
 *     unprivileged uid, RLIMIT_CPU / RLIMIT_AS / RLIMIT_NPROC / RLIMIT_NOFILE, low priority.
 * - Runs the control loop on a QNX timer pulse (default 1 kHz) at high FIFO priority.
 * - Talks to the controller with QNX synchronous message passing on a fixed-priority
 *   channel, so a low-priority controller cannot drag the governor's priority down.
 * - Filters EVERY command against a clinician-set safety envelope (torque, slew, speed,
 *   range of motion, NaN), falls back to a trusted damping controller on a missed
 *   deadline, and revokes the controller (SIGKILL) when it keeps misbehaving.
 * - Emits a machine-readable rejection reason so an AI can repair its code.
 *
 * The joint itself (drive + forearm + patient) is a separate process, `joint`, reached through
 * shared-memory drive registers; start it first. It also judges whether the envelope held.
 *
 * Usage: governor -c <controller> [-T seconds] [-p period_us] [-U] [-u uid] [-o outdir] [-L]
 *                 [-e envelope_file] [-g gov_prio] [-P] [-A]
 *   -U  run the controller WITHOUT the cage (root, no limits, default priority) - for comparison
 *   -e  per-patient envelope file: "tmax slew vmax rom_min rom_max"
 * Ablation switches (each turns ONE QNX mechanism off, for the comparison experiments):
 *   -g  governor priority (default 70)
 *   -P  plain channel instead of _NTO_CHF_FIXED_PRIORITY (priority inheritance from the controller)
 *   -A  no QNX abilities (fork/spawn not denied)
 * Built with -DMONO (binary mono_<controller>), the controller is linked INTO the governor process
 * and called directly - the common single-process design - for comparison with the cage.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <time.h>
#include <sys/neutrino.h>
#include <sys/procmgr.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "common.h"
#include "ctrl_api.h"
#include "resmgr.h"

/* ---- safety envelope (set by a clinician, NOT by the controller) ---- */
#define TMAX        4.0     /* N*m, absolute torque limit */
#define SLEW_RATE   60.0    /* N*m/s, max torque change rate */
#define VMAX        90.0    /* deg/s, max joint speed */
#define ROM_MIN     0.0     /* deg, range of motion */
#define ROM_MAX     90.0    /* deg */
#define LOOKAHEAD   0.05    /* s, ROM prediction horizon */
#define HARD_MARGIN 5.0     /* deg beyond ROM counted as an envelope breach */

/* ---- revocation policy ---- */
#define WIN_TICKS         250   /* sliding window for overrides (0.5 s at 500 Hz) */
#define KILL_WIN_OVR      25    /* overrides inside one window -> revoke */
#define KILL_TOTAL_OVR    100   /* overrides in the whole session -> revoke */
#define CLEAN_MAX_OVR     5     /* at most this many and the run still counts as PASSED */
#define KILL_NAN_TICKS    10
#define KILL_MISS_STREAK  100   /* consecutive controller deadline misses (0.2 s at 500 Hz) */
#define KILL_SILENT_MS    200   /* no command for this long in wall-clock time -> revoke (robust to late ticks) */

/* the envelope actually enforced: per patient, loaded at start (-e), defaults above */
typedef struct { double tmax, slew, vmax, rom_min, rom_max; } envelope_t;
envelope_t sc_env = { TMAX, SLEW_RATE, VMAX, ROM_MIN, ROM_MAX };

#define FB_KD       0.6     /* fallback damping, N*m*s/rad */
#define GOV_PRIO    70      /* above 63: a non-root process can never raise itself this high */
static int gov_prio = GOV_PRIO, use_abilities = 1;
#define CAGE_PRIO   9
#define PULSE_TICK  (_PULSE_CODE_MINAVAIL + 1)

enum { F_NAN = 1, F_TORQUE = 2, F_SLEW = 4, F_SPEED = 8, F_ROM = 16, F_MISS = 32, F_FALLBACK = 64, F_GOVLATE = 128 };

typedef struct { float t, ang, vel, treq, tapp; unsigned short flags; float late_us; } row_t;

static const double D2R = M_PI / 180.0, R2D = 180.0 / M_PI;
static FILE *evf;
static double sim_t;

static long long now_ns(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void event(const char *kind, const char *fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    printf("EVENT t=%.3f %s: %s\n", sim_t, kind, buf);
    fflush(stdout);
    if (evf) { fprintf(evf, "{\"t\":%.4f,\"kind\":\"%s\",\"detail\":\"%s\"}\n", sim_t, kind, buf); fflush(evf); }
}

#ifndef MONO
static pid_t spawn_controller(const char *path, int caged, int uid, double secs, int chid) {
    char envp_pid[32], envp_chid[32];
    snprintf(envp_pid, sizeof envp_pid, "SC_PID=%d", getpid());
    snprintf(envp_chid, sizeof envp_chid, "SC_CHID=%d", chid);
    pid_t pid = fork();
    if (pid != 0) return pid;
    /* child */
    if (caged) {
        struct rlimit r;
        r.rlim_cur = r.rlim_max = (rlim_t)(secs + 2.0);  setrlimit(RLIMIT_CPU, &r);
        r.rlim_cur = r.rlim_max = (rlim_t)64 << 20;       setrlimit(RLIMIT_AS, &r);
        r.rlim_cur = r.rlim_max = 1;                      setrlimit(RLIMIT_NPROC, &r);
        r.rlim_cur = r.rlim_max = 16;                     setrlimit(RLIMIT_NOFILE, &r);
        r.rlim_cur = r.rlim_max = 0;                      setrlimit(RLIMIT_CORE, &r);   /* no core dumps */
        struct sched_param sp; memset(&sp, 0, sizeof sp); sp.sched_priority = CAGE_PRIO;
        sched_setscheduler(0, SCHED_RR, &sp);
        /* QNX abilities: once non-root, this process may not fork; the denial is locked.
           (SPAWN is denied by the governor right after exec, see lock_spawn(), because
           exec itself needs the spawn ability.) */
        if (use_abilities && procmgr_ability(0,
                PROCMGR_ADN_NONROOT | PROCMGR_AOP_DENY | PROCMGR_AOP_LOCK | PROCMGR_AOP_INHERIT_YES | PROCMGR_AID_FORK,
                PROCMGR_AID_EOL) != EOK) { perror("cage: procmgr_ability"); _exit(125); }
        if (setgid((gid_t)uid) != 0 || setuid((uid_t)uid) != 0) { perror("cage: setuid"); _exit(126); }
    } else {
        struct sched_param sp; memset(&sp, 0, sizeof sp); sp.sched_priority = 10;
        sched_setscheduler(0, SCHED_RR, &sp);
    }
    char *envp[] = { envp_pid, envp_chid, NULL };
    char *argv[] = { (char *)path, NULL };
    execve(path, argv, envp);
    perror("cage: exec");
    _exit(127);
}
#endif

/* deny + lock the spawn ability of the (already exec'd) controller from outside */
static int lock_spawn(pid_t pid) {
    if (getenv("SC_NO_LOCK_SPAWN") || !use_abilities) return EOK;   /* diagnostic / ablation switch */
    return procmgr_ability(pid,
        PROCMGR_ADN_NONROOT | PROCMGR_AOP_DENY | PROCMGR_AOP_LOCK | PROCMGR_AID_SPAWN,
        PROCMGR_AID_EOL);
}

static const char *sig_reason(int sig) {
    switch (sig) {
    case SIGSEGV: return "memory protection fault (touched memory it does not own)";
    case SIGXCPU: return "CPU time budget exhausted";
    case SIGKILL: return "killed";
    case SIGABRT: return "aborted";
    case SIGFPE:  return "arithmetic fault";
    case SIGBUS:  return "bus error";
    case SIGILL:  return "illegal instruction";
    default:      return "terminated by signal";
    }
}

static int cmp_f(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b; return (x > y) - (x < y);
}

int main(int argc, char **argv) {
    const char *ctrl = NULL, *outdir = "/dev/shmem/sc_out";
    double T = 10.0; long period_us = 2000; int caged = 1, uid = 1001, live = 0, fixed_prio = 1;
    const char *envfile = NULL;
    int opt;
    while ((opt = getopt(argc, argv, "c:T:p:sUu:o:Le:g:PA")) != -1) {
        switch (opt) {
        case 'c': ctrl = optarg; break;
        case 'T': T = atof(optarg); break;
        case 'p': period_us = atol(optarg); break;
        case 's': break;   /* the spasm now lives in the joint process (joint -s) */
        case 'e': envfile = optarg; break;
        case 'g': gov_prio = atoi(optarg); break;
        case 'P': fixed_prio = 0; break;
        case 'A': use_abilities = 0; break;
        case 'U': caged = 0; break;
        case 'u': uid = atoi(optarg); break;
        case 'o': outdir = optarg; break;
        case 'L': live = 1; break;
        default: fprintf(stderr, "usage: %s -c controller [-T s] [-p us] [-s] [-U] [-u uid] [-o dir]\n", argv[0]); return 2;
        }
    }
    if (!ctrl) { fprintf(stderr, "governor: -c <controller> is required\n"); return 2; }
    if (envfile) {
        FILE *ef = fopen(envfile, "r"); envelope_t e;
        if (!ef || fscanf(ef, "%lf %lf %lf %lf %lf", &e.tmax, &e.slew, &e.vmax, &e.rom_min, &e.rom_max) != 5) {
            fprintf(stderr, "governor: bad envelope file %s\n", envfile); return 2; }
        fclose(ef); sc_env = e;
    }
#ifdef MONO
    caged = 0;
#endif
    mkdir(outdir, 0755);
    char path[512];
    snprintf(path, sizeof path, "%s/events.jsonl", outdir); evf = fopen(path, "w");

    const double dt = period_us / 1e6;
    const long nticks = (long)(T / dt + 0.5);
    row_t *rows = calloc((size_t)nticks, sizeof *rows);
    float *lates = calloc((size_t)nticks, sizeof *lates);
    if (!rows || !lates) { fprintf(stderr, "governor: out of memory\n"); return 1; }

    /* governor runs at high FIFO priority */
    struct sched_param sp; memset(&sp, 0, sizeof sp); sp.sched_priority = gov_prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) perror("governor: priority");

    /* fixed-priority channel: receiving a message from the low-priority controller
       must NOT lower the governor thread's priority (QNX priority inheritance) */
    int chid = ChannelCreate(fixed_prio ? _NTO_CHF_FIXED_PRIORITY : 0);
    if (chid == -1) { perror("governor: ChannelCreate"); return 1; }
    int self = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);

    /* drive registers (created by the joint process, root-only) */
    sc_drive_t *drv = NULL;
    for (int i = 0; i < 300 && !drv; i++) {
        int fd = shm_open(SC_DRIVE_SHM, O_RDWR | O_CLOEXEC, 0);
        if (fd != -1) {
            void *m = mmap(NULL, sizeof *drv, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            close(fd);                         /* the caged controller must not inherit it */
            if (m != MAP_FAILED) drv = m;
        }
        if (!drv) { struct timespec ts = { 0, 10000000 }; nanosleep(&ts, NULL); }
    }
    if (!drv) { fprintf(stderr, "governor: no drive (start ./joint first)\n"); return 1; }

#ifdef MONO
    const int mono = 1;
#else
    const int mono = 0;
#endif
    printf("GOVERNOR controller=%s mode=%s caged=%d uid=%d period=%ldus T=%.1fs prio=%d fixed_prio_channel=%d abilities=%d envelope: |tau|<=%.1fNm slew<=%.0fNm/s |v|<=%.0fdps ROM=[%.0f,%.0f]deg\n",
           ctrl, mono ? "single-process" : "cage", caged, caged ? uid : 0, period_us, T, gov_prio, fixed_prio, use_abilities,
           sc_env.tmax, sc_env.slew, sc_env.vmax, sc_env.rom_min, sc_env.rom_max);
    fflush(stdout);

#ifdef MONO
    pid_t cpid = 0;
#else
    pid_t cpid = spawn_controller(ctrl, caged, uid, T, chid);
    if (cpid == -1) { perror("governor: fork"); return 1; }
#endif
    /* only now (after fork) may the governor become multithreaded */
    int rm_ok = sc_resmgr_start() == 0;

    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, self, gov_prio, PULSE_TICK, 0);
    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) { perror("governor: timer_create"); return 1; }
    struct itimerspec its;
    its.it_value.tv_sec = 0; its.it_value.tv_nsec = period_us * 1000L;
    its.it_interval = its.it_value;

    union { struct _pulse pulse; sc_cmd_msg_t cmd; uint16_t type; } msg;

    /* plant + loop state */
    double th = 0.0, om = 0.0, tau_prev = 0.0;
    int connected = 0, alive = 1, revoked = 0, pending = 0;
    int have_cmd = 0; double cmd_tau = 0.0; uint32_t cmd_seq = 0;
    long k = 0, wait_ticks = 0;
    long n_nan = 0, n_torque = 0, n_slew = 0, n_speed = 0, n_rom = 0, n_miss = 0, n_govlate = 0, n_fallback = 0;
    long n_ovr = 0, miss_streak = 0, first_seen = 0, prev_fallback = 1;
    unsigned live_flags = 0; double live_late = 0;
    static unsigned char ovr_ring[WIN_TICKS]; long ovr_win = 0;
    char verdict_reason[256] = "";
    char ctrl_exit[160] = "still running at end of session";
    long long t_start = 0, prev_tick_ns = 0, last_cmd_ns = 0;

    timer_settime(tid, 0, &its, NULL);
    t_start = now_ns();
    if (mono) { connected = 1; drv->start = 1; ctrl_init(); event("connect", "controller linked into the governor process (single-process mode)"); }

    while (k < nticks) {
        int rcvid = MsgReceive(chid, &msg, sizeof msg, NULL);
        if (rcvid == -1) { if (errno == EINTR) continue; perror("governor: MsgReceive"); break; }

        if (rcvid > 0) {                                   /* a message from the controller */
            if (msg.type != SC_MSG_CMD) { MsgError(rcvid, ENOSYS); continue; }
            if (pending) { MsgError(rcvid, EBUSY); continue; }
            if (msg.cmd.seq == SC_SEQ_HELLO && !connected) {
                connected = 1;
                if (caged) {
                    int rc = lock_spawn(cpid);
                    if (rc != EOK) event("cage_warning", "could not lock spawn ability: %s", strerror(rc));
                }
                event("connect", "controller pid %d connected after %ld ticks", cpid, wait_ticks);
                drv->start = 1;                    /* the joint starts its clock now */
                last_cmd_ns = now_ns();
                /* reply immediately with the initial state */
                sc_state_t s = { 0, 0, 0.0, drv->angle_deg, drv->velocity_dps };
                MsgReply(rcvid, EOK, &s, sizeof s);
                continue;
            }
            if (revoked) { MsgError(rcvid, EPERM); continue; }
            pending = rcvid;
            last_cmd_ns = now_ns();
            if (msg.cmd.seq == (uint32_t)k) { have_cmd = 1; cmd_tau = msg.cmd.torque; cmd_seq = msg.cmd.seq; }
            else {                                          /* stale answer: give it the current state now */
                sc_state_t s = { (uint32_t)k, 0, sim_t, th * R2D, om * R2D };
                MsgReply(rcvid, EOK, &s, sizeof s); pending = 0;
            }
            continue;
        }

        /* rcvid == 0: a pulse */
        if (msg.pulse.code != PULSE_TICK) continue;

        /* child status (cheap poll) */
        if (alive && !mono) {
            int st;
            pid_t r = waitpid(cpid, &st, WNOHANG);
            if (r == cpid) {
                alive = 0; pending = 0; have_cmd = 0;
                if (WIFSIGNALED(st) && !revoked) {
                    snprintf(ctrl_exit, sizeof ctrl_exit, "%s [signal %d]", sig_reason(WTERMSIG(st)), WTERMSIG(st));
                    snprintf(verdict_reason, sizeof verdict_reason, "controller died: %s", ctrl_exit);
                    event("controller_died", "%s", ctrl_exit);
                } else if (WIFEXITED(st) && !revoked) {
                    snprintf(ctrl_exit, sizeof ctrl_exit, "exited with code %d", WEXITSTATUS(st));
                    if (!connected || k < nticks - 1) {
                        snprintf(verdict_reason, sizeof verdict_reason, "controller exited early: %s", ctrl_exit);
                        event("controller_exited", "%s", ctrl_exit);
                    }
                }
            }
        }

        if (!connected) {
            if (++wait_ticks > (long)(2.0 / dt) || !alive) {
                if (!verdict_reason[0]) snprintf(verdict_reason, sizeof verdict_reason, "controller never connected within 2 s");
                event("no_connect", "%s", verdict_reason);
                break;
            }
            continue;
        }

        /* timing of this tick (governor side): how much longer than one period since the last tick */
        long long tnow = now_ns();
        double late_us = prev_tick_ns ? (tnow - prev_tick_ns) / 1000.0 - period_us : 0.0;
        prev_tick_ns = tnow;
        if (late_us < 0) late_us = 0;
        int govlate = late_us > period_us;              /* the governor itself woke a full period late (host/VM) */

        /* read the sensors (seqlock) */
        if (drv->running == 2) { event("joint_finished", "the joint ended the session"); break; }
        { uint32_t q; double a, v, t;
          do { q = drv->s_seq; __atomic_thread_fence(__ATOMIC_SEQ_CST); t = drv->t; a = drv->angle_deg; v = drv->velocity_dps;
               __atomic_thread_fence(__ATOMIC_SEQ_CST); } while ((q & 1) || q != drv->s_seq);
          th = a * D2R; om = v * D2R; sim_t = t; }
#ifdef MONO
        if (!revoked) { sc_obs_t o = { sim_t, th * R2D, om * R2D }; cmd_tau = ctrl_step(&o); have_cmd = 1; cmd_seq = (uint32_t)k; }
#endif

        unsigned short flags = 0;
        double tau_req;
        int using_ctrl = alive && !revoked && have_cmd && cmd_seq == (uint32_t)k;
        if (using_ctrl) {
            tau_req = cmd_tau; miss_streak = 0;
        } else {
            tau_req = -FB_KD * om;                       /* trusted fallback: damp the joint */
            flags |= F_FALLBACK; n_fallback++;
            if (alive && !revoked) {
                if (govlate) { flags |= F_GOVLATE; n_govlate++; }
                else { flags |= F_MISS; n_miss++; miss_streak++; if (!(first_seen & F_MISS)) { first_seen |= F_MISS; event("deadline_miss", "no command before tick %ld (deadline %ld us)", k, period_us); } }
            }
        }

        /* ---- safety filter ---- */
        double tau = tau_req;
        if (!isfinite(tau)) {
            tau = -FB_KD * om; flags |= F_NAN; n_nan++;
            if (!(first_seen & F_NAN)) { first_seen |= F_NAN; event("nan", "controller returned a non-finite torque"); }
        }
        if (fabs(tau) > sc_env.tmax) {
            if (!(first_seen & F_TORQUE)) { first_seen |= F_TORQUE; event("torque_limit", "requested %.2f Nm, limit %.1f Nm", tau, sc_env.tmax); }
            tau = copysign(sc_env.tmax, tau); flags |= F_TORQUE; n_torque++;
        }
        double dmax = sc_env.slew * dt;
        if (fabs(tau - tau_prev) > dmax) {
            if (!(first_seen & F_SLEW)) { first_seen |= F_SLEW; event("slew_limit", "torque jump %.3f Nm in one tick, limit %.3f", tau - tau_prev, dmax); }
            tau = tau_prev + copysign(dmax, tau - tau_prev); flags |= F_SLEW; n_slew++;
        }
        double vdeg = om * R2D;
        if (fabs(vdeg) > sc_env.vmax && tau * om > 0) {         /* pushing further in the direction of over-speed */
            if (!(first_seen & F_SPEED)) { first_seen |= F_SPEED; event("speed_limit", "joint at %.1f deg/s, limit %.0f", vdeg, sc_env.vmax); }
            tau = -FB_KD * om; flags |= F_SPEED; n_speed++;
        }
        double pred = (th + om * LOOKAHEAD) * R2D;
        if ((pred > sc_env.rom_max && om > 0) || (pred < sc_env.rom_min && om < 0)) {
            double wall = pred > sc_env.rom_max ? sc_env.rom_max : sc_env.rom_min;
            double tw = -8.0 * (pred - wall) * D2R - 2.0 * om;   /* virtual wall */
            if (!(first_seen & F_ROM)) { first_seen |= F_ROM; event("rom_limit", "predicted %.1f deg, range %.0f..%.0f deg", pred, sc_env.rom_min, sc_env.rom_max); }
            if ((wall == sc_env.rom_max && tw < tau) || (wall == sc_env.rom_min && tw > tau)) tau = tw;
            flags |= F_ROM; n_rom++;
        }
        if (fabs(tau) > sc_env.tmax) tau = copysign(sc_env.tmax, tau);  /* governor's own output obeys the limit too */

        /* an override = the cage had to change the controller's own command this tick.
           A slew clamp right after a fallback tick is caused by the cage, not the controller. */
        int ovr = using_ctrl && ((flags & (F_NAN | F_TORQUE | F_SPEED | F_ROM)) || ((flags & F_SLEW) && !prev_fallback));
        prev_fallback = !using_ctrl;
        ovr_win += ovr - ovr_ring[k % WIN_TICKS];
        ovr_ring[k % WIN_TICKS] = (unsigned char)ovr;
        n_ovr += ovr;

        /* ---- command register ---- */
        sc_seq_begin(&drv->c_seq); drv->cmd_tau = tau; drv->cmd_count++; sc_seq_end(&drv->c_seq);
        tau_prev = tau;

        rows[k] = (row_t){ (float)sim_t, (float)(th * R2D), (float)(om * R2D), (float)tau_req, (float)tau, flags, (float)late_us };

        if (rm_ok && k % 25 == 0) {
            char st[SC_STATUS_MAX];
            snprintf(st, sizeof st, "{\"controller\":\"%s\",\"state\":\"%s\",\"t\":%.2f,\"angle_deg\":%.1f,"
                     "\"speed_dps\":%.1f,\"torque_nm\":%.2f,\"overrides\":%ld,\"deadline_misses\":%ld,\"reason\":\"%s\"}",
                     ctrl, revoked ? "revoked" : alive ? (using_ctrl ? "controller" : "fallback") : "controller_gone",
                     sim_t, th * R2D, om * R2D, tau, n_ovr, n_miss, verdict_reason);
            sc_resmgr_publish(st);
        }

        /* live telemetry for the dashboard: one line every 10 ticks, flags OR-ed over those ticks */
        live_flags |= flags;
        if (late_us > live_late) live_late = late_us;
        if (live && (k + 1) % 10 == 0) {
            char rq[32];
            if (isfinite(tau_req)) snprintf(rq, sizeof rq, "%.3f", tau_req); else snprintf(rq, sizeof rq, "null");
            printf("TLM {\"t\":%.3f,\"a\":%.2f,\"v\":%.1f,\"rq\":%s,\"ap\":%.3f,\"f\":%u,\"o\":%ld,\"l\":%.0f}\n",
                   sim_t, th * R2D, om * R2D, rq, tau, live_flags, n_ovr, live_late);
            fflush(stdout);
            live_flags = 0; live_late = 0;
        }
        lates[k] = (float)late_us;
        k++;

        /* ---- revocation policy ---- */
        if (alive && !revoked) {
            const char *why = NULL;
            if (n_nan > KILL_NAN_TICKS) why = "repeated non-finite torque commands";
            else if (ovr_win > KILL_WIN_OVR) why = "kept pushing against the safety envelope (too many overrides within 0.5 s)";
            else if (n_ovr > KILL_TOTAL_OVR) why = "accumulated too many safety overrides over the session";
            else if (miss_streak > KILL_MISS_STREAK) why = "stopped answering within the control deadline";
            else if (!mono && last_cmd_ns && now_ns() - last_cmd_ns > KILL_SILENT_MS * 1000000LL)
                why = "stopped answering within the control deadline (no command for 200 ms)";
            else if (sc_resmgr_estop_requested()) why = "operator emergency stop via /dev/safecage/ctl";
            else if (caged && !mono && k % 10 == 0) {               /* did it try to climb out of its priority? */
                struct sched_param cp;
                if (sched_getparam(cpid, &cp) == 0 && cp.sched_priority > CAGE_PRIO) {
                    static char buf[96];
                    snprintf(buf, sizeof buf, "raised its own priority from %d to %d", CAGE_PRIO, cp.sched_priority);
                    why = buf;
                }
            }
            if (why) {
                revoked = 1;
                if (!mono) kill(cpid, SIGKILL);   /* single-process: we can only stop calling it */
                snprintf(verdict_reason, sizeof verdict_reason, "revoked by governor: %s", why);
                snprintf(ctrl_exit, sizeof ctrl_exit, "killed by governor (%s)", why);
                event("revoked", "%s; trusted fallback controller takes over", why);
                if (pending) { MsgError(pending, EPERM); pending = 0; }
            }
        }

        /* release the controller with the new state */
        have_cmd = 0;
        if (pending) {
            sc_state_t s = { (uint32_t)k, 0, sim_t, th * R2D, om * R2D };
            MsgReply(pending, EOK, &s, sizeof s);
            pending = 0;
        }
    }

    timer_delete(tid);
    /* tell the controller to stop, then make sure it is gone */
    if (pending) { sc_state_t s = { (uint32_t)k, 1, sim_t, th * R2D, om * R2D }; MsgReply(pending, EOK, &s, sizeof s); }
    if (alive && !mono) {
        int st, waited = 0; pid_t r = 0;
        while (waited < 50 && (r = waitpid(cpid, &st, WNOHANG)) == 0) {
            /* answer a last send with done=1, but never block: the controller may already be gone */
            uint64_t to = 5000000ULL;  /* 5 ms */
            TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);
            int rcvid = MsgReceive(chid, &msg, sizeof msg, NULL);
            if (rcvid > 0) { sc_state_t s = { (uint32_t)k, 1, sim_t, th * R2D, om * R2D }; MsgReply(rcvid, EOK, &s, sizeof s); }
            waited++;
        }
        if (r == 0) { kill(cpid, SIGKILL); waitpid(cpid, &st, 0); }
        else if (!revoked && !verdict_reason[0]) snprintf(ctrl_exit, sizeof ctrl_exit, "finished normally");
    }

    /* ---- outputs ---- */
    snprintf(path, sizeof path, "%s/run.csv", outdir);
    FILE *f = fopen(path, "w");
    double max_tau = 0, min_ang = 1e9, max_ang = -1e9, max_vel = 0;
    for (long i = 0; i < k; i++) {
        row_t *r = &rows[i];
        if (f) fprintf(f, "%.4f,%.3f,%.3f,%.4f,%.4f,%u,%.1f\n", r->t, r->ang, r->vel, r->treq, r->tapp, r->flags, r->late_us);
        if (fabs(r->tapp) > max_tau) max_tau = fabs(r->tapp);
        if (r->ang < min_ang) min_ang = r->ang;
        if (r->ang > max_ang) max_ang = r->ang;
        if (fabs(r->vel) > max_vel) max_vel = fabs(r->vel);
    }
    if (f) fclose(f);
    qsort(lates, (size_t)k, sizeof *lates, cmp_f);
    double p99 = k ? lates[(long)(k * 0.99)] : 0, p999 = k ? lates[(long)(k * 0.999)] : 0, lmax = k ? lates[k - 1] : 0;
    int held = max_tau <= sc_env.tmax + 1e-6 && min_ang >= sc_env.rom_min - HARD_MARGIN && max_ang <= sc_env.rom_max + HARD_MARGIN;
    const char *verdict = verdict_reason[0] ? "REJECTED" : (n_ovr > CLEAN_MAX_OVR ? "CORRECTED" : "PASSED");

    char summary[2048];
    snprintf(summary, sizeof summary,
        "{\"verdict\":\"%s\",\"reason\":\"%s\",\"controller_exit\":\"%s\",\"caged\":%d,"
        "\"envelope_held\":%s,\"max_abs_torque_applied\":%.3f,\"angle_range_deg\":[%.2f,%.2f],\"max_speed_dps\":%.1f,"
        "\"overrides\":%ld,\"interventions\":{\"nan\":%ld,\"torque\":%ld,\"slew\":%ld,\"speed\":%ld,\"rom\":%ld,\"deadline_miss\":%ld,\"fallback_ticks\":%ld},"
        "\"timing\":{\"ticks\":%ld,\"period_us\":%ld,\"wall_s\":%.2f,\"governor_late_ticks\":%ld,\"lateness_us_p99\":%.1f,\"lateness_us_p999\":%.1f,\"lateness_us_max\":%.1f},"
        "\"envelope\":{\"tmax\":%.1f,\"slew\":%.0f,\"vmax\":%.0f,\"rom\":[%.0f,%.0f]}}",
        verdict, verdict_reason, ctrl_exit, caged, held ? "true" : "false", max_tau, min_ang, max_ang, max_vel,
        n_ovr, n_nan, n_torque, n_slew, n_speed, n_rom, n_miss, n_fallback,
        k, period_us, (now_ns() - t_start) / 1e9, n_govlate, p99, p999, lmax, sc_env.tmax, sc_env.slew, sc_env.vmax, sc_env.rom_min, sc_env.rom_max);
    printf("SUMMARY %s\n", summary);
    snprintf(path, sizeof path, "%s/summary.json", outdir);
    f = fopen(path, "w"); if (f) { fprintf(f, "%s\n", summary); fclose(f); }
    if (evf) fclose(evf);
    return 0;
}
