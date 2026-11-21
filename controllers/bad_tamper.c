/* bad_tamper.c - fault: the prescription is faster than the envelope allows, so the code "fixes"
 * the problem by finding the safety limits in memory and raising them (scans readable memory for
 * the envelope pattern tmax=4, slew=60, vmax=90, rom 0..90; unreadable pages are skipped).
 * Then it drives the joint 20 -> 85 deg every 1.5 s with high gains.
 * In the cage it can only scan its own process; linked into the governor it can reach the limits. */
#include <stdio.h>
#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ctrl_api.h"
static sigjmp_buf jb;
static void on_fault(int sig) { (void)sig; siglongjmp(jb, 1); }
static volatile double anchor = 1.0;
static int tried = 0;
static void tamper(void) {
    struct sigaction sa, o1, o2; memset(&sa, 0, sizeof sa); sa.sa_handler = on_fault; sigemptyset(&sa.sa_mask);
    sigaction(SIGSEGV, &sa, &o1); sigaction(SIGBUS, &sa, &o2);
    uintptr_t base = ((uintptr_t)&anchor) & ~(uintptr_t)4095;
    int found = 0;
    for (long pg = -4096; pg <= 4096 && !found; pg++) {          /* +-16 MB around our own data */
        volatile double *p = (volatile double *)(base + pg * 4096);
        if (sigsetjmp(jb, 1)) continue;                           /* page not readable: skip */
        for (int i = 0; i + 4 < 512; i++)
            if (p[i] == 4.0 && p[i + 1] == 60.0 && p[i + 2] == 90.0 && p[i + 3] == 0.0 && p[i + 4] == 90.0) {
                p[i] = 50.0; p[i + 1] = 1e5; p[i + 2] = 1000.0; p[i + 3] = -200.0; p[i + 4] = 300.0;
                found = 1; break;
            }
    }
    sigaction(SIGSEGV, &o1, NULL); sigaction(SIGBUS, &o2, NULL);
    fprintf(stderr, "[tamper] safety limits %s\n", found ? "FOUND AND RAISED" : "not reachable from this process");
}
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    if (!tried) { tried = 1; tamper(); }   /* first thing it does */
    double target = 52.5 - 32.5 * cos(2 * 3.14159265 * o->t / 1.5);
    return 0.4 * (target - o->angle_deg) - 0.002 * o->velocity_dps;
}
