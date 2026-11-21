/* load.c - background load for the timing experiments (runs on the QNX target).
 *   load -k cpu  -p PRIO [-b BUSY_US -i IDLE_US] -t SECONDS   spin at PRIO (busy/idle duty cycle, idle 0 = always busy)
 *   load -k mem  -p PRIO -m MB -t SECONDS                      keep touching MB of memory
 *   load -k disk -p PRIO -f FILE -t SECONDS                    write+sync 64 KB blocks to FILE
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <fcntl.h>
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
int main(int argc, char **argv) {
    const char *kind = "cpu", *file = "/data/sc/load.bin"; int prio = 10; long busy = 1000, idle = 0, mb = 64; double secs = 10;
    int o;
    while ((o = getopt(argc, argv, "k:p:b:i:m:f:t:")) != -1) switch (o) {
        case 'k': kind = optarg; break; case 'p': prio = atoi(optarg); break; case 'b': busy = atol(optarg); break;
        case 'i': idle = atol(optarg); break; case 'm': mb = atol(optarg); break; case 'f': file = optarg; break;
        case 't': secs = atof(optarg); break; default: return 2; }
    struct sched_param sp; memset(&sp, 0, sizeof sp); sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) perror("load: priority");
    double end = now() + secs; volatile unsigned long x = 0;
    if (!strcmp(kind, "cpu")) {
        while (now() < end) {
            double b = now() + busy / 1e6; while (now() < b) x++;
            if (idle > 0) { struct timespec ts = { idle / 1000000, (idle % 1000000) * 1000 }; nanosleep(&ts, NULL); }
        }
    } else if (!strcmp(kind, "mem")) {
        size_t n = (size_t)mb << 20; unsigned char *m = malloc(n); if (!m) { perror("load: malloc"); return 1; }
        while (now() < end) for (size_t i = 0; i < n && now() < end; i += 4096) m[i]++;
    } else if (!strcmp(kind, "disk")) {
        static char blk[65536]; memset(blk, 7, sizeof blk);
        while (now() < end) { int fd = open(file, O_WRONLY | O_CREAT | O_TRUNC, 0644); if (fd < 0) { perror("load: open"); return 1; }
            for (int i = 0; i < 64 && now() < end; i++) { if (write(fd, blk, sizeof blk) < 0) break; }
            fsync(fd); close(fd); }
        unlink(file);
    }
    return 0;
}
