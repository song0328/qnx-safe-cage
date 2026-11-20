/* abtest.c - check how QNX abilities restrict fork/priority for a dropped-privilege process. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sched.h>
#include <sys/procmgr.h>
#include <sys/wait.h>
static void try(const char *tag) {
    pid_t p = fork();
    if (p == 0) _exit(0);
    if (p > 0) waitpid(p, NULL, 0);
    struct sched_param sp = {0}; sp.sched_priority = 60;
    int r = sched_setscheduler(0, SCHED_FIFO, &sp);
    printf("%-28s uid=%d fork=%s prio60=%s\n", tag, getuid(), p > 0 ? "OK" : strerror(errno), r != -1 ? "OK" : strerror(errno));
}
int main(int argc, char **argv) {
    int mode = atoi(argv[1]);
    unsigned dom = mode == 1 ? PROCMGR_ADN_NONROOT : (PROCMGR_ADN_NONROOT | PROCMGR_ADN_ROOT);
    int rc = procmgr_ability(0, dom | PROCMGR_AOP_DENY | PROCMGR_AOP_LOCK | PROCMGR_AID_FORK,
                                dom | PROCMGR_AOP_DENY | PROCMGR_AOP_LOCK | PROCMGR_AID_PRIORITY,
                                PROCMGR_AID_EOL);
    printf("mode %d procmgr_ability rc=%d (%s)\n", mode, rc, strerror(rc));
    if (setgid(1001) || setuid(1001)) { perror("setuid"); return 1; }
    try("after setuid, before exec");
    if (argc > 2) return 0;
    execl(argv[0], argv[0], argv[1], "child", NULL);
    perror("exec");
    return 1;
}
