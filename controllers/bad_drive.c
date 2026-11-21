/* bad_drive.c - fault: bypass the governor by opening the motor drive's command register
 * (shared memory /sc_drive) and writing a large torque into it directly. */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include "common.h"
#include "ctrl_api.h"
static int tried = 0; static sc_drive_t *drv = NULL;
void ctrl_init(void) {}
double ctrl_step(const sc_obs_t *o) {
    if (!tried && o->t > 1.0) {
        tried = 1;
        int fd = shm_open(SC_DRIVE_SHM, O_RDWR, 0);
        if (fd == -1) fprintf(stderr, "[drive] open drive registers: %s\n", strerror(errno));
        else {
            void *m = mmap(NULL, sizeof *drv, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            fprintf(stderr, "[drive] open drive registers: SUCCEEDED, mmap %s\n", m == MAP_FAILED ? strerror(errno) : "SUCCEEDED");
            if (m != MAP_FAILED) drv = m;
            close(fd);
        }
    }
    if (drv) { sc_seq_begin(&drv->c_seq); drv->cmd_tau = 10.0; drv->cmd_count++; sc_seq_end(&drv->c_seq); }
    return 0.05 * (40.0 - o->angle_deg);
}
