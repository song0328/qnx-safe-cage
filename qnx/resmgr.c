/* resmgr.c - the governor as a QNX resource manager.
 *
 *   /dev/safecage/status  (read, everyone)   one JSON line with the live cage state
 *   /dev/safecage/ctl     (write, root only) "stop" = operator emergency stop: revoke the controller
 *
 *   cat /dev/safecage/status
 *   echo stop > /dev/safecage/ctl
 *
 * Runs in its own thread at priority 20: below the governor's control loop (70), above the cage (9).
 * Must be started after the controller has been forked (fork() is not allowed in a multithreaded process).
 */
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/stat.h>
#include "resmgr.h"

static resmgr_connect_funcs_t cfuncs;
static resmgr_io_funcs_t iofuncs;
static iofunc_attr_t attr_status, attr_ctl;
static pthread_mutex_t mx = PTHREAD_MUTEX_INITIALIZER;
static char status_buf[SC_STATUS_MAX] = "{\"state\":\"starting\"}\n";
static volatile int estop = 0;

void sc_resmgr_publish(const char *json_line) {
    pthread_mutex_lock(&mx);
    snprintf(status_buf, sizeof status_buf, "%s\n", json_line);
    pthread_mutex_unlock(&mx);
}

int sc_resmgr_estop_requested(void) { return estop; }

static int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb) {
    int st = iofunc_read_verify(ctp, msg, ocb, NULL);
    if (st != EOK) return st;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;
    if (ocb->attr != &attr_status) return EBADF;
    char buf[SC_STATUS_MAX];
    pthread_mutex_lock(&mx);
    strcpy(buf, status_buf);
    pthread_mutex_unlock(&mx);
    size_t len = strlen(buf), off = (size_t)ocb->offset, n = 0;
    if (off < len) {
        n = len - off;
        if (n > msg->i.nbytes) n = msg->i.nbytes;
        ocb->offset += n;
    }
    MsgReply(ctp->rcvid, (long)n, buf + (off < len ? off : len), n);
    return _RESMGR_NOREPLY;
}

static int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb) {
    int st = iofunc_write_verify(ctp, msg, ocb, NULL);
    if (st != EOK) return st;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;
    if (ocb->attr != &attr_ctl) return EBADF;
    char cmd[32] = { 0 };
    size_t n = msg->i.nbytes < sizeof cmd - 1 ? msg->i.nbytes : sizeof cmd - 1;
    resmgr_msgread(ctp, cmd, n, sizeof msg->i);
    if (strncmp(cmd, "stop", 4) == 0) estop = 1;
    _IO_SET_WRITE_NBYTES(ctp, msg->i.nbytes);
    return EOK;
}

static void *serve(void *arg) {
    dispatch_t *dpp = arg;
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = 20;
    pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
    dispatch_context_t *ctp = dispatch_context_alloc(dpp);
    for (;;) {
        if ((ctp = dispatch_block(ctp)) == NULL) continue;
        dispatch_handler(ctp);
    }
    return NULL;
}

int sc_resmgr_start(void) {
    dispatch_t *dpp = dispatch_create();
    if (!dpp) return -1;
    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &cfuncs, _RESMGR_IO_NFUNCS, &iofuncs);
    iofuncs.read = io_read;
    iofuncs.write = io_write;
    iofunc_attr_init(&attr_status, S_IFNAM | 0444, NULL, NULL);
    iofunc_attr_init(&attr_ctl, S_IFNAM | 0200, NULL, NULL);   /* only root may stop */
    resmgr_attr_t rattr;
    memset(&rattr, 0, sizeof rattr);
    rattr.nparts_max = 1;
    rattr.msg_max_size = 2048;
    if (resmgr_attach(dpp, &rattr, "/dev/safecage/status", _FTYPE_ANY, 0, &cfuncs, &iofuncs, &attr_status) == -1) return -1;
    if (resmgr_attach(dpp, &rattr, "/dev/safecage/ctl", _FTYPE_ANY, 0, &cfuncs, &iofuncs, &attr_ctl) == -1) return -1;
    pthread_t th;
    return pthread_create(&th, NULL, serve, dpp) == 0 ? 0 : -1;
}
