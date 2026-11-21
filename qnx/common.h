/* common.h - message layout shared by the governor (trusted) and the caged controller (untrusted).
 *
 * Protocol (QNX native message passing, synchronous):
 *   controller --MsgSend(sc_cmd_msg_t)--> governor
 *   governor holds the reply until the next control tick, then
 *   governor --MsgReply(sc_state_t)--> controller
 * So the controller is reply-blocked between ticks and cannot run ahead,
 * and a controller that does not answer before the next tick simply misses it.
 */
#ifndef SAFECAGE_COMMON_H
#define SAFECAGE_COMMON_H

#include <stdint.h>

#define SC_NAME       "safecage"       /* name_attach() path */
#define SC_MSG_CMD    0x5343u          /* 'SC'; above the _IO_* range */
#define SC_SEQ_HELLO  0xFFFFFFFFu      /* first message: "I am ready" */

typedef struct {
    uint16_t type;      /* SC_MSG_CMD */
    uint16_t pad;
    uint32_t seq;       /* seq of the state this command answers */
    double   torque;    /* requested joint torque, N*m */
} sc_cmd_msg_t;

typedef struct {
    uint32_t seq;           /* tick index */
    int32_t  done;          /* 1 = session over, controller should exit */
    double   t;             /* simulated time, s */
    double   angle_deg;     /* joint angle, degrees */
    double   velocity_dps;  /* joint velocity, degrees/s */
} sc_state_t;


/* ---- drive registers (shared memory between the governor and the joint/drive process) ----
 * The joint process stands in for the hardware: motor drive + forearm + patient. It owns the
 * physics, applies whatever torque is in the command register on its own 500 Hz clock, and logs
 * the TRUE trajectory, so whether the envelope held is judged by an observer outside the governor.
 * Single writer per half; each half is a seqlock (odd = being written). */
#define SC_DRIVE_SHM  "/sc_drive"
typedef struct {
    /* sensor half: written by the joint */
    volatile uint32_t s_seq;
    double   t, angle_deg, velocity_dps;
    volatile int32_t running;      /* 0 = waiting for start, 1 = session running, 2 = finished */
    volatile int32_t sto;          /* 1 = drive watchdog cut the torque (safe torque off) */
    /* command half: written by the governor */
    volatile uint32_t c_seq;
    double   cmd_tau;              /* N*m */
    volatile uint32_t cmd_count;   /* +1 per command write; the drive watchdog watches this */
    volatile int32_t start;        /* governor sets 1 once the controller is connected */
} sc_drive_t;

static inline void sc_seq_begin(volatile uint32_t *q) { (*q)++; __atomic_thread_fence(__ATOMIC_SEQ_CST); }
static inline void sc_seq_end(volatile uint32_t *q)   { __atomic_thread_fence(__ATOMIC_SEQ_CST); (*q)++; }

#endif
