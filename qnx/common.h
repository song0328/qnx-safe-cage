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

#endif
