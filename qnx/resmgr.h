#ifndef SAFECAGE_RESMGR_H
#define SAFECAGE_RESMGR_H

#define SC_STATUS_MAX 512

int  sc_resmgr_start(void);                    /* start /dev/safecage/{status,ctl} */
void sc_resmgr_publish(const char *json_line); /* update what `cat /dev/safecage/status` shows */
int  sc_resmgr_estop_requested(void);          /* 1 after `echo stop > /dev/safecage/ctl` */

#endif
