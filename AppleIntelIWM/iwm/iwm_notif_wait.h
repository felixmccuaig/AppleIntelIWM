//
//  iwm_notif_wait.h
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 1/01/2020.
//  Copyright © 2020 Felix McCuaig. All rights reserved.
//

#ifndef iwm_notif_wait_h
#define iwm_notif_wait_h


#include <sys/queue.h>

#define IWM_MAX_NOTIF_CMDS	5

struct iwm_rx_packet;
struct iwm_drv;

/**
 * struct iwm_notification_wait - notification wait entry
 * @entry: link for global list
 * @fn: Function called with the notification. If the function
 *      returns true, the wait is over, if it returns false then
 *      the waiter stays blocked. If no function is given, any
 *      of the listed commands will unblock the waiter.
 * @cmds: command IDs
 * @n_cmds: number of command IDs
 * @triggered: waiter should be woken up
 * @aborted: wait was aborted
 *
 * This structure is not used directly, to wait for a
 * notification declare it on the stack, and call
 * iwm_init_notification_wait() with appropriate
 * parameters. Then do whatever will cause the ucode
 * to notify the driver, and to wait for that then
 * call iwm_wait_notification().
 *
 * Each notification is one-shot. If at some point we
 * need to support multi-shot notifications (which
 * can't be allocated on the stack) we need to modify
 * the code for them.
 */
struct iwm_notification_wait {
    STAILQ_ENTRY(iwm_notification_wait) entry;
    
    int (*fn)(struct iwm_drv *driver, struct iwm_rx_packet *pkt, void *data);
    void *fn_data;
    
    uint16_t cmds[IWM_MAX_NOTIF_CMDS];
    uint8_t n_cmds;
    int triggered, aborted;
};

struct iwm_notif_wait_data {
    IOLock *notifWaitLock;
    char lk_buf[32];
    STAILQ_HEAD(, iwm_notification_wait) list;
    struct iwm_drv *driver;
};

#endif /* iwm_notif_wait_h */
