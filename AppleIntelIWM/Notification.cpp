//
//  Notification.cpp
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 1/01/2020.
//  Copyright © 2020 Felix McCuaig. All rights reserved.
//

#include "AppleIntelIWM.h"

struct iwm_notif_wait_data* AppleIntelIWM::notifWaitInit(struct iwm_drv *driver) {
    struct iwm_notif_wait_data *data;
    
    data = (struct iwm_notif_wait_data*) IOMalloc(sizeof(struct iwm_notif_wait_data));
    bzero(data, sizeof(*data));
    
    if(data) {
        snprintf(data->lk_buf, 32, "iwm wait_notif");
        data->notifWaitLock = IOLockAlloc();
        STAILQ_INIT(&data->list);
        data->driver = driver;
    }
    
    return data;
}

IOReturn AppleIntelIWM::notifWaitFree(struct iwm_notif_wait_data *notifData)
{
    KASSERT(STAILQ_EMPTY(&notifData->list), "Notif list isn't empty\n")
    IOLockFree(notifData->notifWaitLock);
    IOFree(notifData, sizeof(*notifData));
    return KERN_SUCCESS;
}

/* XXX Get rid of separate cmd argument, like in iwlwifi's code */
IOReturn AppleIntelIWM::notifWaitNotify(struct iwm_notif_wait_data *notifData, UInt16 cmd, struct iwm_rx_packet *pkt)
{
    struct iwm_notification_wait *waitEntry;
    
    //IOLockLock(notifData->notifWaitLock); //only called from interrupt
    
    STAILQ_FOREACH(waitEntry, &notifData->list, entry) {
        int found = FALSE;
        
        int i;
        
        if(waitEntry->triggered || waitEntry->aborted) {
            continue;
        }
        
        for(i = 0; i < waitEntry->n_cmds; i++) {
            if(cmd == waitEntry->n_cmds) {
                found = TRUE;
                break;
            }
        }
    
        if(!found) {
            continue;
        }
        
        if(!waitEntry->fn || waitEntry->fn(notifData->driver, pkt, waitEntry->fn_data)) {
            waitEntry->triggered = 1;
            IOLockWakeup(notifData->notifWaitLock, waitEntry, true);
        }
    
    }
    
    //IOLockUnlock(notifData->notifWaitLock);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::abortNotificationWait(struct iwm_notif_wait_data *notifData)
{
    struct iwm_notification_wait *waitEntry;
    
    IOLockLock(notifData->notifWaitLock);
    
    STAILQ_FOREACH(waitEntry, &notifData->list, entry) {
        waitEntry->aborted = 1;
    }
    
    IOLockUnlock(notifData->notifWaitLock);

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::initNotificationWait(struct iwm_notif_wait_data *notifData, struct iwm_notification_wait *waitEntry, const UInt16 *cmds, int nCmds, int (*fn)(struct iwm_drv *, struct iwm_rx_packet *, void *), void *fn_data)
{
    if(nCmds >= IWM_MAX_NOTIF_CMDS) {
        IWM_KERNLOG("n Commands is too large %d", nCmds);
    }
    
    waitEntry->fn = fn;
    waitEntry->fn_data = fn_data;
    waitEntry->n_cmds = nCmds;
    memcpy(waitEntry->cmds, cmds, nCmds * sizeof(UInt16));
    waitEntry->triggered = 0;
    waitEntry->aborted = 0;
    
    IOLockLock(notifData->notifWaitLock);
    STAILQ_INSERT_TAIL(&notifData->list, waitEntry, entry);
    IOLockUnlock(notifData->notifWaitLock);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::waitNotification(struct iwm_notif_wait_data *notifData, struct iwm_notification_wait *waitEntry, int timeout)
{
    int ret = 0;
    
    IOLockLock(notifData->notifWaitLock);
    if(!waitEntry->triggered && !waitEntry->aborted) {
        ret = IWM_WAIT_LOCK_SLEEP(notifData, waitEntry, timeout);
    }
    STAILQ_REMOVE(&notifData->list, waitEntry, iwm_notification_wait, entry);

    IOLockUnlock(notifData->notifWaitLock);
    return ret;
}

IOReturn AppleIntelIWM::removeNotification(struct iwm_notif_wait_data *notifData, struct iwm_notification_wait *waitEntry)
{
    IOLockLock(notifData->notifWaitLock);
    STAILQ_REMOVE(&notifData->list, waitEntry, iwm_notification_wait, entry);
    IOLockUnlock(notifData->notifWaitLock);
    return KERN_SUCCESS;
}








