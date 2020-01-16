//
//  Trans.cpp
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 12/10/19.
//  Copyright © 2019 Felix McCuaig. All rights reserved.
//

#include "AppleIntelIWM.h"

OS_INLINE void _OSWriteInt8(volatile void* base, uintptr_t byteOffset, uint8_t data)
{
    *(volatile uint8_t *)((uintptr_t)base + byteOffset) = data;
}

#define OSWriteLittleInt8(a, b, c) _OSWriteInt8(a, b, c)

IOReturn AppleIntelIWM::iwmWrite8(struct iwm_drv* driver, UInt32 ofs, UInt8 val)
{
    if(!driver) {
        IWM_KERNLOG("Error, driver doesn't exist for write op\n");
        return KERN_FAILURE;
    }
    
    if(!driver->hwBase) {
        IWM_KERNLOG("Error, device addr doesn't exist for write op\n");
        return KERN_FAILURE;
    }

    OSWriteLittleInt8(driver->hwBase, ofs, val);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::iwmWrite32(struct iwm_drv* driver, UInt32 ofs, UInt32 val)
{
    if(!driver) {
        IWM_KERNLOG("Error, driver doesn't exist for write op\n");
        return KERN_FAILURE;
    }
    
    if(!driver->hwBase) {
        IWM_KERNLOG("Error, device addr doesn't exist for write op\n");
        return KERN_FAILURE;
    }
    
    OSWriteLittleInt32(driver->hwBase, ofs, val);
    return KERN_SUCCESS;
}

UInt32 AppleIntelIWM::iwmRead(struct iwm_drv* driver, UInt32 ofs)
{
    if(!driver) {
        IWM_KERNLOG("Error, driver doesn't exist for read op\n");
        return KERN_FAILURE;
    }
    
    if(!driver->hwBase) {
        IWM_KERNLOG("Error, device addr doesn't exist for read op\n");
        return KERN_FAILURE;
    }
    
    return OSReadLittleInt32(driver->hwBase, ofs);
}

IOReturn AppleIntelIWM::iwmSetBits(struct iwm_drv* driver, UInt32 dest, UInt32 mask)
{
    return iwmWrite32(driver, dest, iwmRead(driver, dest) | mask);
}

IOReturn AppleIntelIWM::iwmClearBits(struct iwm_drv* driver, UInt32 dest, UInt32 mask)
{
    return iwmWrite32(driver, dest, iwmRead(driver, dest) & ~(mask));
}

#define POLLING_TIME 10
int AppleIntelIWM::iwmPollBit(struct iwm_drv* driver, UInt32 dest, UInt32 bits, UInt32 mask, int time)
{
    for(;;) {
        if((iwmRead(driver, dest) & mask) == (bits & mask)) {
            return 1;
        }
        
        if(time <= POLLING_TIME) {
            return 0;
        }
        
        time -= POLLING_TIME;
        IOSleep(POLLING_TIME);
    }
}
#undef POLLING_TIME

#define HW_READY_TIMEOUT 50
IOReturn AppleIntelIWM::setHardwareReady(struct iwm_drv* driver)
{
    int ready;
    iwmSetBits(driver, IWM_CSR_HW_IF_CONFIG_REG, IWM_CSR_HW_IF_CONFIG_REG_BIT_NIC_READY);
    ready = iwmPollBit(driver, IWM_CSR_HW_IF_CONFIG_REG, IWM_CSR_HW_IF_CONFIG_REG_BIT_NIC_READY, IWM_CSR_HW_IF_CONFIG_REG_BIT_NIC_READY, HW_READY_TIMEOUT);
    if(ready) {
        iwmSetBits(driver, IWM_CSR_MBOX_SET_REG, IWM_CSR_MBOX_SET_REG_OS_ALIVE);
    }
    IWM_KERNLOG("Hw set ready completed %d", ready);
    return ready;
}
#undef HW_READY_TIMEOUT

IOReturn AppleIntelIWM::prepareCardHardware(struct iwm_drv* driver)
{
    int error = 0;
    int t = 0;
    IWM_DLOG(IWM_DEBUG_RESET, "Preparing card hardware\n");
    
    if(setHardwareReady(driver)) {
        goto out;
    }
    
    iwmSetBits(driver, IWM_CSR_DBG_LINK_PWR_MGMT_REG, IWM_CSR_RESET_LINK_PWR_MGMT_DISABLED);
    IOSleep(1000);
    
    /* If HW is not ready, prepare the conditions to check again */
    iwmSetBits(driver, IWM_CSR_HW_IF_CONFIG_REG, IWM_CSR_HW_IF_CONFIG_REG_PREPARE);
    
    do {
        if(setHardwareReady(driver)) {
            goto out;
        }
        
        IOSleep(200);
        t += 200;
    } while (t < 150000);
    
    error = ETIMEDOUT;
    
    IWM_KERNLOG("Preparing card HW code %d\n", error);
out:
    return error;
}

UInt32 AppleIntelIWM::readPRPH(struct iwm_drv* driver, UInt32 addr)
{
    iwmWrite32(driver, IWM_HBUS_TARG_PRPH_RADDR, ((addr & 0x000FFFFF) | (3 << 24)));
    //TODO: bus space barrier read / write idk what it does
    return iwmRead(driver, IWM_HBUS_TARG_PRPH_RDAT);
}

IOReturn AppleIntelIWM::writePRPH(struct iwm_drv* driver, UInt32 addr, UInt32 value)
{
    iwmWrite32(driver, IWM_HBUS_TARG_PRPH_WADDR, ((addr & 0x000fffff) | (3 << 24)));
    //TODO: bus space barrier write
    iwmWrite32(driver, IWM_HBUS_TARG_PRPH_WDAT, value);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::setBitsMaskPRPH(struct iwm_drv* driver, UInt32 reg, UInt32 bits, UInt32 mask)
{
    UInt32 value;
    
    /* XXX: no error path? */
    if(nicLock(driver)) {
        value = readPRPH(driver, reg) & mask;
        value |= bits;
        writePRPH(driver, reg, value);
        nicUnlock(driver);
    }
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::setBitsPRPH(struct iwm_drv* driver, UInt32 reg, UInt32 bits)
{
    return setBitsMaskPRPH(driver, reg, bits, ~0);
}

IOReturn AppleIntelIWM::clearBitsPRPH(struct iwm_drv* driver, UInt32 reg, UInt32 bits)
{
    return setBitsMaskPRPH(driver, reg, 0, ~bits);
}

IOReturn AppleIntelIWM::apmConfig(struct iwm_drv* driver)
{
    //TODO: IS THIS RIGHT?
    OSNumber *pciLinkCapabilities = OSDynamicCast(OSNumber, driver->pciDevice->getProperty(kIOPCIExpressLinkCapabilitiesKey));
    if(pciLinkCapabilities) {
        UInt32 linkCaps = pciLinkCapabilities->unsigned32BitValue();
        if(linkCaps & 0x800) {
            iwmWrite32(driver, IWM_CSR_GIO_REG, IWM_CSR_GIO_REG_VAL_L0S_ENABLED);
        } else {
            iwmClearBits(driver, IWM_CSR_GIO_REG, IWM_CSR_GIO_REG_VAL_L0S_ENABLED);
        }
        driver->pm_support = !(linkCaps & 0x400);
    }
    //Lets just do L0 for nao
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::apmInit(struct iwm_drv* driver)
{
    IWM_DLOG(IWM_DEBUG_RESET, "Running APM init\n");
    if(driver->config->device_family == IWM_DEVICE_FAMILY_8000) {
        iwmSetBits(driver, IWM_CSR_GIO_CHICKEN_BITS, IWM_CSR_GIO_CHICKEN_BITS_REG_BIT_DIS_L0S_EXIT_TIMER);
    }
    
    /*
     * Disable L0s without affecting L1;
     *  don't wait for ICH L0s (ICH bug W/A)
     */
    iwmSetBits(driver, IWM_CSR_GIO_CHICKEN_BITS, IWM_CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);
    
    /* Set FH wait threshold to maximum (HW error during stress W/A) */
    iwmSetBits(driver, IWM_CSR_DBG_HPET_MEM_REG, IWM_CSR_DBG_HPET_MEM_REG_VAL);
    
    /*
    * Enable HAP INTA (interrupt from management bus) to
    * wake device's PCI Express link L1a -> L0s
    */
    iwmSetBits(driver, IWM_CSR_HW_IF_CONFIG_REG, IWM_CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);
    apmConfig(driver);
    
    
    /*
     * Set "initialization complete" bit to move adapter from
     * D0U* --> D0A* (powered-up active) state.
     */
    iwmSetBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
    
    
    /*
     * Wait for clock stabilization; once stabilized, access to
     * device-internal resources is supported, e.g. iwm_write_prph()
     * and accesses to uCode SRAM.
     */
    if(!iwmPollBit(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY, 25000)) {
        IWM_DLOG(IWM_DEBUG_ERROR, "Timeout waiting for clock to stabilize!\n");
        return KERN_FAILURE;
    }
    
    //TODO: fix iwmPoll bit... returns 1 on success and 0 on failed dont ask me why
    
    if(driver->config->host_interrupt_operation_mode) {
        /*
         * This is a bit of an abuse - This is needed for 7260 / 3160
         * only check host_interrupt_operation_mode even if this is
         * not related to host_interrupt_operation_mode.
         *
         * Enable the oscillator to count wake up time for L1 exit. This
         * consumes slightly more power (100uA) - but allows to be sure
         * that we wake up from L1 on time.
         *
         * This looks weird: read twice the same register, discard the
         * value, set a bit, and yet again, read that same register
         * just to discard the value. But that's the way the hardware
         * seems to like it.
         */
        if(nicLock(driver)) {
            readPRPH(driver, IWM_OSC_CLK);
            readPRPH(driver, IWM_OSC_CLK);
            nicUnlock(driver);
        }
        setBitsPRPH(driver, IWM_OSC_CLK, IWM_OSC_CLK_FORCE_CONTROL);
        
        if(nicLock(driver)) {
            readPRPH(driver, IWM_OSC_CLK);
            readPRPH(driver, IWM_OSC_CLK);
            nicUnlock(driver);
        }
    }
    
    /*
     * Enable DMA clock and wait for it to stabilize.
     *
     * Write to "CLK_EN_REG"; "1" bits enable clocks, while "0" bits
     * do not disable clocks.  This preserves any hardware bits already
     * set by default in "CLK_CTRL_REG" after reset.
     */
    
    if(driver->config->device_family == IWM_DEVICE_FAMILY_7000) {
        if(nicLock(driver)) {
            writePRPH(driver, IWM_APMG_CLK_EN_REG, IWM_APMG_CLK_VAL_DMA_CLK_RQT);
            nicUnlock(driver);
        }
        
        IOSleep(20);
        
        /* Disable L1-Active */
        setBitsPRPH(driver, IWM_APMG_PCIDEV_STT_REG, IWM_APMG_PCIDEV_STT_VAL_L1_ACT_DIS);
        
        /* Clear the interrupt in APMG if the NIC is in RFKILL */
        if(nicLock(driver)) {
            writePRPH(driver, IWM_APMG_RTC_INT_STT_REG, IWM_APMG_RTC_INT_STT_RFKILL);
            nicUnlock(driver);
        }
    }
    
    IWM_DLOG(IWM_DEBUG_INFO, "APM init finished\n");
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::apmStop(struct iwm_drv* driver)
{
    iwmSetBits(driver, IWM_CSR_RESET, IWM_CSR_RESET_REG_FLAG_STOP_MASTER);
    
    if(!iwmPollBit(driver, IWM_CSR_RESET, IWM_CSR_RESET_REG_FLAG_MASTER_DISABLED, IWM_CSR_RESET_REG_FLAG_MASTER_DISABLED, 100)) {
        IWM_DLOG(IWM_DEBUG_ERROR, "apmStop timeout waiting for master!\n");
    }
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::initUcode(struct iwm_drv *driver, int justNVM) {
    struct iwm_notification_wait calibWait;
    
    static const UInt16 initComplete[] = {
        IWM_INIT_COMPLETE_NOTIF,
        IWM_CALIB_RES_NOTIF_PHY_DB
    };
    
    int ret = 0;
    
    /* do not operate with rfkill switch turned on */
    if((driver->flags & IWM_FLAG_RFKILL) && !justNVM) {
        IWM_KERNLOG("Radio is disabled by HW switch!\n");
        return EPERM;
    }
    
    initNotificationWait(driver->notifWait, &calibWait, initComplete, nitems(initComplete), waitPhyDbEntry, driver->phyDb);
    
    /* Will also start the device */
    ret = loadUcodeWaitAlive(driver, IWM_UCODE_INIT);
    if(ret) {
        IWM_KERNLOG("Failed to start INIT UCode!\n");
        goto error;
    }
    
    if(driver->config->device_family < IWM_DEVICE_FAMILY_8000) {
        ret = sendBtInitConfig(driver);
        if(ret) {
            IWM_KERNLOG("Failed to send bt config! %d\n", ret);
            goto error;
        }
    }
    
    if(justNVM) {
        ret = initNvm(driver);
        if(ret) {
            IWM_KERNLOG("Failed to read NVM\n");
            goto error;
        }
    }
    
    /* Send TX valid antennas before triggering calibrations */
    ret = sendTxAntConfig(driver, getValidTxAnt(driver));
    if(ret) {
        IWM_KERNLOG("Failed to send antennas before calib: %d\n", ret);
        goto error;
    }
    
    /*
     * Send phy configurations command to init uCode
     * to start the 16.0 uCode init image internal calibrations.
     */
    ret = sendPhyConfigCmd(driver);
    if(ret) {
        IWM_KERNLOG("Failed to run INIT calibrations: %d\n", ret);
        goto error;
    }
    
    /*
     * Nothing to do but wait for the init complete notification
     * from the firmware.
     */
    IWM_UNLOCK(driver);
    
    ret = waitNotification(driver->notifWait, &calibWait, IWM_UCODE_CALIB_TIMEOUT);
    
    IWM_LOCK(driver);
    
    goto out;
    
error:
    removeNotification(driver->notifWait, &calibWait);
out:
    return ret;
}

IOReturn AppleIntelIWM::sendPhyConfigCmd(struct iwm_drv *driver)
{
    struct iwm_phy_cfg_cmd phyConfigCmd;
    enum iwm_ucode_type ucodeType = driver->currentUcode;
    
    /* Set parameters */
    phyConfigCmd.phy_cfg = getPhyConfig(driver);
    phyConfigCmd.calib_control.event_trigger = driver->default_calib[ucodeType].event_trigger;
    phyConfigCmd.calib_control.flow_trigger = driver->default_calib[ucodeType].flow_trigger;
    
    IWM_KERNLOG("Sending Phy CFG command: 0x%x\n", phyConfigCmd.phy_cfg);
    
    return sendCmdPdu(driver, IWM_PHY_CONFIGURATION_CMD, IWM_CMD_SYNC, sizeof(phyConfigCmd), &phyConfigCmd);
}

IOReturn AppleIntelIWM::sendBtInitConfig(struct iwm_drv *driver)
{
    struct iwm_bt_coex_cmd btCommand = { 0 };
    
    btCommand.mode = IWM_BT_COEX_WIFI;
    btCommand.enabled_modules = IWM_BT_COEX_HIGH_BAND_RET;
    
    return sendCmdPdu(driver, IWM_BT_CONFIG, 0, sizeof(btCommand), &btCommand);
}

IOReturn AppleIntelIWM::startHardware(struct iwm_drv* driver)
{
    int error = 0;
    
    error = prepareCardHardware(driver);
    if(error != KERN_SUCCESS) {
        IWM_DLOG(IWM_DEBUG_INFO, "Prepare hardware failed\n");
        return error;
    }
    
    /* Reset the entire device */
    iwmWrite32(driver, IWM_CSR_RESET, IWM_CSR_RESET_REG_FLAG_SW_RESET);
    
    IOSleep(1000);
    
    error = apmInit(driver);
    if(error != KERN_SUCCESS) {
        IWM_DLOG(IWM_DEBUG_INFO, "Init Apm failed!\n");
        return error;
    }
    
    /* On newer chipsets MSI is disabled by default. */
    if(0) {
        //MQRX stuff
    }
    
    enableRfKillInterrupt(driver);
    checkRfKill(driver);
    
    IWM_DLOG(IWM_DEBUG_INFO, "No issues starting hardware\n");
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::enableRfKillInterrupt(struct iwm_drv *driver)
{
    driver->intMask = IWM_CSR_INT_BIT_RF_KILL;
    
    iwmWrite32(driver, IWM_CSR_INT_MASK, driver->intMask);
    iwmSetBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_RFKILL_WAKE_L1A_EN);
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::checkRfKill(struct iwm_drv *driver)
{
    UInt32 v;
    int rv;
    
    /*
     * "documentation" is not really helpful here:
     *  27:	HW_RF_KILL_SW
     *	Indicates state of (platform's) hardware RF-Kill switch
     *
     * But apparently when it's off, it's on ...
     */
    
    v = iwmRead(driver, IWM_CSR_GP_CNTRL);
    rv = (v & IWM_CSR_GP_CNTRL_REG_FLAG_HW_RF_KILL_SW) == 0;
    if(rv) {
        driver->flags |= IWM_FLAG_RFKILL;
    } else {
        driver->flags &= ~IWM_FLAG_RFKILL;
    }
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::setPowerMode(struct iwm_drv* driver)
{
    setBitsMaskPRPH(driver, IWM_APMG_PS_CTRL_REG, IWM_APMG_PS_CTRL_VAL_PWR_SRC_VMAIN, ~IWM_APMG_PS_CTRL_MSK_PWR_SRC);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::clearCmdInFlight(struct iwm_drv *driver)
{
    if(!driver->config->apmg_wake_up_wa) {
        return KERN_SUCCESS;
    }
    
    if(!driver->cmdHoldNicAwake) {
        IWM_KERNLOG("clear cmd in flight cmdHoldNicAwake not set!\n");
        return KERN_SUCCESS;
    }
    
    driver->cmdHoldNicAwake = 0;
    
    iwmClearBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::setCmdInFlight(struct iwm_drv *driver)
{
    /*
     * wake up the NIC to make sure that the firmware will see the host
     * command - we will let the NIC sleep once all the host commands
     * returned. This needs to be done only on NICs that have
     * apmg_wake_up_wa set.
     */
    
    if(driver->config->apmg_wake_up_wa && !driver->cmdHoldNicAwake) {
        IOReturn result;
        
        iwmSetBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
        
        result = iwmPollBit(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN, (IWM_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY | IWM_CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP), 15000);
        
        if(!result) {
            iwmClearBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
            IWM_KERNLOG("Failed to wake up NIC for hcmd\n");
            return KERN_FAILURE;
        }
        
        driver->cmdHoldNicAwake = 1;
    }

    return KERN_SUCCESS;
}

int AppleIntelIWM::nicLock(struct iwm_drv *driver) //Positive Rv is success and zero is failure
{
    int rv = 0;
    
    if(driver->cmdHoldNicAwake) {
        return 1;
    }

    iwmSetBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
    
    if(driver->config->device_family >= IWM_DEVICE_FAMILY_8000) {
        IOSleep(2);
    }
    
    if(iwmPollBit(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY | IWM_CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP, 15000)) {
        rv = 1;
    } else {
        /* jolt */
        IWM_KERNLOG("Resetting device via NMI\n");
        iwmWrite32(driver, IWM_CSR_RESET, IWM_CSR_RESET_REG_FLAG_FORCE_NMI);
    }

    return rv;
}


IOReturn AppleIntelIWM::nicUnlock(struct iwm_drv *driver)
{
    if(driver->cmdHoldNicAwake) {
        return KERN_SUCCESS;
    }
    
    iwmClearBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::writeMem(struct iwm_drv *driver, UInt32 addr, const void *buf, int dWords)
{
    int offs;
    const UInt32 *vals = (UInt32*) buf;
    
    if(nicLock(driver)) {
        iwmWrite32(driver, IWM_HBUS_TARG_MEM_WADDR, addr);
        /* WADDR auto-increments */
        for(offs = 0; offs < dWords; offs++) {
            UInt32 val = vals ? vals[offs] : 0;
            iwmWrite32(driver, IWM_HBUS_TARG_MEM_WDAT, val);
        }
        nicUnlock(driver);
    } else {
        IWM_KERNLOG("WriteMem failed!\n");
        return EBUSY;
    }
    return 0;
}

IOReturn AppleIntelIWM::writeMem32(struct iwm_drv *driver, UInt32 addr, UInt32 val)
{
    return writeMem(driver, addr, &val, 1);
}

void AppleIntelIWM::IWM_LOCK(struct iwm_drv *driver)
{
    if(!driver->mtxLocked) {
        IOLockLock(driver->pciLock);
        driver->mtxLocked = 1;
        IWM_KERNLOG("Lock has been locked!\n");
    } else {
        IWM_KERNLOG("Lock already held::Illegal operation!\n");
    }
}

void AppleIntelIWM::IWM_UNLOCK(struct iwm_drv *driver)
{
    if(driver->mtxLocked) {
        driver->mtxLocked = 0;
        IOLockUnlock(driver->pciLock);
    } else {
        IWM_KERNLOG("Lock not yet held::Illegal operation!\n");
    }
}

int AppleIntelIWM::IWM_SLEEP(struct iwm_drv *driver, void* event, UInt32 interType, int time)
{
    int ret = THREAD_INTERRUPTED;
    
    if(driver->mtxLocked) {
        driver->mtxLocked = 0; //on wakeup it gets reaquired
        if(time) {
            AbsoluteTime deadline;
            clock_interval_to_deadline(time, kSecondScale, (UInt64*) &deadline);
            ret = IOLockSleepDeadline(driver->pciLock, event, deadline, interType);
        } else {
            ret = IOLockSleep(driver->pciLock, event, interType);
        }
    } else {
        IWM_KERNLOG("Lock not yet held for sleep::Illegal operation!\n");
    }
    
    if(ret == THREAD_AWAKENED) {
        driver->mtxLocked = 1;
    }
    
    return ret;
}

void AppleIntelIWM::IWM_WAKEUP(struct iwm_drv* driver, void* event, bool oneThread)
{
    IOLockWakeup(driver->pciLock, event, oneThread);
}

int AppleIntelIWM::IWM_WAIT_LOCK_SLEEP(struct iwm_notif_wait_data *notif_data, void* event, int time)
{
    AbsoluteTime deadline;
    clock_interval_to_deadline(time, kMillisecondScale, (UInt64*) &deadline);
    return IOLockSleepDeadline(notif_data->notifWaitLock, event, deadline, THREAD_INTERRUPTIBLE);
}

















