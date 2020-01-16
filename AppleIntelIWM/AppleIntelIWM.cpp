#include "AppleIntelIWM.h"

#define super IO80211Controller

OSDefineMetaClassAndStructors(AppleIntelIWM, super);

bool AppleIntelIWM::init(OSDictionary *dict)
{
    bool result = super::init(dict);
    IWM_KERNLOG("Called init\n");
    return result;
}

bool AppleIntelIWM::start(IOService* provider)
{
    if(!super::start(provider)) {
        IWM_KERNLOG("Starting failed\n");
        return false;
    }
    
    IOSleep(200); //So we don't miss any output
    
    IOPCIDevice* pciDevice = OSDynamicCast(IOPCIDevice, provider);
    if(!pciDevice) {
        IWM_KERNLOG("Failed to acquire provider\n");
        return false;
    }
    
    driver = (iwm_drv*) IOMalloc(sizeof(iwm_drv));
    
    if(!driver) {
        IWM_KERNLOG("Failed to alloc driver\n");
        return false;
    }
    
    //We need this memory to be all zeros
    zeroDriver(driver);
    
    driver->pciDevice = pciDevice;
    
    if(!driver->pciDevice) {
        IWM_KERNLOG("Failed to alloc pci device\n");
        return false;
    }
    
    pciDevice->setMemoryEnable(true);
    pciDevice->setBusMasterEnable(true);
    
    driver->config = getConfig(pciDevice);
    if(!driver->config) {
        IWM_KERNLOG("Failed to get config\n");
        return false;
    }
    
    driver->deviceMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
    if(!driver->deviceMap) {
        IWM_KERNLOG("Failed to alloc device map\n");
        return false;
    }
    
    driver->hwBase = reinterpret_cast<volatile void *>(driver->deviceMap->getVirtualAddress());
    
    if(!driver->hwBase) {
        IWM_KERNLOG("Failed to alloc hw base\n");
        return false;
    }
    
    firmwareLoadLock = IOLockAlloc();
    if(!firmwareLoadLock) {
        IWM_KERNLOG("Load lock couldn't be acquired\n");
        return false;
    }
    
    driver->pciLock = IOLockAlloc();
    if(!driver->pciLock) {
        IWM_KERNLOG("PCI lock couldn't be acquired\n");
        return false;
    }
    
    driver->hwRev = iwmRead(driver, IWM_CSR_HW_REV);
    
    IWM_KERNLOG("Device %s detected with hw rev: %d\n", driver->config->name, driver->hwRev);
    
    driver->irqLoop = IO80211WorkLoop::workLoop();
    driver->workLoop = IO80211WorkLoop::workLoop();
    
    if(!driver->irqLoop) {
        IWM_KERNLOG("Creating irq loop failed!\n");
        return false;
    }
    
    int source = findMSIInterruptTypeIndex(driver);
    IWM_KERNLOG("MSI interrupt index is %d\n", source);
    
    driver->interruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(this, (IOInterruptEventAction) &AppleIntelIWM::interruptOccured, (IOFilterInterruptAction) &AppleIntelIWM::interruptFilter, provider);
    
    if(!driver->interruptSource) {
        IWM_KERNLOG("Creating interruptSource failed!\n");
        return false;
    }
    
    if(driver->irqLoop->addEventSource(driver->interruptSource) != KERN_SUCCESS) {
        IWM_KERNLOG("Adding event source to irq loop failed!\n");
        return false;
    }
    
    driver->interruptSource->enable();
    driver->gate = IOCommandGate::commandGate(this, (IOCommandGate::Action) &AppleIntelIWM::gateAction);
    
    if(driver->workLoop->addEventSource(driver->gate) != KERN_SUCCESS) {
        IWM_KERNLOG("Adding event source to workLoop failed!\n");
        return false;
    }

    driver->gate->enable();
    
    driver->mBufCursor = IOMbufNaturalMemoryCursor::withSpecification(PAGE_SIZE, 1);
    
    if(driver->config->device_family >= IWM_DEVICE_FAMILY_8000) {
        //Do this later TODO
    }
    
    //init notif wait
    driver->notifWait = notifWaitInit(driver);
    if(!driver->notifWait) {
        IWM_KERNLOG("Couldn't init notif wait\n");
        return false;
    }
    
    //init phy db
    driver->phyDb = phyDbInit(driver);
    if(!driver->phyDb) {
        IWM_KERNLOG("Couldn't init phyDb!\n");
        return false;
    }
    
    /* Set EBS as successful as long as not stated otherwise by the FW. */
    driver->lastEbsSuccessful = TRUE;
    
    if(driver->config == &iwm7265_cfg && (driver->hwRev & IWM_CSR_HW_REV_TYPE_MSK) == IWM_CSR_HW_REV_TYPE_7265D) {
        driver->config = &iwm7265d_cfg;
    }
    
    //Allocate DMA mem for firmware
    if(allocFwMem(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't allocate DMA mem\n");
        return false;
    }

    //Allocate Keep Warm page
    if(allocKwPage(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't allocate KW page\n");
        return false;
    }
    
    //Allocate Tx sched rings
    if(allocTxSchedRings(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't allocate tx sched rings\n");
        return false;
    }
    
    //Allocate TX rings
    int txqI = 0, nItemsTXQ = 0;
    nItemsTXQ = sizeof(driver->txq) / sizeof(struct iwm_tx_ring);
    
    for(txqI = 0; txqI < nItemsTXQ; txqI++) {
        if(allocTxRing(driver, &driver->txq[txqI], txqI) != KERN_SUCCESS) {
            IWM_KERNLOG("Couldn't allocate tx ring %d \n", txqI);
            return false;
        }
    }
    
    //Allocate RX ring
    if(allocRxRing(driver, &driver->rxq) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't allocate rx ring\n");
        return false;
    }
    
    //Clear pending interrupts
    iwmWrite32(driver, IWM_CSR_INT, 0xffffffff);
    
    driver->noise = -96;
    driver->maxRssi = IWM_MAX_DBM - IWM_MIN_DBM;
    
    if(readFirmware(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't read firmware\n");
        return false;
    }
    
    IWM_LOCK(driver);
    
    enableInterrupts(driver);
    
    resetRxRing(driver, &driver->rxq);
    
    if(initHardware(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't init hardware!\n");
        return false;
    }
    
    IWM_UNLOCK(driver);
    
    registerService();
    
    return true;
}

void AppleIntelIWM::stop(IOService* provider)
{
    super::stop(provider);
    IWM_KERNLOG("Called stop\n");
}

void AppleIntelIWM::free()
{
    super::free();
    IWM_KERNLOG("Called free\n");
}

uint16_t AppleIntelIWM::getVendorID(IOPCIDevice* provider)
{
    return provider->configRead16(kIOPCIConfigVendorID);
}

uint16_t AppleIntelIWM::getDeviceID(IOPCIDevice* provider)
{
    return provider->configRead16(kIOPCIConfigDeviceID);
}

struct iwm_cfg* AppleIntelIWM::getConfig(IOPCIDevice* provider)
{
    uint16_t vendorID = getVendorID(provider);
    
    if(vendorID != PCI_VENDOR_INTEL) {
        IWM_KERNLOG("Must be an Intel Product!\n");
        return NULL;
    }
    
    uint16_t deviceID = getDeviceID(provider);
    
    IWM_KERNLOG("Vendor ID found: %X\n", vendorID);
    IWM_KERNLOG("Device ID found: %X\n", deviceID);
    
    UInt32 i;
    
    for(i = 0; i < (sizeof(iwm_devices) / sizeof(struct iwm_devices)); i++) {
        if(deviceID == iwm_devices[i].device) {
            return iwm_devices[i].cfg;
        }
    }
    
    return NULL;
}

UInt8 AppleIntelIWM::getDmaUpperAddr(UInt32 addr)
{
    return (sizeof(addr) > sizeof(UInt32) ? (addr >> 16) >> 16 : 0) & 0xF;
}

IOReturn AppleIntelIWM::gateAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3)
{
    return KERN_SUCCESS;
}

void AppleIntelIWM::interruptOccured(OSObject *owner, IOInterruptEventSource *sender, int count) //secondary context but dont block
{
    AppleIntelIWM* me = (AppleIntelIWM*) owner;

    if(!me) {
        return;
    }
    
    struct iwm_drv* driver = me->driver;
    
    int handled = 0;
    
    UInt32 r1 = 0;
    int rv = 0;
    int isPeriodic = 0;
    
    r1 = me->iwmRead(driver, IWM_CSR_INT);
    /* "hardware gone" (where, fishing?) */
    if(r1 == 0xffffffff || (r1 & 0xfffffff0) == 0xa5a5a5a0) {
        goto out;
    }
    
    if(r1 == 0) {
        goto outEna;
    }
    
    IWM_KERNLOG("R1 value is: 0x%X and interrupt bits set are 0x%X and r1 ored with mask is 0x%X\n", r1, driver->intMask, r1 | ~driver->intMask);
    
    me->iwmWrite32(driver, IWM_CSR_INT, r1 | ~driver->intMask);
    
    /* Safely ignore these bits for debug checks below */
    r1 &= ~(IWM_CSR_INT_BIT_ALIVE | IWM_CSR_INT_BIT_SCD);
    
    if(r1 & IWM_CSR_INT_BIT_SW_ERR) {
        int i;
        
        IWM_KERNLOG("Driver status: \n");
        for(i = 0; i < IWM_MAX_QUEUES; i++) {
            struct iwm_tx_ring *ring = &driver->txq[i];
            IWM_KERNLOG("Tx ring %d: qid=%d cur=%d queued=%d \n", i, ring->qid, ring->cur, ring->queued);
        }
        
        IWM_KERNLOG("RX ring: cur=%d \n", driver->rxq.cur);
        
        /* Reset our firmware state tracking. */
        driver->firmwareState = 0;
    
        /* Don't stop the device; just do a VAP restart */
        goto outEna;
    }
    
    if(r1 & IWM_CSR_INT_BIT_HW_ERR) {
        handled |= IWM_CSR_INT_BIT_HW_ERR;
        IWM_KERNLOG("Hardware Error, stopping device!\n");
        //iwmStop;
        rv = 1;
        goto out;
    }
     
    /* firmware chunk loaded */
    if(r1 & IWM_CSR_INT_BIT_FH_TX) {
        me->iwmWrite32(driver, IWM_CSR_FH_INT_STATUS, IWM_CSR_FH_INT_TX_MASK);
        handled |= IWM_CSR_INT_BIT_FH_TX;
        driver->fwChunkDone = 1;
        IWM_WAKEUP(driver, &driver->fw_info, true);
    }
    
    if(r1 & IWM_CSR_INT_BIT_RF_KILL) {
        handled |= IWM_CSR_INT_BIT_RF_KILL;
        if(me->checkRfKill(driver)) {
            IWM_KERNLOG("Rfkill switch, disabling interface!\n");
            //TODO: IWM STOP
        }
    }
    
    /*
     * The Linux driver uses periodic interrupts to avoid races.
     * We cargo-cult like it's going out of fashion.
     */
    if(r1 & IWM_CSR_INT_BIT_RX_PERIODIC) {
        handled |= IWM_CSR_INT_BIT_RX_PERIODIC;
        me->iwmWrite32(driver, IWM_CSR_INT, IWM_CSR_INT_BIT_RX_PERIODIC);
        if((r1 & (IWM_CSR_INT_BIT_FH_RX | IWM_CSR_INT_BIT_SW_RX)) == 0) {
            me->iwmWrite8(driver, IWM_CSR_INT_PERIODIC_REG, IWM_CSR_INT_PERIODIC_DIS);
        }
        isPeriodic = 1;
    }
    
    if(r1 & (IWM_CSR_INT_BIT_FH_RX | IWM_CSR_INT_BIT_SW_RX) || isPeriodic) {
        handled |= (IWM_CSR_INT_BIT_FH_RX | IWM_CSR_INT_BIT_SW_RX);
        me->iwmWrite32(driver, IWM_CSR_FH_INT_STATUS, IWM_CSR_FH_INT_RX_MASK);
        
        IWM_KERNLOG("Was going to call notif Intr\n");
        
        notifInterrupt(me, driver);
        
        /* enable periodic interrupt, see above */
        if(r1 & (IWM_CSR_INT_BIT_FH_RX | IWM_CSR_INT_BIT_SW_RX) && !isPeriodic) {
            me->iwmWrite8(driver, IWM_CSR_INT_PERIODIC_REG, IWM_CSR_INT_PERIODIC_ENA);
        }
    }
    
    rv = 1;
    
outEna:
    me->restoreInterrupts(driver);
out:
    return;
}

/*
 * Process an IWM_CSR_INT_BIT_FH_RX or IWM_CSR_INT_BIT_SW_RX interrupt.
 * Basic structure from if_iwn
 */
void AppleIntelIWM::notifInterrupt(AppleIntelIWM* me, struct iwm_drv *driver)
{
    int count;
    UInt32 wReg;
    UInt16 hw;
    
    driver->rxq.statDMA->cmd->readBytes(0, driver->rxq.stat, sizeof(struct iwm_rb_status));
    driver->rxq.freeDescDMA->cmd->readBytes(0, driver->rxq.desc, sizeof(UInt32) * IWM_RX_LEGACY_RING_COUNT);
    
    count = IWM_RX_LEGACY_RING_COUNT;
    wReg = IWM_FH_RSCSR_CHNL0_WPTR;
    
    /* uCode's read index (stored in shared DRAM) indicates the last Rx
     * buffer that the driver may process (last buffer filled by ucode). */
    hw = (driver->rxq.stat->closed_rb_num) & 0xFFF;
    
    /*
     * Process responses
     */
    while(driver->rxq.cur != hw) {
        struct iwm_rx_ring *ring = &driver->rxq;
        struct iwm_rx_data *data = &ring->data[ring->cur];
        
        IWM_KERNLOG("hw = %d cur = %d\n", hw, ring->cur);

        handleRxb(me, driver, data->m);
        
        ring->cur = (ring->cur + 1) % count;
    }
    
    /*
     * Tell the firmware that it can reuse the ring entries that
     * we have just processed.
     * Seems like the hardware gets upset unless we align
     * the write by 8??
     */
    hw = (hw == 0) ? count - 1 : hw - 1;
    me->iwmWrite32(driver, wReg, rounddown2(hw, 8));
}

/*
 * Support for dumping the error log seemed like a good idea ...
 * but it's mostly hex junk and the only sensible thing is the
 * hw/ucode revision (which we know anyway).  Since it's here,
 * I'll just leave it in, just in case e.g. the Intel guys want to
 * help us decipher some "ADVANCED_SYSASSERT" later.
 */
void AppleIntelIWM::handleRxb(AppleIntelIWM *me, struct iwm_drv *driver, mbuf_t m)
{
    IWM_KERNLOG("Handle RXB called!\n");
    //structieee 802211 com
    struct iwm_cmd_response cResp;
    mbuf_t m1;
    
    UInt32 offset = 0;
    UInt32 maxoff = IWM_RBUF_SIZE;
    UInt32 nextoff;
    bool stolen = false;
    
#define HAVEROOM(a) ((a) + sizeof(UInt32) + sizeof(struct iwm_cmd_header) < maxoff)
    
    while(HAVEROOM(offset)) {
        struct iwm_rx_packet *pkt = (struct iwm_rx_packet *) mbuf_data(m);
        
        int qid, idx, code, len;
        
        qid = pkt->hdr.qid;
        idx = pkt->hdr.idx;
        
        code = IWM_WIDE_ID(pkt->hdr.flags, pkt->hdr.code);
        
        /*
         * randomly get these from the firmware, no idea why.
         * they at least seem harmless, so just ignore them for now
         */
        
        IWM_KERNLOG("Code is: %d\n", pkt->hdr.code);
        
        if((pkt->hdr.code == 0 && (qid & ~0x80) == 0 && idx == 0) || pkt->len_n_flags == IWM_FH_RSCSR_FRAME_INVALID) {
            IWM_KERNLOG("About to break in handle RXB\n");
            break;
        }
    
        IWM_KERNLOG("rx packet qid=%d idx=%d type=%x\n", qid & ~0x80, pkt->hdr.idx, code);
        
        len = rxPacketLen(pkt);
        
        len += sizeof(UInt32); /* account for status word */
        nextoff = offset + roundup2(len, IWM_FH_RSCSR_FRAME_ALIGN);

        notifWaitNotify(driver->notifWait, code, pkt);
    
        return;
    }
    
#undef HAVEROOM
}


/**
 * Search for Message Signaled Interrupts source
 */
int AppleIntelIWM::findMSIInterruptTypeIndex(struct iwm_drv *driver)
{
    int i, source = 0;
    
    for(i = 0; ; i++) {
        int interruptType;
        
        if(driver->pciDevice->getInterruptType(i, &interruptType) != KERN_SUCCESS) {
            break;
        }
        
        if(interruptType & kIOInterruptTypePCIMessaged) {
            source = i;
            break;
        }
    }
    return source;
}

bool AppleIntelIWM::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *src)
{
    AppleIntelIWM* me = (AppleIntelIWM*)owner;
    if(!me) {
        IWM_KERNLOG("Interrupt Filter\n");
        return false;
    }
    
    /* Disable (but don't clear!) interrupts here to avoid
     * back-to-back ISRs and sporadic interrupts from our NIC.
     * If we have something to service, the tasklet will re-enable ints.
     * If we *don't* have something, we'll re-enable before leaving here.
     */
    me->iwmWrite32(me->driver, IWM_CSR_INT_MASK, 0x00000000);
    
    return true;
}

IOReturn AppleIntelIWM::startFirmware(struct iwm_drv *driver, const struct iwm_fw_img *fw)
{
    IWM_KERNLOG("Starting firmware!\n");
    
    int ret = 0;
    
    /* This may fail if AMT took ownership of the device */
    if(prepareCardHardware(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Failed to prepare hardware!\n");
        ret = EIO;
        goto out;
    }
    
    iwmWrite32(driver, IWM_CSR_INT, 0xFFFFFFFF);
    
    disableInterrupts(driver);
    
    /* make sure rfkill handshake bits are cleared */
    iwmWrite32(driver, IWM_CSR_UCODE_DRV_GP1_CLR, IWM_CSR_UCODE_SW_BIT_RFKILL);
    iwmWrite32(driver, IWM_CSR_UCODE_DRV_GP1_CLR, IWM_CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);
    
    /* clear (again), then enable host interrupts */
    iwmWrite32(driver, IWM_CSR_INT, 0xFFFFFFFF);
    
    ret = initNic(driver);
    if(ret != KERN_SUCCESS) {
        IWM_KERNLOG("Failed to init NIC!\n");
        goto out;
    }
    
    /*
     * Now, we load the firmware and don't want to be interrupted, even
     * by the RF-Kill interrupt (hence mask all the interrupt besides the
     * FH_TX interrupt which is needed to load the firmware). If the
     * RF-Kill switch is toggled, we will find out after having loaded
     * the firmware and return the proper value to the caller.
     */
    enableFwLoadInt(driver);
    
    /* really make sure rfkill handshake bits are cleared */
    /* maybe we should write a few times more?  just to make sure */
    iwmWrite32(driver, IWM_CSR_UCODE_DRV_GP1_CLR, IWM_CSR_UCODE_SW_BIT_RFKILL);
    iwmWrite32(driver, IWM_CSR_UCODE_DRV_GP1_CLR, IWM_CSR_UCODE_SW_BIT_RFKILL);
    
    /* Load the given image to the HW */
    if(driver->config->device_family >= IWM_DEVICE_FAMILY_8000) {
        //ret = iwm_pcie_load_given_ucode_8000(sc, fw);
    } else {
        ret = loadGivenUcode(driver, fw);
        if(ret) {
            IWM_KERNLOG("load given ucode failed!\n");
            goto out;
        }
    }
    
out:
    return ret;
}

IOReturn AppleIntelIWM::sendTxAntConfig(struct iwm_drv *driver, UInt8 validTxAnt)
{
    struct iwm_tx_ant_cfg_cmd txAntCmd = {
        .valid = validTxAnt,
    };
    
    return sendCmdPdu(driver, IWM_TX_ANT_CONFIGURATION_CMD, IWM_CMD_SYNC, sizeof(txAntCmd), &txAntCmd);
}



IOReturn AppleIntelIWM::loadGivenUcode(struct iwm_drv *driver, const struct iwm_fw_img *image)
{
    int ret = 0;
    int firstUcodeSection;
    
    IWM_KERNLOG("Working with %s CPU\n", image->is_dual_cpus ? "Dual" : "Single");
    
    /* load to FW the binary non secured sections of CPU1 */
    ret = loadCpuSections(driver, image, 1, &firstUcodeSection);
    if(ret) {
        return ret;
    }
    
    if(image->is_dual_cpus) {
        /* set CPU2 header address */
        if(nicLock(driver)) {
            writePRPH(driver, IWM_LMPM_SECURE_UCODE_LOAD_CPU2_HDR_ADDR, IWM_LMPM_SECURE_CPU2_HDR_MEM_SPACE);
            nicUnlock(driver);
        }
        
        /* load to FW the binary sections of CPU2 */
        ret = loadCpuSections(driver, image, 2, &firstUcodeSection);
        if(ret) {
            return ret;
        }
    }
    
    enableInterrupts(driver);

    /* release CPU reset */
    iwmWrite32(driver, IWM_CSR_RESET, 0);
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::preInit(struct iwm_drv *driver)
{
    //do 80211 things here
    IWM_LOCK(driver);

    if(startHardware(driver) != KERN_SUCCESS) {
        IWM_KERNLOG("Couldn't init hardware\n");
        IWM_UNLOCK(driver);
        return KERN_FAILURE;
    }
    
    if(initUcode(driver, 1) != KERN_SUCCESS) {
        IWM_UNLOCK(driver);
        return KERN_FAILURE;
    }
    
    stopDevice(driver);
    
    IWM_UNLOCK(driver);

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::loadCpuSections(struct iwm_drv *driver, const struct iwm_fw_img *image, int cpu, int *firstUcodeSection)
{
    int shiftParam;
    
    int i, ret = 0;
    UInt32 lastReadIndex = 0;
    
    if(cpu == 1) {
        shiftParam = 0;
        *firstUcodeSection = 0;
    } else {
        shiftParam = 16;
        (*firstUcodeSection)++;
    }
    
    for(i = *firstUcodeSection; i < IWM_UCODE_SECTION_MAX; i++) {
        lastReadIndex = i;
        
        /*
         * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between
         * CPU1 to CPU2.
         * PAGING_SEPARATOR_SECTION delimiter - separate between
         * CPU2 non paged to CPU2 paging sec.
         */
        
        if(!image->sec[i].data ||
           image->sec[i].offset == IWM_CPU1_CPU2_SEPARATOR_SECTION ||
           image->sec[i].offset == IWM_PAGING_SEPARATOR_SECTION) {
            IWM_KERNLOG("Break becuase data is not valid or empty section: %d\n", i);
            break;
        }
        
        ret = loadSection(driver, i, &image->sec[i]);
        if(ret != KERN_SUCCESS) {
            IWM_KERNLOG("Load section %d failed\n", i);
            return ret;
        }
    }
    
    *firstUcodeSection = lastReadIndex;
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::loadSection(struct iwm_drv *driver, UInt8 sectionNum, const struct iwm_fw_desc *section)
{
    struct iwm_dma_info *dma = driver->firmwareDMA;
    UInt32 offset, chunkSize = MIN(IWM_FH_MEM_TB_MAX_LENGTH, section->len);
    
    UInt8* vAddr;
    UInt32 pAddr;
    
    IWM_KERNLOG("[%d] uCode section being loaded...\n", sectionNum);
    
    vAddr = (UInt8 *) dma->vAddr;
    pAddr = dma->pAddr;
    
    dma->bmd->prepare();
    dma->cmd->prepare();
    
    for(offset = 0; offset < section->len; offset += chunkSize) {
        UInt32 copySize, destAddr;
        int extendedAddr = FALSE;
        
        copySize = MIN(chunkSize, section->len - offset);
        destAddr = section->offset + offset;
        
        if(destAddr >= IWM_FW_MEM_EXTENDED_START && destAddr <= IWM_FW_MEM_EXTENDED_END) {
            extendedAddr = TRUE;
        }
        
        if(extendedAddr) {
            setBitsPRPH(driver, IWM_LMPM_CHICK, IWM_LMPM_CHICK_EXTENDED_ADDR_SPACE);
        }
        
        dma->cmd->writeBytes(0, (const UInt8 *)section->data + offset, copySize);
        
        int ret = loadFirmwareChunk(driver, destAddr, pAddr, copySize);
    
        if(extendedAddr) {
            clearBitsPRPH(driver, IWM_LMPM_CHICK, IWM_LMPM_CHICK_EXTENDED_ADDR_SPACE);
        }
        
        if(ret != KERN_SUCCESS) {
            IWM_KERNLOG("Couldn't load the [%d] uCcode section\n", sectionNum);
            break;
        }
    }
    
    dma->bmd->complete();
    dma->cmd->complete();
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::loadFirmwareChunk(struct iwm_drv *driver, UInt32 destAddr, UInt32 phyAddr, UInt32 byteCount)
{
    if(!driver->mtxLocked) {
        return ENOLCK;
    }
    
    driver->fwChunkDone = 0;
    
    if(!nicLock(driver)) {
        return EBUSY;
    }
    
    iwmWrite32(driver, IWM_FH_TCSR_CHNL_TX_CONFIG_REG(IWM_FH_SRVC_CHNL), IWM_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_PAUSE);
    
    iwmWrite32(driver, IWM_FH_SRVC_CHNL_SRAM_ADDR_REG(IWM_FH_SRVC_CHNL), destAddr);
    
    iwmWrite32(driver, IWM_FH_TFDIB_CTRL0_REG(IWM_FH_SRVC_CHNL), phyAddr & IWM_FH_MEM_TFDIB_DRAM_ADDR_LSB_MSK);
    
    iwmWrite32(driver, IWM_FH_TFDIB_CTRL1_REG(IWM_FH_SRVC_CHNL), (getDmaUpperAddr(phyAddr) << IWM_FH_MEM_TFDIB_REG1_ADDR_BITSHIFT) | byteCount);
    
    iwmWrite32(driver, IWM_FH_TCSR_CHNL_TX_BUF_STS_REG(IWM_FH_SRVC_CHNL),
               1 << IWM_FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_NUM |
               1 << IWM_FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_IDX |
               IWM_FH_TCSR_CHNL_TX_BUF_STS_REG_VAL_TFDB_VALID);
    
    iwmWrite32(driver, IWM_FH_TCSR_CHNL_TX_CONFIG_REG(IWM_FH_SRVC_CHNL),
               IWM_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE    |
               IWM_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_DISABLE |
               IWM_FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_HOST_ENDTFD);

    nicUnlock(driver);
    
    /* wait up to 5s for this segment to load */
    IWM_SLEEP(driver, &driver->fw_info, THREAD_INTERRUPTIBLE, 5);
    if(!driver->fwChunkDone) {
        IWM_KERNLOG("fw chunk addr 0x%x len %d failed to load\n", destAddr, byteCount);
        return ETIMEDOUT;
    } else {
        IWM_KERNLOG("Fw chunk success!\n");
    }
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::enableFwLoadInt(struct iwm_drv *driver)
{
    IWM_KERNLOG("Enabling FW load interrupt\n");
    driver->intMask = IWM_CSR_INT_BIT_FH_TX;
    iwmWrite32(driver, IWM_CSR_INT_MASK, driver->intMask);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::initHardware(struct iwm_drv *driver)
{
    int error, i, ac;
    
    driver->sfState = IWM_SF_UNINIT;
    
    error = startHardware(driver);
    if(error) {
        IWM_KERNLOG("start hw failed\n");
        return error;
    }
    
    error = initUcode(driver, 0);
    if(error) {
        IWM_KERNLOG("init ucode failed!\n");
        return error;
    } else {
        IWM_KERNLOG("no issues with init ucode\n");
    }
    
    /*
     * should stop and start HW since that INIT
     * image just loaded
     */
    stopDevice(driver);
    driver->psDisabled = FALSE;
    error = startHardware(driver);
    if(error) {
        IWM_KERNLOG("Couldn't init hardware\n");
        return error;
    }
    
    /* omstart, this time with the regular firmware */
    error = loadUcodeWaitAlive(driver, IWM_UCODE_REGULAR);
    if(error) {
        IWM_KERNLOG("Couldn't load ucode!\n");
        return error;
    }
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::loadUcodeWaitAlive(struct iwm_drv *driver, enum iwm_ucode_type ucodeType)
{
    struct iwm_notification_wait aliveWait;
    struct iwm_alive_data aliveData;
    const struct iwm_fw_img *fw;
    enum iwm_ucode_type oldType = driver->currentUcode;
    int error = 0;
    
    static const UInt16 aliveCmd[] = { IWM_ALIVE };
    
    fw = &driver->fw_info.img[ucodeType];
    driver->currentUcode = ucodeType;
    driver->ucodeLoaded = FALSE;
    
    memset(&aliveData, 0, sizeof(aliveData));
    
    initNotificationWait(driver->notifWait, &aliveWait, aliveCmd, nitems(aliveCmd), aliveFn, &aliveData);
    
    error = startFirmware(driver, fw);
    if(error) {
        IWM_KERNLOG("Iwm Start Fw failed with error: %d\n", error);
        driver->currentUcode = oldType;
        removeNotification(driver->notifWait, &aliveWait);
        return error;
    }
    
    /*
     * Some things may run in the background now, but we
     * just wait for the ALIVE notification here.
     */
    IWM_UNLOCK(driver);
    error = waitNotification(driver->notifWait, &aliveWait, IWM_UCODE_ALIVE_TIMEOUT);
    IWM_LOCK(driver);
    if(error) {
        IWM_KERNLOG("Wait notification returned an error! %d\n", error);
        driver->currentUcode = oldType;
        return error;
    }
    
    if(!aliveData.valid) {
        IWM_KERNLOG("Loaded ucode is not valid!\n");
        driver->currentUcode = oldType;
        return EIO;
    } else {
        IWM_KERNLOG("Alive data was valid!\n");
    }
    
    pcieFwAlive(driver, aliveData.scd_base_addr);
    
    /*
     * configure and operate fw paging mechanism.
     * driver configures the paging flow only once, CPU2 paging image
     * included in the IWM_UCODE_INIT image.
     */
    if(fw->paging_mem_size) {
        error = saveFwPaging(driver, fw);
        if(error) {
            IWM_KERNLOG("Failed to save the FW paging image\n");
            return error;
        }
        
        error = sendPagingCmd(driver, fw);
        if(error) {
            IWM_KERNLOG("Failed to send the paging cmd\n");
            freeFwPaging(driver);
            return error;
        }
    }
    
    if(!error) {
        driver->ucodeLoaded = TRUE;
        IWM_KERNLOG("Ucode was successfully loaded!\n");
    } else {
        IWM_KERNLOG("Failed to load Ucode!\n");
    }

    return error;
}


void AppleIntelIWM::enableInterrupts(struct iwm_drv *driver)
{
    driver->intMask = IWM_CSR_INI_SET_MASK;
    iwmWrite32(driver, IWM_CSR_INT_MASK, driver->intMask);
}

IOReturn AppleIntelIWM::restoreInterrupts(struct iwm_drv *driver)
{
    iwmWrite32(driver, IWM_CSR_INT_MASK, driver->intMask);
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::disableInterrupts(struct iwm_drv* driver)
{
    /* disable interrupts */
    iwmWrite32(driver, IWM_CSR_INT_MASK, 0);
    
    /* acknowledge all interrupts */
    iwmWrite32(driver, IWM_CSR_INT, ~0);
    iwmWrite32(driver, IWM_CSR_FH_INT_STATUS, ~0);
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::ictReset(struct iwm_drv *driver)
{
    disableInterrupts(driver);
    
    driver->ictDMA->cmd->synchronize(kIODirectionIn);
    
    /* Reset ICT table. */
    memset(driver->ictTable, 0, IWM_ICT_SIZE);
    driver->ictCur = 0;
    
    IWM_KERNLOG("INT TBL REG=0x%X\n", iwmRead(driver, IWM_CSR_DRAM_INT_TBL_REG));
    
    /* Set physical address of ICT table (4KB aligned). */
    iwmWrite32(driver, IWM_CSR_DRAM_INT_TBL_REG, IWM_CSR_DRAM_INT_TBL_ENABLE
               | IWM_CSR_DRAM_INIT_TBL_WRITE_POINTER
               | IWM_CSR_DRAM_INIT_TBL_WRAP_CHECK
               | driver->ictDMA->pAddr >> IWM_ICT_PADDR_SHIFT);
    
    /* Switch to ICT interrupt mode in driver. */
    driver->flags |= IWM_FLAG_USE_ICT;
    
    /* Re-enable interrupts. */
    iwmWrite32(driver, IWM_CSR_INT, ~0);
    enableInterrupts(driver);
    
    return KERN_SUCCESS;
}


/*
 * Since this .. hard-resets things, it's time to actually
 * mark the first vap (if any) as having no mac context.
 * It's annoying, but since the driver is potentially being
 * stop/start'ed whilst active (thanks openbsd port!) we
 * have to correctly track this.
 */
IOReturn AppleIntelIWM::stopDevice(struct iwm_drv *driver)
{
    int chnl, qid;
    UInt32 mask = 0;
    
    /* tell the device to stop sending interrupts */
    disableInterrupts(driver);
    
    /*
     * FreeBSD-local: mark the first vap as not-uploaded,
     * so the next transition through auth/assoc
     * will correctly populate the MAC context.
     */
    if(0) {
    }
    
    driver->firmwareState = 0;
    driver->flags &= ~IWM_FLAG_TE_ACTIVE;
    
    /* stop tx and rx.  tx and rx bits, as usual, are from if_iwn */
    if(nicLock(driver)) {
        writePRPH(driver, IWM_SCD_TXFACT, 0);
        
        for (chnl = 0; chnl < IWM_FH_TCSR_CHNL_NUM; chnl++) {
            iwmWrite32(driver, IWM_FH_TCSR_CHNL_TX_CONFIG_REG(chnl), 0);
            mask |= IWM_FH_TSSR_TX_STATUS_REG_MSK_CHNL_IDLE(chnl);
        }
        
        /* Wait for DMA channels to be idle */
        if(!iwmPollBit(driver, IWM_FH_TSSR_TX_STATUS_REG, mask, mask, 5000)) {
            IWM_KERNLOG("Failing on timeout while stopping DMA channel: [0x%08x]\n", iwmRead(driver, IWM_FH_TSSR_TX_STATUS_REG));
        }
        nicUnlock(driver);
    }
    pcieRxStop(driver);
    
    /* Stop RX ring. */
    resetRxRing(driver, &driver->rxq);
    
    /* Reset all TX rings. */
    for(qid = 0; qid < nitems(driver->txq); qid++) {
        resetTxRing(driver, &driver->txq[qid]);
    }
    
    if(driver->config->device_family == IWM_DEVICE_FAMILY_7000) {
        /* Power-down device's busmaster DMA clocks */
        if(nicLock(driver)) {
            writePRPH(driver, IWM_APMG_CLK_DIS_REG, IWM_APMG_CLK_VAL_DMA_CLK_RQT);
            nicUnlock(driver);
        }
        IOSleep(5);
    }
    
    /* Make sure (redundant) we've released our request to stay awake */
    iwmClearBits(driver, IWM_CSR_GP_CNTRL, IWM_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
    
    /* Stop the device, and put it in low power state */
    apmStop(driver);
    
    /* stop and reset the on-board processor */
    iwmSetBits(driver, IWM_CSR_RESET, IWM_CSR_RESET_REG_FLAG_SW_RESET);
    IOSleep(5000);
    
    /*
     * Upon stop, the APM issues an interrupt if HW RF kill is set.
     */
    disableInterrupts(driver);
    
    /*
     * Even if we stop the HW, we still want the RF kill
     * interrupt
     */
    enableRfKillInterrupt(driver);
    checkRfKill(driver);
    
    prepareCardHardware(driver);
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::initNic(struct iwm_drv *driver)
{
    
    int error = 0;
    
    apmInit(driver);
    
    if(driver->config->device_family == IWM_DEVICE_FAMILY_7000) {
        setPowerMode(driver);
    }
    
    nicConfig(driver);
    
    error = initNicRx(driver);
    if(error != KERN_SUCCESS) {
        IWM_KERNLOG("Init nic Rx failed!\n");
        return error;
    }
    
    /*
     * Ditto for TX, from iwn
     */
    error = initNicTx(driver);
    if(error != KERN_SUCCESS) {
        IWM_KERNLOG("Init nic Tx failed!\n");
        return error;
    }
    
    IWM_KERNLOG("Shadow registers enabled\n");
    
    iwmSetBits(driver, IWM_CSR_MAC_SHADOW_REG_CTRL, 0x800fffff);
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::aliveFn(struct iwm_drv *driver, struct iwm_rx_packet *pkt, void *data)
{
    IWM_KERNLOG("Alive FN called!\n");
    
    struct iwm_alive_data *aliveData = (struct iwm_alive_data*) data;
    struct iwm_mvm_alive_resp_v3 *pAlive3;
    struct iwm_mvm_alive_resp *pAlive;
    struct iwm_umac_alive *umac;
    struct iwm_lmac_alive *lMac1;
    struct iwm_lmac_alive *lMac2 = NULL;
    UInt16 status;
    
    if(rxPacketPayloadLen(pkt) == sizeof(*pAlive)) {
        pAlive = (struct iwm_mvm_alive_resp*) pkt->data;
        umac = &pAlive->umac_data;
        lMac1 = (struct iwm_lmac_alive*) &pAlive->lmac_data[0];
        lMac2 = (struct iwm_lmac_alive*) &pAlive->lmac_data[1];
        status = pAlive->status;
    } else {
        pAlive3 = (struct iwm_mvm_alive_resp_v3*) pkt->data;
        umac = &pAlive3->umac_data;
        lMac1 = (struct iwm_lmac_alive*) &pAlive3->lmac_data;
        status = pAlive3->status;
    }
    
    driver->error_event_table[0] = lMac1->error_event_table_ptr;
    if(lMac2) {
        driver->error_event_table[1] = lMac2->error_event_table_ptr;
    }
    
    driver->log_event_table = lMac1->log_event_table_ptr;
    driver->umac_error_event_table = umac->error_info_addr;
    aliveData->scd_base_addr = lMac1->scd_base_ptr;
    aliveData->valid = status == IWM_ALIVE_STATUS_OK;
    
    if(driver->umac_error_event_table) {
        driver->support_umac_log = TRUE;
    }
    
    IWM_KERNLOG("Alive ucode status 0x%04x revision 0x%01X 0x%01X\n", status, lMac1->ver_type, lMac1->ver_subtype);
    
    if(lMac2) {
        IWM_KERNLOG("Alive ucode CDB\n");
    }
    
    IWM_KERNLOG("UMAC version: Major - 0x%x, Minor - 0x%x\n", umac->umac_major, umac->umac_minor);
    
    return KERN_SUCCESS;
}

void AppleIntelIWM::firmwareLoadComplete(OSKextRequestTag requestTag, OSReturn result, const void *resourceData, uint32_t resourceDataLength, void *context)
{
    //Interrupt Context
    FirmwareLoadProgress* progress = (FirmwareLoadProgress*)context;
    
    IOLockLock(progress->driver->firmwareLoadLock);
    if(result == kOSReturnSuccess) {
        IWM_KERNLOG("Loading FW worked\n");
        progress->firmwareData = OSData::withBytes(resourceData, resourceDataLength);
    } else {
        IWM_KERNLOG("Loading FW failed: 0x%x\n", result);
    }
    
    IOLockUnlock(progress->driver->firmwareLoadLock);
    IOLockWakeup(progress->driver->firmwareLoadLock, progress->driver, true);
}

IOReturn AppleIntelIWM::zeroDriver(struct iwm_drv *driver)
{
    if(!driver) {
        IWM_KERNLOG("Driver  doesn't exist!\n");
        return KERN_FAILURE;
    }
    
    struct iwm_fw_info fw_info = { 0 };
    
    driver->fw_info = fw_info;
    
    driver->ictCur = 0;
    driver->hwRev = 0;
    driver->hwId = 0;
    driver->intMask = 0;
    driver->cmdHoldNicAwake = 0;
    driver->maxRssi = 0;
    driver->noise = 0;
    driver->flags = 0; //only 8 flags
    driver->fwChunkDone = 0;
    driver->lastEbsSuccessful = 0;
    driver->wantResp = 0;
    driver->generation = 0;
    driver->firmwareState = 0;
    driver->mtxLocked = 0;

    
    return KERN_SUCCESS;
}


OSData* AppleIntelIWM::requestLoadFirmware(struct iwm_drv *driver)
{
    IWM_KERNLOG("Loading Firmware!\n");
    
    IOLockLock(firmwareLoadLock);
    
    FirmwareLoadProgress loadProgress = {.driver = this, .firmwareData = NULL};
    
    int nameSize = (int) strlen(driver->config->fw_name) + sizeof(".ucode");
    
    char fwName[nameSize];
    
    snprintf(fwName, nameSize, "%s.ucode", driver->config->fw_name);
    
    IWM_KERNLOG("Requesting load from file: %s \n", fwName);
    
    OSReturn result = OSKextRequestResource(OSKextGetCurrentIdentifier(), fwName, firmwareLoadComplete, &loadProgress, NULL);
    
    if(result != kIOReturnSuccess) {
        IWM_KERNLOG("Loading Firmware was a failure!\n");
        IOLockUnlock(firmwareLoadLock);
        return NULL;
    }
    
    IOLockSleep(firmwareLoadLock, this, THREAD_INTERRUPTIBLE);
    IOLockUnlock(firmwareLoadLock);
    
    if(loadProgress.firmwareData) {
        IWM_KERNLOG("Returning firmwareData!\n");
        return loadProgress.firmwareData;
    } else {
        return NULL;
    }
}


IOReturn AppleIntelIWM::storeCScheme(struct iwm_drv *driver, const UInt8 *data, size_t dataLen)
{
    const struct iwm_fw_cscheme_list* list = (iwm_fw_cscheme_list*) data;
    
    if(dataLen < sizeof(*list) || dataLen < (sizeof(list->size) + list->size * sizeof(*list->cs))) {
        IWM_KERNLOG("Store CSheme failed\n");
        return KERN_FAILURE;
    }

    /* we don't actually store anything for now, always use s/w crypto */
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::storeFirmwareSection(struct iwm_drv *driver, enum iwm_ucode_type type, const UInt8 *data, UInt32 dataLen)
{
    struct iwm_fw_img* firmwareSection;
    struct iwm_fw_desc* firmwareDescription;
    
    IWM_KERNLOG("img type is: %d \n", type);
    IWM_KERNLOG("size of data is: %X \n", dataLen);
    
    if(type >= IWM_UCODE_TYPE_MAX) {
        IWM_KERNLOG("Store firmware section failed (wrong type)\n");
        return KERN_FAILURE;
    }
    
    if(dataLen < sizeof(UInt32)) {
        IWM_KERNLOG("Store firmware section failed (data len too small)\n");
        return KERN_FAILURE;
    }
    
    firmwareSection = &driver->fw_info.img[type];
    
    IWM_KERNLOG("fw count is: %d max is: %d \n", firmwareSection->fw_count, IWM_UCODE_SECTION_MAX);
    
    if(firmwareSection->fw_count >= IWM_UCODE_SECTION_MAX) {
        IWM_KERNLOG("Store firmware section failed (fwcount bigger than max)\n");
        return KERN_FAILURE;
    }

    firmwareDescription = &firmwareSection->sec[firmwareSection->fw_count];
    
    /* first 32bit are device load offset */
    memcpy(&firmwareDescription->offset, data, sizeof(UInt32));
    
    /* rest is data */
    firmwareDescription->data = data + sizeof(UInt32);
    firmwareDescription->len = (UInt32) dataLen - sizeof(UInt32);
    
    firmwareSection->fw_count++;
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::setUcodeApiFlags(struct iwm_drv *driver, const UInt8 *data, struct iwm_ucode_capabilities *capabilities)
{
    const struct iwm_ucode_api *ucode_api = (struct iwm_ucode_api*)data;
    UInt32 apiIndex = ucode_api->api_index;
    UInt32 apiFlags = ucode_api->api_flags;
    
    if(apiIndex >= howmany(IWM_NUM_UCODE_TLV_API, 32)) {
        //Error but dont return error
        return KERN_SUCCESS;
    }

    int i = 0;

    for(i=0; i < 32; i++) {
        if(apiFlags & (1U << i)) {
            OSTestAndSet(i + 32 * apiIndex, capabilities->enabled_capa);
        }
    }

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::setUcodeCapabilities(struct iwm_drv *driver, const UInt8 *data, struct iwm_ucode_capabilities *capabilities)
{
    const struct iwm_ucode_capa *ucode_capabilities = (iwm_ucode_capa* )data;
    UInt32 apiIndex = ucode_capabilities->api_index;
    UInt32 apiFlags = ucode_capabilities->api_capa;
    
    if(apiIndex >= DIV_ROUND_UP(IWM_NUM_UCODE_TLV_CAPA, 32)) {
        //Error but dont return an error
        return KERN_SUCCESS;
    }
    
    int i = 0;
    
    for(i = 0; i < 32; i++) {
        if(apiFlags & (1U << i)) {
            OSTestAndSet(i + 32 * apiIndex, capabilities->enabled_capa);
        }
    }

    return KERN_SUCCESS;
}


struct iwm_tlv_calib_data {
    uint32_t ucode_type;
    struct iwm_tlv_calib_ctrl calib;
} __packed;

IOReturn AppleIntelIWM::setDefaultCalibration(struct iwm_drv *driver, const void *data)
{
    const struct iwm_tlv_calib_data *default_calib = (struct iwm_tlv_calib_data*) data;
    UInt32 ucodeType = default_calib->ucode_type;
    
    if(ucodeType >= IWM_UCODE_TYPE_MAX) {
        IWM_KERNLOG("Wrong type for default calibration. \n");
        return KERN_FAILURE;
    }
    
    driver->default_calib[ucodeType].flow_trigger = default_calib->calib.flow_trigger;
    driver->default_calib[ucodeType].event_trigger = default_calib->calib.event_trigger;
    
    return KERN_SUCCESS;
}



IOReturn AppleIntelIWM::readFirmware(struct iwm_drv *driver)
{
    struct iwm_fw_info* fw = &driver->fw_info;
    struct iwm_ucode_capabilities* capa = &driver->fw_info.ucode_capa;
    
    OSData* fwData = requestLoadFirmware(driver);
    
    if(!fwData) {
        return KERN_FAILURE;
    }
    
    IWM_KERNLOG("Parsing FW\n");
    
    /*
     * Load firmware into driver memory.
     * pointer to fw data will be set.
     */
    
    
    /* (Re-)Initialize default values. */
    capa->flags = 0;
    capa->max_probe_length = IWM_DEFAULT_MAX_PROBE_LENGTH;
    capa->n_scan_channels = IWM_DEFAULT_SCAN_CHANNELS;
    
    memset(capa->enabled_capa, 0, sizeof(capa->enabled_capa));
    memset(capa->enabled_api, 0, sizeof(capa->enabled_api));
    memset(driver->fw_mcc, 0, sizeof(driver->fw_mcc));
    
    /*
     * Parse firmware contents
     */
    
    struct iwm_tlv_ucode_header* uhdr = (struct iwm_tlv_ucode_header*) fwData->getBytesNoCopy();
    
    if(uhdr->zero) {
        IWM_KERNLOG("FW should start with ZEROS\n");
        return KERN_FAILURE;
    }
    
    if(uhdr->magic != IWM_TLV_UCODE_MAGIC)
    {
        IWM_KERNLOG("Invalid value for magic bits\n");
        return KERN_FAILURE;
    }
    
    fw->ucodeVersion = uhdr->ver;
    
    IWM_KERNLOG("Version info: %u.%u.%u.%u \n", IWM_UCODE_MAJOR(fw->ucodeVersion), IWM_UCODE_MINOR(fw->ucodeVersion), IWM_UCODE_API(fw->ucodeVersion), IWM_UCODE_SERIAL(fw->ucodeVersion));
    
    
    struct iwm_ucode_tlv *tlv;
    UInt32 pagingMemSize;
    UInt32 uSnifferImg;
    enum iwm_ucode_tlv_type tlvType;
    UInt32 tlvLen;
    UInt8* tlvData;
    int numOfCpus = 1;
    
    const UInt8* data = uhdr->data;
    //Take away header and then deserialize the rest
    size_t ucodeLen = fwData->getLength() - sizeof(*uhdr);

    while(ucodeLen >= sizeof(*tlv)) {
        ucodeLen -= sizeof(*tlv);
        tlv = (struct iwm_ucode_tlv*)data;
        
        tlvLen = tlv->length;
        tlvType = (iwm_ucode_tlv_type) tlv->type;
        tlvData = tlv->data;
    
        if(ucodeLen < tlvLen) {
            IWM_KERNLOG("Firmware is too short!\n");
            return KERN_FAILURE;
            //TODO: GOTO ERROR
        }
        ucodeLen -= ALIGN(tlvLen, 4);
        data += sizeof(*tlv) + ALIGN(tlvLen, 4);
        
        IWM_KERNLOG("TLV Len: %X Type is: %d\n", tlvLen, (int)tlvType);
        
        switch(tlvType) {
            case IWM_UCODE_TLV_PROBE_MAX_LEN:
                 if (tlvLen != sizeof(UInt32)) {
                    IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_PROBE_MAX_LEN\n");
                    return KERN_FAILURE;
                 }
                 capa->max_probe_length = *(UInt32*) tlvData;
                 /* limit it to something sensible */
                 if (capa->max_probe_length > IWM_SCAN_OFFLOAD_PROBE_REQ_SIZE) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_PROBE_MAX_LEN\n");
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_PAN:
                 if (tlvLen) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_PAN \n");
                     return KERN_FAILURE;
                 }
                 capa->flags |= IWM_UCODE_TLV_FLAGS_PAN;
                break;
            case IWM_UCODE_TLV_FLAGS:
                 if (tlvLen < sizeof(UInt32)) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_FLAGS\n");
                     return KERN_FAILURE;
                 }
                 if (tlvLen % sizeof(UInt32)) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_FLAGS\n");
                     return KERN_FAILURE;
                 }
                 /*
                 * Apparently there can be many flags, but Linux driver
                 * parses only the first one, and so do we.
                 *
                 * XXX: why does this override IWM_UCODE_TLV_PAN?
                 * Intentional or a bug?  Observations from
                 * current firmware file:
                 *  1) TLV_PAN is parsed first
                 *  2) TLV_FLAGS contains TLV_FLAGS_PAN
                 * ==> this resets TLV_PAN to itself... hnnnk
                 */
                capa->flags = *(UInt32*)tlvData;
                break;
            case IWM_UCODE_TLV_CSCHEME:
                 if (storeCScheme(driver, tlvData, tlvLen) != KERN_SUCCESS) {
                     IWM_KERNLOG("Error trying to parse FW CScheme failed\n");
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_NUM_OF_CPU:
                 if (tlvLen != sizeof(UInt32)) {
                     IWM_KERNLOG("Error trying to parse FW nun of CPU\n");
                     return KERN_FAILURE;
                 }
                
                 numOfCpus = *(UInt32*)tlvData;
                
                IWM_KERNLOG("There are %d CPUS!\n", numOfCpus);
                
                if (numOfCpus == 2) {
                    fw->img[IWM_UCODE_REGULAR].is_dual_cpus = TRUE;
                    fw->img[IWM_UCODE_INIT].is_dual_cpus = TRUE;
                    fw->img[IWM_UCODE_WOWLAN].is_dual_cpus = TRUE;
                 } else if ((numOfCpus > 2) || (numOfCpus < 1)) {
                     IWM_KERNLOG("Error trying to parse FW wrong number of CPUS %d\n", numOfCpus);
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_SEC_RT:
                 IWM_KERNLOG("Calling storeFW from IWM_UCODE_TLV_SEC_RT \n");
                 if(storeFirmwareSection(driver, IWM_UCODE_REGULAR, tlvData, tlvLen) != KERN_SUCCESS) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_SEC_RT\n");
                     return KERN_FAILURE; 
                 }
                break;
            case IWM_UCODE_TLV_SEC_INIT:
                IWM_KERNLOG("Calling storeFW from IWM_UCODE_TLV_SEC_INIT \n");
                 if(storeFirmwareSection(driver, IWM_UCODE_INIT, tlvData, tlvLen) != KERN_SUCCESS) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_SEC_INIT \n");
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_SEC_WOWLAN:
                 IWM_KERNLOG("Calling storeFW from IWM_UCODE_TLV_SEC_WOWLAN \n");
                 if (storeFirmwareSection(driver, IWM_UCODE_WOWLAN, tlvData, tlvLen) != KERN_SUCCESS) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_SEC_WOWLAN\n");
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_DEF_CALIB:
                 if (tlvLen != sizeof(iwm_tlv_calib_data)) {
                     //TODO: ERROR
                     IWM_KERNLOG("Error trying to parse fw def calib\n");
                     return KERN_FAILURE;
                 }
                 if (setDefaultCalibration(driver, tlvData) != KERN_SUCCESS) {
                     //TODO: ERROR
                     IWM_KERNLOG("Error trying to parse FW def calib\n");
                     return KERN_FAILURE;
                 }
                
                break;
            case IWM_UCODE_TLV_PHY_SKU:
                 if (tlvLen != sizeof(UInt32)) {
                    IWM_KERNLOG("Error trying to parse FW PHY SKU\n");
                     return KERN_FAILURE;
                 }
                 driver->fw_info.phy_config = *(UInt32*)tlvData;
                 driver->fw_info.valid_tx_ant = (driver->fw_info.phy_config & IWM_FW_PHY_CFG_TX_CHAIN) >> IWM_FW_PHY_CFG_TX_CHAIN_POS;
                 driver->fw_info.valid_rx_ant = (driver->fw_info.phy_config & IWM_FW_PHY_CFG_RX_CHAIN) >> IWM_FW_PHY_CFG_RX_CHAIN_POS;
                break;
            case IWM_UCODE_TLV_API_CHANGES_SET:
                 if (tlvLen != sizeof(struct iwm_ucode_api)) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_API_CHANGES_SET \n");
                     return KERN_FAILURE;
                 }
                 if (setUcodeApiFlags(driver, tlvData, capa) != KERN_SUCCESS) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_API_CHANGES_SET \n");
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_ENABLED_CAPABILITIES:
                if (tlvLen != sizeof(struct iwm_ucode_capa)) {
                        IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_ENABLED_CAPABILITIES\n");
                        return KERN_FAILURE;
                }
                
                if (setUcodeCapabilities(driver, tlvData, capa) != KERN_SUCCESS) {
                        IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_ENABLED_CAPABILITIES\n");
                        return KERN_FAILURE;
                }
                break;
            case IWM_UCODE_TLV_UNDOCUMENTED: /* undocumented TLV */
            case IWM_UCODE_TLV_SDIO_ADMA_ADDR:
            case IWM_UCODE_TLV_FW_GSCAN_CAPA:
                /* ignore, not used by current driver */
                IWM_KERNLOG("Ignore this code\n");
                break;
            case IWM_UCODE_TLV_SEC_RT_USNIFFER:
                IWM_KERNLOG("Calling storeFW from IWM_UCODE_TLV_SEC_RT_USNIFFER \n");
                 if (storeFirmwareSection(driver, IWM_UCODE_REGULAR_USNIFFER, tlvData, tlvLen) != KERN_SUCCESS) {
                     IWM_KERNLOG("Error trying to parse FW IWM_UCODE_TLV_SEC_RT_USNIFFER\n");
                     return KERN_FAILURE;
                 }
                break;
            case IWM_UCODE_TLV_PAGING:
                 if (tlvLen != sizeof(UInt32)) {
                     IWM_KERNLOG("Error trying to parse FW\n");
                     return KERN_FAILURE;
                 }
                 pagingMemSize = *(UInt32 *)tlvData;
                 
                 IWM_KERNLOG("Paging enabled (size = %u bytes)\n", pagingMemSize);
                 
                 if (pagingMemSize > IWM_MAX_PAGING_IMAGE_SIZE) {
                     IWM_KERNLOG("Error trying to parse FW\n");
                     return KERN_FAILURE;
                 }
                 
                 if (pagingMemSize & (IWM_FW_PAGING_SIZE - 1)) {
                     IWM_KERNLOG("Error trying to parse FW\n");
                     return KERN_FAILURE;
                 }
                 driver->fw_info.img[IWM_UCODE_REGULAR].paging_mem_size = pagingMemSize;
                 uSnifferImg = IWM_UCODE_REGULAR_USNIFFER;
                 driver->fw_info.img[uSnifferImg].paging_mem_size = pagingMemSize;
                break;
            case IWM_UCODE_TLV_N_SCAN_CHANNELS:
                 if (tlvLen != sizeof(UInt32)) {
                     IWM_KERNLOG("Error trying to parse FW\n");
                     return KERN_FAILURE;
                 }
                 capa->n_scan_channels = *(UInt32 *)tlvData;
                break;
            case IWM_UCODE_TLV_FW_VERSION:
                 if(tlvLen != sizeof(UInt32) * 3) {
                     IWM_KERNLOG("Error trying to parse FW\n");
                     return KERN_FAILURE;
                 }
                 
                 snprintf(driver->fwver, sizeof(driver->fwver),
                 "%d.%d.%d",
                 ((UInt32 *)tlvData)[0],
                 ((UInt32 *)tlvData)[1],
                 ((UInt32 *)tlvData)[2]);
                break;
            case IWM_UCODE_TLV_FW_MEM_SEG:
                break;
            default:
                IWM_KERNLOG("Default case called, code is %d \n", (int)tlvType);
                break;
        }
    }
    return KERN_SUCCESS;
}


//used to load firmware onto the card
IOReturn AppleIntelIWM::allocFwMem(struct iwm_drv *driver)
{
    struct iwm_dma_info* fwMem = (struct iwm_dma_info*) IOMalloc(sizeof(struct iwm_dma_info));
    
    if(!fwMem) {
        return KERN_FAILURE;
    }
    
    if(allocContigDMA(driver, fwMem, IWM_FH_MEM_TB_MAX_LENGTH, 16, "Fw mem", kIODirectionOut) != KERN_SUCCESS) {
        return KERN_FAILURE;
    }
    
    driver->firmwareDMA = fwMem;
    
    return KERN_SUCCESS;
}

//Keep warm page used by card
IOReturn AppleIntelIWM::allocKwPage(struct iwm_drv *driver)
{
    struct iwm_dma_info* kwMem = (struct iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    
    if(!kwMem) {
        return KERN_FAILURE;
    }
    
    if(allocContigDMA(driver, kwMem, 4096, 4096, "KW page", kIODirectionInOut) != KERN_SUCCESS) {
        return KERN_FAILURE;
    }
    
    driver->keepWarmDMA = kwMem;
    
    return KERN_SUCCESS;
}

//Interrupt cause table
IOReturn AppleIntelIWM::allocICT(struct iwm_drv *driver)
{
    struct iwm_dma_info* ictMem = (struct iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    
    if(!ictMem) {
        return KERN_FAILURE;
    }
    
    if(allocContigDMA(driver, ictMem, IWM_ICT_SIZE, 1 << IWM_ICT_PADDR_SHIFT, "ICT table", kIODirectionIn) != KERN_SUCCESS) {
        return KERN_FAILURE;
    }

    driver->ictDMA = ictMem;
    driver->ictTable = (UInt32*) ictMem->vAddr;
    
    bzero(driver->ictTable, IWM_ICT_SIZE);
    
    /* just an API sanity check ... it is guaranteed to be aligned */
    if(ictMem->pAddr & (IWM_ICT_SIZE - 1)) {
        IWM_KERNLOG("ICT buffer not aligned!\n");
        return EINVAL;
    }
    
    return KERN_SUCCESS;
}

//Tx sched rings
IOReturn AppleIntelIWM::allocTxSchedRings(struct iwm_drv *driver)
{
    struct iwm_dma_info* txSchedMem = (struct iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    
    if(!txSchedMem) {
        return KERN_FAILURE;
    }
    
    int nItems = sizeof(driver->txq) / sizeof(struct iwm_tx_ring);
    
    //this should be 31 maybe...
    IWM_KERNLOG("There are %d items in TXQ\n", nItems);
    
    if(allocContigDMA(driver, txSchedMem, nItems * sizeof(struct iwm_agn_scd_bc_tbl), 1024, "Tx sched rings", kIODirectionInOut) != KERN_SUCCESS) {
        return KERN_FAILURE;
    }
    
    driver->schedDMA = txSchedMem;

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::initNvm(struct iwm_drv *driver)
{
    struct iwm_nvm_section nvmSections[IWM_NVM_MAX_NUM_SECTIONS];
    
    int i, ret, section;
    
    UInt32 sizeRead = 0;
    UInt8 *nvmBuffer, *temp;
    UInt16 len;
    
    memset(nvmSections, 0, sizeof(nvmSections));
    
    if(driver->config->nvm_hw_section_num >= IWM_NVM_MAX_NUM_SECTIONS) {
        return EINVAL;
    }
    
    /* load NVM values from nic */
    /* Read From FW NVM */
    IWM_KERNLOG("Read from NVM\n");
    
    nvmBuffer = (UInt8 *) IOMalloc(driver->config->eeprom_size);
    
    if(!nvmBuffer) {
        return ENOMEM;
    }
    
    bzero(nvmBuffer, sizeof(*nvmBuffer));
    
    for(section = 0; section < IWM_NVM_MAX_NUM_SECTIONS; section++) {
        ret = NVMReadSection(driver, section, nvmBuffer, &len, sizeRead);
        
        if(ret) {
            continue;
        }
        sizeRead += len;
        temp = (UInt8*) IOMalloc(len);
        
        if(!temp) {
            ret = ENOMEM;
            break;
        }
        memcpy(temp, nvmBuffer, len);
        
        nvmSections[section].data = temp;
        nvmSections[section].length = len;
    }
    
    if(!sizeRead) {
        IWM_KERNLOG("OTP is blank\n");
    }
    IOFree(nvmBuffer, sizeof(*nvmBuffer));
    
    driver->nvm_data = parseNVMSections(driver, nvmSections);
    if(!driver->nvm_data) {
        return EINVAL;
    }
    
    IWM_KERNLOG("nvm version = %x\n", driver->nvm_data->nvm_version);
    
    for(i = 0; i < IWM_NVM_MAX_NUM_SECTIONS; i++) {
        if(nvmSections[i].data != NULL) {
            IOFree(nvmSections[i].data, sizeof(*nvmSections[i].data));
        }
    }
    return 0;
}

IOReturn AppleIntelIWM::allocRxRing(struct iwm_drv *driver, struct iwm_rx_ring *ring)
{
    size_t descSize, size;
    int count, i, error = 0;

    struct iwm_dma_info* rxStatMem = (struct iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    struct iwm_dma_info* rxRingMem = (struct iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    
    if(!rxRingMem || !rxStatMem) {
        return KERN_FAILURE;
    }
    
    ring->cur = 0;
    
    count = IWM_RX_LEGACY_RING_COUNT;
    descSize = sizeof(UInt32);
    
    /* Allocate RX descriptors (256-byte aligned). */
    size = count * descSize;
    
    error = allocContigDMA(driver, rxRingMem, size, 256, "rx desc");
    if(error) {
        IWM_KERNLOG("Couldn't alloc DMA for rx desc\n");
        goto fail;
    }
    
    error = allocContigDMA(driver, rxStatMem, sizeof(struct iwm_rb_status), 16, "stat");
    if(error) {
        IWM_KERNLOG("Couldn't allocate DMA for stat\n");
        goto fail;
    }
    
    
    ring->freeDescDMA = rxRingMem;
    ring->freeDescDMA->bmd->prepare();
    ring->freeDescDMA->cmd->prepare();
    ring->desc = (void *) ring->freeDescDMA->vAddr;
    
    ring->statDMA = rxStatMem;
    ring->statDMA->bmd->prepare();
    ring->statDMA->cmd->prepare();
    ring->stat = (struct iwm_rb_status *) ring->statDMA->vAddr;

    //Allocate and map RX buffers
    for(i = 0; i < count; i++) {
        struct iwm_rx_data *data = &ring->data[i];
        data->m = NULL;
        
        error = rxAddBuf(driver, IWM_RBUF_SIZE, i);
        if(error != KERN_SUCCESS) {
            IWM_KERNLOG("Error adding buffer\n");
            goto fail;
        }
    }
    
    ring->freeDescDMA->cmd->writeBytes(0, ring->desc, size);
    
    return KERN_SUCCESS;
fail:
    freeRxRing(driver, ring);
    return error;
}

IOReturn AppleIntelIWM::resetRxRing(struct iwm_drv *driver, struct iwm_rx_ring *ring)
{
    //Reset ring state
    ring->cur = 0;
    
    /*
     * The hw rx ring index in shared memory must also be cleared,
     * otherwise the discrepancy can cause reprocessing chaos.
     */
    
    if(driver->rxq.stat) {
        memset(driver->rxq.stat, 0, sizeof(*driver->rxq.stat));
    }

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::freeRxRing(struct iwm_drv *driver, struct iwm_rx_ring *ring)
{
    int count, i;
    
    ring->freeDescDMA->bmd->complete();
    ring->freeDescDMA->cmd->complete();
    
    ring->statDMA->bmd->complete();
    ring->statDMA->cmd->complete();
    
    freeContigDMA(driver, ring->freeDescDMA);
    freeContigDMA(driver, ring->statDMA);
    
    count = IWM_RX_LEGACY_RING_COUNT;
    
    for(i = 0; i < count; i++) {
        struct iwm_rx_data *data = &ring->data[i];
        
        if(data->m != NULL) {
            //TODO: here
            mbuf_free(data->m);
            data->m = NULL;
        }
    }
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::allocTxRing(struct iwm_drv *driver, struct iwm_tx_ring *ring, int qid)
{
    
    ring->qid = qid;
    ring->queued = 0;
    ring->cur = 0;
    
    UInt64 pAddr;
    int error;
    int i = 0;
    
    iwm_dma_info *cmdMem;
    
    //Allocate TX desc (256 byte aligned)
    size_t size, maxSize;
    int nSegments = 0;
    
    size = IWM_TX_RING_COUNT * sizeof(struct iwm_tfd);
    
    iwm_dma_info *txDescMem = (iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    
    if(!txDescMem) {
        error = KERN_FAILURE;
        goto fail;
    }
    
    error = allocContigDMA(driver, txDescMem, size, 256, "Tx ring", kIODirectionOut);
    
    if(error) {
        IWM_KERNLOG("Couldn't allocate tx desc ring %d", qid);
        goto fail;
    }
    
    ring->descDMA = txDescMem;
    ring->desc = (struct iwm_tfd*) txDescMem->vAddr;
    
    /*
     * We only use rings 0 through 9 (4 EDCA + cmd) so there is no need
     * to allocate commands space for other rings.
     */
    if(qid > IWM_CMD_QUEUE) {
        return KERN_SUCCESS;
    }
    
    size = IWM_TX_RING_COUNT * sizeof(struct iwm_device_cmd);
    
    cmdMem = (iwm_dma_info*) IOMalloc(sizeof(iwm_dma_info));
    
    if(!cmdMem) {
        error = KERN_FAILURE;
        goto fail;
    }
    
    error = allocContigDMA(driver, cmdMem, size, 4, "Tx ring cmd space", kIODirectionOut);
    
    if(error) {
        IWM_KERNLOG("Couldn't allocate cmd DMA %d", qid);
        goto fail;
    }
    
    ring->cmdDMA = cmdMem;
    ring->cmd = (struct iwm_device_cmd*) cmdMem->vAddr;
    
    /* FW commands may require more mapped space than packets. */
    if(qid == IWM_CMD_QUEUE) {
        maxSize = IWM_RBUF_SIZE;
        nSegments = 1;
    } else {
        maxSize = 1728; //TODO: CHECH
        nSegments = IWM_MAX_SCATTER - 2;
    }
    
    pAddr = ring->cmdDMA->pAddr;
    for(i = 0; i < IWM_TX_RING_COUNT; i++) {
        struct iwm_tx_data *data = &ring->data[i];
        data->cmdPaddr = pAddr;
        data->scratchPaddr = pAddr + sizeof(struct iwm_cmd_header) + __offsetof(struct iwm_tx_cmd, scratch);
        pAddr += sizeof(struct iwm_device_cmd);
    }
    KASSERT(pAddr == ring->cmdDMA->pAddr + size, "Invalid physical address!\n");
    
    return KERN_SUCCESS;
    
fail:
    freeTxRing(driver, ring);
    return error;
}

IOReturn AppleIntelIWM::resetTxRing(struct iwm_drv *driver, struct iwm_tx_ring *ring)
{
    int i = 0;
    
    for(i = 0; i < IWM_TX_RING_COUNT; i++) {
        struct iwm_tx_data *data = &ring->data[i];
        
        if(data->m != NULL) {
            mbuf_freem(*data->m);
            data->m = NULL;
        }
    }
    
    //Clear TX descriptors
    memset(ring->desc, 0, ring->descDMA->pAddr);
    driver->qfullmask &= ~(1 << ring->qid);
    ring->queued = 0;
    ring->cur = 0;
    
    if(ring->qid == IWM_CMD_QUEUE && driver->cmdHoldNicAwake) {
        clearCmdInFlight(driver);
    }

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::freeTxRing(struct iwm_drv *driver, struct iwm_tx_ring *ring)
{
    int i = 0;
    
    freeContigDMA(driver, ring->descDMA);
    freeContigDMA(driver, ring->cmdDMA);
    
    for(i = 0; i < IWM_TX_RING_COUNT; i++) {
        struct iwm_tx_data *data = &ring->data[i];
        
        if(data->m != NULL) {
            mbuf_free(*data->m);
            data->m = NULL;
        }
    }

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::nicConfig(struct iwm_drv *driver)
{
    UInt8 radioConfigType, radioConfigStep, radioConfigDash;
    UInt32 regVal = 0;
    UInt32 phyConfig = getPhyConfig(driver);
    
    radioConfigType = (phyConfig & IWM_FW_PHY_CFG_RADIO_TYPE) >> IWM_FW_PHY_CFG_RADIO_TYPE_POS;
    radioConfigStep = (phyConfig & IWM_FW_PHY_CFG_RADIO_STEP) >> IWM_FW_PHY_CFG_RADIO_STEP_POS;
    radioConfigDash = (phyConfig & IWM_FW_PHY_CFG_RADIO_DASH) >> IWM_FW_PHY_CFG_RADIO_DASH_POS;
    
    /* SKU control */
    regVal |= IWM_CSR_HW_REV_STEP(driver->hwRev) << IWM_CSR_HW_IF_CONFIG_REG_POS_MAC_STEP;
    regVal |= IWM_CSR_HW_REV_DASH(driver->hwRev) << IWM_CSR_HW_IF_CONFIG_REG_POS_MAC_DASH;
    
    /* Radio Config */
    regVal |= radioConfigType << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_TYPE;
    regVal |= radioConfigStep << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_STEP;
    regVal |= radioConfigDash << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_DASH;
    
    iwmWrite32(driver, IWM_CSR_HW_IF_CONFIG_REG,
               IWM_CSR_HW_IF_CONFIG_REG_MSK_MAC_DASH |
               IWM_CSR_HW_IF_CONFIG_REG_MSK_MAC_STEP |
               IWM_CSR_HW_IF_CONFIG_REG_MSK_PHY_STEP |
               IWM_CSR_HW_IF_CONFIG_REG_MSK_PHY_DASH |
               IWM_CSR_HW_IF_CONFIG_REG_MSK_PHY_TYPE |
               IWM_CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI |
               IWM_CSR_HW_IF_CONFIG_REG_BIT_MAC_SI |
               regVal); //write config to config reg
    
    IWM_DLOG(IWM_DEBUG_RESET, "Radio type=0x%x-0x%x-0x%x\n", radioConfigType, radioConfigStep, radioConfigDash);
    /*
     * W/A : NIC is stuck in a reset state after Early PCIe power off
     * (PCIe power is lost before PERST# is asserted), causing ME FW
     * to lose ownership and not being able to obtain it back.
     */
    if (driver->config->device_family == IWM_DEVICE_FAMILY_7000) {
        setBitsMaskPRPH(driver, IWM_APMG_PS_CTRL_REG, IWM_APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS, ~IWM_APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS);
    }
    
    return KERN_SUCCESS;
}

UInt8 AppleIntelIWM::getValidRxAnt(struct iwm_drv *driver)
{
    return driver->nvm_data && driver->nvm_data->valid_rx_ant ? driver->fw_info.valid_rx_ant & driver->nvm_data->valid_rx_ant : driver->fw_info.valid_rx_ant;
}

UInt8 AppleIntelIWM::getValidTxAnt(struct iwm_drv *driver)
{
    return driver->nvm_data && driver->nvm_data->valid_tx_ant ? driver->fw_info.valid_tx_ant & driver->nvm_data->valid_tx_ant : driver->fw_info.valid_tx_ant;
}

IOReturn AppleIntelIWM::initNicRx(struct iwm_drv *driver)
{
    //We dont support MQRX rn
    
    /* Stop Rx DMA */
    pcieRxStop(driver);
    
    if(!nicLock(driver)) {
        return EBUSY;
    }
    
    /* reset and flush pointers */
    iwmWrite32(driver, IWM_FH_MEM_RCSR_CHNL0_RBDCB_WPTR, 0);
    iwmWrite32(driver, IWM_FH_MEM_RCSR_CHNL0_FLUSH_RB_REQ, 0);
    iwmWrite32(driver, IWM_FH_RSCSR_CHNL0_RDPTR, 0);
    iwmWrite32(driver, IWM_FH_RSCSR_CHNL0_RBDCB_WPTR_REG, 0);
    
    /* Set physical address of RX ring (256-byte aligned). */
    iwmWrite32(driver, IWM_FH_RSCSR_CHNL0_RBDCB_BASE_REG, driver->rxq.freeDescDMA->pAddr >> 8);
    
    /* Set physical address of RX status (16-byte aligned). */
    iwmWrite32(driver, IWM_FH_RSCSR_CHNL0_STTS_WPTR_REG, driver->rxq.statDMA->pAddr >> 4);

    /* Enable Rx DMA
     * XXX 5000 HW isn't supported by the iwm(4) driver.
     * IWM_FH_RCSR_CHNL0_RX_IGNORE_RXF_EMPTY is set because of HW bug in
     *      the credit mechanism in 5000 HW RX FIFO
     * Direct rx interrupts to hosts
     * Rx buffer size 4 or 8k or 12k
     * RB timeout 0x10
     * 256 RBDs
     */
    iwmWrite32(driver, IWM_FH_MEM_RCSR_CHNL0_CONFIG_REG,
               IWM_FH_RCSR_RX_CONFIG_CHNL_EN_ENABLE_VAL		|
               IWM_FH_RCSR_CHNL0_RX_IGNORE_RXF_EMPTY		|  /* HW bug */
               IWM_FH_RCSR_CHNL0_RX_CONFIG_IRQ_DEST_INT_HOST_VAL	|
               IWM_FH_RCSR_RX_CONFIG_REG_VAL_RB_SIZE_4K		|
               (IWM_RX_RB_TIMEOUT << IWM_FH_RCSR_RX_CONFIG_REG_IRQ_RBTH_POS) |
               IWM_RX_QUEUE_SIZE_LOG << IWM_FH_RCSR_RX_CONFIG_RBDCB_SIZE_POS);
    
    iwmWrite8(driver, IWM_CSR_INT_COALESCING, IWM_HOST_INT_TIMEOUT_DEF);
    
    /* W/A for interrupt coalescing bug in 7260 and 3160 */
    if(driver->config->host_interrupt_operation_mode) {
        iwmSetBits(driver, IWM_CSR_INT_COALESCING, IWM_HOST_INT_OPER_MODE);
    }
    
    nicUnlock(driver);
    
    iwmWrite32(driver, IWM_FH_RSCSR_CHNL0_WPTR, 8);
    
    IWM_KERNLOG("Wrote statDMA and freeDescDMA pAddrs!\n");
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::initNicTx(struct iwm_drv *driver)
{
    int qid;
    
    if(!nicLock(driver)) {
        return EBUSY;
    }
    
    /* Deactivate TX scheduler. */
    writePRPH(driver, IWM_SCD_TXFACT, 0);
    
    /* Set physical address of "keep warm" page (16-byte aligned). */
    iwmWrite32(driver, IWM_FH_KW_MEM_ADDR_REG, driver->keepWarmDMA->pAddr >> 4);
    
    /* Initialize TX rings. */
    for(qid = 0; qid < sizeof(driver->txq) / sizeof(struct iwm_tx_ring); qid++) {
        struct iwm_tx_ring *txq = &driver->txq[qid];
        
        /* Set physical address of TX ring (256-byte aligned). */
        iwmWrite32(driver, IWM_FH_MEM_CBBC_QUEUE(qid), txq->descDMA->pAddr >> 8);
    }
    
    setBitsPRPH(driver, IWM_SCD_GP_CTRL, IWM_SCD_GP_CTRL_AUTO_ACTIVE_MODE | IWM_SCD_GP_CTRL_ENABLE_31_QUEUES);
    
    nicUnlock(driver);

    IWM_KERNLOG("Tx ring descriptors enabled!\n");
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::pcieRxStop(struct iwm_drv *driver)
{
    int ret = 0;
    
    if(nicLock(driver)) {
        iwmWrite32(driver, IWM_FH_MEM_RCSR_CHNL0_CONFIG_REG, 0);
        ret = iwmPollBit(driver, IWM_FH_MEM_RSSR_RX_STATUS_REG, IWM_FH_RSSR_CHNL0_RX_STATUS_CHNL_IDLE, IWM_FH_RSSR_CHNL0_RX_STATUS_CHNL_IDLE, 1000);
        nicUnlock(driver);
    }
    return ret;
}
 
UInt32 AppleIntelIWM::getPhyConfig(struct iwm_drv *driver)
{
    UInt32 phyConfig = ~(IWM_FW_PHY_CFG_TX_CHAIN | IWM_FW_PHY_CFG_RX_CHAIN);
    
    UInt32 validTxAnt = getValidTxAnt(driver);
    UInt32 validRxAnt = getValidRxAnt(driver);
    
    phyConfig |= validTxAnt << IWM_FW_PHY_CFG_TX_CHAIN_POS | validRxAnt << IWM_FW_PHY_CFG_RX_CHAIN_POS;
    
    return driver->fw_info.phy_config & phyConfig;
}

IOReturn AppleIntelIWM::rxAddBuf(struct iwm_drv *driver, int size, int idx)
{
    struct iwm_rx_ring *ring = &driver->rxq;
    struct iwm_rx_data *data = &ring->data[idx];
    
    mbuf_t packet = allocatePacket(size);
    
    if(!packet) {
        IWM_KERNLOG("Alloc packet failed!\n");
        return KERN_FAILURE;
    }
    
    UInt32 pAddr = mBufDmaMap(driver, data->m);
    
    data->m = packet;
    data->pAddr = pAddr;
    
    ((UInt32*)ring->desc)[idx] = pAddr >> 8;
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::NVMReadSection(struct iwm_drv *driver, UInt16 section, UInt8 *data, UInt16 *len, UInt32 sizeRead)
{
    UInt16 segLen, length, offset = 0;
    int ret;
    
    /* Set nvm section read length */
    length = IWM_NVM_DEFAULT_CHUNK_SIZE;
    
    segLen = length;
    
    /* Read the NVM until exhausted (reading less than requested) */
    while(segLen == length) {
        /* Check no memory assumptions fail and cause an overflow */
        if((sizeRead + offset + length) > driver->config->eeprom_size) {
            IWM_KERNLOG("EEPROM size is too small for NVM!\n");
            return ENOBUFS;
        }
        
        ret = NVMReadChunk(driver, section, offset, length, data, &segLen);
        if(ret) {
            IWM_KERNLOG("Cannot read NVM from section %d offset %d, length %d\n", section, offset, length);
            return ret;
        }
        offset += segLen;
    }
    
    IWM_KERNLOG("NVM section %d read completed\n", section);
    *len = section;
    return 0;
}

IOReturn AppleIntelIWM::NVMReadChunk(struct iwm_drv *driver, UInt16 section, UInt16 offset, UInt16 length, UInt8 *data, UInt16* len)
{
    struct iwm_nvm_access_cmd nvmAccessCmd = {
        .offset = offset,
        .length = length,
        .type = section,
        .op_code = IWM_NVM_READ_OPCODE,
    };
    
    struct iwm_nvm_access_resp *nvmResp;
    struct iwm_rx_packet *pkt;
    struct iwm_host_cmd cmd = {
        .id = IWM_NVM_ACCESS_CMD,
        .flags = IWM_CMD_WANT_SKB | IWM_CMD_SEND_IN_RFKILL,
        .data = { &nvmAccessCmd, },
    };
    
    int ret, bytesRead, offsetRead;
    UInt8 *respData;
    
    cmd.len[0] = sizeof(struct iwm_nvm_access_cmd);
    
    ret = sendCmd(driver, &cmd);
    if(ret) {
        IWM_KERNLOG("Could not send NVM_ACCESS command (error=%d)\n", ret);
        return ret;
    }
    
    pkt = (struct iwm_rx_packet *) cmd.resp_pkt;
    
    /* Extract NVM response */
    nvmResp = (struct iwm_nvm_access_resp *)pkt->data;
    ret = nvmResp->status;
    bytesRead = nvmResp->length;
    offsetRead = nvmResp->offset;
    respData = nvmResp->data;
    if(ret) {
        if((offset != 0) && (ret == IWM_READ_NVM_CHUNK_NOT_VALID_ADDRESS)) {
            /*
             * meaning of NOT_VALID_ADDRESS:
             * driver try to read chunk from address that is
             * multiple of 2K and got an error since addr is empty.
             * meaning of (offset != 0): driver already
             * read valid data from another chunk so this case
             * is not an error.
             */
            IWM_KERNLOG("NVM access command failed on offset 0x%x since that section size is multiple 2K\n", offset);
            *len = 0;
            ret = 0;
        } else {
            IWM_KERNLOG("NVM access command failed with status %d\n", ret);
            ret = EIO;
        }
        goto exit;
    }
    
    if(offsetRead != offset) {
        IWM_KERNLOG("NVM ACCESS response with invalid offset %d\n", offsetRead);
        ret = EINVAL;
        goto exit;
    }
    
    if(bytesRead > length) {
        IWM_KERNLOG("NVM ACCESS response with too much data (%d bytes requested, %d bytes received)\n", length, bytesRead);
        ret = EINVAL;
        goto exit;
    }
    
    /* Write data to NVM */
    memcpy(data + offset, respData, bytesRead);
    *len = bytesRead;
exit:
    freeResp(driver, &cmd);
    return ret;
}

struct iwm_nvm_data* AppleIntelIWM::parseNVMSections(struct iwm_drv *driver, struct iwm_nvm_section *sections)
{
    UInt16 *hw, *sw, *calib, *regulatory, *macOverride, *phySku;
    
    /* Checking for required sections */
    if(driver->config->device_family == IWM_DEVICE_FAMILY_7000) {
        if(!sections[IWM_NVM_SECTION_TYPE_SW].data || !sections[driver->config->nvm_hw_section_num].data) {
            IWM_KERNLOG("Can't parse empty OTP/NVM sections\n");
            return NULL;
        }
    } else if(driver->config->device_family >= IWM_DEVICE_FAMILY_8000) {
        //TODO: ignore for now
    } else {
        panic("Unknown device family %d!\n", driver->config->device_family);
    }
    
    hw = (UInt16 *) sections[driver->config->nvm_hw_section_num].data;
    sw = (UInt16 *) sections[IWM_NVM_SECTION_TYPE_SW].data;
    calib = (UInt16 *) sections[IWM_NVM_SECTION_TYPE_CALIBRATION].data;
    regulatory = driver->config->nvm_type == IWM_NVM_SDP ? (UInt16 *)sections[IWM_NVM_SECTION_TYPE_REGULATORY_SDP].data : (UInt16 *)sections[IWM_NVM_SECTION_TYPE_REGULATORY].data;
    macOverride = (UInt16 *) sections[IWM_NVM_SECTION_TYPE_MAC_OVERRIDE].data;
    phySku = (UInt16 *)sections[IWM_NVM_SECTION_TYPE_PHY_SKU].data;
    
    return parseNVMData(driver, hw, sw, calib, macOverride, phySku, regulatory);
}

struct iwm_nvm_data* AppleIntelIWM::parseNVMData(struct iwm_drv *driver, UInt16 *nvmHw, UInt16 *nvmSw, UInt16 *nvmCalib, UInt16 *macOverride, UInt16 *phySku, UInt16 *regulatory)
{
    struct iwm_nvm_data *data;
    UInt32 sku, radioConfig;
    UInt16 larConfig;
    
    if(driver->config->device_family < IWM_DEVICE_FAMILY_8000) {
        data = (struct iwm_nvm_data*) IOMalloc(sizeof(struct iwm_nvm_data) + sizeof(UInt16) * IWM_NUM_CHANNELS);
    } else {
        
    }
    
    if(!data) {
        return NULL;
    }
    
    bzero(data, sizeof(*data));
    
    //only for 7000 series
    data->nvm_version = getNvmVersion(driver, nvmSw);
    radioConfig = getRadioConfig(driver, nvmSw);
    setRadioCfg(driver, data, radioConfig);
    sku = getSku(driver, nvmSw);
    
    data->sku_cap_band_24GHz_enable = sku & IWM_NVM_SKU_CAP_BAND_24GHZ;
    data->sku_cap_band_52GHz_enable = sku & IWM_NVM_SKU_CAP_BAND_52GHZ;
    data->sku_cap_11n_enable = 0;
    
    data->n_hw_addrs = getNHWAddr(driver, nvmSw);
    
    /* If no valid mac address was found - bail out */
    if(setHwAddress(driver, data, nvmHw, macOverride)) {
        IOFree(data, sizeof(*data));
        return NULL;
    }
    
    if(driver->config->device_family == IWM_DEVICE_FAMILY_7000) {
        int nvmTypeSdp = driver->config->nvm_type == IWM_NVM_SDP;
        memcpy(data->nvm_ch_flags, nvmTypeSdp ? &regulatory[0] : &nvmSw[IWM_NVM_CHANNELS], IWM_NUM_CHANNELS * sizeof(UInt16));
    }
    return data;
}

IOReturn AppleIntelIWM::pcieFwAlive(struct iwm_drv *driver, UInt32 scdBaseAddr)
{
    int error, channel;
    
    int clearDwords = (IWM_SCD_TRANS_TBL_MEM_UPPER_BOUND - IWM_SCD_CONTEXT_MEM_LOWER_BOUND) / sizeof(UInt32);
    
    if(!nicLock(driver)) {
        return EBUSY;
    }
    
    //ictReset(driver);
    
    driver->scdBase = readPRPH(driver, IWM_SCD_SRAM_BASE_ADDR);
    if(scdBaseAddr != 0 && scdBaseAddr != driver->scdBase) {
        IWM_KERNLOG("sched addr mismatch: alive: 0x%x prph: 0x%x\n", driver->scdBase, scdBaseAddr);
    }
    
    nicUnlock(driver);
    
    /* reset context data, TX status and translation data */
    error = writeMem(driver, driver->scdBase + IWM_SCD_CONTEXT_MEM_LOWER_BOUND, NULL, clearDwords);
    
    if(error) {
        return EBUSY;
    }
    
    if(!nicLock(driver)) {
        return EBUSY;
    }
    
    /* Set physical address of TX scheduler rings (1KB aligned). */
    writePRPH(driver, IWM_SCD_DRAM_BASE_ADDR, driver->schedDMA->pAddr >> 10);
    
    writePRPH(driver, IWM_SCD_CHAINEXT_EN, 0);
    
    nicUnlock(driver);

    /* enable command channel */
    error = enableTxq(driver, 0 /* unused */, IWM_CMD_QUEUE, 7);
    if(error) {
        return error;
    }
    
    if(!nicLock(driver)) {
        return EBUSY;
    }
    
    writePRPH(driver, IWM_SCD_TXFACT, 0xFF);
    
    /* Enable DMA channels. */
    for(channel = 0; channel < IWM_FH_TCSR_CHNL_NUM; channel++) {
        iwmWrite32(driver, IWM_FH_TCSR_CHNL_TX_CONFIG_REG(channel), IWM_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE | IWM_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_ENABLE);
    }
    
    iwmSetBits(driver, IWM_FH_TX_CHICKEN_BITS_REG, IWM_FH_TX_CHICKEN_BITS_SCD_AUTO_RETRY_EN);
    
    nicUnlock(driver);
    
    /* Enable L1-Active */
    if(driver->config->device_family < IWM_DEVICE_FAMILY_8000) {
        clearBitsPRPH(driver, IWM_APMG_PCIDEV_STT_REG, IWM_APMG_PCIDEV_STT_VAL_L1_ACT_DIS);
    }
    
    return error;
}

IOReturn AppleIntelIWM::enableTxq(struct iwm_drv *driver, int staId, int qid, int fifo)
{
    int qmsk;
    
    qmsk = 1 << qid;
    
    if(!nicLock(driver)) {
        IWM_KERNLOG("Can't enable Txq!\n");
        return EBUSY;
    }
    
    iwmWrite32(driver, IWM_HBUS_TARG_WRPTR, qid << 8 | 0);
    
    if(qid == IWM_CMD_QUEUE) {
        /* Disable the scheduler. */
        writePRPH(driver, IWM_SCD_EN_CTRL, 0);
        
        /* Stop the TX queue prior to configuration. */
        writePRPH(driver, IWM_SCD_QUEUE_STATUS_BITS(qid), (0 << IWM_SCD_QUEUE_STTS_REG_POS_ACTIVE) | (1 << IWM_SCD_QUEUE_STTS_REG_POS_SCD_ACT_EN));
        
        nicUnlock(driver);
    
        /* Disable aggregations for this queue. */
        clearBitsPRPH(driver, IWM_SCD_AGGR_SEL, qmsk);
        
        if(!nicLock(driver)) {
            IWM_KERNLOG("Cannot enable Txq %d\n", qid);
            return EBUSY;
        }
        
        writePRPH(driver, IWM_SCD_QUEUE_RDPTR(qid), 0);
        nicUnlock(driver);
        
        writeMem32(driver, driver->scdBase + IWM_SCD_CONTEXT_QUEUE_OFFSET(qid), 0);
        
        /* Set scheduler window size and frame limit. */
        writeMem32(driver, driver->scdBase + IWM_SCD_CONTEXT_QUEUE_OFFSET(qid) + sizeof(UInt32), ((IWM_FRAME_LIMIT << IWM_SCD_QUEUE_CTX_REG2_WIN_SIZE_POS) & ((IWM_FRAME_LIMIT << IWM_SCD_QUEUE_CTX_REG2_FRAME_LIMIT_POS) & IWM_SCD_QUEUE_CTX_REG2_FRAME_LIMIT_MSK)));
                   
        if(!nicLock(driver)) {
            IWM_KERNLOG("Cannot enable Txq %d\n", qid);
            return EBUSY;
        }
                   
        writePRPH(driver, IWM_SCD_QUEUE_STATUS_BITS(qid), (1 << IWM_SCD_QUEUE_STTS_REG_POS_ACTIVE) | (fifo << IWM_SCD_QUEUE_STTS_REG_POS_TXF) | (1 << IWM_SCD_QUEUE_STTS_REG_POS_WSL) | IWM_SCD_QUEUE_STTS_REG_MSK);
                   
        /* Enable the scheduler for this queue. */
        writePRPH(driver, IWM_SCD_EN_CTRL, qmsk);
    } else {
        struct iwm_scd_txq_cfg_cmd cmd;
        int error;
        
        nicUnlock(driver);
        
        memset(&cmd, 0, sizeof(cmd));
        
        cmd.scd_queue = qid;
        cmd.enable = 1;
        cmd.sta_id = staId;
        cmd.tx_fifo = fifo;
        cmd.aggregate = 0;
        cmd.window = IWM_FRAME_LIMIT;
        
        error = sendCmdPdu(driver, IWM_SCD_QUEUE_CFG, IWM_CMD_SYNC, sizeof(cmd), &cmd);
        if(error) {
            IWM_KERNLOG("Cannot enable txq %d\n", qid);
            return error;
        }
        
        if(!nicLock(driver)) {
            return EBUSY;
        }
    }
    
    nicUnlock(driver);
    
    IWM_KERNLOG("enabled txq %d FIFO %d\n", qid, fifo);
    
    return 0;
}

UInt32 AppleIntelIWM::mBufDmaMap(struct iwm_drv *driver, mbuf_t m)
{
    IOPhysicalSegment rxSeg;
    driver->mBufCursor->getPhysicalSegments(m, &rxSeg, 1);
    
    return rxSeg.location;
}










