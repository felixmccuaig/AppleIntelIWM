//
//  Utils.cpp
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 27/12/2019.
//  Copyright © 2019 Felix McCuaig. All rights reserved.
//

#include "AppleIntelIWM.h"

IOReturn AppleIntelIWM::allocContigDMA(struct iwm_drv *driver, struct iwm_dma_info *result, size_t size, size_t alignment, char* desc, IOOptionBits direction)
{
    IOOptionBits options = kIOMemoryPhysicallyContiguous | kIOMapInhibitCache | direction;
    IOBufferMemoryDescriptor* bmd = new IOBufferMemoryDescriptor();
    bmd->initWithPhysicalMask(kernel_task, options, size, alignment, 0x00000000FFFFFFFFULL); //must be in 32 bit space!
    
    bzero(bmd->getBytesNoCopy(), size);
    
    IODMACommand* cmd = IODMACommand::withSpecification(kIODMACommandOutputHost32, 32, 0, IODMACommand::kMapped, 0, 1);
    
    if(!cmd) {
        IWM_KERNLOG("No mem!\n");
        return KERN_FAILURE;
    }
    
    cmd->setMemoryDescriptor(bmd);
    
    IODMACommand::Segment32 seg;
    UInt64 ofs = 0;
    UInt32 numSegs = 1;
    
    //Lets just make one segement
    if(cmd->gen32IOVMSegments(&ofs, &seg, &numSegs) != KERN_SUCCESS) {
        IWM_KERNLOG("failed to gen IOVM segs\n");
        
        //Cleanup routine
        cmd->release();
        cmd = NULL;
        
        bmd->release();
        bmd = NULL;
        
        IOFree(result, sizeof(*result));
        
        return KERN_FAILURE;
    }
    
    if(!result) {
        IWM_KERNLOG("Pointer to DMA info does't exist\n");
        return KERN_FAILURE;
    }
    
    result->bmd = bmd;
    result->cmd = cmd;
    result->pAddr = seg.fIOVMAddr;
    result->vAddr = bmd->getBytesNoCopy();
    result->size = seg.fLength;
    
    IWM_KERNLOG("Creating DMA segment for %s with address: 0x%X and length 0x%X\n", desc, result->pAddr, result->size);
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::freeContigDMA(struct iwm_drv *driver, struct iwm_dma_info *dma)
{
    dma->cmd->complete();
    dma->cmd->release();
    dma->cmd = NULL;
    
    dma->bmd->complete();
    dma->bmd->release();
    dma->bmd = NULL;
    
    dma->pAddr = NULL;
    dma->size = NULL;
    
    IOFree(dma, sizeof(*dma));

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::sendCmdStatus(struct iwm_drv *driver, struct iwm_host_cmd *cmd, UInt32 *status)
{
    struct iwm_rx_packet *pkt;
    struct iwm_cmd_response *resp;
    int error = 0, respLen;
    
    KASSERT((cmd->flags & IWM_CMD_WANT_SKB) == 0, "Invalid Command!\n");
    
    cmd->flags |= IWM_CMD_SYNC | IWM_CMD_WANT_SKB;

    error = sendCmd(driver, cmd);
    
    if(error) {
        return error;
    }
    
    pkt = cmd->resp_pkt;
    
    /* Can happen if RFKILL is asserted */
    if(!pkt) {
        error = 0;
        goto outFreeResp;
    }
    
    if(pkt->hdr.flags & IWM_CMD_FAILED_MSK) {
        error = EIO;
        goto outFreeResp;
    }
    
    respLen = rxPacketPayloadLen(pkt);
    if(respLen != sizeof(*resp)) {
        error = EIO;
        goto outFreeResp;
    }
    
    resp = (struct iwm_cmd_response*) pkt->data;
    *status = resp->status;
outFreeResp:
    freeResp(driver, cmd);
    return error;
}

IOReturn AppleIntelIWM::sendCmdPduStatus(struct iwm_drv *driver, UInt32 id, UInt16 len, const void *data, UInt32 *status)
{
    struct iwm_host_cmd cmd = {
        .id = id,
        .len = { len, },
        .data = { data, },
    };

    return 0; //TODO
}

IOReturn AppleIntelIWM::sendCmdPdu(struct iwm_drv *driver, UInt32 id, UInt32 flags, UInt16 len, const void *data)
{
    struct iwm_host_cmd cmd = {
        .id = id,
        .len = { len, },
        .data = { data, },
        .flags = flags,
    };
    
    return sendCmd(driver, &cmd);
}

IOReturn AppleIntelIWM::sendCmd(struct iwm_drv *driver, struct iwm_host_cmd *hCmd)
{
    struct iwm_tx_ring *ring = &driver->txq[IWM_CMD_QUEUE];
    struct iwm_tfd *desc;
    struct iwm_tx_data *txData = NULL;
    struct iwm_device_cmd *cmd;
    mbuf_t m;
    
    UInt32 addrLo;
    int i, payLen, offset;
    int code;
    int async, wantResp;
    int groupId;
    int nSegs;
    size_t hdrLen, dataSize;
    UInt8* data;
    int error = 0;
    UInt64 pAddr;
    
    code = hCmd->id;
    async = hCmd->flags & IWM_CMD_ASYNC;
    wantResp = hCmd->flags & IWM_CMD_WANT_SKB;
    data = NULL;
    
    for(i = 0; i < sizeof(hCmd->len) / sizeof(UInt16); i++) {
        payLen += hCmd->len[i];
    }
    
    /* if the command wants an answer, busy sc_cmd_resp */
    if(wantResp) {
        KASSERT(!async, "Invalid Async param in sendCMD!\n")
        
        while(driver->wantResp != -1) {
            IOLockSleep(driver->pciLock, &driver->wantResp, THREAD_INTERRUPTIBLE);
        }
        
        driver->wantResp = ring->qid << 16 | ring->cur;
        IWM_KERNLOG("Want resp is %x\n", driver->wantResp);
    }
    
    /*
     * Is the hardware still available?  (after e.g. above wait).
     */
    if(driver->flags & IWM_FLAG_STOPPED) {
        error = ENXIO;
        goto out;
    }
    
    desc = &ring->desc[ring->cur];
    txData = &ring->data[ring->cur];
    
    groupId = cmdGroupId(code);
    
    if(groupId != 0) {
        hdrLen = sizeof(cmd->hdr_wide);
        dataSize = sizeof(cmd->data_wide);
    } else {
        hdrLen = sizeof(cmd->hdr);
        dataSize = sizeof(cmd->data);
    }
    
    if(payLen > dataSize) {
        IWM_KERNLOG("Large command payload len\n");
        
        //Command is too large
        size_t totLen = hdrLen + payLen;
        
        if(payLen > IWM_MAX_CMD_PAYLOAD_SIZE) {
            IWM_KERNLOG("Firmware command too long (%zd bytes)\n", totLen);
            error = EINVAL;
            goto out;
        }
        
        mbuf_allocpacket(MBUF_DONTWAIT, IWM_RBUF_SIZE, 0, &m);
        
        if(!m) {
            error = ENOBUFS;
            goto out;
        }
        
        txData->m = &m;
        cmd = (struct iwm_device_cmd *) mbuf_data(m);
        pAddr = ring->descDMA->pAddr;
    } else {
        cmd = &ring->cmd[ring->cur];
        pAddr = txData->cmdPaddr;
    }
    
    if(groupId != 0) {
        cmd->hdr_wide.opcode = cmdOpcode(code);
        cmd->hdr_wide.group_id = groupId;
        cmd->hdr_wide.qid = ring->qid;
        cmd->hdr_wide.idx = ring->cur;
        cmd->hdr_wide.length = payLen;
        cmd->hdr_wide.version = cmdVersion(code);
        data = cmd->data_wide;
    } else {
        cmd->hdr.code = cmdOpcode(code);
        cmd->hdr.flags = 0;
        cmd->hdr.qid = ring->qid;
        cmd->hdr.idx = ring->cur;
        data = cmd->data;
    }
    
    for(i = 0, offset = 0; i < sizeof(hCmd->data) / sizeof(void *); i++) {
        if(hCmd->len[i] == 0) {
            continue;
        }
        
        memcpy(data + offset, hCmd->data[i], hCmd->len[i]);
        offset += hCmd->len[i];
    }
    KASSERT(offset == payLen, "Off %d != paylen %d\n", offset, payLen);
    
    /* lo field is not aligned */
    addrLo = (UInt32) pAddr;
    memcpy(&desc->tbs[0].lo, &addrLo, sizeof(UInt32));
    desc->tbs[0].hi_n_len = getDmaUpperAddr(pAddr) | ((hdrLen + payLen) << 4);
    desc->num_tbs = 1;
    
    IWM_KERNLOG("Sending command 0x%x size=%lu %s\n", code, (unsigned long) (hCmd->len[0] + hCmd->len[1] + hdrLen), async ? " (async)" : "");
    
    error = setCmdInFlight(driver);
    if (error) {
        goto out;
    }
    ring->queued++;
    
    IWM_KERNLOG("sending command 0x%x qid %d, idx %d\n", code, ring->qid, ring->cur);
    
    //Kick command ring
    ring->cur = (ring->cur + 1) % IWM_TX_RING_COUNT;
    iwmWrite32(driver, IWM_HBUS_TARG_WRPTR, ring->qid << 8 | ring->cur);

    if(!async) {
        /* m..m-mmyy-mmyyyy-mym-ym m-my generation */
        int generation = driver->generation;
        error = IOLockSleep(driver->pciLock, desc, THREAD_INTERRUPTIBLE);
        if(error == 0) {
            /* if hardware is no longer up, return error */
            if(generation != driver->generation) {
                error = ENXIO;
            } else {
                hCmd->resp_pkt = (struct iwm_rx_packet *)driver->cmdResp;
            }
        }
    
    }
    
out:
    if(wantResp && error) {
        freeResp(driver, hCmd);
    }

    return error;
}

void AppleIntelIWM::freeResp(struct iwm_drv *driver, struct iwm_host_cmd *hCmd)
{
    KASSERT(driver->wantResp != -1, "Already freed!\n");
    KASSERT((hCmd->flags & (IWM_CMD_WANT_SKB | IWM_CMD_SYNC)) == (IWM_CMD_WANT_SKB | IWM_CMD_SYNC), "Invalid flags!\n");
    driver->wantResp = -1;
    IOLockWakeup(driver->pciLock, &driver->wantResp, false); //TODO: true or false idk
}


/*
 * These functions retrieve specific information from the id field in
 * the iwm_host_cmd struct which contains the command id, the group id,
 * and the version of the command and vice versa.
 */
UInt8 AppleIntelIWM::cmdGroupId(UInt32 cmdId)
{
    return ((cmdId & 0xFF00) >> 8);
}

UInt8 AppleIntelIWM::cmdOpcode(UInt32 cmdId)
{
    return cmdId & 0xFF;
}

UInt8 AppleIntelIWM::cmdVersion(UInt32 cmdId)
{
    return ((cmdId & 0xFF0000) >> 16);
}

UInt32 AppleIntelIWM::cmdId(UInt8 opCode, UInt8 groupId, UInt8 version)
{
    return opCode + (groupId << 8) + (version << 16);
}

UInt32 AppleIntelIWM::rxPacketLen(const struct iwm_rx_packet *pkt)
{
    return pkt->len_n_flags & IWM_FH_RSCSR_FRAME_SIZE_MSK;
}

UInt32 AppleIntelIWM::rxPacketPayloadLen(const struct iwm_rx_packet *pkt)
{
    return rxPacketLen(pkt) - sizeof(pkt->hdr);
}

UInt16 AppleIntelIWM::getNvmVersion(const struct iwm_drv *driver, UInt16 *nvmSw)
{   //7000 series only
    return *(nvmSw + IWM_NVM_VERSION);
}

UInt16 AppleIntelIWM::getRadioConfig(const struct iwm_drv *driver, UInt16 *nvmSw)
{   //7000 series only
    return *(nvmSw + IWM_RADIO_CFG);
}

UInt16 AppleIntelIWM::getNHWAddr(const struct iwm_drv *driver, UInt16 *nvmSw)
{   //7000 series only
    return *(nvmSw + IWM_N_HW_ADDRS);
}

UInt16 AppleIntelIWM::getSku(const struct iwm_drv *driver, UInt16 *nvmSw)
{
    return *(nvmSw + IWM_SKU);
}

void AppleIntelIWM::setRadioCfg(const struct iwm_drv *driver, struct iwm_nvm_data *data, UInt32 radioConfig)
{
    data->radio_cfg_type = IWM_NVM_RF_CFG_TYPE_MSK(radioConfig);
    data->radio_cfg_step = IWM_NVM_RF_CFG_STEP_MSK(radioConfig);
    data->radio_cfg_dash = IWM_NVM_RF_CFG_DASH_MSK(radioConfig);
    data->radio_cfg_pnum = IWM_NVM_RF_CFG_PNUM_MSK(radioConfig);
    return;
}

IOReturn AppleIntelIWM::setHwAddress(struct iwm_drv *driver, struct iwm_nvm_data *data, UInt16 *nvmHw, UInt16 *macOverride)
{   //7000 series only
    
    UInt8 *hwAddr = (UInt8 *)(nvmHw + IWM_HW_ADDR);
    /* The byte order is little endian 16 bit, meaning 214365 */
    data->hw_addr[0] = hwAddr[0];
    data->hw_addr[1] = hwAddr[1];
    data->hw_addr[2] = hwAddr[2];
    data->hw_addr[3] = hwAddr[3];
    data->hw_addr[4] = hwAddr[4];
    data->hw_addr[5] = hwAddr[5];
    
    IWM_KERNLOG("Loaded hw addr, not checking if its valid or not\n");
    return KERN_SUCCESS;
}













