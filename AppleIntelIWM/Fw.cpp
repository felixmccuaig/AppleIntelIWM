//
//  Fw.cpp
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 8/01/2020.
//  Copyright © 2020 Felix McCuaig. All rights reserved.
//

#include "AppleIntelIWM.h"


void AppleIntelIWM::freeFwPaging(struct iwm_drv *driver)
{
    int i;
    
    if(driver->fw_paging_db[0].fwPagingBlock->vAddr == NULL) {
        return;
    }
    
    for(i = 0; i < IWM_NUM_OF_FW_PAGING_BLOCKS; i++) {
        freeContigDMA(driver, driver->fw_paging_db[i].fwPagingBlock);
    }
    
    memset(driver->fw_paging_db, 0, sizeof(driver->fw_paging_db));
}

IOReturn AppleIntelIWM::fillPagingMem(struct iwm_drv *driver, const struct iwm_fw_img *image)
{
    int secIdx, idx;
    UInt32 offset = 0;
    
    /*
     * find where is the paging image start point:
     * if CPU2 exist and it's in paging format, then the image looks like:
     * CPU1 sections (2 or more)
     * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between CPU1 to CPU2
     * CPU2 sections (not paged)
     * PAGING_SEPARATOR_SECTION delimiter - separate between CPU2
     * non paged to CPU2 paging sec
     * CPU2 paging CSS
     * CPU2 paging image (including instruction and data)
     */
    for(secIdx = 0; secIdx < IWM_UCODE_SECTION_MAX; secIdx++) {
        if(image->sec[secIdx].offset == IWM_PAGING_SEPARATOR_SECTION) {
            secIdx++;
            break;
        }
    }
    
    /*
     * If paging is enabled there should be at least 2 more sections left
     * (one for CSS and one for Paging data)
     */
    if(secIdx >= (sizeof(image->sec) / sizeof(struct iwm_fw_desc)) - 1) {
        IWM_KERNLOG("Missing either CSS and / or paging sectrions!\n");
        freeFwPaging(driver);
        return EINVAL;
    }
    
    /* copy the CSS block to the dram */
    IWM_KERNLOG("Paging: load paging CSS to FW, sec = %d\n", secIdx);
    
    memcpy(driver->fw_paging_db[0].fwPagingBlock->vAddr, image->sec[secIdx].data, driver->fw_paging_db[0].fw_paging_size);
    
    IWM_KERNLOG("Paging: copied %d CSS bytes to first block\n", driver->fw_paging_db[0].fw_paging_size);
    
    secIdx++;
    
    /*
     * copy the paging blocks to the dram
     * loop index start from 1 since that CSS block already copied to dram
     * and CSS index is 0.
     * loop stop at num_of_paging_blk since that last block is not full.
     */
    for(idx = 1; idx < driver->num_of_paging_blk; idx++) {
        memcpy(driver->fw_paging_db[idx].fwPagingBlock->vAddr, (const char*)image->sec[secIdx].data + offset, driver->fw_paging_db[idx].fw_paging_size);
        
        IWM_KERNLOG("Paging: copied %d paging bytes to block %d\n", driver->fw_paging_db[idx].fw_paging_size, idx);
        
        offset += driver->fw_paging_db[idx].fw_paging_size;
    }
    
    /* copy the last paging block */
    if(driver->num_of_pages_in_last_blk) {
        memcpy(driver->fw_paging_db[idx].fwPagingBlock->vAddr, (const char *)image->sec[secIdx].data + offset, IWM_FW_PAGING_SIZE * driver->num_of_pages_in_last_blk);
    
    
        IWM_KERNLOG("Paging: copied %d pages in the last block %d\n", driver->num_of_pages_in_last_blk, idx);
    }
    
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::allocFwPagingMem(struct iwm_drv *driver, const struct iwm_fw_img *image)
{
    int blkIdx = 0;
    int error, numOfPages;
    
    if(driver->fw_paging_db[0].fwPagingBlock->vAddr != NULL) {
        int i;
        /* Device got reset, and we setup firmware paging again */
        for(i = 0; i < driver->num_of_paging_blk + 1; i++) {
            //bus dma map sync here //TODO
        }
        return 0;
    }
    
    KASSERT((1 << IWM_BLOCK_2_EXP_SIZE) == IWM_PAGING_BLOCK_SIZE, "IWM_BLOCK_2_EXP_SIZE must be power of 2 of IWM_PAGING_BLOCK_SIZE");
    
    numOfPages = image->paging_mem_size / IWM_FW_PAGING_SIZE;
    driver->num_of_paging_blk = ((numOfPages - 1) / IWM_NUM_OF_PAGE_PER_GROUP) + 1;
    
    driver->num_of_pages_in_last_blk = numOfPages - IWM_NUM_OF_PAGE_PER_GROUP * (driver->num_of_paging_blk - 1);
    
    IWM_KERNLOG("Paging: allocating mem for %d paging blocks, each block holds 8 pages, last block holds %d pages\n", driver->num_of_paging_blk, driver->num_of_pages_in_last_blk);
    
    /* allocate block of 4Kbytes for paging CSS */
    driver->fw_paging_db[blkIdx].fwPagingBlock = (struct iwm_dma_info*) IOMalloc(sizeof(struct iwm_dma_info));
    
    if(!driver->fw_paging_db[blkIdx].fwPagingBlock) {
        /* free all the previous pages since we failed */
        freeFwPaging(driver);
        return ENOMEM;
    }
    
    error = allocContigDMA(driver, driver->fw_paging_db[blkIdx].fwPagingBlock, IWM_FW_PAGING_SIZE, 4096, "Fw paging block", kIODirectionInOut);
    if(error) {
        freeFwPaging(driver);
        return ENOMEM;
    }
    
    driver->fw_paging_db[blkIdx].fw_paging_size = IWM_FW_PAGING_SIZE;
    
    IWM_KERNLOG("Paging: allocated 4K(CSS) bytes for firmware paging.\n");
    
    /*
     * allocate blocks in dram.
     * since that CSS allocated in fw_paging_db[0] loop start from index 1
     */
    for(blkIdx = 1; blkIdx < driver->num_of_paging_blk + 1; blkIdx ++) {
        /* allocate block of IWM_PAGING_BLOCK_SIZE (32K) */
        /* XXX Use iwm_dma_contig_alloc for allocating */
        driver->fw_paging_db[blkIdx].fwPagingBlock = (struct iwm_dma_info*) IOMalloc(sizeof(struct iwm_dma_info));
        
        if(!driver->fw_paging_db[blkIdx].fwPagingBlock) {
            /* free all the previous pages since we failed */
            freeFwPaging(driver);
            return ENOMEM;
        }
        
        error = allocContigDMA(driver, driver->fw_paging_db[blkIdx].fwPagingBlock, IWM_PAGING_BLOCK_SIZE, 4096, "FW Paging block DRAM", kIODirectionInOut);
        
        if(error) {
            /* free all the previous pages since we failed */
            freeFwPaging(driver);
            return ENOMEM;
        }
        
        driver->fw_paging_db[blkIdx].fw_paging_size = IWM_PAGING_BLOCK_SIZE;
        
        IWM_KERNLOG("Paging: allocated 32K bytes for firmware paging.\n");
    }
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::saveFwPaging(struct iwm_drv *driver, const struct iwm_fw_img *image)
{
    int ret;
    
    ret = allocFwPagingMem(driver, image);
    if(ret) {
        return ret;
    }
    
    return fillPagingMem(driver, image);
}

IOReturn AppleIntelIWM::sendPagingCmd(struct iwm_drv *driver, const struct iwm_fw_img *image)
{
    int blkIdx;
    UInt32 devPhyAddr;
    struct iwm_fw_paging_cmd fwPagingCmd = {
        .flags = (UInt32) (IWM_PAGING_CMD_IS_SECURED | IWM_PAGING_CMD_IS_ENABLED | (driver->num_of_pages_in_last_blk << IWM_PAGING_CMD_NUM_OF_PAGES_IN_LAST_GRP_POS)),
        .block_size = IWM_BLOCK_2_EXP_SIZE,
        .block_num = driver->num_of_paging_blk,
    };
    
    /* loop for for all paging blocks + CSS block */
    for(blkIdx = 0; blkIdx < driver->num_of_paging_blk + 1; blkIdx ++) {
        devPhyAddr = (UInt32) ((UInt64)driver->fw_paging_db[blkIdx].fwPagingBlock->vAddr) >> IWM_PAGE_2_EXP_SIZE;
        fwPagingCmd.device_phy_addr[blkIdx] = devPhyAddr;
        //TODO: DMA SYNC
    }

    return sendCmdPdu(driver, cmdId(IWM_FW_PAGING_BLOCK_CMD, IWM_ALWAYS_LONG_GROUP, 0), 0, sizeof(fwPagingCmd), &fwPagingCmd);
}
