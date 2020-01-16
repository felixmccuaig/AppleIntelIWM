//
//  iwm_var.h
//  AppleIntelIWM
//
//  Created by felixmccuaig on 5/10/19.
//  Copyright © 2019 felixmccuaig. All rights reserved.
//

#ifndef iwm_var_h
#define iwm_var_h

#pragma once

#include <stdint.h>

#define IWM_UCODE_SECTION_MAX 16

enum iwm_ucode_type {
    IWM_UCODE_REGULAR,
    IWM_UCODE_INIT,
    IWM_UCODE_WOWLAN,
    IWM_UCODE_REGULAR_USNIFFER,
    IWM_UCODE_TYPE_MAX
};

struct iwm_ucode_capabilities {
    uint32_t max_probe_length;
    uint32_t n_scan_channels;
    uint32_t flags;
    uint8_t enabled_api[howmany(IWM_NUM_UCODE_TLV_API, NBBY)];
    uint8_t enabled_capa[howmany(IWM_NUM_UCODE_TLV_CAPA, NBBY)];
};

struct iwm_fw_desc {
    const void *data;    /* vmalloc'ed data */
    uint32_t len;        /* size in bytes */
    uint32_t offset;    /* offset in the device */
};

struct iwm_fw_img {
    struct iwm_fw_desc sec[IWM_UCODE_SECTION_MAX];
    int fw_count;
    int is_dual_cpus;
    uint32_t paging_mem_size;
};

struct iwm_fw_info {
    //FW POINTER
    
    /* ucode images */
    struct iwm_fw_img img[IWM_UCODE_TYPE_MAX];
    
    struct iwm_ucode_capabilities ucode_capa;
    
    uint32_t phy_config;
    uint8_t valid_tx_ant;
    uint8_t valid_rx_ant;
};

struct iwm_drv {
    struct iwm_cfg* config;
    
    struct iwm_fw_info fw_info;
    //operation mode
    //trans
    int fw_index;
    char firmware_name[32];
};

#endif /* iwm_var_h */
