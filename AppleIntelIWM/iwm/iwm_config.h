//
//  iwm_config.h
//  AppleIntelIWM
//
//  Created by felixmccuaig on 4/10/19.
//  Copyright © 2019 felixmccuaig. All rights reserved.
//

#ifndef iwm_config_h
#define iwm_config_h

#pragma once

enum iwm_device_family {
    IWM_DEVICE_FAMILY_UNDEFINED,
    IWM_DEVICE_FAMILY_7000,
    IWM_DEVICE_FAMILY_8000,
};

#define IWM_DEFAULT_MAX_TX_POWER    22

/* Antenna presence definitions */
#define    IWM_ANT_NONE    0x0
#define    IWM_ANT_A    (1 << 0)
#define    IWM_ANT_B    (1 << 1)
#define IWM_ANT_C    (1 << 2)
#define    IWM_ANT_AB    (IWM_ANT_A | IWM_ANT_B)
#define    IWM_ANT_AC    (IWM_ANT_A | IWM_ANT_C)
#define IWM_ANT_BC    (IWM_ANT_B | IWM_ANT_C)
#define IWM_ANT_ABC    (IWM_ANT_A | IWM_ANT_B | IWM_ANT_C)

static inline uint8_t num_of_ant(uint8_t mask)
{
    return  !!((mask) & IWM_ANT_A) +
    !!((mask) & IWM_ANT_B) +
    !!((mask) & IWM_ANT_C);
}

/* lower blocks contain EEPROM image and calibration data */
#define IWM_OTP_LOW_IMAGE_SIZE_FAMILY_7000    (16 * 512 * sizeof(uint16_t)) /* 16 KB */
#define IWM_OTP_LOW_IMAGE_SIZE_FAMILY_8000    (32 * 512 * sizeof(uint16_t)) /* 32 KB */
#define IWM_OTP_LOW_IMAGE_SIZE_FAMILY_9000    IWM_OTP_LOW_IMAGE_SIZE_FAMILY_8000


/**
 * enum iwl_nvm_type - nvm formats
 * @IWM_NVM: the regular format
 * @IWM_NVM_EXT: extended NVM format
 * @IWM_NVM_SDP: NVM format used by 3168 series
 */
enum iwm_nvm_type {
    IWM_NVM,
    IWM_NVM_EXT,
    IWM_NVM_SDP,
};

/**
 * struct iwm_cfg
 * @name: Official name of the device
 * @fw_name: Firmware filename.
 * @host_interrupt_operation_mode: device needs host interrupt operation
 *      mode set
 * @nvm_hw_section_num: the ID of the HW NVM section
 * @apmg_wake_up_wa: should the MAC access REQ be asserted when a command
 *      is in flight. This is due to a HW bug in 7260, 3160 and 7265.
 * @nvm_type: see &enum iwl_nvm_type
 */
struct iwm_cfg {
    const char *name;
    const char *fw_name;
    uint16_t eeprom_size;
    enum iwm_device_family device_family;
    int host_interrupt_operation_mode;
    uint8_t nvm_hw_section_num;
    int apmg_wake_up_wa;
    enum iwm_nvm_type nvm_type;
};

#define IWM7260_FW    "iwm7260fw"
#define IWM3160_FW    "iwm3160fw"
#define IWM3168_FW    "iwm3168fw"
#define IWM7265_FW    "iwm7265fw"
#define IWM7265D_FW   "iwm7265Dfw"

#define IWM8000_FW    "iwm8000Cfw"
#define IWM8265_FW    "iwm8265fw"

#define IWM_NVM_HW_SECTION_NUM_FAMILY_7000    0

#define IWM_DEVICE_7000_COMMON                        \
.device_family = IWM_DEVICE_FAMILY_7000,            \
.eeprom_size = IWM_OTP_LOW_IMAGE_SIZE_FAMILY_7000,        \
.nvm_hw_section_num = IWM_NVM_HW_SECTION_NUM_FAMILY_7000,    \
.apmg_wake_up_wa = 1

#define IWM_NVM_HW_SECTION_NUM_FAMILY_8000    10

#define IWM_DEVICE_8000_COMMON                        \
.device_family = IWM_DEVICE_FAMILY_8000,            \
.eeprom_size = IWM_OTP_LOW_IMAGE_SIZE_FAMILY_8000,        \
.nvm_hw_section_num = IWM_NVM_HW_SECTION_NUM_FAMILY_8000

/*
 * Autoconf glue-sniffing
 */
#define    PCI_VENDOR_INTEL        0x8086
#define    PCI_PRODUCT_INTEL_WL_3160_1    0x08b3
#define    PCI_PRODUCT_INTEL_WL_3160_2    0x08b4
#define    PCI_PRODUCT_INTEL_WL_3165_1    0x3165
#define    PCI_PRODUCT_INTEL_WL_3165_2    0x3166
#define    PCI_PRODUCT_INTEL_WL_3168_1    0x24fb
#define    PCI_PRODUCT_INTEL_WL_7260_1    0x08b1
#define    PCI_PRODUCT_INTEL_WL_7260_2    0x08b2
#define    PCI_PRODUCT_INTEL_WL_7265_1    0x095a
#define    PCI_PRODUCT_INTEL_WL_7265_2    0x095b
#define    PCI_PRODUCT_INTEL_WL_8260_1    0x24f3
#define    PCI_PRODUCT_INTEL_WL_8260_2    0x24f4
#define    PCI_PRODUCT_INTEL_WL_8265_1    0x24fd

static struct iwm_cfg iwm7260_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 7260",
    .fw_name = IWM7260_FW,
    IWM_DEVICE_7000_COMMON,
    .host_interrupt_operation_mode = 1,
};

static struct iwm_cfg iwm3160_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 3160",
    .fw_name = IWM3160_FW,
    IWM_DEVICE_7000_COMMON,
    .host_interrupt_operation_mode = 1,
};

static struct iwm_cfg iwm3165_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 3165",
    .fw_name = IWM7265D_FW,
    IWM_DEVICE_7000_COMMON,
    .host_interrupt_operation_mode = 0,
};

static struct iwm_cfg iwm3168_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 3168",
    .fw_name = IWM3168_FW,
    IWM_DEVICE_7000_COMMON,
    .host_interrupt_operation_mode = 0,
    .nvm_type = IWM_NVM_SDP,
};

static struct iwm_cfg iwm7265_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 7265",
    .fw_name = IWM7265_FW,
    IWM_DEVICE_7000_COMMON,
    .host_interrupt_operation_mode = 0,
};

static struct iwm_cfg iwm7265d_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 7265",
    .fw_name = IWM7265D_FW,
    IWM_DEVICE_7000_COMMON,
    .host_interrupt_operation_mode = 0,
};

static struct iwm_cfg iwm8260_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 8260",
    .fw_name = IWM8000_FW,
    IWM_DEVICE_8000_COMMON,
    .host_interrupt_operation_mode = 0,
};

static struct iwm_cfg iwm8265_cfg = {
    .name = "Intel(R) Dual Band Wireless AC 8265",
    .fw_name = IWM8265_FW,
    IWM_DEVICE_8000_COMMON,
    .host_interrupt_operation_mode = 0,
};

static struct iwm_devices {
    uint16_t        device;
    struct iwm_cfg    *cfg;
} iwm_devices[] = {
    { PCI_PRODUCT_INTEL_WL_3160_1, &iwm3160_cfg },
    { PCI_PRODUCT_INTEL_WL_3160_2, &iwm3160_cfg },
    { PCI_PRODUCT_INTEL_WL_3165_1, &iwm3165_cfg },
    { PCI_PRODUCT_INTEL_WL_3165_2, &iwm3165_cfg },
    { PCI_PRODUCT_INTEL_WL_3168_1, &iwm3168_cfg },
    { PCI_PRODUCT_INTEL_WL_7260_1, &iwm7260_cfg },
    { PCI_PRODUCT_INTEL_WL_7260_2, &iwm7260_cfg },
    { PCI_PRODUCT_INTEL_WL_7265_1, &iwm7265_cfg },
    { PCI_PRODUCT_INTEL_WL_7265_2, &iwm7265_cfg },
    { PCI_PRODUCT_INTEL_WL_8260_1, &iwm8260_cfg },
    { PCI_PRODUCT_INTEL_WL_8260_2, &iwm8260_cfg },
    { PCI_PRODUCT_INTEL_WL_8265_1, &iwm8265_cfg },
};

#endif /* iwm_config_h */
