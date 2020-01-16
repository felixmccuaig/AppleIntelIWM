//
//  iwm_phy_db.h
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 2/01/2020.
//  Copyright © 2020 Felix McCuaig. All rights reserved.
//

#ifndef iwm_phy_db_h
#define iwm_phy_db_h

#define __packed __attribute__((__packed__))

#define CHANNEL_NUM_SIZE	4	/* num of channels in calib_ch size */

struct iwm_phy_db_entry {
    uint16_t	size;
    uint8_t		*data;
};

/**
 * struct iwm_phy_db - stores phy configuration and calibration data.
 *
 * @cfg: phy configuration.
 * @calib_nch: non channel specific calibration data.
 * @calib_ch: channel specific calibration data.
 * @n_group_papd: number of entries in papd channel group.
 * @calib_ch_group_papd: calibration data related to papd channel group.
 * @n_group_txp: number of entries in tx power channel group.
 * @calib_ch_group_txp: calibration data related to tx power chanel group.
 */
struct iwm_phy_db {
    struct iwm_phy_db_entry cfg;
    struct iwm_phy_db_entry calib_nch;
    int n_group_papd;
    struct iwm_phy_db_entry *calib_ch_group_papd;
    int n_group_txp;
    struct iwm_phy_db_entry *calib_ch_group_txp;
    
    struct iwm_drv *driver;
};

enum iwm_phy_db_section_type {
    IWM_PHY_DB_CFG = 1,
    IWM_PHY_DB_CALIB_NCH,
    IWM_PHY_DB_UNUSED,
    IWM_PHY_DB_CALIB_CHG_PAPD,
    IWM_PHY_DB_CALIB_CHG_TXP,
    IWM_PHY_DB_MAX
};

#define PHY_DB_CMD 0x6c

/*
 * phy db - configure operational ucode
 */
struct iwm_phy_db_cmd {
    uint16_t type;
    uint16_t length;
    uint8_t data[];
} __packed;

/* for parsing of tx power channel group data that comes from the firmware*/
struct iwm_phy_db_chg_txp {
    uint32_t space;
    uint16_t max_channel_idx;
} __packed;

/*
 * phy db - Receive phy db chunk after calibrations
 */
struct iwm_calib_res_notif_phy_db {
    uint16_t type;
    uint16_t length;
    uint8_t data[];
} __packed;


#endif /* iwm_phy_db_h */
