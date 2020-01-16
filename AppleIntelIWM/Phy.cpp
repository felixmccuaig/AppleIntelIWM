//
//  Phy.cpp
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 2/01/2020.
//  Copyright © 2020 Felix McCuaig. All rights reserved.
//

#include "AppleIntelIWM.h"

IOReturn AppleIntelIWM::waitPhyDbEntry(struct iwm_drv *driver, struct iwm_rx_packet *pkt, void *data)
{
    struct iwm_phy_db *phyDb = (struct iwm_phy_db*) data;
    
    if(pkt->hdr.code != IWM_CALIB_RES_NOTIF_PHY_DB) {
        if(pkt->hdr.code != IWM_INIT_COMPLETE_NOTIF) {
            IWM_KERNLOG("Unexpected cmd!\n");
        }
        return KERN_FAILURE;
    }
    
    if(phyDbSetSection(phyDb, pkt) != KERN_SUCCESS) {
        IWM_KERNLOG("Phy Db set section failed!\n");
        return KERN_FAILURE;
    }

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::phyDbSetSection(struct iwm_phy_db *phyDb, struct iwm_rx_packet *pkt)
{
    if(!phyDb) {
        return KERN_FAILURE;
    }
    
    struct iwm_calib_res_notif_phy_db *phyDbNotif = (struct iwm_calib_res_notif_phy_db*) pkt->data;
    enum iwm_phy_db_section_type type = (iwm_phy_db_section_type) phyDbNotif->type;
    
    UInt16 size = phyDbNotif->length;
    
    struct iwm_phy_db_entry *entry;
    UInt16 chgId = 0;
    
    if(type == IWM_PHY_DB_CALIB_CHG_PAPD) {
        chgId = *(UInt16*)phyDbNotif->data;
        if(!phyDb->calib_ch_group_papd) {
            /*
             * Firmware sends the largest index first, so we can use
             * it to know how much we should allocate.
             */
            phyDb->calib_ch_group_papd = (struct iwm_phy_db_entry*) IOMalloc((chgId + 1) * sizeof(struct iwm_phy_db_entry));
            
            if(!phyDb->calib_ch_group_papd) {
                return KERN_FAILURE;
            }
            
            memset(phyDb->calib_ch_group_papd, 0, sizeof(*phyDb->calib_ch_group_papd));
            
            phyDb->n_group_txp = chgId + 1;
        }
    } else if(type == IWM_PHY_DB_CALIB_CHG_TXP)  {
        chgId = *(UInt16 *)phyDbNotif->data;
        if(!phyDb->calib_ch_group_txp) {
            /*
             * Firmware sends the largest index first, so we can use
             * it to know how much we should allocate.
             */
            phyDb->calib_ch_group_txp = (struct iwm_phy_db_entry*) IOMalloc((chgId + 1) * sizeof(struct iwm_phy_db_entry));
            if(!phyDb->calib_ch_group_txp) {
                return KERN_FAILURE;
            }
            
            memset(phyDb->calib_ch_group_txp, 0, sizeof(*phyDb->calib_ch_group_txp));
            
            phyDb->n_group_txp = chgId + 1;
        }
    }
    
    entry = phyDbGetSection(phyDb, type, chgId);
    
    if(!entry) {
        return KERN_FAILURE;
    }
    
    if(entry->data != NULL) {
        IOFree(entry->data, sizeof(*entry->data)); //TODO: we might have issues here
    }
    
    entry->data = (UInt8 *) IOMalloc(size);
    
    if(!entry->data) {
        entry->size = 0;
        return KERN_FAILURE;
    }
    
    memcpy(entry->data, phyDbNotif->data, size);
    
    entry->size = size;
    
    IWM_KERNLOG("[PHYDB]SET: Type %d , Size: %d\n", type, size);
    
    return KERN_SUCCESS;
}

 struct iwm_phy_db_entry *AppleIntelIWM::phyDbGetSection(struct iwm_phy_db *phyDb, enum iwm_phy_db_section_type type, UInt16 chgId)
{
    if(!phyDb || type >= IWM_PHY_DB_MAX) {
        return NULL;
    }
    
    switch(type) {
        case IWM_PHY_DB_CFG:
            return &phyDb->cfg;
        case IWM_PHY_DB_CALIB_NCH:
            return &phyDb->calib_nch;
        case IWM_PHY_DB_CALIB_CHG_PAPD:
            if(chgId >= phyDb->n_group_papd) {
                return NULL;
            }
            return &phyDb->calib_ch_group_papd[chgId];
        case IWM_PHY_DB_CALIB_CHG_TXP:
            if(chgId >= phyDb->n_group_txp) {
                return NULL;
            }
            return &phyDb->calib_ch_group_txp[chgId];
        default:
            return NULL;
    }
    
    return NULL;
}

IOReturn AppleIntelIWM::phyDbFreeSection(struct iwm_phy_db *phyDb, enum iwm_phy_db_section_type type, UInt16 chgId)
{
    struct iwm_phy_db_entry *entry = phyDbGetSection(phyDb, type, chgId);
    
    if(!entry) {
        return KERN_SUCCESS;
    }
    
    if(entry->data != NULL) {
        IOFree(entry->data, sizeof(*entry->data));
    }
    
    entry->data = NULL;
    entry->size = 0;

    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::phyDbFree(struct iwm_phy_db *phyDb)
{
    int i;
    
    if(!phyDb) {
        return KERN_SUCCESS;
    }
    
    phyDbFreeSection(phyDb, IWM_PHY_DB_CFG, 0);
    phyDbFreeSection(phyDb, IWM_PHY_DB_CALIB_NCH, 0);
    
    for(i = 0; i< phyDb->n_group_papd; i++) {
        phyDbFreeSection(phyDb, IWM_PHY_DB_CALIB_CHG_PAPD, i);
    }
    
    if(phyDb->calib_ch_group_papd != NULL) {
        IOFree(phyDb->calib_ch_group_papd, sizeof(*phyDb->calib_ch_group_papd));
    }
    
    for(i = 0; i < phyDb->n_group_txp; i++) {
        phyDbFreeSection(phyDb, IWM_PHY_DB_CALIB_CHG_TXP, i);
    }
    
    if(phyDb->calib_ch_group_txp != NULL) {
    
        IOFree(phyDb->calib_ch_group_txp, sizeof(*phyDb->calib_ch_group_txp));
    }
    
    IOFree(phyDb, sizeof(*phyDb));

    return KERN_SUCCESS;
}

struct iwm_phy_db* AppleIntelIWM::phyDbInit(struct iwm_drv *driver)
{
    struct iwm_phy_db *phyDb = (struct iwm_phy_db*) IOMalloc(sizeof(struct iwm_phy_db));
    
    if(!phyDb) {
        return NULL;
    }
    
    memset(phyDb, 0, sizeof(*phyDb));
    
    phyDb->driver = driver;
    phyDb->n_group_txp = -1;
    phyDb->n_group_papd = -1;

    /* TODO: add default values of the phy db. */
    return phyDb;
}



