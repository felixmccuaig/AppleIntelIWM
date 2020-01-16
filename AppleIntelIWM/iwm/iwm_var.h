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


//alive data
struct iwm_alive_data {
    int valid;
    uint32_t scd_base_addr;
};

struct iwm_nvm_section {
    uint16_t length;
    uint8_t *data;
};

#define IWM_UCODE_SECTION_MAX 16

#define IEEE80211_ADDR_LEN 6

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
    OSData* fwPointer;
    uint32_t ucodeVersion;

    /* ucode images */
    struct iwm_fw_img img[IWM_UCODE_TYPE_MAX];
    
    struct iwm_ucode_capabilities ucode_capa;
    
    uint32_t phy_config;
    uint8_t valid_tx_ant;
    uint8_t valid_rx_ant;
};

/**
 * struct iwm_fw_paging
 * @fw_paging_block: dma memory info
 * @fw_paging_size: page size
 */
struct iwm_fw_paging {
    struct iwm_dma_info *fwPagingBlock;
    uint32_t fw_paging_size;
};

#define IWM_TX_RING_COUNT    256
#define IWM_TX_RING_LOMARK    192
#define IWM_TX_RING_HIMARK    224

#define IWM_STATION_ID 0
#define IWM_AUX_STA_ID 1

#define	IWM_DEFAULT_MACID	0
#define	IWM_DEFAULT_COLOR	0
#define	IWM_DEFAULT_TSFID	0

#define IWM_ICT_SIZE		4096
#define IWM_ICT_COUNT		(IWM_ICT_SIZE / sizeof (uint32_t))
#define IWM_ICT_PADDR_SHIFT	12

struct iwm_node {
    //leave this for now
};

struct iwm_nvm_data {
    int n_hw_addrs;
    uint8_t hw_addr[IEEE80211_ADDR_LEN];
    
    int sku_cap_band_24GHz_enable;
    int sku_cap_band_52GHz_enable;
    int sku_cap_11n_enable;
    int sku_cap_amt_enable;
    int sku_cap_ipan_enable;
    
    uint8_t radio_cfg_type;
    uint8_t radio_cfg_step;
    uint8_t radio_cfg_dash;
    uint8_t radio_cfg_pnum;
    uint8_t valid_tx_ant, valid_rx_ant;
#define IWM_NUM_CHANNELS	39
#define IWM_NUM_CHANNELS_8000	51
    
    uint16_t nvm_version;
    uint8_t max_tx_pwr_half_dbm;
    
    boolean_t lar_enabled;
    uint16_t nvm_ch_flags[];
};

/* max bufs per tfd the driver will use */
#define IWM_MAX_CMD_TBS_PER_TFD 2

struct iwm_rx_packet;
struct iwm_host_cmd {
    const void *data[IWM_MAX_CMD_TBS_PER_TFD];
    struct iwm_rx_packet *resp_pkt;
    unsigned long _rx_page_addr;
    uint32_t _rx_page_order;
    int handler_status;
    
    uint32_t flags;
    uint32_t id;
    uint16_t len[IWM_MAX_CMD_TBS_PER_TFD];
    uint8_t dataflags[IWM_MAX_CMD_TBS_PER_TFD];
};

struct iwm_dma_info {
    IOBufferMemoryDescriptor *bmd;
    IODMACommand *cmd;
    void* vAddr;
    UInt32 pAddr;
    UInt32 size;
};

struct iwm_tx_data {
    mbuf_t *m;
    struct iwm_node *input;
    int done;
    UInt64 cmdPaddr;
    UInt64 scratchPaddr;
};

struct iwm_tx_ring {
    struct iwm_dma_info* descDMA;
    struct iwm_dma_info* cmdDMA;
    struct iwm_tfd *desc;
    struct iwm_device_cmd *cmd;
    //Data dma tag
    struct iwm_tx_data  data[IWM_TX_RING_COUNT];
    int qid;
    int queued;
    int cur;
};

#define IWM_RX_LEGACY_RING_COUNT	256
#define IWM_RX_MQ_RING_COUNT		512

#define IWM_RBUF_SIZE		4096

#define IWM_MAX_SCATTER		20

struct iwm_rx_data {
    mbuf_t m; //4096 bytes
    UInt32 pAddr; //DMA address
};

struct iwm_rx_ring {
    struct iwm_dma_info* freeDescDMA;
    struct iwm_dma_info* statDMA;
    
    void *desc;
    struct iwm_rb_status *stat;
    struct iwm_rx_data data[512];
    int cur;
};

#define IWM_CMD_RESP_MAX PAGE_SIZE

#define IWM_TE_SESSION_PROTECTION_MAX_TIME_MS 500
#define IWM_TE_SESSION_PROTECTION_MIN_TIME_MS 400

/*
 * Command headers are in iwl-trans.h, which is full of all
 * kinds of other junk, so we just replicate the structures here.
 * First the software bits:
 */
enum IWM_CMD_MODE {
    IWM_CMD_SYNC		= 0,
    IWM_CMD_ASYNC		= (1 << 0),
    IWM_CMD_WANT_SKB	= (1 << 1),
    IWM_CMD_SEND_IN_RFKILL	= (1 << 2),
};
enum iwm_hcmd_dataflag {
    IWM_HCMD_DFL_NOCOPY     = (1 << 0),
    IWM_HCMD_DFL_DUP        = (1 << 1),
};

struct iwm_int_sta {
    uint32_t sta_id;
    uint32_t tfd_queue_msk;
};

struct iwm_phy_ctxt {
    uint16_t id;
    uint16_t color;
    uint32_t ref;
    struct ieee80211_channel *channel;
};

struct iwm_bf_data {
    int bf_enabled;		/* filtering	*/
    int ba_enabled;		/* abort	*/
    int ave_beacon_signal;
    int last_cqm_event;
};


struct iwm_drv {
    struct iwm_fw_info fw_info;
    enum iwm_ucode_type currentUcode;
    int ucodeLoaded = 0;
    
    
    struct iwm_tlv_calib_ctrl default_calib[IWM_UCODE_TYPE_MAX];
    
    struct iwm_nvm_data* nvm_data;
    struct iwm_cfg* config;
    
    //operation mode
    //trans
    int fw_index;
    char firmware_name[32];
    char fwver[32];
    
    int pm_support = 0;
    
    bool firmwareLoaded = false;
    
    //iwm_rx_ring* rxq;
    //iwm_tx_ring* txq[IWM_MVM_MAX_QUEUES];
    
    UInt8 cmdResp[IWM_CMD_RESP_MAX];
    
    struct iwm_tx_ring txq[IWM_MAX_QUEUES];
    struct iwm_rx_ring rxq;
    int qfullmask = 0;
    
    /* last smart fifo state that was successfully sent to firmware */
    enum iwm_sf_state sfState;
    
    int ictCur = 0;
    int hwRev = 0;
    int hwId = 0;
    int intMask = 0;
    int cmdHoldNicAwake = 0;
    int maxRssi = 0;
    int noise = 0;
    UInt8 flags = 0; //only 8 flags
    int fwChunkDone = 0;
    int lastEbsSuccessful = 0;
    int wantResp = 0;
    int generation = 0;
    int firmwareState = 0;
    int mtxLocked = 0;
#define IWM_FLAG_USE_ICT	(1 << 0)
#define IWM_FLAG_HW_INITED	(1 << 1)
#define IWM_FLAG_STOPPED	(1 << 2)
#define IWM_FLAG_RFKILL		(1 << 3)
#define IWM_FLAG_BUSY		(1 << 4)
#define IWM_FLAG_SCANNING	(1 << 5)
#define IWM_FLAG_SCAN_RUNNING	(1 << 6)
#define IWM_FLAG_TE_ACTIVE	(1 << 7)
    
    /*
     * Paging parameters - All of the parameters should be set by the
     * opmode when paging is enabled
     */
    struct iwm_fw_paging	fw_paging_db[IWM_NUM_OF_FW_PAGING_BLOCKS];
    uint16_t		num_of_paging_blk;
    uint16_t		num_of_pages_in_last_blk;
    
    struct iwm_notif_wait_data *notifWait;
    
    struct iwm_phy_db* phyDb;
    
    struct iwm_dma_info* keepWarmDMA;
    struct iwm_dma_info* firmwareDMA;
    
    struct iwm_dma_info* ictDMA;
    UInt32* ictTable; //array of UInt32 with 1024 entries
    
    //Tx Sched ring
    struct iwm_dma_info* schedDMA;
    
    /* Firmware status */
    uint32_t		error_event_table[2];
    uint32_t		log_event_table;
    uint32_t		umac_error_event_table;
    int			support_umac_log;
    
    char fw_mcc[3];
    
    boolean_t psDisabled; //is powersave disabled
    
    IOLock* pciLock;
    IOPCIDevice* pciDevice;
    IOMemoryMap* deviceMap;
    
    IO80211Interface *netif;
    IOWorkLoop *workLoop;
    IOWorkLoop *irqLoop;
    IOCommandGate *gate;
    
    IONetworkStats *networkStats;
    IOEthernetStats *ethernetStats;
    IOFilterInterruptEventSource* interruptSource;
    IOMbufNaturalMemoryCursor* mBufCursor;
    
    volatile void* hwBase;
    volatile UInt32 scdBase;
};

#endif /* iwm_var_h */
