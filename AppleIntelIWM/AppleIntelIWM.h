#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOMemoryCursor.h>
#include <IOKit/IODMACommand.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOCommandGate.h>

#include <IOKit/network/IOPacketQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>

#include <IOKit/pci/IOPCIDevice.h>

#include <libkern/OSKextLib.h>

#include <sys/kpi_mbuf.h>

#include <sys/param.h>

#include "apple80211/IO80211Controller.h"
#include "apple80211/IO80211Interface.h"
#include "apple80211/IO80211WorkLoop.h"

extern "C" {
#include "iwm/iwm_reg.h"
#include "iwm/iwm_config.h"
#include "iwm/iwm_var.h"
#include "iwm/if_iwm_fw.h"
#include "iwm/iwm_notif_wait.h"
#include "iwm/iwm_phy_db.h"
};

class AppleIntelIWM : public IO80211Controller {
    OSDeclareDefaultStructors(AppleIntelIWM);
public:
    virtual bool init(OSDictionary* dictionary) override;
    virtual bool start(IOService* provider) override;
    virtual void stop(IOService* provider) override;
    virtual void free() override;
    
    //80211
    virtual IOReturn configureReport(IOReportChannelList*, unsigned int, void*, void*) override;
    virtual IOReturn updateReport(IOReportChannelList*, unsigned int, void*, void*) override;
    virtual void requestPacketTx(void*, unsigned int) override;
    virtual IOReturn getHardwareAddressForInterface(IO80211Interface*, IOEthernetAddress*) override;
    virtual void inputMonitorPacket(mbuf_t, unsigned int, void*, unsigned long) override;
    virtual int outputRaw80211Packet(IO80211Interface*, mbuf_t) override;
    virtual int outputActionFrame(IO80211Interface*, mbuf_t) override;
    virtual int bpfOutputPacket(OSObject*, unsigned int, mbuf_t) override;
    virtual SInt32 monitorModeSetEnabled(IO80211Interface*, bool, unsigned int) override;
    virtual IO80211Interface * getNetworkInterface() override;
    virtual SInt32 apple80211_ioctl(IO80211Interface*, IO80211VirtualInterface*, ifnet_t, unsigned long, void*) override;
    virtual SInt32 apple80211_ioctl(IO80211Interface*, ifnet_t, unsigned long, void*) override;
    virtual SInt32 apple80211Request(unsigned int, int, IO80211Interface*, void*) override;
    virtual SInt32 apple80211VirtualRequest(unsigned int, int, IO80211VirtualInterface*, void*) override;
    virtual SInt32 stopDMA() override;
    virtual UInt32 hardwareOutputQueueDepth(IO80211Interface*) override;
    virtual SInt32 performCountryCodeOperation(IO80211Interface*, IO80211CountryCodeOp) override;
    virtual bool useAppleRSNSupplicant(IO80211Interface*) override;
    virtual bool useAppleRSNSupplicant(IO80211VirtualInterface*) override;
    virtual void dataLinkLayerAttachComplete(IO80211Interface*) override;
    virtual SInt32 enableFeature(IO80211FeatureCode, void*) override;
    virtual SInt32 setVirtualHardwareAddress(IO80211VirtualInterface*, ether_addr*) override;
    virtual SInt32 enableVirtualInterface(IO80211VirtualInterface*) override;
    virtual SInt32 disableVirtualInterface(IO80211VirtualInterface*) override;
    virtual IOReturn requiresExplicitMBufRelease() override;
    virtual IOReturn flowIdSupported() override;
    virtual IO80211FlowQueueLegacy * requestFlowQueue(FlowIdMetadata const*) override;
    virtual void releaseFlowQueue(IO80211FlowQueue*) override;
    virtual IOReturn enablePacketTimestamping() override;
    virtual IOReturn disablePacketTimestamping() override;
    virtual UInt32 selfDiagnosticsReport(int, char const*, unsigned int) override;
    virtual UInt32 getDataQueueDepth(OSObject*) override;
    virtual mbuf_flags_t inputPacket(mbuf_t) override;
    virtual SInt32 apple80211_ioctl_get(IO80211Interface*, IO80211VirtualInterface*, ifnet_t, void*) override;
    virtual SInt32 apple80211_ioctl_set(IO80211Interface*, IO80211VirtualInterface*, ifnet_t, void*) override;
    virtual IO80211VirtualInterface * createVirtualInterface(ether_addr*, unsigned int) override;
    virtual bool attachVirtualInterface(IO80211VirtualInterface**, ether_addr*, unsigned int, bool) override;
    virtual bool detachVirtualInterface(IO80211VirtualInterface*, bool) override;
    //end 80211
    
private:
    uint16_t getVendorID(IOPCIDevice* provider);
    uint16_t getDeviceID(IOPCIDevice* provider);
    struct iwm_cfg* getConfig(IOPCIDevice* provider);
    
    int findMSIInterruptTypeIndex(struct iwm_drv* driver);
    
    static void interruptOccured(OSObject* owner, IOInterruptEventSource* sender, int count);
    
    static void notifInterrupt(AppleIntelIWM* me, struct iwm_drv* driver);
    
    static bool interruptFilter(OSObject* owner, IOFilterInterruptEventSource* src);
    
    static void handleRxb(AppleIntelIWM* me, struct iwm_drv* driver, mbuf_t m);
    
    static IOReturn gateAction(OSObject *owner, void *arg0, void *arg1, void *arg2, void *arg3);
    
    int iwmPollBit(struct iwm_drv* driver, UInt32 dest, UInt32 bits, UInt32 mask, int time);
    
    IOReturn iwmWrite8(struct iwm_drv* driver, UInt32 ofs, UInt8 val);
    IOReturn iwmWrite32(struct iwm_drv* driver, UInt32 ofs, UInt32 val);
    
    UInt32 iwmRead(struct iwm_drv* driver, UInt32 ofs);
    IOReturn iwmSetBits(struct iwm_drv* driver, UInt32 dest, UInt32 mask);
    IOReturn iwmClearBits(struct iwm_drv* driver, UInt32 dest, UInt32 mask);
    
    UInt32 readPRPH(struct iwm_drv* driver, UInt32 addr);
    IOReturn writePRPH(struct iwm_drv* driver, UInt32 addr, UInt32 value);
    IOReturn setBitsMaskPRPH(struct iwm_drv* driver, UInt32 reg, UInt32 bits, UInt32 mask);
    IOReturn setBitsPRPH(struct iwm_drv* driver, UInt32 reg, UInt32 bits);
    IOReturn clearBitsPRPH(struct iwm_drv* driver, UInt32 reg, UInt32 bits);
    
    IOReturn enableRfKillInterrupt(struct iwm_drv* driver);
    
    IOReturn checkRfKill(struct iwm_drv* driver);
    
    IOReturn setHardwareReady(struct iwm_drv* driver);
    
    IOReturn prepareCardHardware(struct iwm_drv* driver);
    
    UInt8 getValidRxAnt(struct iwm_drv* driver);
    UInt8 getValidTxAnt(struct iwm_drv* driver);
    
    UInt32 getPhyConfig(struct iwm_drv* driver);
    
    IOReturn zeroDriver(struct iwm_drv* driver);
    
    IOReturn setUcodeApiFlags(struct iwm_drv* driver, const UInt8 *data, struct iwm_ucode_capabilities *capabilities);
    
    IOReturn setUcodeCapabilities(struct iwm_drv* driver, const UInt8 *data, struct iwm_ucode_capabilities *capabilities);
    
    IOReturn setDefaultCalibration(struct iwm_drv *driver, const void *data);
    
    IOReturn freeFwInfo(struct iwm_drv *driver);
    
    IOReturn readFirmware(struct iwm_drv *driver);
    
    static void firmwareLoadComplete(OSKextRequestTag requestTag, OSReturn result, const void *resourceData, uint32_t resourceDataLength, void *context);
    
    OSData* requestLoadFirmware(struct iwm_drv *driver);
    
    IOReturn storeCScheme(struct iwm_drv *driver, const UInt8* data, size_t dataLen);
    
    IOReturn storeFirmwareSection(struct iwm_drv *driver, enum iwm_ucode_type type, const UInt8 *data, UInt32 dataLen);
    
    IOReturn allocFwMem(struct iwm_drv *driver);
    
    IOReturn allocKwPage(struct iwm_drv *driver);
    
    IOReturn allocICT(struct iwm_drv *driver);
    
    IOReturn allocRxRing(struct iwm_drv *driver, struct iwm_rx_ring *ring);
    
    IOReturn resetRxRing(struct iwm_drv *driver, struct iwm_rx_ring *ring);
    
    IOReturn freeRxRing(struct iwm_drv *driver, struct iwm_rx_ring *ring);
    
    IOReturn allocTxSchedRings(struct iwm_drv *driver);
    
    IOReturn allocTxRing(struct iwm_drv *driver, struct iwm_tx_ring *ring, int qid);
    
    IOReturn resetTxRing(struct iwm_drv *driver, struct iwm_tx_ring *ring);
    
    IOReturn freeTxRing(struct iwm_drv *driver, struct iwm_tx_ring *ring);
    
    void enableInterrupts(struct iwm_drv* driver);
    IOReturn restoreInterrupts(struct iwm_drv* driver);
    IOReturn disableInterrupts(struct iwm_drv* driver);
    
    IOReturn ictReset(struct iwm_drv* driver);
    
    IOReturn stopDevice(struct iwm_drv *driver);
    
    IOReturn initNicRx(struct iwm_drv *driver);
    IOReturn initNicTx(struct iwm_drv *driver);
    
    IOReturn pcieRxStop(struct iwm_drv *driver);
    
    IOReturn initNic(struct iwm_drv* driver);
    
    IOReturn nicConfig(struct iwm_drv* driver);
    
    IOReturn preInit(struct iwm_drv* driver);
    
    IOReturn sendBtInitConfig(struct iwm_drv* driver);
    
    IOReturn initNvm(struct iwm_drv* driver);
    
    IOReturn NVMReadSection(struct iwm_drv* driver, UInt16 section, UInt8 *data, UInt16 *len, UInt32 sizeRead);
    
    IOReturn NVMReadChunk(struct iwm_drv* driver, UInt16 section, UInt16 offset, UInt16 length, UInt8 *data, UInt16* len);
    
    static struct iwm_nvm_data* parseNVMSections(struct iwm_drv* driver, struct iwm_nvm_section *sections);
    
    static struct iwm_nvm_data* parseNVMData(struct iwm_drv* driver, UInt16 *nvmHw, UInt16 *nvmSw, UInt16 *nvmCalib,  UInt16 *macOverride, UInt16 *phySku, UInt16 *regulatory);
    
    void eepromChannelFlags();
    
    void addChannelBand();
    void initChannelMap();
    
    static UInt16 getSku(const struct iwm_drv *driver, UInt16 *nvmSw);
    
    void setHardwareAddress();
    void getHardwareAddress();
    
    void transmit();
    void receive();
    
    void changeAuth();
    
    IOReturn loadGivenUcode(struct iwm_drv* driver, const struct iwm_fw_img *image);
    
    IOReturn loadCpuSections(struct iwm_drv* driver, const struct iwm_fw_img *image, int cpu, int *firstUcodeSection);
    
    IOReturn loadSection(struct iwm_drv* driver, UInt8 sectionNum, const struct iwm_fw_desc *section);

    IOReturn loadFirmwareChunk(struct iwm_drv* driver, UInt32 destAddr, UInt32 phyAddr, UInt32 byteCount);
    
    IOReturn loadUcodeWaitAlive(struct iwm_drv* driver, enum iwm_ucode_type ucodeType);
    
    static IOReturn aliveFn(struct iwm_drv* driver, struct iwm_rx_packet *pkt, void *data);
    
    IOReturn startFirmware(struct iwm_drv* driver, const struct iwm_fw_img *fw);
    
    IOReturn sendTxAntConfig(struct iwm_drv* driver, UInt8 validTxAnt);
    
    IOReturn stopFirmware();
    
    IOReturn enableFwLoadInt(struct iwm_drv* driver);
    
    void umacScanAbort();
    void lmacScanAbort();
    
    void umacScan();
    void lmacScan();
    
    void rxUmacScanComplete();
    void rxLmacScanComplete();
    
    void startScan();
    void stopScan();
    
    IOReturn setPowerMode(struct iwm_drv* driver);
    
    IOReturn apmConfig(struct iwm_drv* driver);
    IOReturn apmInit(struct iwm_drv* driver);
    IOReturn apmStop(struct iwm_drv* driver);
    
    IOReturn initHardware(struct iwm_drv* driver);
    IOReturn startHardware(struct iwm_drv* driver);
    
    IOReturn initUcode(struct iwm_drv* driver, int justNVM);
    
    IOReturn rxAddBuf(struct iwm_drv* driver, int size, int idx);
    
    IOReturn allocContigDMA(struct iwm_drv* driver, struct iwm_dma_info *dma, size_t size, size_t alignment, char* desc, IOOptionBits direction = kIODirectionInOut);
    
    IOReturn freeContigDMA(struct iwm_drv* driver, struct iwm_dma_info *dma);
    
    IOReturn clearCmdInFlight(struct iwm_drv *driver);
    
    IOReturn setCmdInFlight(struct iwm_drv *driver);
    
    IOReturn pcieFwAlive(struct iwm_drv* driver, UInt32 scdBaseAddr);
    
    void resume();
    void suspend();
    
    int nicLock(struct iwm_drv *driver);
    
    IOReturn nicUnlock(struct iwm_drv *driver);
    
    IOReturn sendPhyConfigCmd(struct iwm_drv *driver);
    
    UInt32 mBufDmaMap(struct iwm_drv *driver, mbuf_t m);
    
    //notif
    struct iwm_notif_wait_data* notifWaitInit(struct iwm_drv* driver);
    
    IOReturn notifWaitFree(struct iwm_notif_wait_data *notifData);
    
    static IOReturn notifWaitNotify(struct iwm_notif_wait_data *notifData, UInt16 cmd, struct iwm_rx_packet *pkt);
    
    IOReturn abortNotificationWait(struct iwm_notif_wait_data *notifData);
    
    IOReturn initNotificationWait(struct iwm_notif_wait_data *notifData, struct iwm_notification_wait *waitEntry, const UInt16 *cmds, int nCmds, int (*fn)(struct iwm_drv* driver, struct iwm_rx_packet *pkt, void *data), void *fn_data);
    
    IOReturn waitNotification(struct iwm_notif_wait_data *notifData, struct iwm_notification_wait *wait_entry, int timeout);
    
    IOReturn removeNotification(struct iwm_notif_wait_data *notifData, struct iwm_notification_wait *waitEntry);
    
    IOReturn enableTxq(struct iwm_drv* driver, int staId, int qid, int fifo);
    
    //end notif
    
    //phy
    
    struct iwm_phy_db* phyDbInit(struct iwm_drv* driver);
    
    static IOReturn waitPhyDbEntry(struct iwm_drv* driver, struct iwm_rx_packet *pkt, void *data);
    
    static IOReturn phyDbSetSection(struct iwm_phy_db *phyDb, struct iwm_rx_packet *pkt);
    
    static struct iwm_phy_db_entry* phyDbGetSection(struct iwm_phy_db *phyDb, enum iwm_phy_db_section_type type, UInt16 chgId);
    
    static IOReturn phyDbFreeSection(struct iwm_phy_db *phyDb, enum iwm_phy_db_section_type type, UInt16 chgId);
    
    static IOReturn phyDbFree(struct iwm_phy_db *phyDb);
    
    UInt8 getDmaUpperAddr(UInt32 addr);
    
    //end phy
    
    //util
    IOReturn sendCmdPduStatus(struct iwm_drv* driver, UInt32 id, UInt16 len, const void* data, UInt32 *status);
    
    IOReturn sendCmdPdu(struct iwm_drv* driver, UInt32 id, UInt32 flags, UInt16 len, const void *data);
    
    IOReturn sendCmd(struct iwm_drv* driver, struct iwm_host_cmd* hCmd);
    
    UInt8 cmdGroupId(UInt32 cmdId);
    
    UInt8 cmdOpcode(UInt32 cmdId);
    
    UInt8 cmdVersion(UInt32 cmdId);
    
    UInt32 cmdId(UInt8 opCode, UInt8 groupId, UInt8 version);
    
    static UInt32 rxPacketLen(const struct iwm_rx_packet *pkt);
    
    static UInt32 rxPacketPayloadLen(const struct iwm_rx_packet *pkt);
    
    static UInt16 getNvmVersion(const struct iwm_drv* driver, UInt16 *nvmSw);
    
    static UInt16 getRadioConfig(const struct iwm_drv* driver, UInt16 *nvmSw);
    
    static UInt16 getNHWAddr(const struct iwm_drv* driver, UInt16* nvmSw);
    
    static void setRadioCfg(const struct iwm_drv* driver, struct iwm_nvm_data *data, UInt32 radioConfig);
    
    static IOReturn setHwAddress(struct iwm_drv* driver, struct iwm_nvm_data *data, UInt16 *nvmHw, UInt16 *macOverride);
    
    IOReturn sendCmdStatus(struct iwm_drv* driver, struct iwm_host_cmd *cmd, UInt32 *status);
    
    void freeResp(struct iwm_drv* driver, struct iwm_host_cmd *hCmd);
    
    //end util
    
    //SMART FIFO
    //sfUpdate(struct iwm_drv* driver, );
    
    
    //END SMARTFIFO
    
    //trans
    IOReturn writeMem(struct iwm_drv* driver, UInt32 addr, const void* buf, int dWords);
    
    IOReturn writeMem32(struct iwm_drv* driver, UInt32 addr, UInt32 val);
    
    void IWM_LOCK(struct iwm_drv* driver);
    
    void IWM_UNLOCK(struct iwm_drv* driver);
    
    int IWM_SLEEP(struct iwm_drv *driver, void* event, UInt32 interType, int time);
    
    static void IWM_WAKEUP(struct iwm_drv* driver, void* event, bool oneThread);
    
    int IWM_WAIT_LOCK_SLEEP(struct iwm_notif_wait_data *notif_data, void* event, int time);
    
    //end trans
    
    //fw.cpp
    void freeFwPaging(struct iwm_drv* driver);
    
    IOReturn fillPagingMem(struct iwm_drv* driver, const struct iwm_fw_img *image);
    
    IOReturn allocFwPagingMem(struct iwm_drv *driver, const struct iwm_fw_img *image);
    
    IOReturn saveFwPaging(struct iwm_drv* driver, const struct iwm_fw_img *image);
    
    IOReturn sendPagingCmd(struct iwm_drv* driver, const struct iwm_fw_img *image);
    
    //end
    
    struct FirmwareLoadProgress {
        AppleIntelIWM* driver;
        OSData* firmwareData;
    };

    struct iwm_drv* driver;
    IOLock* firmwareLoadLock;
};

#define KASSERT(a, b, ...) do { if(!(a)) { IWM_KERNLOG(b, ##__VA_ARGS__); } } while(0); //TODO: CHECK THIS

#define MYNAME "AppleIntelIWM"
#define IWM_KERNLOG(buffer, ...) do { IOLog("%s::" buffer, MYNAME, ##__VA_ARGS__); } while(0)
//#define IWM_KERNLOG(buffer, ...) //os_log(OS_LOG_DEFAULT, "%s::" buffer, "AppleIntelIWMDebug", ##__VA_ARGS__)
#define super IO80211Controller
#define IWM_DLOG(type, buffer, ...) IOLog("%s::" buffer, MYNAME, ##__VA_ARGS__)
#define IWM_DEBUG_RESET
#define IWM_DEBUG_ERROR
#define IWM_DEBUG_INFO

