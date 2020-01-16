//
//  AppleIntelWifi80211.cpp
//  AppleIntelIWM
//
//  Created by Felix McCuaig on 29/12/2019.
//  Copyright © 2019 Felix McCuaig. All rights reserved.
//

#include "AppleIntelIWM.h"

IOReturn AppleIntelIWM::configureReport(IOReportChannelList *, unsigned int, void *, void *)
{
    return KERN_SUCCESS;
}

IOReturn AppleIntelIWM::updateReport(IOReportChannelList *, unsigned int, void *, void *)
{
    return KERN_SUCCESS;
}

void AppleIntelIWM::requestPacketTx(void *, unsigned int)
{
    return;
}

IOReturn AppleIntelIWM::getHardwareAddressForInterface(IO80211Interface *, IOEthernetAddress *)
{
    return KERN_SUCCESS;
}

void AppleIntelIWM::inputMonitorPacket(mbuf_t, unsigned int, void *, unsigned long)
{
    return;
}

int AppleIntelIWM::outputRaw80211Packet(IO80211Interface *, mbuf_t)
{
    return 0;
}

int AppleIntelIWM::outputActionFrame(IO80211Interface *, mbuf_t)
{
    return 0;
}

int AppleIntelIWM::bpfOutputPacket(OSObject *, unsigned int, mbuf_t)
{
    return 0;
}

SInt32 AppleIntelIWM::monitorModeSetEnabled(IO80211Interface *, bool, unsigned int)
{
    return 0;
}

IO80211Interface* AppleIntelIWM::getNetworkInterface()
{
    return NULL;
}

SInt32 AppleIntelIWM::apple80211_ioctl(IO80211Interface *, IO80211VirtualInterface *, ifnet_t, unsigned long, void *)
{
    return 0;
}

SInt32 AppleIntelIWM::apple80211_ioctl(IO80211Interface *, ifnet_t, unsigned long, void *)
{
    return 0;
}

SInt32 AppleIntelIWM::apple80211Request(unsigned int, int, IO80211Interface *, void *)
{
    return 0;
}

SInt32 AppleIntelIWM::apple80211VirtualRequest(unsigned int, int, IO80211VirtualInterface *, void *)
{
    return 0;
}

SInt32 AppleIntelIWM::stopDMA()
{
    return 0;
}

UInt32 AppleIntelIWM::hardwareOutputQueueDepth(IO80211Interface*)
{
    return 0;
}

SInt32 AppleIntelIWM::performCountryCodeOperation(IO80211Interface*, IO80211CountryCodeOp)
{
    return 0;
}

bool AppleIntelIWM::useAppleRSNSupplicant(IO80211Interface*)
{
    return 0;
}

bool AppleIntelIWM::useAppleRSNSupplicant(IO80211VirtualInterface*)
{
    return 0;
}

void AppleIntelIWM::dataLinkLayerAttachComplete(IO80211Interface*)
{
    return;
}

SInt32 AppleIntelIWM::enableFeature(IO80211FeatureCode, void*)
{
    return 0;
}

SInt32 AppleIntelIWM::setVirtualHardwareAddress(IO80211VirtualInterface*, ether_addr*)
{
    return 0;
}

SInt32 AppleIntelIWM::enableVirtualInterface(IO80211VirtualInterface*)
{
    return 0;
}

SInt32 AppleIntelIWM::disableVirtualInterface(IO80211VirtualInterface*)
{
    return 0;
}

IOReturn AppleIntelIWM::requiresExplicitMBufRelease()
{
    return 0;
}

IOReturn AppleIntelIWM::flowIdSupported()
{
    return 0;
}

IO80211FlowQueueLegacy * AppleIntelIWM::requestFlowQueue(FlowIdMetadata const*)
{
    return 0;
}

void AppleIntelIWM::releaseFlowQueue(IO80211FlowQueue*)
{
    return;
}

IOReturn AppleIntelIWM::enablePacketTimestamping()
{
    return 0;
}

IOReturn AppleIntelIWM::disablePacketTimestamping()
{
    return 0;
}

UInt32 AppleIntelIWM::selfDiagnosticsReport(int, char const*, unsigned int)
{
    return 0;
}

UInt32 AppleIntelIWM::getDataQueueDepth(OSObject*)
{
    return 0;
}

mbuf_flags_t AppleIntelIWM::inputPacket(mbuf_t)
{
    return 0;
}

SInt32 AppleIntelIWM::apple80211_ioctl_get(IO80211Interface*, IO80211VirtualInterface*, ifnet_t, void*)
{
    return 0;
}

SInt32 AppleIntelIWM::apple80211_ioctl_set(IO80211Interface*, IO80211VirtualInterface*, ifnet_t, void*)
{
    return 0;
}

IO80211VirtualInterface * AppleIntelIWM::createVirtualInterface(ether_addr*, unsigned int)
{
    return 0;
}

bool AppleIntelIWM::attachVirtualInterface(IO80211VirtualInterface**, ether_addr*, unsigned int, bool)
{
    return 0;
}

bool AppleIntelIWM::detachVirtualInterface(IO80211VirtualInterface*, bool)
{
    return 0;
}






