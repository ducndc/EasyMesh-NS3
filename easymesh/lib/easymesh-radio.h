#ifndef EASYMESH_RADIO_H
#define EASYMESH_RADIO_H

#include "easymesh-types.h"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <vector>

using namespace ns3;

// ────────────────────────────────────────────────────────────────
// EasyMeshRadio – one fronthaul BSS (AP radio) of an Agent.
// Measures channel utilization from the PHY state machine (time not
// IDLE / SLEEP / OFF) and UL/DL bytes from the MAC, like the AP
// Metrics an EasyMesh Agent reports to the Controller.
// ────────────────────────────────────────────────────────────────
class EasyMeshRadio : public SimpleRefCount<EasyMeshRadio>
{
  public:
    EasyMeshRadio(uint32_t agentId, EmBand band, Ptr<WifiNetDevice> device, EmChannel channel);

    void StartMonitoring();
    void SampleUtilization();       // closes the current measurement window

    uint32_t           GetAgentId()  const { return m_agentId; }
    EmBand             GetBand()     const { return m_band; }
    Ptr<WifiNetDevice> GetDevice()   const { return m_device; }
    Ptr<WifiPhy>       GetPhy()      const { return m_device->GetPhy(); }
    Mac48Address       GetBssid()    const;
    const EmChannel&   GetChannel()  const { return m_channel; }
    // retunes the AP PHY; countSwitch=false for the initial plan
    void               SetChannel(const EmChannel& ch, bool countSwitch = true);

    Ipv4Address GetIp()      const { return m_ip; }
    uint32_t    GetIfIndex() const { return m_ifIndex; }
    void        SetIpv4(Ipv4Address ip, uint32_t ifIndex) { m_ip = ip; m_ifIndex = ifIndex; }

    void AddClient(uint32_t clientId);
    void RemoveClient(uint32_t clientId);
    const std::vector<uint32_t>& GetClients()     const { return m_clients; }
    uint32_t                     GetClientCount() const { return (uint32_t)m_clients.size(); }

    double   GetChanUtil()     const { return m_chanUtil; }
    double   GetMeanChanUtil() const { return m_utilSamples ? m_utilSum / m_utilSamples : 0.0; }
    uint64_t GetUlBytes()      const { return m_ulBytes; }
    uint64_t GetDlBytes()      const { return m_dlBytes; }
    uint32_t GetChannelSwitches() const { return m_channelSwitches; }

  private:
    void OnPhyState(Time start, Time duration, WifiPhyState state);
    void OnMacRx(Ptr<const Packet> packet);
    void OnMacTx(Ptr<const Packet> packet);

    uint32_t           m_agentId;
    EmBand             m_band;
    Ptr<WifiNetDevice> m_device;
    EmChannel          m_channel;
    Ipv4Address        m_ip;
    uint32_t           m_ifIndex = 0;

    std::vector<uint32_t> m_clients;

    Time     m_windowStart;
    Time     m_busyInWindow;
    double   m_chanUtil    = 0.0;
    double   m_utilSum     = 0.0;
    uint32_t m_utilSamples = 0;
    uint64_t m_ulBytes     = 0;
    uint64_t m_dlBytes     = 0;
    uint32_t m_channelSwitches = 0;
};

#endif // EASYMESH_RADIO_H
