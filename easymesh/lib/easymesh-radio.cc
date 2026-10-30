#include "easymesh-radio.h"

#include <algorithm>

using namespace ns3;

EasyMeshRadio::EasyMeshRadio(uint32_t agentId,
                             EmBand band,
                             Ptr<WifiNetDevice> device,
                             uint8_t linkId,
                             EmChannel channel)
    : m_agentId(agentId), m_band(band), m_device(device), m_linkId(linkId), m_channel(channel) {}

Mac48Address
EasyMeshRadio::GetBssid() const
{
    // Same as the device address for a single-link AP; the affiliated AP's
    // own address for a link of an AP MLD
    return m_device->GetMac()->GetFrameExchangeManager(m_linkId)->GetAddress();
}

void
EasyMeshRadio::StartMonitoring()
{
    m_windowStart = Simulator::Now();
    m_busyInWindow = Seconds(0);

    GetPhy()->GetState()->TraceConnectWithoutContext(
        "State", MakeCallback(&EasyMeshRadio::OnPhyState, this));

    // The MAC traces (MacRx / MacTx) are per device, i.e. shared by all the
    // links of an AP MLD, so count data on this link's PHY instead:
    // received unicast data addressed to this AP (uplink) and first
    // transmissions of unicast data sent by it (downlink).
    GetPhy()->TraceConnectWithoutContext("MonitorSnifferRx",
                                         MakeCallback(&EasyMeshRadio::OnSnifferRx, this));
    GetPhy()->TraceConnectWithoutContext("MonitorSnifferTx",
                                         MakeCallback(&EasyMeshRadio::OnSnifferTx, this));
}

uint32_t
EasyMeshRadio::DataBytes(Ptr<const Packet> packet, const MpduInfo& aMpdu, WifiMacHeader& hdr)
{
    auto copy = packet->Copy();
    uint32_t mpduSize = copy->GetSize();
    if (aMpdu.type != NORMAL_MPDU)
    {
        AmpduSubframeHeader subframe;
        copy->RemoveHeader(subframe);
        mpduSize = subframe.GetLength();
    }
    copy->RemoveHeader(hdr);
    if (!hdr.IsData() || !hdr.HasData() || hdr.GetAddr1().IsGroup())
    {
        return 0;
    }
    const uint32_t overhead = hdr.GetSerializedSize() + WIFI_MAC_FCS_LENGTH;
    return mpduSize > overhead ? mpduSize - overhead : 0;
}

void
EasyMeshRadio::OnSnifferRx(Ptr<const Packet> packet, uint16_t, WifiTxVector, MpduInfo aMpdu,
                           SignalNoiseDbm, uint16_t)
{
    WifiMacHeader hdr;
    const uint32_t bytes = DataBytes(packet, aMpdu, hdr);
    if (bytes && hdr.GetAddr1() == GetBssid())
    {
        m_ulBytes += bytes;
    }
}

void
EasyMeshRadio::OnSnifferTx(Ptr<const Packet> packet, uint16_t, WifiTxVector, MpduInfo aMpdu,
                           uint16_t)
{
    WifiMacHeader hdr;
    const uint32_t bytes = DataBytes(packet, aMpdu, hdr);
    if (bytes && !hdr.IsRetry() && hdr.GetAddr2() == GetBssid())
    {
        m_dlBytes += bytes;
    }
}

void
EasyMeshRadio::OnPhyState(Time start, Time duration, WifiPhyState state)
{
    if (state == WifiPhyState::IDLE || state == WifiPhyState::SLEEP || state == WifiPhyState::OFF)
    {
        return;
    }
    // Only count the part of the period that falls inside the current window
    const Time end = start + duration;
    const Time from = std::max(start, m_windowStart);
    if (end > from)
    {
        m_busyInWindow += end - from;
    }
}

void
EasyMeshRadio::SampleUtilization()
{
    const Time now = Simulator::Now();
    const Time window = now - m_windowStart;
    if (window.IsStrictlyPositive())
    {
        m_chanUtil = std::min(1.0, m_busyInWindow.GetSeconds() / window.GetSeconds());
        m_utilSum += m_chanUtil;
        m_utilSamples++;
    }
    m_windowStart = now;
    m_busyInWindow = Seconds(0);
}

void
EasyMeshRadio::SetChannel(const EmChannel& ch, bool countSwitch)
{
    if (ch == m_channel)
    {
        return;
    }
    m_channel = ch;
    if (countSwitch)
    {
        m_channelSwitches++;
    }
    GetPhy()->SetAttribute("ChannelSettings", StringValue(ch.ToSettings()));
}

void
EasyMeshRadio::AddClient(uint32_t clientId)
{
    if (std::find(m_clients.begin(), m_clients.end(), clientId) == m_clients.end())
    {
        m_clients.push_back(clientId);
    }
}

void
EasyMeshRadio::RemoveClient(uint32_t clientId)
{
    m_clients.erase(std::remove(m_clients.begin(), m_clients.end(), clientId), m_clients.end());
}
