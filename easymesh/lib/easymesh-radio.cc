#include "easymesh-radio.h"

#include <algorithm>

using namespace ns3;

EasyMeshRadio::EasyMeshRadio(uint32_t agentId, EmBand band, Ptr<WifiNetDevice> device, EmChannel channel)
    : m_agentId(agentId), m_band(band), m_device(device), m_channel(channel) {}

Mac48Address
EasyMeshRadio::GetBssid() const
{
    return m_device->GetMac()->GetAddress();
}

void
EasyMeshRadio::StartMonitoring()
{
    m_windowStart = Simulator::Now();
    m_busyInWindow = Seconds(0);

    GetPhy()->GetState()->TraceConnectWithoutContext(
        "State", MakeCallback(&EasyMeshRadio::OnPhyState, this));

    // MacRx: frames from associated STAs handed up (uplink);
    // MacTx: frames from the IP stack queued towards STAs (downlink).
    m_device->GetMac()->TraceConnectWithoutContext("MacRx",
                                                   MakeCallback(&EasyMeshRadio::OnMacRx, this));
    m_device->GetMac()->TraceConnectWithoutContext("MacTx",
                                                   MakeCallback(&EasyMeshRadio::OnMacTx, this));
}

void
EasyMeshRadio::OnMacRx(Ptr<const Packet> packet)
{
    m_ulBytes += packet->GetSize();
}

void
EasyMeshRadio::OnMacTx(Ptr<const Packet> packet)
{
    m_dlBytes += packet->GetSize();
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
