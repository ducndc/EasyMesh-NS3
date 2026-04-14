#include "easymesh-link.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace ns3;

EasyMeshLink::EasyMeshLink(uint32_t id, Ptr<Node> src, Ptr<Node> dst, LinkType type, double freq)
    : m_id(id), m_src(src), m_dst(dst), m_type(type),
      m_frequency(freq), m_rssi(-100), m_snr(0),
      m_throughputMbps(0), m_airtime(0), m_mcs(0), m_active(true) {}

void 
EasyMeshLink::UpdateMetrics(double rssi, double snr, uint32_t mcs, double throughput, double airtime) 
{
    m_rssi = rssi;
    m_snr = snr;
    m_mcs = mcs;
    m_throughputMbps = throughput;
    m_airtime = airtime;
}

void 
EasyMeshLink::PrintStatus() const 
{
    std::string typeStr = (m_type == LinkType::FRONTHAUL) ? "FRONTHAUL" : 
                          (m_type == LinkType::BACKHAUL) ? "BACKHAUL(WiFi)" : "BACKHAUL(ETH)";
    
    std::cout << "  Link[" << m_id << "] " << typeStr
              << "  Node" << m_src->GetId() << "->Node" << m_dst->GetId()
              << "  RSSI=" << std::fixed << std::setprecision(1) << m_rssi << "dBm"
              << "  Active=" << (m_active ? "YES" : "NO") << "\n";
}