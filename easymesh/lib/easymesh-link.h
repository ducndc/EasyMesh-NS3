#ifndef EASYMESH_LINK_H
#define EASYMESH_LINK_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"

using namespace ns3;

enum class LinkType { FRONTHAUL, BACKHAUL, ETHERNET_BACKHAUL };

class EasyMeshLink : public SimpleRefCount<EasyMeshLink> {
public:
    EasyMeshLink(uint32_t id, Ptr<Node> src, Ptr<Node> dst,
                 LinkType type, double freq);

    void UpdateMetrics(double rssi, double snr, uint32_t mcs,
                       double throughputMbps, double airtime);
    void PrintStatus() const;

    // Getters
    uint32_t GetId()          const { return m_id; }
    Ptr<Node> GetSrc()        const { return m_src; }
    Ptr<Node> GetDst()        const { return m_dst; }
    LinkType  GetType()       const { return m_type; }
    double    GetFrequency()  const { return m_frequency; }
    double    GetRssi()       const { return m_rssi; }
    double    GetSnr()        const { return m_snr; }
    uint32_t  GetMcs()        const { return m_mcs; }
    double    GetThroughput() const { return m_throughputMbps; }
    double    GetAirtime()    const { return m_airtime; }
    bool      IsActive()      const { return m_active; }
    void      SetActive(bool a)     { m_active = a; }

    // Traffic counters (updated by TrafficManager)
    void   AddRxBytes(uint64_t b) { m_rxBytes += b; }
    void   AddTxBytes(uint64_t b) { m_txBytes += b; }
    uint64_t GetRxBytes() const   { return m_rxBytes; }
    uint64_t GetTxBytes() const   { return m_txBytes; }

private:
    uint32_t  m_id;
    Ptr<Node> m_src, m_dst;
    LinkType  m_type;
    double    m_frequency;
    double    m_rssi      = -100.0;
    double    m_snr       = 0.0;
    double    m_throughputMbps = 0.0;
    double    m_airtime   = 0.0;
    uint32_t  m_mcs       = 0;
    bool      m_active    = true;
    uint64_t  m_rxBytes   = 0;
    uint64_t  m_txBytes   = 0;
};

#endif // EASYMESH_LINK_H