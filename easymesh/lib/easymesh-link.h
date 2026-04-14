#ifndef EASYMESH_LINK_H
#define EASYMESH_LINK_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"

using namespace ns3;

enum class LinkType { FRONTHAUL, BACKHAUL, ETHERNET_BACKHAUL };

class EasyMeshLink : public SimpleRefCount<EasyMeshLink> {
public:
    EasyMeshLink(uint32_t id, Ptr<Node> src, Ptr<Node> dst, LinkType type, double freq);

    void UpdateMetrics(double rssi, double snr, uint32_t mcs, double throughputMbps, double airtime);
    void PrintStatus() const;

    // Getters
    uint32_t GetId() const { return m_id; }
    Ptr<Node> GetSrc() const { return m_src; }
    Ptr<Node> GetDst() const { return m_dst; }
    LinkType GetType() const { return m_type; }
    double GetRssi() const { return m_rssi; }
    bool IsActive() const { return m_active; }
    void SetActive(bool a) { m_active = a; }

private:
    uint32_t m_id;
    Ptr<Node> m_src, m_dst;
    LinkType m_type;
    double m_frequency, m_rssi, m_snr, m_throughputMbps, m_airtime;
    uint32_t m_mcs;
    bool m_active;
};

#endif