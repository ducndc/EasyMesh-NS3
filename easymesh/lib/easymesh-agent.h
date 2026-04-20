#ifndef EASYMESH_AGENT_H
#define EASYMESH_AGENT_H

#include "easymesh-link.h"
#include <vector>

enum class AgentRole { FRONTHAUL_ONLY, BACKHAUL_ONLY, HYBRID };

class EasyMeshAgent : public SimpleRefCount<EasyMeshAgent> {
public:
    EasyMeshAgent(uint32_t id, Ptr<Node> node, AgentRole role, Vector position);

    void AddFronthaulLink(Ptr<EasyMeshLink> link);
    void AddBackhaulLink(Ptr<EasyMeshLink> link);
    void RemoveFronthaulLink(Ptr<EasyMeshLink> link);
    void AddClient(uint32_t clientId);
    void RemoveClient(uint32_t clientId);

    // Core getters / setters
    uint32_t  GetId()       const { return m_id; }
    Ptr<Node> GetNode()     const { return m_node; }
    AgentRole GetRole()     const { return m_role; }
    Vector    GetPosition() const { return m_position; }

    double GetLoad()    const { return m_load; }
    void   SetLoad(double v)  { m_load = v; }

    double GetChanUtil()    const { return m_chanUtil; }
    void   SetChanUtil(double v)  { m_chanUtil = v; }

    const std::vector<uint32_t>&          GetClients()        const { return m_clients; }
    uint32_t                              GetClientCount()    const { return (uint32_t)m_clients.size(); }
    const std::vector<Ptr<EasyMeshLink>>& GetFronthaulLinks() const { return m_fronthaulLinks; }
    const std::vector<Ptr<EasyMeshLink>>& GetBackhaulLinks()  const { return m_backhaulLinks; }

    void IncrementSteeredOut() { m_steeredOut++; }
    void IncrementSteeredIn()  { m_steeredIn++;  }
    uint32_t GetSteeredOut() const { return m_steeredOut; }
    uint32_t GetSteeredIn()  const { return m_steeredIn;  }

    // Traffic accumulators (per-agent aggregate filled by TrafficManager)
    void   AddUlThroughput(double v) { m_ulTput += v; }
    void   AddDlThroughput(double v) { m_dlTput += v; }
    double GetUlThroughput()   const { return m_ulTput; }
    double GetDlThroughput()   const { return m_dlTput; }
    void   SetMeanDelay(double v)    { m_meanDelay = v; }
    double GetMeanDelay()      const { return m_meanDelay; }
    void   SetMeanPdr(double v)      { m_meanPdr = v; }
    double GetMeanPdr()        const { return m_meanPdr; }

private:
    uint32_t  m_id;
    Ptr<Node> m_node;
    AgentRole m_role;
    Vector    m_position;

    std::vector<Ptr<EasyMeshLink>> m_fronthaulLinks;
    std::vector<Ptr<EasyMeshLink>> m_backhaulLinks;
    std::vector<uint32_t>          m_clients;

    double   m_load      = 0.0;
    double   m_chanUtil  = 0.0;
    uint32_t m_steeredOut = 0;
    uint32_t m_steeredIn  = 0;

    // traffic stats filled externally
    double m_ulTput    = 0.0;
    double m_dlTput    = 0.0;
    double m_meanDelay = 0.0;
    double m_meanPdr   = 1.0;
};

#endif // EASYMESH_AGENT_H