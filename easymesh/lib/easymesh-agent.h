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
    void AddClient(uint32_t clientId);
    void RemoveClient(uint32_t clientId);
    
    // Getters/Setters
    uint32_t GetId() const { return m_id; }
    double GetLoad() const { return m_load; }
    void SetLoad(double load) { m_load = load; }
    const std::vector<uint32_t>& GetClients() const { return m_clients; }

private:
    uint32_t m_id;
    Ptr<Node> m_node;
    AgentRole m_role;
    Vector m_position;
    std::vector<Ptr<EasyMeshLink>> m_fronthaulLinks, m_backhaulLinks;
    std::vector<uint32_t> m_clients;
    double m_load = 0.0;
};

#endif