#ifndef EASYMESH_AGENT_H
#define EASYMESH_AGENT_H

#include "easymesh-link.h"
#include "easymesh-radio.h"
#include <vector>

enum class AgentRole { FRONTHAUL_ONLY, BACKHAUL_ONLY, HYBRID };

class EasyMeshAgent : public SimpleRefCount<EasyMeshAgent> {
public:
    EasyMeshAgent(uint32_t id, Ptr<Node> node, AgentRole role, Vector position);

    void AddRadio(Ptr<EasyMeshRadio> radio)        { m_radios.push_back(radio); }
    void SetBackhaulLink(Ptr<EasyMeshLink> link)   { m_backhaulLink = link; }

    // Core getters / setters
    uint32_t  GetId()       const { return m_id; }
    Ptr<Node> GetNode()     const { return m_node; }
    AgentRole GetRole()     const { return m_role; }
    Vector    GetPosition() const { return m_position; }

    bool IsOnline() const    { return m_online; }
    void SetOnline(bool v)   { m_online = v; }

    // Backhaul parent: -1 = Controller, otherwise an Agent id
    int32_t GetParent() const        { return m_parent; }
    void    SetParent(int32_t p)     { m_parent = p; }
    double  GetBackhaulCapacity() const  { return m_backhaulCapacity; }   // bottleneck Mbps to Controller
    void    SetBackhaulCapacity(double v) { m_backhaulCapacity = v; }

    const std::vector<Ptr<EasyMeshRadio>>& GetRadios() const { return m_radios; }
    Ptr<EasyMeshRadio> GetRadio(EmBand band) const;
    Ptr<EasyMeshLink>  GetBackhaulLink() const { return m_backhaulLink; }

    uint32_t GetClientCount() const;
    double   GetLoad() const;   // highest channel utilization over its radios

    void IncrementSteeredOut() { m_steeredOut++; }
    void IncrementSteeredIn()  { m_steeredIn++;  }
    uint32_t GetSteeredOut() const { return m_steeredOut; }
    uint32_t GetSteeredIn()  const { return m_steeredIn;  }

private:
    uint32_t  m_id;
    Ptr<Node> m_node;
    AgentRole m_role;
    Vector    m_position;

    bool    m_online = true;
    int32_t m_parent = -1;
    double  m_backhaulCapacity = 0.0;

    std::vector<Ptr<EasyMeshRadio>> m_radios;
    Ptr<EasyMeshLink>               m_backhaulLink;

    uint32_t m_steeredOut = 0;
    uint32_t m_steeredIn  = 0;
};

#endif // EASYMESH_AGENT_H
