#include "easymesh-agent.h"

#include <algorithm>

using namespace ns3;

EasyMeshAgent::EasyMeshAgent(uint32_t id, Ptr<Node> node, AgentRole role, Vector pos)
    : m_id(id), m_node(node), m_role(role), m_position(pos) {}

Ptr<EasyMeshRadio>
EasyMeshAgent::GetRadio(EmBand band) const
{
    for (const auto& radio : m_radios)
    {
        if (radio->GetBand() == band)
        {
            return radio;
        }
    }
    return nullptr;
}

uint32_t
EasyMeshAgent::GetClientCount() const
{
    uint32_t count = 0;
    for (const auto& radio : m_radios)
    {
        count += radio->GetClientCount();
    }
    return count;
}

double
EasyMeshAgent::GetLoad() const
{
    double load = 0.0;
    for (const auto& radio : m_radios)
    {
        load = std::max(load, radio->GetChanUtil());
    }
    return load;
}
