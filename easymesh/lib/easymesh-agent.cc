#include "easymesh-agent.h"

#include <algorithm>
#include <iostream>

using namespace ns3;

EasyMeshAgent::EasyMeshAgent(uint32_t id, Ptr<Node> node, AgentRole role, Vector pos)
    : m_id(id), m_node(node), m_role(role), m_position(pos) {}

void 
EasyMeshAgent::AddFronthaulLink(Ptr<EasyMeshLink> link) 
{
    if (std::find(m_fronthaulLinks.begin(), m_fronthaulLinks.end(), link) == m_fronthaulLinks.end())
    {
        m_fronthaulLinks.push_back(link);
    }
}

void 
EasyMeshAgent::AddBackhaulLink(Ptr<EasyMeshLink> link) 
{
    m_backhaulLinks.push_back(link);
}

void 
EasyMeshAgent::RemoveFronthaulLink(Ptr<EasyMeshLink> link)
{
    m_fronthaulLinks.erase(std::remove(m_fronthaulLinks.begin(), m_fronthaulLinks.end(), link),
                           m_fronthaulLinks.end());
}

void 
EasyMeshAgent::AddClient(uint32_t clientId) 
{
    if (std::find(m_clients.begin(), m_clients.end(), clientId) == m_clients.end()) 
    {
        m_clients.push_back(clientId);
    }
}

void 
EasyMeshAgent::RemoveClient(uint32_t clientId) 
{
    m_clients.erase(std::remove(m_clients.begin(), m_clients.end(), clientId), m_clients.end());
}