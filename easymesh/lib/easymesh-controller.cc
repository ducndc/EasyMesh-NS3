#include "easymesh-controller.h"
#include "ns3/log.h"
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshController");

EasyMeshController::EasyMeshController(uint32_t id, Ptr<Node> node)
    : m_id(id), m_node(node) {}

void 
EasyMeshController::RegisterAgent(Ptr<EasyMeshAgent> agent) 
{
    m_agents.push_back(agent);
}

void 
EasyMeshController::CollectApMetrics() 
{
    NS_LOG_INFO("[Controller] Collecting metrics from " << m_agents.size() << " agents.");
    for (auto& agent : m_agents) 
    {
        // Simulate load: 15% per client
        agent->SetLoad(std::min(1.0, agent->GetClients().size() * 0.15));
    }
}

void 
EasyMeshController::RunSteeringEngine() 
{
    NS_LOG_INFO("[Controller] Running Steering Engine Tick");
    for (auto& agent : m_agents) 
    {
        if (agent->GetLoad() > 0.75) 
        {   
            // Load threshold
            NS_LOG_INFO("[Controller] Agent " << agent->GetId() << " is overloaded. Attempting steering...");
            // Steering logic would go here
        }
    }
}