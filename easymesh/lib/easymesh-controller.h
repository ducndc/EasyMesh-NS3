#ifndef EASYMESH_CONTROLLER_H
#define EASYMESH_CONTROLLER_H

#include "easymesh-agent.h"

class EasyMeshController : public SimpleRefCount<EasyMeshController> {
public:
    EasyMeshController(uint32_t id, Ptr<Node> node);

    void RegisterAgent(Ptr<EasyMeshAgent> agent);
    void RunSteeringEngine();
    void CollectApMetrics();
    
private:
    uint32_t m_id;
    Ptr<Node> m_node;
    std::vector<Ptr<EasyMeshAgent>> m_agents;
};

#endif