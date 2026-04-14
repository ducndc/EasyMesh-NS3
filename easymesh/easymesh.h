#ifndef EASYMESH_H
#define EASYMESH_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

// Include the individual component headers
#include "easymesh-link.h"
#include "easymesh-agent.h"
#include "easymesh-controller.h"

#include <vector>

using namespace ns3;

class EasyMeshSimulation {
public:
    EasyMeshSimulation();
    ~EasyMeshSimulation(); // Added destructor back to declaration

    void BuildTopology();
    void InstallProtocolStack();
    void InstallApplications();
    void ScheduleEvents();
    void Run();

    // Helper used in main.cc
    double ComputeDistance(Vector a, Vector b);

private:
    void CreateNodes();
    void CreateWifiChannels();
    void CreateBackhaulLinks();
    void CreateFronthaulLinks();
    void CreateEthernetBackhaul();
    void InstallMobility(); // Added missing declaration

    NodeContainer m_controllerNode, m_agentNodes, m_clientNodes;
    Ptr<EasyMeshController> m_controller;
    std::vector<Ptr<EasyMeshAgent>> m_agents;
    std::vector<Ptr<EasyMeshLink>>  m_links;
    
    // Wifi Helpers needed in main.cc
    WifiHelper m_wifiHelper;
    WifiMacHelper m_macHelper;
    YansWifiPhyHelper m_phyHelper;
    YansWifiChannelHelper m_channelHelper;

    double m_duration = 60.0;
    uint32_t m_numClients = 8;

    static const Vector AGENT_POSITIONS[4];
    static const Vector CONTROLLER_POS;
};

#endif