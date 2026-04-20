#ifndef EASYMESH_H
#define EASYMESH_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/csma-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/bulk-send-helper.h"
#include "ns3/bridge-module.h"

#include "lib/easymesh-link.h"
#include "lib/easymesh-agent.h"
#include "lib/easymesh-controller.h"
#include "lib/traffic-stats.h"
#include "parameters.h"

#include <vector>
#include <map>

using namespace ns3;

class EasyMeshSimulation {
public:
    EasyMeshSimulation();
    ~EasyMeshSimulation();

    // ── Build phases ──────────────────────────────────────────
    void BuildTopology();
    void InstallProtocolStack();
    void InstallApplications();     // ← UDP CBR + TCP bulk + BH stress
    void ScheduleEvents();
    void Run();
    void PrintResults();            // ← topology + steering + flow stats
    void ConfigureAnimation(AnimationInterface& anim);
    void UpdateAnimationLinks(AnimationInterface* anim);

    // ── CLI overrides ─────────────────────────────────────────
    void SetDuration(double s)   { m_duration   = s; }
    void SetScenarioRun(uint32_t run) { m_scenarioRun = run; }
    void SetNumClients(uint32_t n)
    {
        m_numClients = n;
        m_clientIp.resize(n, Ipv4Address::GetAny());
        m_clientToAgent.resize(n, 0);
        m_clientFronthaulDevices.resize(n);
        m_clientFronthaulIfIndex.resize(n, 0);
    }
    void EnablePcap(bool e)      { m_pcap       = e; }

    double ComputeDistance(Vector a, Vector b) const;

private:
    // ── Topology builders ─────────────────────────────────────
    void CreateNodes();
    void InstallMobility();
    void CreateWifiChannels();
    void CreateBackhaulLinks();
    void CreateFronthaulLinks();
    void CreateEthernetBackhaul();
    void AssignClientsToAgents();
    void GenerateRandomAgentPositions();
    void OptimizeNetworkTopology();
    void BuildBackhaulTree();
    uint32_t SelectBestUplinkAgent(uint32_t clientId) const;
    Ptr<EasyMeshLink> FindBackhaulLinkForAgent(uint32_t agentId) const;
    Ptr<EasyMeshLink> FindFronthaulLinkForClient(uint32_t clientId) const;
    double EstimateLinkRssi(Vector a, Vector b) const;
    double EstimateLinkThroughput(double rssi) const;
    Vector GetAgentPosition(uint32_t agentId) const;
    uint32_t FindAgentForFronthaulBssid(Mac48Address bssid) const;
    void UpdateClientAssociation(uint32_t clientId, uint32_t agentId);
    void RebuildClientRoutes();
    void SyncClientAssociations();

    // ── Traffic installers ────────────────────────────────────
    void InstallUdpUplink();        // STA → Controller  (CBR)
    void InstallUdpDownlink();      // Controller → STA  (CBR)
    void InstallTcpUplink();        // STA (even) → Controller (BulkSend)
    void InstallBackhaulStress();   // Agent → Agent 0   (UDP CBR)

    // ── NS-3 containers ──────────────────────────────────────
    NodeContainer m_controllerNode;
    NodeContainer m_agentNodes;
    NodeContainer m_clientNodes;

    NetDeviceContainer m_ethDevices;
    NetDeviceContainer m_backhaulDevices;
    NetDeviceContainer m_fronthaulDevices;
    std::vector<Ptr<NetDevice>> m_agentFronthaulDevices;
    std::vector<Ptr<NetDevice>> m_clientFronthaulDevices;

    Ipv4InterfaceContainer m_ctrlIfaces;        // eth segment
    Ipv4InterfaceContainer m_backhaulIfaces;
    std::vector<Ipv4InterfaceContainer> m_fronthaulIfaces; // one per agent

    // IP address bookkeeping
    Ipv4Address m_controllerIp;
    std::vector<Ipv4Address> m_controllerBackhaulIp;
    std::vector<Ipv4Address> m_agentIp;     // backhaul side
    std::vector<Ipv4Address> m_agentFronthaulIp;
    std::vector<Ipv4Address> m_clientIp;    // fronthaul side
    std::vector<uint32_t>    m_clientToAgent;
    std::vector<uint32_t>    m_parentIfIndex;
    std::vector<uint32_t>    m_agentBackhaulIfIndex;
    std::vector<uint32_t>    m_agentFronthaulIfIndex;
    std::vector<uint32_t>    m_clientFronthaulIfIndex;
    std::vector<Vector>      m_agentPositions;
    std::vector<int32_t>     m_agentParent;

    // ── EasyMesh logical objects ──────────────────────────────
    Ptr<EasyMeshController>          m_controller;
    std::vector<Ptr<EasyMeshAgent>>  m_agents;
    std::vector<Ptr<EasyMeshLink>>   m_links;

    // ── Wi-Fi helpers (backhaul) ──────────────────────────────
    WifiHelper          m_wifiHelper;
    WifiMacHelper       m_macHelper;
    YansWifiPhyHelper   m_phyHelper;
    YansWifiChannelHelper m_channelHelper;
    Ptr<YansWifiChannel> m_sharedWifiChannel;

    // ── FlowMonitor ───────────────────────────────────────────
    Ptr<FlowMonitor>   m_monitor;
    FlowMonitorHelper  m_monHelper;
    TrafficStats       m_trafficStats;

    // ── Config ────────────────────────────────────────────────
    double   m_duration   = DURATION;
    uint32_t m_numClients = NUM_STA;
    uint32_t m_scenarioRun = 1;
    bool     m_pcap       = false;

    // ── Static topology ───────────────────────────────────────
    static const Vector AGENT_POSITIONS[NUM_AGENT];
    static const Vector CONTROLLER_POS;
};

#endif // EASYMESH_H