#include "easymesh.h"
#include "easymesh-link.h"
#include "easymesh-agent.h"
#include "easymesh-controller.h"
#include "ns3/csma-module.h"
#include "ns3/mobility-model.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshSim");

// Static data initialization
const Vector EasyMeshSimulation::CONTROLLER_POS = Vector(25.0, 25.0, 0.0);
const Vector EasyMeshSimulation::AGENT_POSITIONS[4] = {
    Vector(5.0, 25.0, 0.0),   // Agent 0 – West
    Vector(25.0, 45.0, 0.0),  // Agent 1 – North
    Vector(45.0, 25.0, 0.0),  // Agent 2 – East
    Vector(25.0, 5.0, 0.0)    // Agent 3 – South
};

EasyMeshSimulation::EasyMeshSimulation() {}
EasyMeshSimulation::~EasyMeshSimulation() {}

double 
EasyMeshSimulation::ComputeDistance(Vector a, Vector b) 
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

void 
EasyMeshSimulation::CreateNodes() 
{
    m_controllerNode.Create(1);
    m_agentNodes.Create(4);
    m_clientNodes.Create(m_numClients);
    NS_LOG_INFO("Created nodes: 1 controller + 4 agents + " << m_numClients << " clients");
}

void 
EasyMeshSimulation::InstallMobility() 
{
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    Ptr<ListPositionAllocator> ctrlAlloc = CreateObject<ListPositionAllocator>();
    ctrlAlloc->Add(CONTROLLER_POS);
    mob.SetPositionAllocator(ctrlAlloc);
    mob.Install(m_controllerNode);

    Ptr<ListPositionAllocator> agentAlloc = CreateObject<ListPositionAllocator>();
    for (int i = 0; i < 4; i++) agentAlloc->Add(AGENT_POSITIONS[i]);
    mob.SetPositionAllocator(agentAlloc);
    mob.Install(m_agentNodes);

    mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                         "Bounds", RectangleValue(Rectangle(0.0, 50.0, 0.0, 50.0)),
                         "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=3.0]"),
                         "Distance", DoubleValue(5.0));
    
    Ptr<RandomBoxPositionAllocator> clientAlloc = CreateObject<RandomBoxPositionAllocator>();
    clientAlloc->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=45.0]"));
    clientAlloc->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=45.0]"));
    clientAlloc->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
    mob.SetPositionAllocator(clientAlloc);
    mob.Install(m_clientNodes);
}

void 
EasyMeshSimulation::CreateWifiChannels() 
{
    m_channelHelper = YansWifiChannelHelper::Default();
    m_channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    m_channelHelper.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                       "Exponent", DoubleValue(3.0),
                                       "ReferenceDistance", DoubleValue(1.0),
                                       "ReferenceLoss", DoubleValue(40.0));
}

void 
EasyMeshSimulation::CreateBackhaulLinks() 
{
    m_wifiHelper.SetStandard(WIFI_STANDARD_80211ac);
    m_phyHelper = YansWifiPhyHelper();
    m_phyHelper.SetChannel(m_channelHelper.Create());
    m_macHelper.SetType("ns3::AdhocWifiMac");

    for (uint32_t i = 1; i < 4; i++) 
    {
        NodeContainer pair;
        pair.Add(m_agentNodes.Get(0));
        pair.Add(m_agentNodes.Get(i));
        m_wifiHelper.Install(m_phyHelper, m_macHelper, pair);
        
        Ptr<EasyMeshLink> link = Create<EasyMeshLink>(i, m_agentNodes.Get(0), m_agentNodes.Get(i), LinkType::BACKHAUL, 5.0);
        m_links.push_back(link);
    }
}

void 
EasyMeshSimulation::CreateEthernetBackhaul() 
{
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1Gbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(100)));

    NodeContainer pair;
    pair.Add(m_controllerNode.Get(0));
    pair.Add(m_agentNodes.Get(0));
    csma.Install(pair);

    Ptr<EasyMeshLink> ethLink = Create<EasyMeshLink>(100, m_controllerNode.Get(0), m_agentNodes.Get(0), LinkType::ETHERNET_BACKHAUL, 0.0);
    m_links.push_back(ethLink);
}

void 
EasyMeshSimulation::CreateFronthaulLinks() 
{
    WifiHelper frontWifi;
    frontWifi.SetStandard(WIFI_STANDARD_80211n);
    YansWifiPhyHelper frontPhy;
    frontPhy.SetChannel(m_channelHelper.Create());
    WifiMacHelper frontMac;

    for (uint32_t a = 0; a < 4; a++) 
    {
        frontMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(Ssid("EasyMesh-AP" + std::to_string(a))));
        frontWifi.Install(frontPhy, frontMac, m_agentNodes.Get(a));
    }
}

void 
EasyMeshSimulation::BuildTopology() 
{
    CreateNodes();
    InstallMobility();
    CreateWifiChannels();
    CreateEthernetBackhaul();
    CreateBackhaulLinks();
    CreateFronthaulLinks();

    m_controller = Create<EasyMeshController>(0, m_controllerNode.Get(0));

    for (uint32_t i = 0; i < 4; i++) 
    {
        auto agent = Create<EasyMeshAgent>(i, m_agentNodes.Get(i), AgentRole::HYBRID, AGENT_POSITIONS[i]);
        m_agents.push_back(agent);
        m_controller->RegisterAgent(agent);
    }
}

void 
EasyMeshSimulation::InstallProtocolStack() 
{
    InternetStackHelper internet;
    internet.Install(m_controllerNode);
    internet.Install(m_agentNodes);
    internet.Install(m_clientNodes);
}

void 
EasyMeshSimulation::InstallApplications() 
{
    UdpEchoServerHelper echoServer(7);
    ApplicationContainer srv = echoServer.Install(m_controllerNode.Get(0));
    srv.Start(Seconds(1.0));
}

void 
EasyMeshSimulation::ScheduleEvents() 
{
    Simulator::Schedule(Seconds(5.0), &EasyMeshController::CollectApMetrics, m_controller);
    Simulator::Schedule(Seconds(10.0), &EasyMeshController::RunSteeringEngine, m_controller);
}

void 
EasyMeshSimulation::Run() 
{
    Simulator::Stop(Seconds(m_duration));
    Simulator::Run();
    Simulator::Destroy();
}

int 
main(int argc, char* argv[]) 
{
    LogComponentEnable("EasyMeshSim", LOG_LEVEL_INFO);
    EasyMeshSimulation sim;
    sim.BuildTopology();
    sim.InstallProtocolStack();
    sim.InstallApplications();
    sim.ScheduleEvents();
    sim.Run();
    return 0;
}