// easymesh.cc – EasyMesh Multi-AP Simulation with Traffic Evaluation
// ======================================================================

#include "easymesh.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshSim");

const Vector EasyMeshSimulation::CONTROLLER_POS   = Vector(25.0, 25.0, 0.0);
const Vector EasyMeshSimulation::AGENT_POSITIONS[4] = {
    Vector( 5.0, 25.0, 0.0),   // Agent 0 – West (root, Ethernet to controller)
    Vector(25.0, 45.0, 0.0),   // Agent 1 – North
    Vector(45.0, 25.0, 0.0),   // Agent 2 – East
    Vector(25.0,  5.0, 0.0),   // Agent 3 – South
};

EasyMeshSimulation::EasyMeshSimulation()  {
    m_agentIp.resize(NUM_AGENT, Ipv4Address::GetAny()); 
    m_clientIp.resize(NUM_STA, Ipv4Address::GetAny());
}

EasyMeshSimulation::~EasyMeshSimulation() {}

double EasyMeshSimulation::ComputeDistance(Vector a, Vector b) {
    return std::sqrt(std::pow(a.x-b.x,2) + std::pow(a.y-b.y,2) + std::pow(a.z-b.z,2));
}

void EasyMeshSimulation::CreateNodes() {
    m_controllerNode.Create(1);
    m_agentNodes.Create(4);
    m_clientNodes.Create(m_numClients);
}

void EasyMeshSimulation::InstallMobility() {
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> ca = CreateObject<ListPositionAllocator>();
    ca->Add(CONTROLLER_POS);
    mob.SetPositionAllocator(ca);
    mob.Install(m_controllerNode);

    Ptr<ListPositionAllocator> aa = CreateObject<ListPositionAllocator>();
    for (int i = 0; i < 4; i++) aa->Add(AGENT_POSITIONS[i]);
    mob.SetPositionAllocator(aa);
    mob.Install(m_agentNodes);

    mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds",    RectangleValue(Rectangle(0.0, 50.0, 0.0, 50.0)),
        "Speed",     StringValue("ns3::UniformRandomVariable[Min=1.0|Max=3.0]"),
        "Distance",  DoubleValue(5.0));

    Ptr<RandomBoxPositionAllocator> cla = CreateObject<RandomBoxPositionAllocator>();
    cla->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=45.0]"));
    cla->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=45.0]"));
    cla->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
    mob.SetPositionAllocator(cla);
    mob.Install(m_clientNodes);
}

void EasyMeshSimulation::CreateWifiChannels() {
    m_channelHelper = YansWifiChannelHelper::Default();
    m_channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    m_channelHelper.AddPropagationLoss(
        "ns3::LogDistancePropagationLossModel",
        "Exponent",          DoubleValue(3.0),
        "ReferenceDistance", DoubleValue(1.0),
        "ReferenceLoss",     DoubleValue(40.0));
}

void EasyMeshSimulation::CreateEthernetBackhaul() {
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1Gbps"));
    csma.SetChannelAttribute("Delay",    TimeValue(NanoSeconds(100)));

    NodeContainer pair;
    pair.Add(m_controllerNode.Get(0));
    pair.Add(m_agentNodes.Get(0));
    m_ethDevices = csma.Install(pair);

    Ptr<EasyMeshLink> link = Create<EasyMeshLink>(100, m_controllerNode.Get(0), m_agentNodes.Get(0), LinkType::ETHERNET_BACKHAUL, 0.0);
    link->UpdateMetrics(-30, 60, 11, 940.0, 0.01);
    m_links.push_back(link);
}

void EasyMeshSimulation::CreateBackhaulLinks() {
    m_wifiHelper.SetStandard(WIFI_STANDARD_80211ac);
    m_wifiHelper.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
    m_phyHelper = YansWifiPhyHelper();
    m_phyHelper.Set("TxPowerStart", DoubleValue(20.0));
    m_phyHelper.Set("TxPowerEnd",   DoubleValue(20.0));
    m_macHelper.SetType("ns3::AdhocWifiMac");

    for (uint32_t i = 1; i < 4; i++) {
        m_phyHelper.SetChannel(m_channelHelper.Create()); 
        NodeContainer pair;
        pair.Add(m_agentNodes.Get(0));
        pair.Add(m_agentNodes.Get(i));
        NetDeviceContainer devs = m_wifiHelper.Install(m_phyHelper, m_macHelper, pair);
        m_backhaulDevices.Add(devs);
    }
}

void EasyMeshSimulation::CreateFronthaulLinks() {
    WifiHelper fh;
    fh.SetStandard(WIFI_STANDARD_80211n);
    fh.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    for (uint32_t a = 0; a < 4; a++) {
        YansWifiPhyHelper fhPhy;
        fhPhy.SetChannel(m_channelHelper.Create()); 
        fhPhy.Set("TxPowerStart", DoubleValue(17.0));
        fhPhy.Set("TxPowerEnd",   DoubleValue(17.0));

        std::string ssidString = "EasyMesh-AP-" + std::to_string(a);
        Ssid ssid = Ssid(ssidString);

        WifiMacHelper fhMac;
        fhMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true));
        NetDeviceContainer apDev = fh.Install(fhPhy, fhMac, m_agentNodes.Get(a));
        m_fronthaulDevices.Add(apDev);

        fhMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(true));

        uint32_t base = (m_numClients * a) / 4;
        uint32_t top  = (m_numClients * (a+1)) / 4;

        NodeContainer staNodes;
        for (uint32_t c = base; c < top; c++) staNodes.Add(m_clientNodes.Get(c));

        if (staNodes.GetN() > 0) {
            NetDeviceContainer staDevs = fh.Install(fhPhy, fhMac, staNodes);
            m_fronthaulDevices.Add(staDevs);
        }
    }
}

void EasyMeshSimulation::BuildTopology() {
    CreateNodes();
    InstallMobility();
    CreateWifiChannels();
    CreateEthernetBackhaul();
    CreateBackhaulLinks();
    CreateFronthaulLinks();

    m_controller = Create<EasyMeshController>(0, m_controllerNode.Get(0));
    AgentRole roles[4] = {AgentRole::HYBRID, AgentRole::FRONTHAUL_ONLY, AgentRole::FRONTHAUL_ONLY, AgentRole::FRONTHAUL_ONLY};
    for (uint32_t i = 0; i < 4; i++) {
        auto agent = Create<EasyMeshAgent>(i, m_agentNodes.Get(i), roles[i], AGENT_POSITIONS[i]);
        m_agents.push_back(agent);
        m_controller->RegisterAgent(agent);
    }
    for (auto& l : m_links) m_controller->RegisterLink(l);
    AssignClientsToAgents();
}

void EasyMeshSimulation::AssignClientsToAgents() {
    for (uint32_t c = 0; c < m_numClients; c++) {
        Ptr<MobilityModel> mob = m_clientNodes.Get(c)->GetObject<MobilityModel>();
        double minDist = 1e9; uint32_t best = 0;
        for (uint32_t a = 0; a < 4; a++) {
            double d = ComputeDistance(mob->GetPosition(), AGENT_POSITIONS[a]);
            if (d < minDist) { minDist = d; best = a; }
        }
        m_agents[best]->AddClient(c);
    }
}

void EasyMeshSimulation::InstallProtocolStack() {
    InternetStackHelper inet;
    inet.Install(m_controllerNode);
    inet.Install(m_agentNodes);
    inet.Install(m_clientNodes);

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    m_ctrlIfaces = addr.Assign(m_ethDevices);
    m_controllerIp = m_ctrlIfaces.GetAddress(0);
    m_agentIp[0] = m_ctrlIfaces.GetAddress(1); 

    for (uint32_t i = 0; i < 3; i++) {
        std::stringstream ss; ss << "10.1." << (i + 1) << ".0";
        addr.SetBase(ss.str().c_str(), "255.255.255.0");
        NetDeviceContainer linkDevs;
        linkDevs.Add(m_backhaulDevices.Get(i * 2));
        linkDevs.Add(m_backhaulDevices.Get(i * 2 + 1));
        Ipv4InterfaceContainer bhIfaces = addr.Assign(linkDevs);
        m_agentIp[i+1] = bhIfaces.GetAddress(1);
    }

    for (uint32_t a = 0; a < 4; a++) {
        std::stringstream ss; ss << "192.168." << a << ".0";
        addr.SetBase(ss.str().c_str(), "255.255.255.0");
        NetDeviceContainer subnetDevs;
        for (uint32_t i = 0; i < m_fronthaulDevices.GetN(); i++) {
            if (m_fronthaulDevices.Get(i)->GetNode() == m_agentNodes.Get(a)) {
                subnetDevs.Add(m_fronthaulDevices.Get(i)); break;
            }
        }
        uint32_t base = (m_numClients * a) / 4;
        uint32_t top  = (m_numClients * (a + 1)) / 4;
        for (uint32_t c = base; c < top; c++) {
            for (uint32_t i = 0; i < m_fronthaulDevices.GetN(); i++) {
                if (m_fronthaulDevices.Get(i)->GetNode() == m_clientNodes.Get(c)) {
                    subnetDevs.Add(m_fronthaulDevices.Get(i)); break;
                }
            }
        }
        if (subnetDevs.GetN() > 0) {
            Ipv4InterfaceContainer fhIfaces = addr.Assign(subnetDevs);
            for (uint32_t idx = 1; idx < fhIfaces.GetN(); idx++) {
                m_clientIp[base + idx - 1] = fhIfaces.GetAddress(idx);
            }
        }
    }

    m_controllerNode.Get(0)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    for (uint32_t i = 0; i < 4; i++) {
        m_agentNodes.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
}

void EasyMeshSimulation::InstallUdpUplink() {
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT_UDP_UL));
    ApplicationContainer sinks = sink.Install(m_controllerNode.Get(0));
    sinks.Start(Seconds(0.5)); sinks.Stop(Seconds(m_duration));

    for (uint32_t c = 0; c < m_numClients; c++) {
        OnOffHelper src("ns3::UdpSocketFactory", InetSocketAddress(m_controllerIp, PORT_UDP_UL));
        src.SetAttribute("DataRate", StringValue(UL_DATA_RATE));
        src.SetAttribute("PacketSize", UintegerValue(UL_PKT_SIZE));
        src.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        src.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer app = src.Install(m_clientNodes.Get(c));
        app.Start(Seconds(UL_START + c * 0.1)); app.Stop(Seconds(m_duration - 1.0));
    }
}

void EasyMeshSimulation::InstallUdpDownlink() {
    for (uint32_t c = 0; c < m_numClients; c++) {
        if (m_clientIp[c] == Ipv4Address::GetAny()) continue;
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT_UDP_DL));
        sink.Install(m_clientNodes.Get(c)).Start(Seconds(0.5));

        OnOffHelper src("ns3::UdpSocketFactory", InetSocketAddress(m_clientIp[c], PORT_UDP_DL));
        src.SetAttribute("DataRate", StringValue(DL_DATA_RATE));
        src.SetAttribute("PacketSize", UintegerValue(DL_PKT_SIZE));
        ApplicationContainer app = src.Install(m_controllerNode.Get(0));
        app.Start(Seconds(DL_START + c * 0.1)); app.Stop(Seconds(m_duration - 1.0));
    }
}

void EasyMeshSimulation::InstallTcpUplink() {
    PacketSinkHelper tcpSink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT_TCP_UL));
    tcpSink.Install(m_controllerNode.Get(0)).Start(Seconds(0.5));

    for (uint32_t c = 0; c < m_numClients; c += 2) {
        BulkSendHelper bulk("ns3::TcpSocketFactory", InetSocketAddress(m_controllerIp, PORT_TCP_UL));
        ApplicationContainer app = bulk.Install(m_clientNodes.Get(c));
        app.Start(Seconds(TCP_START + c * 0.05)); app.Stop(Seconds(m_duration - 1.0));
    }
}

void EasyMeshSimulation::InstallBackhaulStress() {
    PacketSinkHelper bhSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT_UDP_BH));
    bhSink.Install(m_agentNodes.Get(0)).Start(Seconds(0.5));

    for (uint32_t i = 1; i < 4; i++) {
        OnOffHelper src("ns3::UdpSocketFactory", InetSocketAddress(m_agentIp[0], PORT_UDP_BH));
        src.SetAttribute("DataRate", StringValue(BH_DATA_RATE));
        src.SetAttribute("PacketSize", UintegerValue(BH_PKT_SIZE));
        ApplicationContainer app = src.Install(m_agentNodes.Get(i));
        app.Start(Seconds(BH_START + (i - 1) * 0.2)); app.Stop(Seconds(m_duration - 1.0));
    }
}

void EasyMeshSimulation::InstallApplications() {
    InstallUdpUplink();
    InstallUdpDownlink();
    InstallTcpUplink();
    InstallBackhaulStress();
}

void EasyMeshSimulation::ScheduleEvents() {
    for (double t = 5.0; t < m_duration; t += 5.0)
        Simulator::Schedule(Seconds(t), &EasyMeshController::CollectApMetrics, m_controller);
    for (double t = 10.0; t < m_duration; t += 10.0)
        Simulator::Schedule(Seconds(t), &EasyMeshController::RunSteeringEngine, m_controller);
    for (double t = 20.0; t < m_duration; t += 20.0)
        Simulator::Schedule(Seconds(t), &EasyMeshController::OptimizeBackhaulTopology, m_controller);
}

void EasyMeshSimulation::Run() {
    m_monitor = m_monHelper.InstallAll();
    Simulator::Stop(Seconds(m_duration));
    Simulator::Run();

    Ptr<Ipv4FlowClassifier> clf = DynamicCast<Ipv4FlowClassifier>(m_monHelper.GetClassifier());
    m_trafficStats.Collect(m_monitor, clf);
    m_controller->ApplyTrafficStats(m_trafficStats);
    Simulator::Destroy();
}

void EasyMeshSimulation::PrintResults() {
    m_controller->PrintTopology();
    m_trafficStats.PrintPerFlowTable();
    m_trafficStats.PrintNetworkSummary();
}

int main(int argc, char* argv[]) {
    EasyMeshSimulation sim;
    sim.BuildTopology();
    sim.InstallProtocolStack();
    sim.InstallApplications();
    sim.ScheduleEvents();
    sim.Run();
    sim.PrintResults();
    return 0;
}