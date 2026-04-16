// easymesh.cc  –  EasyMesh Multi-AP Simulation with Traffic Evaluation
// ======================================================================
// Sửa đổi: 1 controller + 2 agents (Wi‑Fi backhaul) + 1 STA
// ======================================================================

#include "easymesh.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshSim");

// ── Static topology positions ────────────────────────────────────
const Vector EasyMeshSimulation::CONTROLLER_POS   = Vector(25.0, 25.0, 0.0);
const Vector EasyMeshSimulation::AGENT_POSITIONS[NUM_AGENT] = {
    Vector(15.0, 25.0, 0.0),   // Agent 0 – West
    Vector(25.0, 15.0, 0.0)    // Agent 1 – North
};

// ════════════════════════════════════════════════════════════════
// Ctor / Dtor
// ════════════════════════════════════════════════════════════════
EasyMeshSimulation::EasyMeshSimulation()  
{
    m_agentIp.resize(NUM_AGENT, Ipv4Address::GetAny()); 
    m_clientIp.resize(NUM_STA, Ipv4Address::GetAny());
}

EasyMeshSimulation::~EasyMeshSimulation() {}

double 
EasyMeshSimulation::ComputeDistance(Vector a, Vector b) 
{
    return std::sqrt(std::pow(a.x-b.x,2) +
                     std::pow(a.y-b.y,2) +
                     std::pow(a.z-b.z,2));
}

// ════════════════════════════════════════════════════════════════
// CreateNodes
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::CreateNodes()
{
    m_controllerNode.Create(NUM_CONTROLLER);
    m_agentNodes.Create(NUM_AGENT);
    m_clientNodes.Create(m_numClients);
    NS_LOG_INFO("Nodes: " << NUM_CONTROLLER << " controller + "<< NUM_AGENT << " agents + " << m_numClients << " clients");
}

// ════════════════════════════════════════════════════════════════
// InstallMobility
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::InstallMobility()
{
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Controller
    Ptr<ListPositionAllocator> ca = CreateObject<ListPositionAllocator>();
    ca->Add(CONTROLLER_POS);
    mob.SetPositionAllocator(ca);
    mob.Install(m_controllerNode);

    // Agents – fixed
    Ptr<ListPositionAllocator> aa = CreateObject<ListPositionAllocator>();
    for (int i = 0; i < NUM_AGENT; i++) 
        aa->Add(AGENT_POSITIONS[i]);
    mob.SetPositionAllocator(aa);
    mob.Install(m_agentNodes);

    // Clients – random walk inside 50×50 m area (chỉ 1 client nên di chuyển)
    // mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
    //     "Bounds",    RectangleValue(Rectangle(0.0, 50.0, 0.0, 50.0)),
    //     "Speed",     StringValue("ns3::UniformRandomVariable[Min=1.0|Max=3.0]"),
    //     "Distance",  DoubleValue(5.0));

    // Ptr<RandomBoxPositionAllocator> cla = CreateObject<RandomBoxPositionAllocator>();
    // cla->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=45.0]"));
    // cla->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=5.0|Max=45.0]"));
    // cla->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        Ptr<ListPositionAllocator> cla = CreateObject<ListPositionAllocator>();
    cla->Add(Vector(25.0, 15.0, 0.0)); // cùng vị trí Agent1
    MobilityHelper mobClient;
    mobClient.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobClient.SetPositionAllocator(cla);
    mobClient.Install(m_clientNodes);
    
    // mob.SetPositionAllocator(cla);
    // mob.Install(m_clientNodes);
}

// ════════════════════════════════════════════════════════════════
// CreateWifiChannels
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::CreateWifiChannels()
{
    m_channelHelper = YansWifiChannelHelper::Default();
    m_channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    m_channelHelper.AddPropagationLoss(
        "ns3::LogDistancePropagationLossModel",
        "Exponent",          DoubleValue(3.0),
        "ReferenceDistance", DoubleValue(1.0),
        "ReferenceLoss",     DoubleValue(40.0));
}

// ════════════════════════════════════════════════════════════════
// CreateBackhaulLinks   Controller ↔ Agent i  (802.11ac 5GHz)
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::CreateBackhaulLinks()
{
    m_wifiHelper.SetStandard(WIFI_STANDARD_80211ac);
    m_wifiHelper.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    m_phyHelper = YansWifiPhyHelper();
    m_phyHelper.SetChannel(m_channelHelper.Create());
    m_phyHelper.Set("TxPowerStart", DoubleValue(20.0));
    m_phyHelper.Set("TxPowerEnd",   DoubleValue(20.0));

    m_macHelper.SetType("ns3::AdhocWifiMac");

    for (uint32_t i = 0; i < NUM_AGENT; i++) 
    {
        NodeContainer pair;
        pair.Add(m_controllerNode.Get(0));
        pair.Add(m_agentNodes.Get(i));
        NetDeviceContainer devs = m_wifiHelper.Install(m_phyHelper, m_macHelper, pair);
        m_backhaulDevices.Add(devs);

        double dist = ComputeDistance(CONTROLLER_POS, AGENT_POSITIONS[i]);
        double pl   = 40.0 + 30.0 * std::log10(std::max(1.0, dist));
        double rssi = 20.0 + 6.0 - pl;
        double snr  = rssi - (-95.0);
        uint32_t mcs = rssi >= -65 ? 9 : rssi >= -70 ? 7 : rssi >= -76 ? 5 : 3;
        double tput  = (mcs + 1) * 5.5;

        Ptr<EasyMeshLink> link = Create<EasyMeshLink>(
            i, m_controllerNode.Get(0), m_agentNodes.Get(i),
            LinkType::BACKHAUL, 5.0);
        link->UpdateMetrics(rssi, snr, mcs, tput, 0.0);
        m_links.push_back(link);

        NS_LOG_INFO("BH Wi-Fi: Controller <-> Agent" << i
            << "  dist=" << dist << "m  RSSI=" << rssi
            << "dBm  MCS=" << mcs << "  Tput=" << tput << "Mbps");
    }
}

// ════════════════════════════════════════════════════════════════
// CreateFronthaulLinks   Agent i ↔ Clients  (802.11n 2.4GHz)
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::CreateFronthaulLinks()
{
    WifiHelper fh;
    fh.SetStandard(WIFI_STANDARD_80211n);
    fh.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
    NS_LOG_INFO("=== CreateFronthaulLinks ===");

    for (uint32_t a = 0; a < NUM_AGENT; a++) 
    {
        YansWifiPhyHelper fhPhy;
        fhPhy.SetChannel(m_channelHelper.Create());
        fhPhy.Set("TxPowerStart", DoubleValue(17.0));
        fhPhy.Set("TxPowerEnd",   DoubleValue(17.0));

        WifiMacHelper fhMac;
        Ssid ssid = Ssid("EasyMesh-AP");

        // Install AP
        fhMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer apDev = fh.Install(fhPhy, fhMac, m_agentNodes.Get(a));
        m_fronthaulDevices.Add(apDev);

        // Install STAs for this agent (round-robin partition)
        fhMac.SetType("ns3::StaWifiMac",
            "Ssid",          SsidValue(ssid),
            "ActiveProbing", BooleanValue(false));

        uint32_t base = (m_numClients * a)     / NUM_AGENT;
        uint32_t top  = (m_numClients * (a+1)) / NUM_AGENT;

        NodeContainer staNd;
        for (uint32_t c = base; c < top; c++) 
            staNd.Add(m_clientNodes.Get(c));

        if (staNd.GetN() > 0) 
        {
            NetDeviceContainer staDev = fh.Install(fhPhy, fhMac, staNd);
            m_fronthaulDevices.Add(staDev);
        }
    }
}

// ════════════════════════════════════════════════════════════════
// BuildTopology
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::BuildTopology()
{
    NS_LOG_INFO("=== BuildTopology ===");
    CreateNodes();
    InstallMobility();
    CreateWifiChannels();
    // Không dùng Ethernet backhaul nữa
    CreateBackhaulLinks();      // Controller ↔ Agent qua Wi-Fi
    CreateFronthaulLinks();

    // Logical EasyMesh objects
    m_controller = Create<EasyMeshController>(0, m_controllerNode.Get(0));

    AgentRole roles[NUM_AGENT] = { AgentRole::HYBRID, AgentRole::HYBRID };
    for (uint32_t i = 0; i < NUM_AGENT; i++) 
    {
        auto agent = Create<EasyMeshAgent>(
            i, m_agentNodes.Get(i), roles[i], AGENT_POSITIONS[i]);
        m_agents.push_back(agent);
        m_controller->RegisterAgent(agent);
    }

    for (auto& l : m_links) 
        m_controller->RegisterLink(l);

    AssignClientsToAgents();
    NS_LOG_INFO("=== Topology ready ===");
}

// ════════════════════════════════════════════════════════════════
// AssignClientsToAgents  (nearest-agent heuristic)
// ════════════════════════════════════════════════════════════════
void 
EasyMeshSimulation::AssignClientsToAgents()
{
    for (uint32_t c = 0; c < m_numClients; c++) 
    {
        Ptr<MobilityModel> mob = m_clientNodes.Get(c)->GetObject<MobilityModel>();
        double minDist  = 1e9;
        uint32_t best   = 0;
        for (uint32_t a = 0; a < NUM_AGENT; a++) 
        {
            double d = mob ? ComputeDistance(mob->GetPosition(), AGENT_POSITIONS[a])
                           : (double)(c % NUM_AGENT == a ? 0 : 1e9);
            if (d < minDist) 
            { 
                minDist = d; 
                best = a; 
            }
        }
        m_agents[best]->AddClient(c);
    }
}

// ════════════════════════════════════════════════════════════════
// InstallProtocolStack
// ════════════════════════════════════════════════════════════════
void EasyMeshSimulation::InstallProtocolStack()
{
    NS_LOG_INFO("=== Install Protocol Stack (Wi‑Fi Backhaul + Static Routes) ===");
    
    InternetStackHelper inet;
    inet.Install(m_controllerNode);
    inet.Install(m_agentNodes);
    inet.Install(m_clientNodes);

    Ipv4AddressHelper addr;
    
    // Mảng lưu địa chỉ IP của controller trên mỗi backhaul link
    Ipv4Address ctrlBackhaulIp[NUM_AGENT];
    
    // 1. Gán IP cho Wi-Fi backhaul: Controller <-> Agent i (mỗi link một subnet)
    for (uint32_t i = 0; i < NUM_AGENT; i++) 
    {
        std::string subnet = "10.1." + std::to_string(i) + ".0";
        addr.SetBase(subnet.c_str(), "255.255.255.0");
        NetDeviceContainer linkDevs;
        linkDevs.Add(m_backhaulDevices.Get(2*i));     // device trên Controller
        linkDevs.Add(m_backhaulDevices.Get(2*i+1));   // device trên Agent i
        Ipv4InterfaceContainer bhIfaces = addr.Assign(linkDevs);
        
        ctrlBackhaulIp[i] = bhIfaces.GetAddress(0);   // IP của controller trên link này
        m_agentIp[i] = bhIfaces.GetAddress(1);        // IP của agent i
        if (i == 0) m_controllerIp = ctrlBackhaulIp[0];
        
        NS_LOG_INFO("Backhaul link " << i << ": Controller " << ctrlBackhaulIp[i]
                    << " <-> Agent" << i << " " << m_agentIp[i]);
    }
    
    // 2. Gán IP cho Fronthaul Wi-Fi: mỗi agent tạo một BSS riêng (subnet 192.168.x.0/24)
    for (uint32_t a = 0; a < NUM_AGENT; a++) 
    {
        std::string subnet = "192.168." + std::to_string(a) + ".0";
        addr.SetBase(subnet.c_str(), "255.255.255.0");
        
        // Tìm AP device của agent a
        NetDeviceContainer apDev;
        for (uint32_t i = 0; i < m_fronthaulDevices.GetN(); i++) 
        {
            Ptr<WifiNetDevice> wifiDev = m_fronthaulDevices.Get(i)->GetObject<WifiNetDevice>();
            if (wifiDev && wifiDev->GetNode() == m_agentNodes.Get(a) &&
                wifiDev->GetMac()->GetTypeOfStation() == AP) 
            {
                apDev.Add(m_fronthaulDevices.Get(i));
                break;
            }
        }
        
        // Tìm các STA devices thuộc về agent a
        NetDeviceContainer staDevs;
        uint32_t base = (m_numClients * a) / NUM_AGENT;
        uint32_t top  = (m_numClients * (a+1)) / NUM_AGENT;
        for (uint32_t c = base; c < top; c++) 
        {
            for (uint32_t i = 0; i < m_fronthaulDevices.GetN(); i++) 
            {
                if (m_fronthaulDevices.Get(i)->GetNode() == m_clientNodes.Get(c)) 
                {
                    staDevs.Add(m_fronthaulDevices.Get(i));
                    break;
                }
            }
        }
        
        // Gán IP cho AP và các STA
        Ipv4InterfaceContainer fhIfaces;
        if (apDev.GetN() > 0) 
            fhIfaces = addr.Assign(apDev);
        if (staDevs.GetN() > 0) 
            fhIfaces.Add(addr.Assign(staDevs));
        
        // Lưu IP cho client (index 0 là AP, index 1.. là STA)
        for (uint32_t idx = 1; idx < fhIfaces.GetN(); idx++) 
        {
            m_clientIp[base + idx - 1] = fhIfaces.GetAddress(idx);
            NS_LOG_INFO("Fronthaul: Agent" << a << " AP=" << fhIfaces.GetAddress(0)
                        << " STA" << (base+idx-1) << "=" << fhIfaces.GetAddress(idx));
        }
    }
    
    // 3. Enable IP forwarding
    m_controllerNode.Get(0)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    for (uint32_t i = 0; i < NUM_AGENT; i++) 
        m_agentNodes.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    
    // 4. Static routes
    Ipv4StaticRoutingHelper staticRoutingHelper;
    
    // Trên Controller: route đến các subnet fronthaul của từng agent (192.168.x.0/24)
    Ptr<Ipv4StaticRouting> ctrlRouting = staticRoutingHelper.GetStaticRouting(
        m_controllerNode.Get(0)->GetObject<Ipv4>());
    for (uint32_t a = 0; a < NUM_AGENT; a++) 
    {
        ctrlRouting->AddNetworkRouteTo(
            Ipv4Address(("192.168." + std::to_string(a) + ".0").c_str()),
            Ipv4Mask("255.255.255.0"),
            m_agentIp[a],      // next hop là IP của agent a trên backhaul link
            1 + a);            // interface index (backhaul interfaces bắt đầu từ 1)
    }
    
    // Trên mỗi agent: route đến các subnet của agent khác và đến controller (nếu cần)
    for (uint32_t a = 0; a < NUM_AGENT; a++) 
    {
        Ptr<Ipv4StaticRouting> agentRouting = staticRoutingHelper.GetStaticRouting(
            m_agentNodes.Get(a)->GetObject<Ipv4>());
        
        // Route đến controller (có thể không cần vì đã cùng mạng backhaul, nhưng để chắc chắn)
        // Route đến các backhaul subnet khác và các fronthaul subnet khác
        for (uint32_t b = 0; b < NUM_AGENT; b++) 
        {
            if (b == a) continue;
            // Backhaul subnet của agent b
            std::string bhSubnet = "10.1." + std::to_string(b) + ".0";
            agentRouting->AddNetworkRouteTo(
                Ipv4Address(bhSubnet.c_str()), Ipv4Mask("255.255.255.0"),
                ctrlBackhaulIp[a], 1); // gateway là IP của controller trên link của agent a
            // Fronthaul subnet của agent b
            std::string fhSubnet = "192.168." + std::to_string(b) + ".0";
            agentRouting->AddNetworkRouteTo(
                Ipv4Address(fhSubnet.c_str()), Ipv4Mask("255.255.255.0"),
                ctrlBackhaulIp[a], 1);
        }
    }
    
    // 5. Default route cho các STA (client)
    for (uint32_t c = 0; c < m_numClients; c++) {
        Ptr<Ipv4StaticRouting> clientRouting = staticRoutingHelper.GetStaticRouting(
            m_clientNodes.Get(c)->GetObject<Ipv4>());
        // Tìm agent quản lý client này
        uint32_t agentIdx = 0;
        for (uint32_t a = 0; a < NUM_AGENT; a++) {
            uint32_t base = (m_numClients * a) / NUM_AGENT;
            uint32_t top  = (m_numClients * (a+1)) / NUM_AGENT;
            if (c >= base && c < top) {
                agentIdx = a;
                break;
            }
        }
        Ipv4Address apAddr = Ipv4Address(("192.168." + std::to_string(agentIdx) + ".1").c_str());
        // Interface 1 là Wi-Fi (0 là loopback)
        clientRouting->SetDefaultRoute(apAddr, 1);
        NS_LOG_INFO("Added default route on STA" << c << " via " << apAddr);
    }

    NS_LOG_INFO("=== END Protocol Stack ===");
}

// ════════════════════════════════════════════════════════════════
// ── TRAFFIC INSTALLATION ────────────────────────────────────────
// (giữ nguyên, chỉ chạy với 1 STA)
// ════════════════════════════════════════════════════════════════

void EasyMeshSimulation::InstallUdpUplink()
{
    PacketSinkHelper sink("ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), PORT_UDP_UL));
    ApplicationContainer sinks = sink.Install(m_controllerNode.Get(0));
    sinks.Start(Seconds(0.5));
    sinks.Stop(Seconds(m_duration));

    for (uint32_t c = 0; c < m_numClients; c++) {
        OnOffHelper src("ns3::UdpSocketFactory",
            InetSocketAddress(m_controllerIp, PORT_UDP_UL));
        src.SetAttribute("DataRate",  StringValue(UL_DATA_RATE));
        src.SetAttribute("PacketSize", UintegerValue(UL_PKT_SIZE));
        src.SetAttribute("OnTime",    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        src.SetAttribute("OffTime",   StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer app = src.Install(m_clientNodes.Get(c));
        double start = UL_START + c * 0.1;
        app.Start(Seconds(start));
        app.Stop(Seconds(m_duration - 1.0));
        NS_LOG_INFO("UDP-UL: STA" << c << " -> Controller  " << UL_DATA_RATE);
    }
}

void EasyMeshSimulation::InstallUdpDownlink()
{
    for (uint32_t c = 0; c < m_numClients; c++) {
        Ipv4Address staIp = m_clientIp[c];
        if (staIp == Ipv4Address()) continue;

        PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), PORT_UDP_DL));
        ApplicationContainer sinks = sink.Install(m_clientNodes.Get(c));
        sinks.Start(Seconds(0.5));
        sinks.Stop(Seconds(m_duration));

        OnOffHelper src("ns3::UdpSocketFactory",
            InetSocketAddress(staIp, PORT_UDP_DL));
        src.SetAttribute("DataRate",   StringValue(DL_DATA_RATE));
        src.SetAttribute("PacketSize", UintegerValue(DL_PKT_SIZE));
        src.SetAttribute("OnTime",     StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        src.SetAttribute("OffTime",    StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer app = src.Install(m_controllerNode.Get(0));
        double start = DL_START + c * 0.1;
        app.Start(Seconds(start));
        app.Stop(Seconds(m_duration - 1.0));
        NS_LOG_INFO("UDP-DL: Controller -> STA" << c << "  " << DL_DATA_RATE);
    }
}

void EasyMeshSimulation::InstallTcpUplink()
{
    PacketSinkHelper tcpSink("ns3::TcpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), PORT_TCP_UL));
    ApplicationContainer sinks = tcpSink.Install(m_controllerNode.Get(0));
    sinks.Start(Seconds(0.5));
    sinks.Stop(Seconds(m_duration));

    for (uint32_t c = 0; c < m_numClients; c += 2) {
        BulkSendHelper bulk("ns3::TcpSocketFactory",
            InetSocketAddress(m_controllerIp, PORT_TCP_UL));
        bulk.SetAttribute("SendSize", UintegerValue(TCP_SEND_SIZE));
        bulk.SetAttribute("MaxBytes", UintegerValue(0));

        ApplicationContainer app = bulk.Install(m_clientNodes.Get(c));
        app.Start(Seconds(TCP_START + c * 0.05));
        app.Stop(Seconds(m_duration - 1.0));
        NS_LOG_INFO("TCP-BulkSend: STA" << c << " -> Controller");
    }
}

void EasyMeshSimulation::InstallBackhaulStress()
{
    // Không cần backhaul stress trong cấu hình nhỏ này
    NS_LOG_INFO("BackhaulStress disabled (only 2 agents)");
}

void EasyMeshSimulation::InstallApplications()
{
    NS_LOG_INFO("=== InstallApplications ===");
    InstallUdpUplink();
    InstallUdpDownlink();
    InstallTcpUplink();
    // InstallBackhaulStress();  // tắt
    NS_LOG_INFO("Applications installed: "
                << m_numClients << " UDP-UL  "
                << m_numClients << " UDP-DL  "
                << (m_numClients / 2) << " TCP-UL");
}

// ════════════════════════════════════════════════════════════════
// ScheduleEvents, Run, PrintResults (giữ nguyên)
// ════════════════════════════════════════════════════════════════

void EasyMeshSimulation::ScheduleEvents()
{
    for (double t = 5.0; t < m_duration; t += 5.0)
        Simulator::Schedule(Seconds(t),
            &EasyMeshController::CollectApMetrics, m_controller);
    for (double t = 10.0; t < m_duration; t += 10.0)
        Simulator::Schedule(Seconds(t),
            &EasyMeshController::RunSteeringEngine, m_controller);
    for (double t = 20.0; t < m_duration; t += 20.0)
        Simulator::Schedule(Seconds(t),
            &EasyMeshController::OptimizeBackhaulTopology, m_controller);
}

void EasyMeshSimulation::Run()
{
    NS_LOG_INFO("=== Simulation start  duration=" << m_duration << "s ===");
    m_monitor = m_monHelper.InstallAll();
    if (m_pcap) m_phyHelper.EnablePcapAll("easymesh");
    Simulator::Stop(Seconds(m_duration));
    Simulator::Run();

    Ptr<Ipv4FlowClassifier> clf =
        DynamicCast<Ipv4FlowClassifier>(m_monHelper.GetClassifier());
    m_trafficStats.Collect(m_monitor, clf);
    m_controller->ApplyTrafficStats(m_trafficStats);

    Simulator::Destroy();
    NS_LOG_INFO("=== Simulation complete ===");
}

void EasyMeshSimulation::PrintResults()
{
    m_controller->PrintTopology();
    m_controller->PrintSteeringLog();
    m_trafficStats.PrintPerFlowTable();
    m_controller->PrintAgentTraffic();
    m_trafficStats.PrintNetworkSummary();
}

// ════════════════════════════════════════════════════════════════
// main()
// ════════════════════════════════════════════════════════════════
int main(int argc, char* argv[])
{
    LogComponentEnable("EasyMeshSim",        LOG_LEVEL_INFO);
    LogComponentEnable("EasyMeshController", LOG_LEVEL_INFO);

    double   duration   = DURATION;
    uint32_t numClients = NUM_STA;          // CHỈ 1 STA
    bool     pcap       = false;

    CommandLine cmd;
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("clients",  "Number of STA clients",   numClients);
    cmd.AddValue("pcap",     "Enable PCAP capture",     pcap);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(42);
    RngSeedManager::SetRun(1);

    EasyMeshSimulation sim;
    sim.SetDuration(duration);
    sim.SetNumClients(numClients);
    sim.EnablePcap(pcap);

    sim.BuildTopology();
    sim.InstallProtocolStack();
    sim.InstallApplications();
    sim.ScheduleEvents();
    sim.Run();
    sim.PrintResults();

    return 0;
}