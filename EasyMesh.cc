/* ==========================================================================
 * easymesh-wifi6.cc
 *
 * Mô phỏng mạng EasyMesh đơn giản trên ns-3.46.1
 *   - 01 Controller  (AP  – BSS1, backhaul)
 *   - 01 Agent       (STA của BSS1  +  AP của BSS2)
 *   - 01 STA         (STA – BSS2, fronthaul)
 *
 * Chuẩn:   Wi-Fi 6 (IEEE 802.11ax)
 * Băng tần: 5 GHz
 * Đo lường: FlowMonitor  →  Throughput, Packet-loss, Delay, Jitter
 *
 * Build:
 *   cp easymesh-wifi6.cc <ns3-root>/scratch/
 *   cd <ns3-root>
 *   ./ns3 run scratch/easymesh-wifi6
 * ========================================================================== */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshWifi6");

/* --------------------------------------------------------------------------
 * Hàm in thông tin luồng từ FlowMonitor
 * -------------------------------------------------------------------------- */
static void
PrintFlowStats(Ptr<FlowMonitor> monitor, FlowMonitorHelper& fmHelper, double simTime)
{
    monitor->CheckForLostPackets();
    auto classifier = DynamicCast<Ipv4FlowClassifier>(fmHelper.GetClassifier());
    auto stats      = monitor->GetFlowStats();

    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "        EasyMesh Wi-Fi 6 (5 GHz)  —  Kết quả FlowMonitor\n";
    std::cout << "        Thời gian mô phỏng: " << simTime << " s\n";
    std::cout << "================================================================\n";

    for (auto& [id, s] : stats)
    {
        auto   t    = classifier->FindFlow(id);
        double dur  = s.timeLastRxPacket.GetSeconds() - s.timeFirstTxPacket.GetSeconds();
        double tput = (dur > 0) ? (s.rxBytes * 8.0 / dur / 1e6) : 0.0; // Mbps
        double lossRate = (s.txPackets > 0)
                              ? (100.0 * (s.txPackets - s.rxPackets) / s.txPackets)
                              : 0.0;
        double avgDelay  = (s.rxPackets > 0) ? (s.delaySum.GetMilliSeconds() / s.rxPackets)
                                              : 0.0;
        double avgJitter = (s.rxPackets > 1)
                               ? (s.jitterSum.GetMilliSeconds() / (s.rxPackets - 1))
                               : 0.0;

        std::cout << "\n  Flow " << id
                  << "  [" << t.sourceAddress << ":" << t.sourcePort
                  << "  →  " << t.destinationAddress << ":" << t.destinationPort << "]\n";
        std::cout << "  ┌─────────────────────────────────┐\n";
        std::cout << "  │ Throughput   : " << std::fixed << std::setprecision(3) << tput
                  << " Mbps\n";
        std::cout << "  │ Tx Packets   : " << s.txPackets << "\n";
        std::cout << "  │ Rx Packets   : " << s.rxPackets << "\n";
        std::cout << "  │ Lost Packets : " << (s.txPackets - s.rxPackets) << "\n";
        std::cout << "  │ Packet Loss  : " << std::fixed << std::setprecision(2) << lossRate
                  << " %\n";
        std::cout << "  │ Mean Delay   : " << std::fixed << std::setprecision(3) << avgDelay
                  << " ms\n";
        std::cout << "  │ Mean Jitter  : " << std::fixed << std::setprecision(3) << avgJitter
                  << " ms\n";
        std::cout << "  │ Tx Bytes     : " << s.txBytes << " B\n";
        std::cout << "  │ Rx Bytes     : " << s.rxBytes << " B\n";
        std::cout << "  └─────────────────────────────────┘\n";
    }
    std::cout << "================================================================\n\n";
}

/* ==========================================================================
 * main()
 * ========================================================================== */
int
main(int argc, char* argv[])
{
    /* ------------------------------------------------------------------
     * 0. Tham số điều chỉnh qua command line
     * ------------------------------------------------------------------ */
    double   simTime    = 15.0;   // [s]  thời gian mô phỏng
    uint32_t payloadB   = 1472;   // [B]  kích thước UDP payload (MTU-safe)
    double   offeredMbps = 100.0; // [Mbps] tải mong muốn từ STA
    bool     verbose    = false;  // bật NS_LOG chi tiết

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime",    "Thời gian mô phỏng (s)",    simTime);
    cmd.AddValue("payload",    "Kích thước UDP payload (B)", payloadB);
    cmd.AddValue("offered",    "Tải UDP (Mbps)",             offeredMbps);
    cmd.AddValue("verbose",    "In log chi tiết",            verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("EasyMeshWifi6", LOG_LEVEL_INFO);
        LogComponentEnable("UdpClient",     LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer",     LOG_LEVEL_INFO);
    }

    /* Khoảng thời gian gửi gói để đạt offeredMbps */
    double intervalUs = (payloadB * 8.0) / (offeredMbps * 1e6) * 1e6; // μs
    Time beaconInterval = MicroSeconds(102400);

    std::cout << "\n[EasyMesh] Cấu hình:\n";
    std::cout << "  simTime    = " << simTime     << " s\n";
    std::cout << "  payload    = " << payloadB    << " B\n";
    std::cout << "  offered    = " << offeredMbps << " Mbps\n";
    std::cout << "  interval   = " << intervalUs  << " μs\n\n";

    /* ------------------------------------------------------------------
     * 1. Tạo nodes
     *    n0 = Controller  (AP  trong BSS1)
     *    n1 = Agent       (STA trong BSS1  +  AP trong BSS2)
     *    n2 = STA         (STA trong BSS2)
     * ------------------------------------------------------------------ */
    NodeContainer controller, agent, sta;
    controller.Create(1);
    agent.Create(1);
    sta.Create(1);

    /* ------------------------------------------------------------------
     * 2. Wi-Fi 6 helper chung
     * ------------------------------------------------------------------ */
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    /* ------------------------------------------------------------------
     * 3. BSS1 – Backhaul: Controller(AP) ↔ Agent(STA)
     *    Kênh trung tâm 42, 80 MHz, 5 GHz
     * ------------------------------------------------------------------ */
    YansWifiChannelHelper chanBH = YansWifiChannelHelper::Default();
    YansWifiPhyHelper     phyBH;
    phyBH.SetChannel(chanBH.Create());
    phyBH.Set("ChannelSettings", StringValue("{42, 80, BAND_5GHZ, 0}"));
    phyBH.Set("TxPowerStart",    DoubleValue(20.0));  // dBm
    phyBH.Set("TxPowerEnd",      DoubleValue(20.0));

    WifiMacHelper mac;
    Ssid ssidBH = Ssid("EasyMesh-Backhaul");

    /* Controller → AP */
    mac.SetType("ns3::ApWifiMac",
                "Ssid",  SsidValue(ssidBH),
                "BeaconInterval", TimeValue(beaconInterval));
    NetDeviceContainer devCtrl = wifi.Install(phyBH, mac, controller.Get(0));

    /* Agent → STA (kết nối backhaul lên Controller) */
    mac.SetType("ns3::StaWifiMac",
                "Ssid",          SsidValue(ssidBH),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer devAgentBH = wifi.Install(phyBH, mac, agent.Get(0));

    /* ------------------------------------------------------------------
     * 4. BSS2 – Fronthaul: Agent(AP) ↔ STA
     *    Kênh trung tâm 58, 80 MHz, 5 GHz  (không chồng lấn với BSS1)
     * ------------------------------------------------------------------ */
    YansWifiChannelHelper chanFH = YansWifiChannelHelper::Default();
    YansWifiPhyHelper     phyFH;
    phyFH.SetChannel(chanFH.Create());
    phyFH.Set("ChannelSettings", StringValue("{58, 80, BAND_5GHZ, 0}"));
    phyFH.Set("TxPowerStart",    DoubleValue(20.0));
    phyFH.Set("TxPowerEnd",      DoubleValue(20.0));

    Ssid ssidFH = Ssid("EasyMesh-Fronthaul");

    /* Agent → AP (phục vụ STA) */
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssidFH),
                "BeaconInterval", TimeValue(beaconInterval));
    NetDeviceContainer devAgentFH = wifi.Install(phyFH, mac, agent.Get(0));

    /* STA → kết nối fronthaul lên Agent */
    mac.SetType("ns3::StaWifiMac",
                "Ssid",          SsidValue(ssidFH),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer devSta = wifi.Install(phyFH, mac, sta.Get(0));

    /* ------------------------------------------------------------------
     * 5. Mobility – vị trí cố định (đơn vị: mét)
     *
     *   Controller (0,0)  ←20m→  Agent (20,0)  ←15m→  STA (35,0)
     * ------------------------------------------------------------------ */
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(controller);
    mob.Install(agent);
    mob.Install(sta);

    controller.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0,  0.0, 1.5));
    agent.Get(0)     ->GetObject<MobilityModel>()->SetPosition(Vector(20.0, 0.0, 1.5));
    sta.Get(0)       ->GetObject<MobilityModel>()->SetPosition(Vector(35.0, 0.0, 1.0));

    /* ------------------------------------------------------------------
     * 6. Internet Stack + Địa chỉ IP
     *
     *   BSS1 (backhaul) : 10.1.1.0/24
     *     Controller    : 10.1.1.1
     *     Agent (BH)    : 10.1.1.2
     *
     *   BSS2 (fronthaul): 10.1.2.0/24
     *     Agent (FH)    : 10.1.2.1
     *     STA           : 10.1.2.2
     * ------------------------------------------------------------------ */
    InternetStackHelper internet;
    internet.Install(controller);
    internet.Install(agent);
    internet.Install(sta);

    Ipv4AddressHelper ipv4;

    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifCtrl   = ipv4.Assign(devCtrl);
    /*Ipv4InterfaceContainer ifAgBH =*/ ipv4.Assign(devAgentBH);

    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    /*Ipv4InterfaceContainer ifAgFH =*/ ipv4.Assign(devAgentFH);
    Ipv4InterfaceContainer ifSta    = ipv4.Assign(devSta);

    /* Agent tự động trở thành router IPv4 nhờ hai giao diện khác subnet */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    /* ------------------------------------------------------------------
     * 7. Ứng dụng: UDP uplink  STA → Controller
     * ------------------------------------------------------------------ */
    uint16_t port      = 9000;
    double   startApp  = 2.0;   // Chờ 2 s để BSS association hoàn tất

    /* UDP Server trên Controller */
    UdpServerHelper udpServer(port);
    ApplicationContainer srvApps = udpServer.Install(controller.Get(0));
    srvApps.Start(Seconds(0.0));
    srvApps.Stop(Seconds(simTime + 1.0));

    /* UDP Client trên STA  →  gửi tới Controller */
    UdpClientHelper udpClient(ifCtrl.GetAddress(0), port);
    udpClient.SetAttribute("MaxPackets", UintegerValue(UINT32_MAX));
    udpClient.SetAttribute("Interval",   TimeValue(MicroSeconds(intervalUs)));
    udpClient.SetAttribute("PacketSize", UintegerValue(payloadB));

    ApplicationContainer cltApps = udpClient.Install(sta.Get(0));
    cltApps.Start(Seconds(startApp));
    cltApps.Stop(Seconds(simTime));

    /* ------------------------------------------------------------------
     * 8. FlowMonitor – theo dõi toàn bộ luồng IP
     * ------------------------------------------------------------------ */
    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor>  monitor = fmHelper.InstallAll();

    /* ------------------------------------------------------------------
     * 9. Chạy mô phỏng
     * ------------------------------------------------------------------ */
    std::cout << "[EasyMesh] Bắt đầu mô phỏng...\n";
    Simulator::Stop(Seconds(simTime + 2.0));
    Simulator::Run();

    /* ------------------------------------------------------------------
     * 10. In kết quả & lưu XML
     * ------------------------------------------------------------------ */
    PrintFlowStats(monitor, fmHelper, simTime);
    monitor->SerializeToXmlFile("easymesh-flowmon.xml", true, true);
    std::cout << "[EasyMesh] Đã lưu dữ liệu FlowMonitor → easymesh-flowmon.xml\n\n";

    Simulator::Destroy();
    return 0;
}