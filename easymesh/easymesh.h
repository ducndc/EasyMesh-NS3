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
#include "ns3/propagation-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/bulk-send-helper.h"

#include "lib/easymesh-types.h"
#include "lib/easymesh-link.h"
#include "lib/easymesh-radio.h"
#include "lib/easymesh-agent.h"
#include "lib/easymesh-assoc-manager.h"
#include "lib/easymesh-controller.h"
#include "lib/traffic-stats.h"
#include "parameters.h"

#include <vector>
#include <map>
#include <memory>
#include <set>

using namespace ns3;

// ── MST edge-weight strategy for ComputeBackhaulTree() ─────────
enum class MstMetric { THROUGHPUT, RSSI };

// ── Fronthaul channel planning mode ────────────────────────────
//   NONE    – every radio on the first channel of its band (baseline)
//   STATIC  – interference-graph plan computed once before start
//   DYNAMIC – STATIC plus periodic re-planning from measured utilization
enum class ChannelPlanMode { NONE, STATIC, DYNAMIC };

class EasyMeshSimulation {
public:
    EasyMeshSimulation();
    ~EasyMeshSimulation();

    // ── Build phases ──────────────────────────────────────────
    void BuildTopology();
    void InstallProtocolStack();
    void InstallApplications();     // ← UDP CBR uplink + downlink
    void ScheduleEvents();
    void Run();
    void PrintResults();            // ← topology + channels + transitions + flow stats

    // ── CLI overrides ─────────────────────────────────────────
    void SetDuration(double s)   { m_duration   = s; }
    void SetScenarioRun(uint32_t run) { m_scenarioRun = run; }
    void SetOfflineAgentIds(std::vector<uint32_t> agentIds) { m_offlineAgentIds = std::move(agentIds); }
    void SetOfflineTime(double timeSeconds) { m_offlineTime = timeSeconds; }
    void SetRecoverTime(double timeSeconds) { m_recoverTime = timeSeconds; }
    void SetNumClients(uint32_t n);
    void SetNumAgents(uint32_t n);
    void EnablePcap(bool e)      { m_pcap       = e; }
    void SetMstMetric(MstMetric m) { m_mstMetric = m; }
    void SetChannelPlanMode(ChannelPlanMode m) { m_channelPlanMode = m; }
    void EnableSteering(bool e)        { m_steering = e; }
    void EnableRoaming(bool e)         { m_roaming = e; }
    void EnableLoadBalancing(bool e)   { m_loadBalancing = e; }
    void EnableNetworkOptimization(bool e) { m_netOpt = e; }
    void SetStaMaxSpeed(double v)      { m_staMaxSpeed = v; }
    // ── Wi-Fi 7 ───────────────────────────────────────────────
    void EnableBackhaulMlo(bool e)     { m_bhMlo = e; }
    void EnableFronthaulMlo(bool e)    { m_fhMlo = e; }
    void SetNumMldClients(uint32_t n)  { m_numMldClients = n; }
    void EnableEmlsr(bool e)           { m_emlsr = e; }
    void SetBackhaul6Width(uint16_t w) { m_bh6Width = w; }

    double ComputeDistance(Vector a, Vector b) const;

private:
    // Where a BSS transition goes. For a multi-link STA: the AP MLD address
    // (what the Assoc trace reports), every affiliated AP it may associate
    // through, and the channel of each of the STA's links.
    struct TransitionTarget {
        Mac48Address            expected;   // address the Assoc trace reports on success
        std::set<Mac48Address>  bssids;     // BSSIDs the STA is allowed to join through
        std::vector<EmChannel>  channels;   // one per STA PHY (link)
    };

    // One STA whose BSS membership the simulation manages:
    // a client STA, or the backhaul STA (bSTA) of an Agent.
    struct ManagedSta {
        Ptr<WifiNetDevice>         device;
        Ptr<StaWifiMac>            mac;
        Ptr<EasyMeshAssocManager>  assocMgr;
        bool                       isBackhaul = false;
        uint32_t                   ownerId    = 0;   // client id or agent id
        std::vector<EmBand>        linkBands;       // band of each PHY (link) of the device

        // in-flight BSS transition
        bool         pending  = false;
        TransitionTarget target;
        Time         start;
        EventId      hopEvent;
        EventId      timeoutEvent;
        uint32_t     hopTries = 0;
        SteeringEvent ev;
    };

    // Who owns a BSSID (or an AP MLD address)
    struct BssInfo {
        int32_t agentId;    // -1 = Controller
        bool    backhaul;
        EmBand  band;       // AP MLD: band of its first link
        bool    mld;        // the address is an AP MLD address
    };

    // ── Topology builders ─────────────────────────────────────
    void CreateNodes();
    void InstallMobility();
    void CreateSpectrumChannels();
    void InstallBackhaulDevices();
    void InstallFronthaulDevices();
    void InstallClientDevices();
    void GenerateRandomAgentPositions();
    void OptimizeNetworkTopology();
    std::vector<int32_t> ComputeBackhaulTree() const;
    double TreeWeight(const std::vector<int32_t>& parents) const;
    double EdgeWeight(Vector a, Vector b) const;
    Ptr<EasyMeshRadio> SelectInitialRadio(uint32_t clientId) const;

    // ── Wi-Fi device helpers ──────────────────────────────────
    Ptr<MultiModelSpectrumChannel> SpectrumChannel(EmBand band) const;
    // one PHY per link: spectrum channel, operating channel, power, CCA
    // fixedBand=false: every PHY attaches to all the links' bands (EMLSR moves
    // its main PHY between links)
    void ConfigureLinks(SpectrumWifiPhyHelper& phy, const std::vector<EmChannel>& links,
                        bool fixedBand = true) const;
    // map the device's (MLD) address and the address of each of its links
    void RegisterBss(Ptr<WifiNetDevice> dev, int32_t agentId, bool backhaul,
                     const std::vector<EmBand>& linkBands);
    static std::set<Mac48Address> LinkBssids(Ptr<WifiNetDevice> dev);
    void InstallClientDevice(uint32_t clientId, NetDeviceContainer& pcapDevices);

    // ── Radio estimates (same log-distance model as the channels) ──
    static double TxPowerDbm(const EmChannel& channel);
    static double CcaEdThresholdDbm(uint16_t widthMhz);
    double EstimateRssi(Vector a, Vector b, const EmChannel& channel) const;
    double ClientRssi(uint32_t clientId, Ptr<EasyMeshRadio> radio) const;
    double RadioRssi(Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b) const;
    static double PhyRateMbps(double rssiDbm, uint16_t widthMhz, uint32_t maxMcs = EHT_MAX_MCS);
    static uint32_t RssiToMcs(double rssiDbm, uint16_t widthMhz, uint32_t maxMcs = EHT_MAX_MCS);
    // every link of the backhaul between two positions; MLO capacity = sum (STR)
    std::vector<EmLinkMetrics> BackhaulLinkMetrics(Vector a, Vector b) const;
    double BackhaulRate(Vector a, Vector b) const;
    Vector GetAgentPosition(uint32_t agentId) const;
    Vector GetClientPosition(uint32_t clientId) const;
    bool IsAgentOnline(uint32_t agentId) const;
    Ptr<WifiNetDevice> BackhaulDevice(int32_t parent) const;
    Mac48Address BackhaulBssid(int32_t parent) const;

    // ── BSS transitions (steering, roaming, backhaul re-parenting) ──
    TransitionTarget ClientTarget(uint32_t clientId, Ptr<EasyMeshRadio> radio) const;
    TransitionTarget BackhaulTarget(int32_t parent) const;
    const EmChannel& ParkingChannel(const ManagedSta& s, size_t link) const;
    void WakeStaLinks(uint32_t staIdx);
    void ForgetStaAtAp(uint32_t staIdx);
    void StartTransition(uint32_t staIdx, TransitionTarget target, const SteeringEvent& ev);
    void HopToTarget(uint32_t staIdx);
    void OnTransitionTimeout(uint32_t staIdx);
    void FinishTransition(uint32_t staIdx, bool success);
    void OnStaAssoc(uint32_t staIdx, Mac48Address bssid);
    void OnStaDeAssoc(uint32_t staIdx, Mac48Address bssid);
    bool SteerClient(uint32_t clientId, Ptr<EasyMeshRadio> target, SteeringReason reason);
    void SwitchRadioChannel(Ptr<EasyMeshRadio> radio, const EmChannel& channel);
    void ReparentAgent(uint32_t agentId, int32_t newParent, SteeringReason reason);
    uint32_t BstaIndex(uint32_t agentId) const { return m_numClients + agentId; }

    // ── Periodic / event handlers ─────────────────────────────
    void CheckRoaming();
    void OptimizeBackhaul(bool force, SteeringReason reason);
    void HandleAgentOffline();
    void HandleAgentRecover();
    void SetAgentRadiosPower(uint32_t agentId, bool on);
    void UpdateBackhaulMetrics();
    void RebuildRoutes();
    uint32_t AssignIpv4(Ptr<Node> node, Ptr<NetDevice> dev, Ipv4Address addr);
    void UpdateAnimationNodes();

    // ── Traffic installers ────────────────────────────────────
    void InstallUdpUplink();        // STA → Controller  (CBR)
    void InstallUdpDownlink();      // Controller → STA  (CBR)
    void InstallTcpUplink();        // STA (even) → Controller (BulkSend)

    // ── NS-3 containers ──────────────────────────────────────
    NodeContainer m_controllerNode;
    NodeContainer m_agentNodes;
    NodeContainer m_clientNodes;

    Ptr<MultiModelSpectrumChannel> m_channel24;
    Ptr<MultiModelSpectrumChannel> m_channel5;
    Ptr<MultiModelSpectrumChannel> m_channel6;
    std::vector<EmChannel>         m_bhChannels;   // backhaul links: 5 GHz [+ 6 GHz with MLO]

    Ptr<WifiNetDevice>              m_ctrlBap;      // Controller backhaul BSS
    std::vector<Ptr<WifiNetDevice>> m_agentBap;     // Agent backhaul BSS (for children)
    std::vector<ManagedSta>         m_stas;         // [clients..., agent bSTAs...]
    std::map<Mac48Address, BssInfo> m_bssMap;

    // IP address bookkeeping
    Ipv4Address              m_controllerIp;
    uint32_t                 m_ctrlBapIf = 0;
    std::vector<Ipv4Address> m_agentBapIp;
    std::vector<uint32_t>    m_agentBapIf;
    std::vector<Ipv4Address> m_agentBstaIp;
    std::vector<uint32_t>    m_agentBstaIf;
    std::vector<Ipv4Address> m_clientIp;
    std::vector<uint32_t>    m_clientIf;

    std::vector<Vector>      m_agentPositions;
    std::vector<int32_t>     m_agentParent;     // parent the bSTA is associated with
    std::vector<int32_t>     m_plannedParent;   // parent chosen by the backhaul MST
    std::vector<bool>        m_agentOnline;

    // ── EasyMesh logical objects ──────────────────────────────
    Ptr<EasyMeshController>          m_controller;
    std::vector<Ptr<EasyMeshAgent>>  m_agents;

    // ── FlowMonitor ───────────────────────────────────────────
    Ptr<FlowMonitor>   m_monitor;
    FlowMonitorHelper  m_monHelper;
    TrafficStats       m_trafficStats;

    // ── Config ────────────────────────────────────────────────
    double   m_duration   = DURATION;
    uint32_t m_numClients = NUM_STA;
    uint32_t m_numAgents  = NUM_AGENT;
    uint32_t m_scenarioRun = 1;
    std::vector<uint32_t> m_offlineAgentIds = { OFFLINE_AGENT_ID };
    double   m_offlineTime = OFFLINE_TIME;
    double   m_recoverTime = RECOVER_TIME;
    bool     m_pcap       = false;
    MstMetric m_mstMetric = MstMetric::THROUGHPUT;
    ChannelPlanMode m_channelPlanMode = ChannelPlanMode::DYNAMIC;
    bool     m_steering      = true;
    bool     m_roaming       = true;
    bool     m_loadBalancing = true;
    bool     m_netOpt        = true;
    double   m_staMaxSpeed   = STA_MAX_SPEED;
    bool     m_bhMlo         = true;
    bool     m_fhMlo         = true;
    uint32_t m_numMldClients = NUM_MLD_STA;
    bool     m_emlsr         = false;
    uint16_t m_bh6Width      = BH6_WIDTH_MHZ;
    uint32_t m_partialSetups = 0;   // multi-link setups that left links unused
    // Created before the Wi-Fi devices exist: NetAnim calls GetChannel() on
    // every device at start-up, which aborts for a multi-band MLD.
    std::unique_ptr<AnimationInterface> m_animation;

    // ── Static topology ───────────────────────────────────────
    static const Vector CONTROLLER_POS;
};

#endif // EASYMESH_H
