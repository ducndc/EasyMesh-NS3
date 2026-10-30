#include "easymesh.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("EasyMeshSim");

const Vector EasyMeshSimulation::CONTROLLER_POS   = Vector(0.0, 0.0, 1.5);

static const EmChannel kBackhaulChannel{BACKHAUL_CHANNEL_NUMBER, BACKHAUL_WIDTH_MHZ, EmBand::BAND_5GHZ};
static const EmChannel kParkingChannel{PARKING_CHANNEL_NUMBER, PARKING_WIDTH_MHZ, EmBand::BAND_5GHZ};
static const EmChannel kParking6Channel{PARKING6_CHANNEL_NUMBER, 20, EmBand::BAND_6GHZ};

// 6 GHz backhaul channel of the given width, inside 20 MHz channels 1..61
// (20→1, 40→3, 80→7, 160→15, 320→31) so it never overlaps FH6_CHANNELS
static EmChannel
Backhaul6Channel(uint16_t widthMhz)
{
    return EmChannel{static_cast<uint16_t>(widthMhz / 10 - 1), widthMhz, EmBand::BAND_6GHZ};
}

static FrequencyRange
SpectrumRange(EmBand band)
{
    switch (band)
    {
    case EmBand::BAND_2_4GHZ: return WIFI_SPECTRUM_2_4_GHZ;
    case EmBand::BAND_6GHZ:   return WIFI_SPECTRUM_6_GHZ;
    default:                  return WIFI_SPECTRUM_5_GHZ;
    }
}

EasyMeshSimulation::EasyMeshSimulation()
{
    SetNumAgents(NUM_AGENT);
    SetNumClients(NUM_STA);
}

EasyMeshSimulation::~EasyMeshSimulation() {}

void
EasyMeshSimulation::SetNumAgents(uint32_t n)
{
    m_numAgents = n;
    m_agentBap.resize(n);
    m_agentBapIp.resize(n, Ipv4Address::GetAny());
    m_agentBapIf.resize(n, 0);
    m_agentBstaIp.resize(n, Ipv4Address::GetAny());
    m_agentBstaIf.resize(n, 0);
    m_agentPositions.resize(n, CONTROLLER_POS);
    m_agentParent.resize(n, -1);
    m_plannedParent.resize(n, -1);
    m_agentOnline.resize(n, true);
}

void
EasyMeshSimulation::SetNumClients(uint32_t n)
{
    m_numClients = n;
    m_clientIp.resize(n, Ipv4Address::GetAny());
    m_clientIf.resize(n, 0);
}

// ════════════════════════════════════════════════════════════════
//  Radio estimates
// ════════════════════════════════════════════════════════════════

double
EasyMeshSimulation::ComputeDistance(Vector a, Vector b) const
{
    return std::sqrt(std::pow(a.x-b.x,2) +
                     std::pow(a.y-b.y,2) +
                     std::pow(a.z-b.z,2));
}

double
EasyMeshSimulation::TxPowerDbm(const EmChannel& channel)
{
    // 6 GHz LPI: power spectral density limit, so total power grows with width
    if (channel.band == EmBand::BAND_6GHZ)
    {
        return std::min(TX_POWER_6GHZ_MAX_DBM,
                        TX_PSD_6GHZ_DBM_PER_MHZ + 10.0 * std::log10(channel.widthMhz));
    }
    return TX_POWER_DBM;
}

double
EasyMeshSimulation::CcaEdThresholdDbm(uint16_t widthMhz)
{
    // CCA-ED looks at the primary 20 MHz only: keep it below the per-20 MHz
    // power of a PPDU detected at -82 dBm over the whole (wider) channel
    // (see CCA_ED_THRESHOLD_DBM, set for 40 MHz).
    return std::min(CCA_ED_THRESHOLD_DBM,
                    CCA_ED_THRESHOLD_DBM - 10.0 * std::log10(widthMhz / 40.0));
}

double
EasyMeshSimulation::EstimateRssi(Vector a, Vector b, const EmChannel& channel) const
{
    // Same log-distance model the spectrum channels use, so the Controller's
    // estimates match what the PHYs actually receive.
    const double refLoss = (channel.band == EmBand::BAND_2_4GHZ) ? REF_LOSS_2_4GHZ_DB
                         : (channel.band == EmBand::BAND_6GHZ)   ? REF_LOSS_6GHZ_DB
                                                                 : REF_LOSS_5GHZ_DB;
    const double dist = std::max(1.0, ComputeDistance(a, b));
    return TxPowerDbm(channel) - (refLoss + 10.0 * PATHLOSS_EXPONENT * std::log10(dist));
}

double
EasyMeshSimulation::ClientRssi(uint32_t clientId, Ptr<EasyMeshRadio> radio) const
{
    return EstimateRssi(GetClientPosition(clientId), GetAgentPosition(radio->GetAgentId()),
                        radio->GetChannel());
}

double
EasyMeshSimulation::RadioRssi(Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b) const
{
    return EstimateRssi(GetAgentPosition(a->GetAgentId()), GetAgentPosition(b->GetAgentId()),
                        a->GetChannel());
}

uint32_t
EasyMeshSimulation::RssiToMcs(double rssiDbm, uint16_t widthMhz, uint32_t maxMcs)
{
    // HE / EHT minimum-sensitivity style thresholds for 20 MHz (MCS 12/13 =
    // 4096-QAM, EHT only); a wider channel spreads the same power over more
    // noise (-3 dB per doubling).
    static const double kThreshold20[] = {-82, -79, -77, -74, -70, -66, -65,
                                          -64, -59, -57, -54, -52, -49, -46};
    const double effective = rssiDbm - 10.0 * std::log10(widthMhz / 20.0);
    uint32_t mcs = 0;
    for (uint32_t m = 0; m <= std::min<uint32_t>(maxMcs, EHT_MAX_MCS); ++m)
    {
        if (effective >= kThreshold20[m]) mcs = m;
    }
    return mcs;
}

double
EasyMeshSimulation::PhyRateMbps(double rssiDbm, uint16_t widthMhz, uint32_t maxMcs)
{
    // 1 spatial stream, 0.8 us GI, 20 MHz HE/EHT rates scaled by the number
    // of data subcarriers (242-tone RU: 234; 484: 468; 996: 980; 2x996: 1960;
    // 4x996: 3920), so 80+ MHz gain a little more than the width ratio.
    static const double kRate20[] = {8.6, 17.2, 25.8, 34.4, 51.6, 68.8, 77.4,
                                     86.0, 103.2, 114.7, 129.0, 143.4, 154.9, 172.1};
    const double effective = rssiDbm - 10.0 * std::log10(widthMhz / 20.0);
    if (effective < -82.0)
    {
        return 0.0;
    }
    const double dataTones = (widthMhz <= 20) ? 234 : (widthMhz == 40) ? 468 : 980.0 * widthMhz / 80;
    return kRate20[RssiToMcs(rssiDbm, widthMhz, maxMcs)] * dataTones / 234.0;
}

std::vector<EmLinkMetrics>
EasyMeshSimulation::BackhaulLinkMetrics(Vector a, Vector b) const
{
    std::vector<EmLinkMetrics> links;
    for (const auto& channel : m_bhChannels)
    {
        EmLinkMetrics link;
        link.channel  = channel;
        link.rssi     = EstimateRssi(a, b, channel);
        link.mcs      = RssiToMcs(link.rssi, channel.widthMhz);
        link.rateMbps = PhyRateMbps(link.rssi, channel.widthMhz);
        links.push_back(link);
    }
    return links;
}

double
EasyMeshSimulation::BackhaulRate(Vector a, Vector b) const
{
    // MLO with simultaneous transmit/receive: the links add up
    double rate = 0.0;
    for (const auto& link : BackhaulLinkMetrics(a, b))
    {
        rate += link.rateMbps;
    }
    return rate;
}

Vector
EasyMeshSimulation::GetAgentPosition(uint32_t agentId) const
{
    if (agentId < m_agentNodes.GetN())
    {
        auto mobility = m_agentNodes.Get(agentId)->GetObject<MobilityModel>();
        if (mobility)
        {
            return mobility->GetPosition();
        }
    }
    return m_agentPositions.at(agentId);
}

Vector
EasyMeshSimulation::GetClientPosition(uint32_t clientId) const
{
    return m_clientNodes.Get(clientId)->GetObject<MobilityModel>()->GetPosition();
}

bool
EasyMeshSimulation::IsAgentOnline(uint32_t agentId) const
{
    return agentId < m_agentOnline.size() && m_agentOnline[agentId];
}

Ptr<WifiNetDevice>
EasyMeshSimulation::BackhaulDevice(int32_t parent) const
{
    return (parent < 0) ? m_ctrlBap : m_agentBap[parent];
}

Mac48Address
EasyMeshSimulation::BackhaulBssid(int32_t parent) const
{
    // bBSS address, i.e. the AP MLD address with an MLO backhaul
    return BackhaulDevice(parent)->GetMac()->GetAddress();
}

std::set<Mac48Address>
EasyMeshSimulation::LinkBssids(Ptr<WifiNetDevice> dev)
{
    std::set<Mac48Address> bssids;
    for (uint8_t linkId : dev->GetMac()->GetLinkIds())
    {
        bssids.insert(dev->GetMac()->GetFrameExchangeManager(linkId)->GetAddress());
    }
    return bssids;
}

// ════════════════════════════════════════════════════════════════
//  Agent placement
// ════════════════════════════════════════════════════════════════

void
EasyMeshSimulation::GenerateRandomAgentPositions()
{
    auto angleVar = CreateObject<UniformRandomVariable>();
    auto radiusVar = CreateObject<UniformRandomVariable>();
    angleVar->SetAttribute("Min", DoubleValue(0.0));
    angleVar->SetAttribute("Max", DoubleValue(2.0 * M_PI));

    const double minRadius = std::min(8.0, TOPOLOGY_TARGET_RADIUS * 0.5);
    const double minRadiusSquared = minRadius * minRadius;
    const double maxRadiusSquared = TOPOLOGY_MAX_RADIUS * TOPOLOGY_MAX_RADIUS;
    radiusVar->SetAttribute("Min", DoubleValue(0.0));
    radiusVar->SetAttribute("Max", DoubleValue(1.0));

    for (uint32_t agentId = 0; agentId < m_numAgents; ++agentId)
    {
        const double angle = angleVar->GetValue();
        const double radius = std::sqrt(minRadiusSquared +
                                        radiusVar->GetValue() *
                                            (maxRadiusSquared - minRadiusSquared));

        m_agentPositions[agentId] = Vector(
            std::clamp(CONTROLLER_POS.x + radius * std::cos(angle),
                       TOPOLOGY_AREA_MIN_X,
                       TOPOLOGY_AREA_MAX_X),
            std::clamp(CONTROLLER_POS.y + radius * std::sin(angle),
                       TOPOLOGY_AREA_MIN_Y,
                       TOPOLOGY_AREA_MAX_Y),
            1.5);
    }
}

void
EasyMeshSimulation::OptimizeNetworkTopology()
{
    const auto clampCoordinate = [](double value, double lower, double upper) {
        return std::max(lower, std::min(value, upper));
    };

    for (uint32_t iteration = 0; iteration < 15; ++iteration)
    {
        for (uint32_t agentId = 0; agentId < m_numAgents; ++agentId)
        {
            Vector delta = m_agentPositions[agentId] - CONTROLLER_POS;
            double distance = ComputeDistance(m_agentPositions[agentId], CONTROLLER_POS);
            if (distance > TOPOLOGY_MAX_RADIUS)
            {
                double scale = TOPOLOGY_TARGET_RADIUS / distance;
                m_agentPositions[agentId].x = CONTROLLER_POS.x + delta.x * scale;
                m_agentPositions[agentId].y = CONTROLLER_POS.y + delta.y * scale;
            }

            m_agentPositions[agentId].x = clampCoordinate(m_agentPositions[agentId].x,
                                                          TOPOLOGY_AREA_MIN_X,
                                                          TOPOLOGY_AREA_MAX_X);
            m_agentPositions[agentId].y = clampCoordinate(m_agentPositions[agentId].y,
                                                          TOPOLOGY_AREA_MIN_Y,
                                                          TOPOLOGY_AREA_MAX_Y);
        }

        for (uint32_t left = 0; left < m_numAgents; ++left)
        {
            for (uint32_t right = left + 1; right < m_numAgents; ++right)
            {
                Vector diff = m_agentPositions[right] - m_agentPositions[left];
                double distance = ComputeDistance(m_agentPositions[left], m_agentPositions[right]);
                if (distance >= TOPOLOGY_MIN_AGENT_SEPARATION)
                {
                    continue;
                }

                if (distance < 1e-6)
                {
                    diff = Vector(1.0 + right, 0.5 + left, 0.0);
                    distance = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                }

                double push = (TOPOLOGY_MIN_AGENT_SEPARATION - distance) * 0.5;
                Vector unit(diff.x / distance, diff.y / distance, 0.0);
                m_agentPositions[left].x -= unit.x * push;
                m_agentPositions[left].y -= unit.y * push;
                m_agentPositions[right].x += unit.x * push;
                m_agentPositions[right].y += unit.y * push;
            }
        }
    }

    for (uint32_t agentId = 0; agentId < m_numAgents; ++agentId)
    {
        NS_LOG_INFO("Optimized Agent" << agentId << " position=("
                    << m_agentPositions[agentId].x << ","
                    << m_agentPositions[agentId].y << ")");
    }
}

// ════════════════════════════════════════════════════════════════
//  Backhaul tree (network optimization)
// ════════════════════════════════════════════════════════════════

double
EasyMeshSimulation::EdgeWeight(Vector a, Vector b) const
{
    // RSSI: signal of the 5 GHz (first) backhaul link; THROUGHPUT: sum of the
    // expected rates of all backhaul links (MLO)
    return (m_mstMetric == MstMetric::RSSI) ? EstimateRssi(a, b, m_bhChannels.front())
                                            : BackhaulRate(a, b);
}

std::vector<int32_t>
EasyMeshSimulation::ComputeBackhaulTree() const
{
    // ── Prim's Maximum-Spanning-Tree algorithm ─────────────────
    // Selects, for every online agent, the best possible uplink AP (either
    // the Controller or another already-connected agent). The edge
    // weight is pluggable via m_mstMetric:
    //   • MstMetric::THROUGHPUT — maximizes total achievable Wi-Fi
    //     throughput (Mbps) across the whole backhaul mesh.
    //   • MstMetric::RSSI       — maximizes total/weakest-link signal
    //     strength (dBm) across the mesh, ignoring MCS/throughput.
    //
    // Graph: kNodeCount = m_numAgents + 1 nodes — index [0, m_numAgents)
    // are agents, index m_numAgents is a virtual node for the Controller
    // (the MST root).
    //
    // This is the textbook Prim's MST procedure with weights maximized
    // instead of minimized: starting from the root, repeatedly attach the
    // outside node reachable by the single strongest direct edge from any
    // node already in the tree. Complexity is O(m_numAgents^2).
    std::vector<int32_t> parent(m_numAgents, -1);

    const uint32_t kControllerNode = m_numAgents;   // virtual root index
    const uint32_t kNodeCount      = m_numAgents + 1;

    auto positionOf = [this, kControllerNode](uint32_t node) -> Vector {
        return (node == kControllerNode) ? CONTROLLER_POS : GetAgentPosition(node);
    };

    const bool useRssi = (m_mstMetric == MstMetric::RSSI);
    const char* metricUnit  = useRssi ? "dBm"  : "Mbps";
    const char* metricLabel = useRssi ? "RSSI" : "capacity";

    std::vector<bool>    inTree(kNodeCount, false);
    std::vector<double>  bestWeight(kNodeCount, -std::numeric_limits<double>::infinity());
    std::vector<int32_t> bestParent(kNodeCount, -1);

    inTree[kControllerNode] = true;

    // Seed frontier with direct Controller→Agent edges.
    for (uint32_t node = 0; node < m_numAgents; ++node)
    {
        if (!IsAgentOnline(node))
        {
            continue;
        }
        bestWeight[node] = EdgeWeight(CONTROLLER_POS, positionOf(node));
        bestParent[node] = static_cast<int32_t>(kControllerNode);
    }

    for (uint32_t step = 0; step < m_numAgents; ++step)
    {
        // Pick the outside node with the strongest edge into the current tree.
        int32_t chosen       = -1;
        double  chosenWeight = -std::numeric_limits<double>::infinity();
        for (uint32_t node = 0; node < m_numAgents; ++node)
        {
            if (inTree[node] || !IsAgentOnline(node))
            {
                continue;
            }
            if (bestWeight[node] > chosenWeight)
            {
                chosenWeight = bestWeight[node];
                chosen       = static_cast<int32_t>(node);
            }
        }

        if (chosen < 0)
        {
            break; // remaining agents are offline / unreachable
        }

        inTree[chosen] = true;
        parent[chosen] = (bestParent[chosen] == static_cast<int32_t>(kControllerNode))
                             ? -1
                             : bestParent[chosen];

        NS_LOG_DEBUG("MST(" << metricLabel << "): Backhaul parent for Agent" << chosen << " is "
                     << (parent[chosen] < 0 ? std::string("Controller")
                                            : "Agent" + std::to_string(parent[chosen]))
                     << "  (" << metricLabel << "=" << chosenWeight << metricUnit << ")");

        // Relax: does the newly attached node offer a stronger direct edge
        // to any node still outside the tree?
        for (uint32_t node = 0; node < m_numAgents; ++node)
        {
            if (inTree[node] || !IsAgentOnline(node))
            {
                continue;
            }

            const auto w = EdgeWeight(positionOf(chosen), positionOf(node));

            if (w > bestWeight[node])
            {
                bestWeight[node] = w;
                bestParent[node] = static_cast<int32_t>(chosen);
            }
        }
    }
    return parent;
}

double
EasyMeshSimulation::TreeWeight(const std::vector<int32_t>& parents) const
{
    double total = 0.0;
    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        if (!IsAgentOnline(a)) continue;
        const int32_t p = parents[a];
        if (p >= 0 && !IsAgentOnline(p))
        {
            return -std::numeric_limits<double>::infinity();   // orphaned: tree is broken
        }
        total += EdgeWeight(p < 0 ? CONTROLLER_POS : GetAgentPosition(p), GetAgentPosition(a));
    }
    return total;
}

void
EasyMeshSimulation::UpdateBackhaulMetrics()
{
    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        // Link object reflects the physical association
        const int32_t parent = m_agentParent[a];
        const Vector parentPos = (parent < 0) ? CONTROLLER_POS : GetAgentPosition(parent);
        const auto links = BackhaulLinkMetrics(parentPos, GetAgentPosition(a));
        double rate = 0.0;
        for (const auto& l : links)
        {
            rate += l.rateMbps;
        }
        auto link = m_agents[a]->GetBackhaulLink();
        link->SetEndpoints(parent < 0 ? m_controllerNode.Get(0) : m_agentNodes.Get(parent),
                           m_agentNodes.Get(a));
        link->UpdateMetrics(links.front().rssi, links.front().rssi - (-95.0), links.front().mcs,
                            rate, 0.0);
        link->SetAffiliatedLinks(links);
        link->SetActive(IsAgentOnline(a));
        m_agents[a]->SetParent(parent);

        // Path capacity along the planned tree: all hops share the backhaul
        // channel(s), so airtime adds up → C = 1 / Σ(1 / rate_hop), where an
        // MLO hop's rate is the sum over its links.
        double inverse = 0.0;
        bool reachable = IsAgentOnline(a);
        uint32_t current = a;
        for (uint32_t hops = 0; reachable && hops <= m_numAgents; ++hops)
        {
            const int32_t p = m_plannedParent[current];
            const Vector pp = (p < 0) ? CONTROLLER_POS : GetAgentPosition(p);
            const double rate = BackhaulRate(pp, GetAgentPosition(current));
            if (rate <= 0.0) { reachable = false; break; }
            inverse += 1.0 / rate;
            if (p < 0) break;
            if (!IsAgentOnline(p) || hops == m_numAgents) { reachable = false; break; }
            current = p;
        }
        m_agents[a]->SetBackhaulCapacity(reachable && inverse > 0 ? 1.0 / inverse : 0.0);
    }
}

void
EasyMeshSimulation::OptimizeBackhaul(bool force, SteeringReason reason)
{
    const auto tree = ComputeBackhaulTree();
    const double current = TreeWeight(m_agentParent);
    const double proposed = TreeWeight(tree);
    const bool broken = !std::isfinite(current);

    if (!force && !broken && proposed - current <= NETOPT_MIN_GAIN * std::abs(current))
    {
        NS_LOG_DEBUG("NETWORK_OPT: current tree kept (" << current << " vs " << proposed << ")");
        return;
    }

    m_plannedParent = tree;
    UpdateBackhaulMetrics();

    uint32_t changes = 0;
    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        if (!IsAgentOnline(a) || tree[a] == m_agentParent[a]) continue;
        const auto& sta = m_stas[BstaIndex(a)];
        if (sta.pending && sta.target.expected == BackhaulBssid(tree[a])) continue;
        ReparentAgent(a, tree[a], reason);
        changes++;
    }

    NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds() << "] NETWORK_OPT ("
                << ReasonStr(reason) << "): tree weight "
                << (broken ? std::string("broken") : std::to_string(current)) << " -> " << proposed
                << ", " << changes << " backhaul STA(s) re-parented");
}

// ════════════════════════════════════════════════════════════════
//  Topology construction
// ════════════════════════════════════════════════════════════════

void
EasyMeshSimulation::CreateNodes()
{
    m_controllerNode.Create(NUM_CONTROLLER);
    m_agentNodes.Create(m_numAgents);
    m_clientNodes.Create(m_numClients);
    NS_LOG_INFO("Nodes: " << NUM_CONTROLLER << " controller + "<< m_numAgents << " agents + " << m_numClients << " clients");
}

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

    GenerateRandomAgentPositions();
    OptimizeNetworkTopology();

    // Agents – optimized after random placement
    Ptr<ListPositionAllocator> aa = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < m_numAgents; i++)
        aa->Add(m_agentPositions[i]);
    mob.SetPositionAllocator(aa);
    mob.Install(m_agentNodes);

    // Clients – random start, then RandomWaypoint inside the coverage area
    std::ostringstream xRange, yRange;
    xRange << "ns3::UniformRandomVariable[Min=" << TOPOLOGY_AREA_MIN_X << "|Max=" << TOPOLOGY_AREA_MAX_X << "]";
    yRange << "ns3::UniformRandomVariable[Min=" << TOPOLOGY_AREA_MIN_Y << "|Max=" << TOPOLOGY_AREA_MAX_Y << "]";
    auto area = CreateObject<RandomRectanglePositionAllocator>();
    area->SetAttribute("X", StringValue(xRange.str()));
    area->SetAttribute("Y", StringValue(yRange.str()));
    area->SetAttribute("Z", DoubleValue(1.0));

    MobilityHelper mobClient;
    mobClient.SetPositionAllocator(area);
    if (m_staMaxSpeed > 0.0)
    {
        std::ostringstream speed, pause;
        speed << "ns3::UniformRandomVariable[Min=" << std::min(STA_MIN_SPEED, m_staMaxSpeed)
              << "|Max=" << m_staMaxSpeed << "]";
        pause << "ns3::UniformRandomVariable[Min=0|Max=" << STA_MAX_PAUSE << "]";
        mobClient.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                   "Speed", StringValue(speed.str()),
                                   "Pause", StringValue(pause.str()),
                                   "PositionAllocator", PointerValue(area));
    }
    else
    {
        mobClient.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
    mobClient.Install(m_clientNodes);

    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        const auto pos = GetClientPosition(c);
        NS_LOG_INFO("STA" << c << " start position=(" << pos.x << "," << pos.y << ")"
                    << (m_staMaxSpeed > 0.0 ? "  mobile" : "  static"));
    }
}

void
EasyMeshSimulation::CreateSpectrumChannels()
{
    // One spectrum channel per band with its own reference path loss, so
    // 2.4 GHz naturally reaches further than 5 GHz. Within a band the
    // spectrum PHY only decodes PPDUs overlapping its primary channel, so
    // the fronthaul channel plan really separates BSSs.
    auto makeChannel = [](double refLoss) {
        auto channel = CreateObject<MultiModelSpectrumChannel>();
        auto loss = CreateObject<LogDistancePropagationLossModel>();
        loss->SetAttribute("Exponent", DoubleValue(PATHLOSS_EXPONENT));
        loss->SetAttribute("ReferenceLoss", DoubleValue(refLoss));
        channel->AddPropagationLossModel(loss);
        channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
        // Do not deliver PPDUs to receivers whose channel they cannot overlap
        // (e.g. 5 GHz fronthaul vs. backhaul): big speed-up, no effect on results.
        channel->AddSpectrumTransmitFilter(CreateObject<WifiBandwidthFilter>());
        return channel;
    };
    m_channel24 = makeChannel(REF_LOSS_2_4GHZ_DB);
    m_channel5  = makeChannel(REF_LOSS_5GHZ_DB);
    m_channel6  = makeChannel(REF_LOSS_6GHZ_DB);
}

Ptr<MultiModelSpectrumChannel>
EasyMeshSimulation::SpectrumChannel(EmBand band) const
{
    switch (band)
    {
    case EmBand::BAND_2_4GHZ: return m_channel24;
    case EmBand::BAND_6GHZ:   return m_channel6;
    default:                  return m_channel5;
    }
}

void
EasyMeshSimulation::ConfigureLinks(SpectrumWifiPhyHelper& phy,
                                   const std::vector<EmChannel>& links,
                                   bool fixedBand) const
{
    // Each link's PHY only attaches to its own band's spectrum channel and
    // never leaves that band (an MLD has one radio per link).
    for (uint8_t linkId = 0; linkId < links.size(); ++linkId)
    {
        const auto& channel = links[linkId];
        phy.AddChannel(SpectrumChannel(channel.band), SpectrumRange(channel.band));
        if (fixedBand)
        {
            phy.AddPhyToFreqRangeMapping(linkId, SpectrumRange(channel.band));
        }
        phy.Set(linkId, "ChannelSettings", StringValue(channel.ToSettings()));
        phy.Set(linkId, "FixedPhyBand", BooleanValue(fixedBand));
        phy.Set(linkId, "TxPowerStart", DoubleValue(TxPowerDbm(channel)));
        phy.Set(linkId, "TxPowerEnd", DoubleValue(TxPowerDbm(channel)));
        phy.Set(linkId, "CcaEdThreshold", DoubleValue(CcaEdThresholdDbm(channel.widthMhz)));
    }
}

void
EasyMeshSimulation::RegisterBss(Ptr<WifiNetDevice> dev, int32_t agentId, bool backhaul,
                                const std::vector<EmBand>& linkBands)
{
    // A single-link device's link address is its device address
    m_bssMap[dev->GetMac()->GetAddress()] = {agentId, backhaul, linkBands.front(), dev->GetNPhys() > 1};
    for (uint8_t linkId = 0; linkId < dev->GetNPhys(); ++linkId)
    {
        m_bssMap[dev->GetMac()->GetFrameExchangeManager(linkId)->GetAddress()] =
            {agentId, backhaul, linkBands[linkId], false};
    }
}

void
EasyMeshSimulation::InstallBackhaulDevices()
{
    // Every node runs a backhaul BSS (bBSS) on the dedicated backhaul
    // channel(s); every Agent also has a backhaul STA (bSTA) pinned to its
    // parent's bBSS. With MLO both are Wi-Fi 7 MLDs with a 5 GHz and a 6 GHz
    // link, and the bSTA sets up both links with its parent AP MLD.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    SpectrumWifiPhyHelper phy(m_bhChannels.size());
    ConfigureLinks(phy, m_bhChannels);
    std::vector<EmBand> linkBands;
    for (const auto& channel : m_bhChannels)
    {
        linkBands.push_back(channel.band);
    }

    WifiMacHelper mac;
    const Ssid ssid("EasyMesh-Backhaul");
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid),
                "BeaconInterval", TimeValue(MicroSeconds(WIFI_BEACON_INTERVAL_US)));

    m_ctrlBap = DynamicCast<WifiNetDevice>(wifi.Install(phy, mac, m_controllerNode.Get(0)).Get(0));
    RegisterBss(m_ctrlBap, -1, true, linkBands);
    NetDeviceContainer pcapDevices(m_ctrlBap);

    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        m_agentBap[a] = DynamicCast<WifiNetDevice>(wifi.Install(phy, mac, m_agentNodes.Get(a)).Get(0));
        RegisterBss(m_agentBap[a], static_cast<int32_t>(a), true, linkBands);
        pcapDevices.Add(m_agentBap[a]);
    }

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(true));
    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        auto dev = DynamicCast<WifiNetDevice>(wifi.Install(phy, mac, m_agentNodes.Get(a)).Get(0));
        auto& sta = m_stas[BstaIndex(a)];
        sta.device     = dev;
        sta.mac        = DynamicCast<StaWifiMac>(dev->GetMac());
        sta.assocMgr   = CreateObject<EasyMeshAssocManager>();
        sta.isBackhaul = true;
        sta.ownerId    = a;
        sta.linkBands  = linkBands;
        sta.mac->SetAssocManager(sta.assocMgr);
        sta.assocMgr->SetTargetBssids(LinkBssids(BackhaulDevice(m_plannedParent[a])));
        pcapDevices.Add(dev);

        auto link = Create<EasyMeshLink>(a, m_controllerNode.Get(0), m_agentNodes.Get(a),
                                         LinkType::BACKHAUL, 5.0);
        m_agents[a]->SetBackhaulLink(link);

        NS_LOG_INFO("BH: Agent" << a << " bSTA -> "
                    << (m_plannedParent[a] < 0 ? std::string("Controller")
                                               : "Agent" + std::to_string(m_plannedParent[a]))
                    << " bBSS");
    }

    if (m_pcap) phy.EnablePcap("easymesh-bh", pcapDevices);
}

void
EasyMeshSimulation::InstallFronthaulDevices()
{
    // Tri-band Agents: a 2.4, a 5 and a 6 GHz fronthaul BSS, same SSID.
    // With MLO they are the three affiliated APs of one Wi-Fi 7 AP MLD
    // (one device, one IP interface); otherwise three single-link APs.
    // Radios start on the first channel of their band; the channel planner
    // (if enabled) retunes them before the simulation starts.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    if (m_emlsr)
    {
        wifi.ConfigEhtOptions("EmlsrActivated", BooleanValue(true));
    }

    WifiMacHelper mac;
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(Ssid("EasyMesh-Fronthaul")),
                "BeaconInterval", TimeValue(MicroSeconds(WIFI_BEACON_INTERVAL_US)));

    const std::vector<uint16_t> ch24 FH24_CHANNELS;
    const std::vector<uint16_t> ch5  FH5_CHANNELS;
    const std::vector<uint16_t> ch6  FH6_CHANNELS;
    // Link order matters for the AP MLD: at multi-link setup a non-AP MLD
    // takes over the AP's link IDs, and ns-3.48 rescans assuming link IDs
    // 0..n-1 (WifiDefaultAssocManager::DoStartScanning). Putting the bands of
    // the Wi-Fi 7 clients (5 + 6 GHz) first keeps their link IDs at 0 and 1.
    const std::vector<EmChannel> channels = {
        EmChannel{ch5.front(),  FH5_WIDTH_MHZ,  EmBand::BAND_5GHZ},
        EmChannel{ch6.front(),  FH6_WIDTH_MHZ,  EmBand::BAND_6GHZ},
        EmChannel{ch24.front(), FH24_WIDTH_MHZ, EmBand::BAND_2_4GHZ},
    };

    // One device per entry: all three bands (AP MLD) or one band each
    std::vector<std::vector<EmChannel>> devices;
    if (m_fhMlo)
        devices.push_back(channels);
    else
        for (const auto& channel : channels) devices.push_back({channel});

    for (const auto& links : devices)
    {
        SpectrumWifiPhyHelper phy(links.size());
        ConfigureLinks(phy, links);
        std::vector<EmBand> linkBands;
        for (const auto& channel : links)
        {
            linkBands.push_back(channel.band);
        }

        NetDeviceContainer pcapDevices;
        for (uint32_t a = 0; a < m_numAgents; ++a)
        {
            auto dev = DynamicCast<WifiNetDevice>(wifi.Install(phy, mac, m_agentNodes.Get(a)).Get(0));
            for (uint8_t linkId = 0; linkId < links.size(); ++linkId)
            {
                auto radio = Create<EasyMeshRadio>(a, links[linkId].band, dev, linkId, links[linkId]);
                radio->StartMonitoring();
                m_agents[a]->AddRadio(radio);
            }
            RegisterBss(dev, static_cast<int32_t>(a), false, linkBands);
            pcapDevices.Add(dev);
        }
        if (m_pcap)
        {
            phy.EnablePcap(links.size() > 1 ? "easymesh-fh"
                                            : std::string("easymesh-fh") + BandStr(links[0].band),
                           pcapDevices);
        }
    }
}

Ptr<EasyMeshRadio>
EasyMeshSimulation::SelectInitialRadio(uint32_t clientId) const
{
    auto radio = m_controller->SelectBestRadio(clientId, nullptr, BandFilter::ANY, MIN_ASSOC_RSSI_DBM);
    if (!radio)
    {
        radio = m_controller->SelectBestRadio(clientId, nullptr, BandFilter::ANY,
                                              -std::numeric_limits<double>::infinity());
    }
    return radio;
}

void
EasyMeshSimulation::InstallClientDevice(uint32_t c, NetDeviceContainer& pcapDevices)
{
    // Clients [0, m_numMldClients) are Wi-Fi 7: with fronthaul MLO a 5 + 6 GHz
    // MLD (STR, or EMLSR), else a single-radio tri-band EHT STA. The others
    // are single-radio dual-band (2.4 / 5 GHz) Wi-Fi 6 STAs. A single-radio
    // PHY is attached to all its band channels and retunes between them.
    ClientCaps caps;
    if (c < m_numMldClients)
    {
        caps.eht   = true;
        caps.mlo   = m_fhMlo;
        caps.bands = m_fhMlo ? std::vector<EmBand>{EmBand::BAND_5GHZ, EmBand::BAND_6GHZ}
                             : std::vector<EmBand>{EmBand::BAND_5GHZ, EmBand::BAND_6GHZ,
                                                   EmBand::BAND_2_4GHZ};
    }
    else
    {
        caps.bands = {EmBand::BAND_5GHZ, EmBand::BAND_2_4GHZ};
    }
    m_controller->SetClientCaps(c, caps);

    // Initial association: Controller-guided choice of BSS. The PHYs are
    // created directly on its channels (no band change before start).
    auto radio  = SelectInitialRadio(c);
    auto target = ClientTarget(c, radio);

    WifiHelper wifi;
    wifi.SetStandard(caps.eht ? WIFI_STANDARD_80211be : WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    WifiMacHelper mac;
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(Ssid("EasyMesh-Fronthaul")),
                "ActiveProbing", BooleanValue(true));

    Ptr<WifiNetDevice> dev;
    auto& sta = m_stas[c];
    if (caps.mlo)
    {
        if (m_emlsr)
        {
            wifi.ConfigEhtOptions("EmlsrActivated", BooleanValue(true));
            mac.SetEmlsrManager("ns3::DefaultEmlsrManager", "EmlsrLinkSet", StringValue("0,1"));
        }
        SpectrumWifiPhyHelper phy(target.channels.size());
        ConfigureLinks(phy, target.channels, !m_emlsr);
        dev = DynamicCast<WifiNetDevice>(wifi.Install(phy, mac, m_clientNodes.Get(c)).Get(0));
        sta.linkBands = caps.bands;
    }
    else
    {
        SpectrumWifiPhyHelper phy;
        uint16_t maxWidth = 20;
        for (EmBand band : caps.bands)
        {
            phy.AddChannel(SpectrumChannel(band), SpectrumRange(band));
            maxWidth = std::max<uint16_t>(maxWidth, band == EmBand::BAND_2_4GHZ ? FH24_WIDTH_MHZ
                                                    : band == EmBand::BAND_5GHZ ? FH5_WIDTH_MHZ
                                                                                : FH6_WIDTH_MHZ);
        }
        phy.Set("ChannelSettings", StringValue(radio->GetChannel().ToSettings()));
        phy.Set("TxPowerStart", DoubleValue(TX_POWER_DBM));
        phy.Set("TxPowerEnd",   DoubleValue(TX_POWER_DBM));
        phy.Set("CcaEdThreshold", DoubleValue(CcaEdThresholdDbm(maxWidth)));
        // a single-radio client only listens on the band it is tuned to
        phy.Set("TrackSignalsFromInactiveInterfaces", BooleanValue(false));
        dev = DynamicCast<WifiNetDevice>(wifi.Install(phy, mac, m_clientNodes.Get(c)).Get(0));
    }

    sta.device   = dev;
    sta.mac      = DynamicCast<StaWifiMac>(dev->GetMac());
    sta.assocMgr = CreateObject<EasyMeshAssocManager>();
    sta.ownerId  = c;
    sta.mac->SetAssocManager(sta.assocMgr);
    sta.assocMgr->SetTargetBssids(target.bssids);
    pcapDevices.Add(dev);

    NS_LOG_INFO("STA" << c << (caps.eht ? (caps.mlo ? " [Wi-Fi 7 MLD]" : " [Wi-Fi 7]") : " [Wi-Fi 6]")
                << " initial BSS A" << radio->GetAgentId() << "/"
                << (caps.mlo ? std::string("MLD") : BandStr(radio->GetBand()))
                << " ch" << radio->GetChannel().number
                << "  RSSI=" << ClientRssi(c, radio) << "dBm");
}

void
EasyMeshSimulation::InstallClientDevices()
{
    NetDeviceContainer pcapDevices;
    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        InstallClientDevice(c, pcapDevices);
    }
    if (m_pcap) SpectrumWifiPhyHelper().EnablePcap("easymesh-sta", pcapDevices);
}

void
EasyMeshSimulation::BuildTopology()
{
    NS_LOG_INFO("=== BuildTopology ===");
    m_bhChannels = {kBackhaulChannel};
    if (m_bhMlo)
    {
        m_bhChannels.push_back(Backhaul6Channel(m_bh6Width));
    }
    CreateNodes();
    InstallMobility();
    std::ostringstream animationFile;
    animationFile << "easymesh-sim-animation-run" << m_scenarioRun << ".xml";
    m_animation = std::make_unique<AnimationInterface>(animationFile.str());
    CreateSpectrumChannels();

    m_plannedParent = ComputeBackhaulTree();
    m_agentParent   = m_plannedParent;

    // Logical EasyMesh objects
    m_controller = Create<EasyMeshController>(0, m_controllerNode.Get(0));
    m_controller->SetNumClients(m_numClients);
    m_controller->EnableSteering(m_steering);
    m_controller->EnableLoadBalancing(m_loadBalancing);
    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        auto agent = Create<EasyMeshAgent>(a,
                                           m_agentNodes.Get(a),
                                           AgentRole::HYBRID,
                                           GetAgentPosition(a));
        m_agents.push_back(agent);
        m_controller->RegisterAgent(agent);
    }

    ControllerOps ops;
    ops.clientRssi    = [this](uint32_t c, Ptr<EasyMeshRadio> r) { return ClientRssi(c, r); };
    ops.radioRssi     = [this](Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b) { return RadioRssi(a, b); };
    ops.phyRate       = [](double rssi, uint16_t width, uint32_t maxMcs) {
        return PhyRateMbps(rssi, width, maxMcs);
    };
    ops.steerClient   = [this](uint32_t c, Ptr<EasyMeshRadio> r, SteeringReason reason) {
        return SteerClient(c, r, reason);
    };
    ops.switchChannel = [this](Ptr<EasyMeshRadio> r, const EmChannel& ch) { SwitchRadioChannel(r, ch); };
    m_controller->SetOps(ops);

    m_stas.resize(m_numClients + m_numAgents);
    InstallBackhaulDevices();
    InstallFronthaulDevices();
    UpdateBackhaulMetrics();

    if (m_channelPlanMode == ChannelPlanMode::NONE)
        m_controller->RecordInitialChannels();
    else
        m_controller->PlanChannels(true);

    InstallClientDevices();

    for (uint32_t i = 0; i < m_stas.size(); ++i)
    {
        m_stas[i].mac->TraceConnectWithoutContext(
            "Assoc", MakeCallback(&EasyMeshSimulation::OnStaAssoc, this).Bind(i));
        m_stas[i].mac->TraceConnectWithoutContext(
            "DeAssoc", MakeCallback(&EasyMeshSimulation::OnStaDeAssoc, this).Bind(i));
    }
    NS_LOG_INFO("=== Topology ready ===");
}

// ════════════════════════════════════════════════════════════════
//  IP layer
// ════════════════════════════════════════════════════════════════

uint32_t
EasyMeshSimulation::AssignIpv4(Ptr<Node> node, Ptr<NetDevice> dev, Ipv4Address addr)
{
    auto ipv4 = node->GetObject<Ipv4>();
    int32_t ifIndex = ipv4->GetInterfaceForDevice(dev);
    if (ifIndex < 0)
    {
        ifIndex = ipv4->AddInterface(dev);
    }
    ipv4->AddAddress(ifIndex, Ipv4InterfaceAddress(addr, Ipv4Mask("255.255.0.0")));
    ipv4->SetMetric(ifIndex, 1);
    ipv4->SetForwarding(ifIndex, true);
    ipv4->SetUp(ifIndex);
    return static_cast<uint32_t>(ifIndex);
}

void
EasyMeshSimulation::InstallProtocolStack()
{
    NS_LOG_INFO("=== Install Protocol Stack ===");

    InternetStackHelper inet;
    inet.Install(m_controllerNode);
    inet.Install(m_agentNodes);
    inet.Install(m_clientNodes);

    // Backhaul 10.1.0.0/16: Controller 10.1.0.1, Agent a bBSS 10.1.(a+1).1, bSTA 10.1.(a+1).2
    // Fronthaul 10.2.0.0/16: Agent a 2.4G 10.2.1.(a+1), 5G 10.2.2.(a+1), 6G 10.2.3.(a+1)
    //   or one AP MLD 10.2.4.(a+1); clients 10.2.10+.x
    // All forwarding uses explicit host/default routes (RebuildRoutes), so the
    // shared /16 on several interfaces of one node is never ambiguous.
    auto addr = [](uint32_t b2, uint32_t b3, uint32_t b4) {
        return Ipv4Address((10u << 24) | (b2 << 16) | (b3 << 8) | b4);
    };

    m_controllerIp = addr(1, 0, 1);
    m_ctrlBapIf = AssignIpv4(m_controllerNode.Get(0), m_ctrlBap, m_controllerIp);

    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        m_agentBapIp[a]  = addr(1, a + 1, 1);
        m_agentBstaIp[a] = addr(1, a + 1, 2);
        m_agentBapIf[a]  = AssignIpv4(m_agentNodes.Get(a), m_agentBap[a], m_agentBapIp[a]);
        m_agentBstaIf[a] = AssignIpv4(m_agentNodes.Get(a), m_stas[BstaIndex(a)].device, m_agentBstaIp[a]);

        // The links of an AP MLD share its device, hence its IP interface
        std::map<Ptr<WifiNetDevice>, std::pair<Ipv4Address, uint32_t>> fhIf;
        for (auto& radio : m_agents[a]->GetRadios())
        {
            auto it = fhIf.find(radio->GetDevice());
            if (it == fhIf.end())
            {
                const uint32_t net = radio->IsMldLink() ? 4
                                   : radio->GetBand() == EmBand::BAND_2_4GHZ ? 1
                                   : radio->GetBand() == EmBand::BAND_5GHZ   ? 2
                                                                             : 3;
                const auto ip = addr(2, net, a + 1);
                it = fhIf.emplace(radio->GetDevice(),
                                  std::make_pair(ip, AssignIpv4(m_agentNodes.Get(a), radio->GetDevice(), ip)))
                         .first;
            }
            radio->SetIpv4(it->second.first, it->second.second);
        }
    }

    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        m_clientIp[c] = addr(2, 10 + c / 250, c % 250 + 1);
        m_clientIf[c] = AssignIpv4(m_clientNodes.Get(c), m_stas[c].device, m_clientIp[c]);
    }

    RebuildRoutes();
    NS_LOG_INFO("=== END Protocol Stack ===");
}

void
EasyMeshSimulation::RebuildRoutes()
{
    Ipv4StaticRoutingHelper helper;
    auto routingOf = [&](Ptr<Node> node) { return helper.GetStaticRouting(node->GetObject<Ipv4>()); };

    // Drop every host/default route we installed; keep connected networks
    auto clear = [&](Ptr<Node> node) {
        auto routing = routingOf(node);
        for (int32_t i = static_cast<int32_t>(routing->GetNRoutes()) - 1; i >= 0; --i)
        {
            const auto route = routing->GetRoute(static_cast<uint32_t>(i));
            if (route.IsHost() || route.IsDefault())
            {
                routing->RemoveRoute(static_cast<uint32_t>(i));
            }
        }
    };
    clear(m_controllerNode.Get(0));
    for (uint32_t a = 0; a < m_numAgents; ++a) clear(m_agentNodes.Get(a));
    for (uint32_t c = 0; c < m_numClients; ++c) clear(m_clientNodes.Get(c));

    // Uplink: every online Agent points at its (associated) parent bBSS
    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        const int32_t p = m_agentParent[a];
        if (!IsAgentOnline(a) || (p >= 0 && !IsAgentOnline(p))) continue;
        const Ipv4Address gw = (p < 0) ? m_controllerIp : m_agentBapIp[p];
        auto routing = routingOf(m_agentNodes.Get(a));
        routing->SetDefaultRoute(gw, m_agentBstaIf[a]);
        routing->AddHostRouteTo(m_controllerIp, gw, m_agentBstaIf[a]);
    }

    // Downlink: host route to each client along its serving Agent's path
    auto ctrlRouting = routingOf(m_controllerNode.Get(0));
    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        auto radio = m_controller->GetServingRadio(c);
        if (!radio || !IsAgentOnline(radio->GetAgentId())) continue;
        const uint32_t serving = radio->GetAgentId();

        routingOf(m_clientNodes.Get(c))->SetDefaultRoute(radio->GetIp(), m_clientIf[c]);
        routingOf(m_agentNodes.Get(serving))->AddHostRouteTo(m_clientIp[c], radio->GetIfIndex());

        uint32_t current = serving;
        for (uint32_t hops = 0; hops <= m_numAgents; ++hops)   // bounded: transient cycles
        {
            const int32_t p = m_agentParent[current];
            if (p < 0)
            {
                ctrlRouting->AddHostRouteTo(m_clientIp[c], m_agentBstaIp[current], m_ctrlBapIf);
                break;
            }
            if (!IsAgentOnline(p)) break;
            routingOf(m_agentNodes.Get(p))->AddHostRouteTo(m_clientIp[c], m_agentBstaIp[current],
                                                           m_agentBapIf[p]);
            current = p;
        }
    }
}

// ════════════════════════════════════════════════════════════════
//  BSS transitions
// ════════════════════════════════════════════════════════════════

EasyMeshSimulation::TransitionTarget
EasyMeshSimulation::ClientTarget(uint32_t clientId, Ptr<EasyMeshRadio> radio) const
{
    TransitionTarget target;
    if (!m_controller->IsMloClient(clientId))
    {
        target.expected = radio->GetBssid();
        target.bssids   = {target.expected};
        target.channels = {radio->GetChannel()};
        return target;
    }

    // Multi-link setup with the Agent's AP MLD: accept any affiliated AP,
    // tune each client link to the AP MLD's channel on that band
    auto agent = m_agents[radio->GetAgentId()];
    target.expected = radio->GetMldAddress();
    target.bssids   = LinkBssids(radio->GetDevice());
    for (EmBand band : m_controller->GetClientCaps(clientId).bands)
    {
        target.channels.push_back(agent->GetRadio(band)->GetChannel());
    }
    return target;
}

EasyMeshSimulation::TransitionTarget
EasyMeshSimulation::BackhaulTarget(int32_t parent) const
{
    TransitionTarget target;
    target.expected = BackhaulBssid(parent);
    target.bssids   = LinkBssids(BackhaulDevice(parent));
    target.channels = m_bhChannels;
    return target;
}

const EmChannel&
EasyMeshSimulation::ParkingChannel(const ManagedSta& s, size_t link) const
{
    // A 6 GHz MLD link cannot leave its band; any other PHY parks on 5 GHz
    return (link < s.linkBands.size() && s.linkBands[link] == EmBand::BAND_6GHZ) ? kParking6Channel
                                                                                 : kParkingChannel;
}

void
EasyMeshSimulation::WakeStaLinks(uint32_t staIdx)
{
    // Links left out of a multi-link setup are switched off by the MAC; turn
    // them back on so the next scan / setup can use them again.
    auto& s = m_stas[staIdx];
    if (s.isBackhaul && !IsAgentOnline(s.ownerId))
    {
        return;
    }
    for (uint8_t i = 0; i < s.device->GetNPhys(); ++i)
    {
        if (s.device->GetPhy(i)->IsStateOff())
        {
            s.device->GetPhy(i)->ResumeFromOff();
        }
    }
}

void
EasyMeshSimulation::ForgetStaAtAp(uint32_t staIdx)
{
    // An ns-3 STA leaves without sending a Disassociation frame, and neither
    // side tears down its Block Ack agreements, so the AP keeps the STA as
    // associated. Re-associating later through another affiliated AP of the
    // same AP MLD (e.g. band steering) then breaks: the stale AID makes
    // ApWifiMac::SetAid abort, and the AP MLD keeps sending A-MPDUs under a BA
    // agreement the STA (now talking to a different link address) no longer
    // has. Do what a Disassociation + DELBA would: reset the AP's record of
    // the STA on every link and drop the BA agreements on both sides.
    auto& s = m_stas[staIdx];
    Ptr<WifiNetDevice> ap;
    if (s.isBackhaul)
        ap = BackhaulDevice(m_agentParent[s.ownerId]);
    else if (auto serving = m_controller->GetServingRadio(s.ownerId))
        ap = serving->GetDevice();
    if (!ap) return;

    // (a non-AP MLD takes over the AP MLD's link IDs at multi-link setup)
    std::set<Mac48Address> staAddresses{s.mac->GetAddress()};
    for (uint8_t linkId : s.mac->GetLinkIds())
    {
        staAddresses.insert(s.mac->GetFrameExchangeManager(linkId)->GetAddress());
    }
    for (uint8_t linkId = 0; linkId < ap->GetNPhys(); ++linkId)
    {
        auto manager = ap->GetMac()->GetWifiRemoteStationManager(linkId);
        for (const auto& address : staAddresses)
        {
            if (manager->IsAssociated(address))
            {
                manager->RecordDisassociated(address);
            }
        }
    }

    auto apAddresses = LinkBssids(ap);
    apAddresses.insert(ap->GetMac()->GetAddress());
    for (uint8_t tid = 0; tid < 8; ++tid)
    {
        auto apBa  = ap->GetMac()->GetQosTxop(tid)->GetBaManager();
        auto staBa = s.mac->GetQosTxop(tid)->GetBaManager();
        for (const auto& address : staAddresses)
        {
            apBa->DestroyOriginatorAgreement(address, tid, std::nullopt);
            apBa->DestroyRecipientAgreement(address, tid, std::nullopt);
        }
        for (const auto& address : apAddresses)
        {
            staBa->DestroyOriginatorAgreement(address, tid, std::nullopt);
            staBa->DestroyRecipientAgreement(address, tid, std::nullopt);
        }
    }
}

void
EasyMeshSimulation::StartTransition(uint32_t staIdx, TransitionTarget target, const SteeringEvent& ev)
{
    auto& s = m_stas[staIdx];
    if (s.pending)
    {
        FinishTransition(staIdx, false);    // superseded by a newer decision
    }

    s.pending       = true;
    s.target        = std::move(target);
    s.start         = Simulator::Now();
    s.hopTries      = 0;
    s.ev            = ev;
    s.assocMgr->SetTargetBssids(s.target.bssids);
    if (!s.isBackhaul)
    {
        m_controller->NotifyTransitionStarted(s.ownerId);
    }

    // BTM-like transition: leave the current BSS by retuning to an idle
    // parking channel (forces disassociation + re-scan), then tune to the
    // target channel(s) where only the target BSSIDs are accepted. A
    // multi-link STA parks and retunes every link.
    WakeStaLinks(staIdx);
    for (size_t i = 0; i < s.target.channels.size(); ++i)
    {
        s.device->GetPhy(i)->SetAttribute("ChannelSettings",
                                          StringValue(ParkingChannel(s, i).ToSettings()));
    }
    s.hopEvent = Simulator::Schedule(MilliSeconds(1), &EasyMeshSimulation::HopToTarget, this, staIdx);
    s.timeoutEvent = Simulator::Schedule(Seconds(STEER_TIMEOUT),
                                         &EasyMeshSimulation::OnTransitionTimeout, this, staIdx);
}

void
EasyMeshSimulation::HopToTarget(uint32_t staIdx)
{
    auto& s = m_stas[staIdx];
    // A channel switch can be postponed while the PHY is busy: wait until
    // every parking switch has happened before moving on to the target.
    bool parked = true;
    for (size_t i = 0; i < s.target.channels.size(); ++i)
    {
        if (s.device->GetPhy(i)->GetChannelNumber() != ParkingChannel(s, i).number)
        {
            parked = false;
        }
    }
    if (parked || s.hopTries >= 50)
    {
        for (size_t i = 0; i < s.target.channels.size(); ++i)
        {
            s.device->GetPhy(i)->SetAttribute("ChannelSettings",
                                              StringValue(s.target.channels[i].ToSettings()));
        }
        return;
    }
    s.hopTries++;
    for (size_t i = 0; i < s.target.channels.size(); ++i)
    {
        auto phy = s.device->GetPhy(i);
        if (phy->GetChannelNumber() != ParkingChannel(s, i).number)
        {
            phy->SetAttribute("ChannelSettings", StringValue(ParkingChannel(s, i).ToSettings()));
        }
    }
    s.hopEvent = Simulator::Schedule(MilliSeconds(1), &EasyMeshSimulation::HopToTarget, this, staIdx);
}

void
EasyMeshSimulation::FinishTransition(uint32_t staIdx, bool success)
{
    auto& s = m_stas[staIdx];
    s.pending = false;
    s.hopEvent.Cancel();
    s.timeoutEvent.Cancel();

    s.ev.success   = success;
    s.ev.latencyMs = success ? (Simulator::Now() - s.start).GetSeconds() * 1000.0 : 0.0;
    if (success)
    {
        if (s.isBackhaul)
        {
            const int32_t p = s.ev.toAgent;
            s.ev.rssiAfterDbm = EstimateRssi(p < 0 ? CONTROLLER_POS : GetAgentPosition(p),
                                             GetAgentPosition(s.ownerId), m_bhChannels.front());
        }
        else
        {
            s.ev.rssiAfterDbm = ClientRssi(s.ownerId, m_agents[s.ev.toAgent]->GetRadio(s.ev.toBand));
        }
    }
    m_controller->RecordTransition(s.ev);
}

void
EasyMeshSimulation::OnTransitionTimeout(uint32_t staIdx)
{
    auto& s = m_stas[staIdx];
    if (!s.pending) return;

    NS_LOG_INFO("[t=" << Simulator::Now().GetSeconds() << "] "
                << (s.isBackhaul ? "bSTA" : "STA") << s.ownerId
                << " transition to " << s.target.expected << " timed out");
    FinishTransition(staIdx, false);
    if (!s.isBackhaul)
    {
        // Client falls back to its own choice among BSSs on this channel.
        // A bSTA stays pinned: it must never pick a descendant's bBSS.
        s.assocMgr->ClearTargetBssid();
    }
}

void
EasyMeshSimulation::OnStaAssoc(uint32_t staIdx, Mac48Address bssid)
{
    auto it = m_bssMap.find(bssid);
    if (it == m_bssMap.end())
    {
        NS_LOG_WARN("STA index " << staIdx << " associated to unknown BSSID " << bssid);
        return;
    }
    auto& s = m_stas[staIdx];
    const BssInfo& bss = it->second;

    // Links actually set up (all links of a full multi-link setup)
    std::vector<EmBand> setupBands;
    for (uint8_t linkId : s.mac->GetSetupLinkIds())
    {
        auto link = m_bssMap.find(s.mac->GetBssid(linkId));
        if (link != m_bssMap.end())
        {
            setupBands.push_back(link->second.band);
        }
    }
    if (setupBands.empty())
    {
        setupBands.push_back(bss.band);
    }
    std::string bandList;
    for (EmBand band : setupBands)
    {
        bandList += (bandList.empty() ? "" : "+") + std::string(BandStr(band));
    }
    if (bss.mld && setupBands.size() < s.device->GetNPhys())
    {
        m_partialSetups++;
        NS_LOG_INFO("[t=" << Simulator::Now().GetSeconds() << "] "
                    << (s.isBackhaul ? "bSTA" : "STA") << s.ownerId << " partial multi-link setup: "
                    << setupBands.size() << "/" << +s.device->GetNPhys() << " links");
    }

    if (s.isBackhaul)
    {
        m_agentParent[s.ownerId] = bss.agentId;
        UpdateBackhaulMetrics();
        NS_LOG_INFO("[t=" << Simulator::Now().GetSeconds() << "] bSTA" << s.ownerId
                    << " associated to " << (bss.agentId < 0 ? std::string("Controller")
                                                             : "Agent" + std::to_string(bss.agentId))
                    << " (" << bandList << ")");
    }
    else
    {
        auto agent = m_agents[bss.agentId];
        std::vector<Ptr<EasyMeshRadio>> links;
        for (EmBand band : setupBands)
        {
            links.push_back(agent->GetRadio(band));
        }
        // the anchor radio (client's first band) represents the AP MLD
        auto anchor = bss.mld ? m_controller->AnchorRadio(s.ownerId, bss.agentId) : nullptr;
        auto pos = std::find(links.begin(), links.end(), anchor);
        if (pos != links.end())
        {
            std::iter_swap(links.begin(), pos);
        }
        m_controller->NotifyClientAssociated(s.ownerId, links);
        NS_LOG_INFO("[t=" << Simulator::Now().GetSeconds() << "] STA" << s.ownerId
                    << " associated to A" << bss.agentId << "/" << (bss.mld ? "MLD " : "") << bandList
                    << " ch" << links.front()->GetChannel().number
                    << "  RSSI=" << ClientRssi(s.ownerId, links.front()) << "dBm");
    }

    if (s.pending && (bssid == s.target.expected || s.target.bssids.count(bssid)))
    {
        FinishTransition(staIdx, true);
    }
    RebuildRoutes();
    UpdateAnimationNodes();
}

void
EasyMeshSimulation::OnStaDeAssoc(uint32_t staIdx, Mac48Address bssid)
{
    const auto& s = m_stas[staIdx];
    NS_LOG_DEBUG("[t=" << Simulator::Now().GetSeconds() << "] "
                 << (s.isBackhaul ? "bSTA" : "STA") << s.ownerId << " left " << bssid);
    ForgetStaAtAp(staIdx);
    if (!s.isBackhaul)
    {
        m_controller->NotifyClientDisassociated(s.ownerId);
    }
    WakeStaLinks(staIdx);
}

bool
EasyMeshSimulation::SteerClient(uint32_t clientId, Ptr<EasyMeshRadio> target, SteeringReason reason)
{
    if (!target || !IsAgentOnline(target->GetAgentId()))
    {
        return false;
    }
    auto serving = m_controller->GetServingRadio(clientId);

    SteeringEvent ev{};
    ev.timeS         = Simulator::Now().GetSeconds();
    ev.isBackhaul    = false;
    ev.mlo           = m_controller->IsMloClient(clientId);
    ev.staId         = clientId;
    ev.fromAgent     = serving ? static_cast<int32_t>(serving->GetAgentId()) : -1;
    ev.fromBand      = serving ? serving->GetBand() : EmBand::BAND_5GHZ;
    ev.toAgent       = static_cast<int32_t>(target->GetAgentId());
    ev.toBand        = target->GetBand();
    ev.reason        = reason;
    ev.rssiBeforeDbm = serving ? ClientRssi(clientId, serving) : -100.0;
    ev.rssiAfterDbm  = ClientRssi(clientId, target);

    NS_LOG_INFO("[t=" << ev.timeS << "] " << ReasonStr(reason) << " STA" << clientId << " "
                << (serving ? "A" + std::to_string(ev.fromAgent) + "/" +
                                  (ev.mlo ? "MLD" : BandStr(ev.fromBand))
                            : std::string("-"))
                << " -> A" << ev.toAgent << "/" << (ev.mlo ? "MLD" : BandStr(ev.toBand))
                << " ch" << target->GetChannel().number
                << "  RSSI " << ev.rssiBeforeDbm << " -> " << ev.rssiAfterDbm << "dBm");

    StartTransition(clientId, ClientTarget(clientId, target), ev);
    return true;
}

void
EasyMeshSimulation::SwitchRadioChannel(Ptr<EasyMeshRadio> radio, const EmChannel& channel)
{
    radio->SetChannel(channel);

    // No CSA in ns-3: the Controller moves the radio's clients along with it
    // (an MLD client re-sets up all its links with the same AP MLD)
    const auto clients = radio->GetClients();
    for (uint32_t c : clients)
    {
        if (!m_controller->IsServedBy(c, radio)) continue;
        auto serving = m_controller->GetServingRadio(c);
        SteeringEvent ev{};
        ev.timeS         = Simulator::Now().GetSeconds();
        ev.mlo           = m_controller->IsMloClient(c);
        ev.staId         = c;
        ev.fromAgent     = ev.toAgent = static_cast<int32_t>(radio->GetAgentId());
        ev.fromBand      = ev.toBand  = serving->GetBand();
        ev.reason        = SteeringReason::CHANNEL_SWITCH;
        ev.rssiBeforeDbm = ev.rssiAfterDbm = ClientRssi(c, serving);
        StartTransition(c, ClientTarget(c, serving), ev);
    }
}

void
EasyMeshSimulation::ReparentAgent(uint32_t agentId, int32_t newParent, SteeringReason reason)
{
    const int32_t oldParent = m_agentParent[agentId];
    const Vector here = GetAgentPosition(agentId);

    SteeringEvent ev{};
    ev.timeS         = Simulator::Now().GetSeconds();
    ev.isBackhaul    = true;
    ev.staId         = agentId;
    ev.fromAgent     = oldParent;
    ev.toAgent       = newParent;
    ev.fromBand      = ev.toBand = m_bhChannels.front().band;
    ev.reason        = reason;
    ev.rssiBeforeDbm = EstimateRssi(oldParent < 0 ? CONTROLLER_POS : GetAgentPosition(oldParent),
                                    here, m_bhChannels.front());
    ev.rssiAfterDbm  = EstimateRssi(newParent < 0 ? CONTROLLER_POS : GetAgentPosition(newParent),
                                    here, m_bhChannels.front());

    NS_LOG_INFO("  Agent" << agentId << " backhaul "
                << (oldParent < 0 ? std::string("C") : "A" + std::to_string(oldParent)) << " -> "
                << (newParent < 0 ? std::string("C") : "A" + std::to_string(newParent)));
    StartTransition(BstaIndex(agentId), BackhaulTarget(newParent), ev);
}

// ════════════════════════════════════════════════════════════════
//  Periodic tasks and failure events
// ════════════════════════════════════════════════════════════════

void
EasyMeshSimulation::CheckRoaming()
{
    // STA-initiated roaming (e.g. from background scans / 802.11k reports):
    // the client only looks at signal strength, not at load or backhaul.
    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        if (!m_controller->IsTransitionAllowed(c)) continue;
        auto serving = m_controller->GetServingRadio(c);
        if (!serving) continue;

        const double rssi = ClientRssi(c, serving);
        if (rssi >= ROAM_RSSI_THRESHOLD) continue;

        // An MLD client roams between AP MLDs (Agents), not between links
        const bool mlo = m_controller->IsMloClient(c);
        Ptr<EasyMeshRadio> best;
        double bestRssi = -std::numeric_limits<double>::infinity();
        for (const auto& radio : m_controller->CandidateRadios(c))
        {
            const bool same = mlo ? radio->GetAgentId() == serving->GetAgentId() : radio == serving;
            const double r = ClientRssi(c, radio);
            if (!same && r > bestRssi)
            {
                bestRssi = r;
                best = radio;
            }
        }
        if (best && bestRssi >= rssi + ROAM_HYSTERESIS_DB)
        {
            SteerClient(c, best, SteeringReason::ROAMING);
        }
    }
}

void
EasyMeshSimulation::SetAgentRadiosPower(uint32_t agentId, bool on)
{
    // every PHY (link) of the bBSS, the bSTA and the fronthaul device(s)
    std::set<Ptr<WifiPhy>> phys;
    auto addDevice = [&phys](Ptr<WifiNetDevice> dev) {
        for (uint8_t i = 0; i < dev->GetNPhys(); ++i) phys.insert(dev->GetPhy(i));
    };
    addDevice(m_agentBap[agentId]);
    addDevice(m_stas[BstaIndex(agentId)].device);
    for (const auto& radio : m_agents[agentId]->GetRadios())
    {
        addDevice(radio->GetDevice());
    }
    for (const auto& phy : phys)
    {
        if (on && phy->IsStateOff())
            phy->ResumeFromOff();
        else if (!on && !phy->IsStateOff())
            phy->SetOffMode();
    }
}

void
EasyMeshSimulation::HandleAgentOffline()
{
    // Take every requested agent offline first (validating + deduping),
    // then rebuild the backhaul MST and steer clients exactly once for the
    // whole set — rebuilding per-agent would let an already-degraded
    // topology bias which parent gets picked for the next offline agent.
    std::vector<uint32_t> newlyOffline;
    for (uint32_t agentId : m_offlineAgentIds)
    {
        if (agentId >= m_numAgents)
        {
            NS_LOG_WARN("Ignoring offline event: invalid agent id " << agentId);
            continue;
        }
        if (!IsAgentOnline(agentId))
        {
            continue; // already offline (or duplicate entry) — nothing to do
        }
        newlyOffline.push_back(agentId);
    }

    if (newlyOffline.empty())
    {
        return;
    }

    std::ostringstream offlineList;
    for (size_t i = 0; i < newlyOffline.size(); ++i)
    {
        offlineList << (i ? "," : "") << "A" << newlyOffline[i];
    }
    NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds() << "] AGENT_OFFLINE "
                << offlineList.str() << "  (" << newlyOffline.size() << " agent(s))");

    for (uint32_t agentId : newlyOffline)
    {
        m_agentOnline[agentId] = false;
        m_agents[agentId]->SetOnline(false);
        auto& bsta = m_stas[BstaIndex(agentId)];
        bsta.pending = false;
        bsta.hopEvent.Cancel();
        bsta.timeoutEvent.Cancel();
        SetAgentRadiosPower(agentId, false);   // every radio goes dark
    }

    OptimizeBackhaul(true, SteeringReason::AGENT_OFFLINE);
    for (uint32_t agentId : newlyOffline)
    {
        m_controller->HandleAgentOffline(agentId);
    }
    RebuildRoutes();
    UpdateAnimationNodes();
}

void
EasyMeshSimulation::HandleAgentRecover()
{
    std::vector<uint32_t> recovered;
    for (uint32_t agentId : m_offlineAgentIds)
    {
        if (agentId < m_numAgents && !IsAgentOnline(agentId))
        {
            m_agentOnline[agentId] = true;
            m_agents[agentId]->SetOnline(true);
            SetAgentRadiosPower(agentId, true);
            recovered.push_back(agentId);
        }
    }
    if (recovered.empty()) return;

    NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds() << "] AGENT_ONLINE "
                << recovered.size() << " agent(s) back");
    // The recovered Agents rejoin the backhaul; steering / load balancing
    // pulls clients onto them over the next ticks.
    OptimizeBackhaul(true, SteeringReason::BACKHAUL_OPT);
    UpdateAnimationNodes();
}

// ════════════════════════════════════════════════════════════════
//  Traffic
// ════════════════════════════════════════════════════════════════

void
EasyMeshSimulation::InstallUdpUplink()
{
    UdpServerHelper server(PORT_UDP_UL);
    ApplicationContainer serverApps = server.Install(m_controllerNode.Get(0));
    serverApps.Start(Seconds(0.0));
    serverApps.Stop(Seconds(m_duration + 1.0));

    double intervalUs = (UL_PKT_SIZE * 8.0) / (UL_OFFERED_MBPS * 1e6) * 1e6;

    for (uint32_t c = 0; c < m_numClients; c++)
    {
        UdpClientHelper client(m_controllerIp, PORT_UDP_UL);
        client.SetAttribute("MaxPackets", UintegerValue(UINT32_MAX));
        client.SetAttribute("Interval", TimeValue(MicroSeconds(intervalUs)));
        client.SetAttribute("PacketSize", UintegerValue(UL_PKT_SIZE));

        ApplicationContainer app = client.Install(m_clientNodes.Get(c));
        app.Start(Seconds(UL_START));
        app.Stop(Seconds(m_duration));
        NS_LOG_INFO("UDP-UL: STA" << c << " -> Controller  payload="
                    << UL_PKT_SIZE << "B offered=" << UL_OFFERED_MBPS << "Mbps");
    }
}

void
EasyMeshSimulation::InstallUdpDownlink()
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

void
EasyMeshSimulation::InstallTcpUplink()
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

void
EasyMeshSimulation::InstallApplications()
{
    NS_LOG_INFO("=== InstallApplications ===");
    InstallUdpUplink();
    InstallUdpDownlink();
    NS_LOG_INFO("Applications installed: " << m_numClients << " UDP uplink + downlink flow pair(s)");
}

// ════════════════════════════════════════════════════════════════
//  Animation, scheduling, run, results
// ════════════════════════════════════════════════════════════════

void
EasyMeshSimulation::UpdateAnimationNodes()
{
    if (!m_animation) return;
    auto anim = m_animation.get();

    anim->UpdateNodeDescription(m_controllerNode.Get(0), "Controller");
    anim->UpdateNodeColor(m_controllerNode.Get(0), 0, 102, 204);
    anim->UpdateNodeSize(m_controllerNode.Get(0), 2.0, 2.0);

    for (uint32_t a = 0; a < m_numAgents; ++a)
    {
        std::ostringstream name;
        if (!IsAgentOnline(a))
        {
            name << "Agent " << a << " OFFLINE";
            anim->UpdateNodeColor(m_agentNodes.Get(a), 127, 140, 141);
        }
        else
        {
            name << "Agent " << a << " UL->"
                 << (m_agentParent[a] < 0 ? std::string("C") : "A" + std::to_string(m_agentParent[a]));
            for (const auto& radio : m_agents[a]->GetRadios())
            {
                name << " " << BandStr(radio->GetBand()) << ":" << radio->GetChannel().number;
            }
            anim->UpdateNodeColor(m_agentNodes.Get(a), 46, 204, 113);
        }
        anim->UpdateNodeDescription(m_agentNodes.Get(a), name.str());
        anim->UpdateNodeSize(m_agentNodes.Get(a), 2.0, 2.0);
    }

    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        std::ostringstream name;
        auto radio = m_controller->GetServingRadio(c);
        name << "STA " << c << " -> ";
        if (radio)
            name << "A" << radio->GetAgentId() << "/"
                 << (m_controller->IsMloClient(c) ? "MLD" : BandStr(radio->GetBand()));
        else
            name << "none";
        anim->UpdateNodeDescription(m_clientNodes.Get(c), name.str());
        anim->UpdateNodeColor(m_clientNodes.Get(c), 231, 126, 35);
        anim->UpdateNodeSize(m_clientNodes.Get(c), 1.0, 1.0);
    }
}

void
EasyMeshSimulation::ScheduleEvents()
{
    for (double t = METRICS_INTERVAL; t < m_duration; t += METRICS_INTERVAL)
        Simulator::Schedule(Seconds(t), &EasyMeshController::CollectApMetrics, m_controller);

    for (double t = UL_START + STEERING_INTERVAL; t < m_duration; t += STEERING_INTERVAL)
        Simulator::Schedule(Seconds(t), &EasyMeshController::RunSteeringEngine, m_controller);

    if (m_roaming)
    {
        for (double t = UL_START; t < m_duration; t += ROAM_CHECK_INTERVAL)
            Simulator::Schedule(Seconds(t), &EasyMeshSimulation::CheckRoaming, this);
    }

    if (m_channelPlanMode == ChannelPlanMode::DYNAMIC)
    {
        for (double t = CHANNEL_PLAN_INTERVAL; t < m_duration; t += CHANNEL_PLAN_INTERVAL)
            Simulator::Schedule(Seconds(t), &EasyMeshController::PlanChannels, m_controller, false);
    }

    if (m_netOpt)
    {
        for (double t = NETOPT_INTERVAL; t < m_duration; t += NETOPT_INTERVAL)
            Simulator::Schedule(Seconds(t), &EasyMeshSimulation::OptimizeBackhaul, this,
                                false, SteeringReason::BACKHAUL_OPT);
    }

    if (m_offlineTime > UL_START && m_offlineTime < m_duration)
    {
        Simulator::Schedule(Seconds(m_offlineTime),
                            &EasyMeshSimulation::HandleAgentOffline,
                            this);
        if (m_recoverTime > m_offlineTime && m_recoverTime < m_duration)
        {
            Simulator::Schedule(Seconds(m_recoverTime), &EasyMeshSimulation::HandleAgentRecover, this);
        }
    }
}

void
EasyMeshSimulation::Run()
{
    NS_LOG_INFO("=== Simulation start  duration=" << m_duration << "s ===");
    m_monitor = m_monHelper.InstallAll();
    m_animation->SetStartTime(Seconds(UL_START));
    m_animation->SetStopTime(Seconds(m_duration));
    m_animation->SetMobilityPollInterval(Seconds(1.0));
    m_animation->SetMaxPktsPerTraceFile(200000);
    m_animation->SkipPacketTracing();
    UpdateAnimationNodes();

    Simulator::Stop(Seconds(m_duration + 2.0));
    Simulator::Run();

    Ptr<Ipv4FlowClassifier> clf =
        DynamicCast<Ipv4FlowClassifier>(m_monHelper.GetClassifier());
    m_trafficStats.Collect(m_monitor, clf);

    Simulator::Destroy();
    m_animation.reset();
    std::cout << "[EasyMeshSim] Animation saved to easymesh-sim-animation-run" << m_scenarioRun
              << ".xml\n";
    NS_LOG_INFO("=== Simulation complete ===");
}

void
EasyMeshSimulation::PrintResults()
{
    m_controller->PrintTopology();
    m_controller->PrintChannelPlan();
    m_controller->PrintSteeringLog();
    if (PRINT_PER_FLOW) m_trafficStats.PrintPerFlowTable();
    m_controller->PrintRadioTraffic(m_duration - UL_START);
    std::cout << "  Wi-Fi 7: backhaul " << (m_bhMlo ? "MLO" : "single-link");
    for (const auto& channel : m_bhChannels)
    {
        std::cout << " " << BandStr(channel.band) << " ch" << channel.number << "/"
                  << channel.widthMhz << "MHz";
    }
    std::cout << "  |  fronthaul " << (m_fhMlo ? "tri-band AP MLD" : "single-link APs")
              << "  |  " << std::min(m_numMldClients, m_numClients) << " Wi-Fi 7 client(s)"
              << (m_fhMlo ? (m_emlsr ? " (MLD, EMLSR)" : " (MLD, STR)") : "")
              << "  |  partial multi-link setups: " << m_partialSetups << "\n";
    m_trafficStats.PrintNetworkSummary();
}

int main(int argc, char* argv[])
{
    LogComponentEnable("EasyMeshSim",        LOG_LEVEL_INFO);
    LogComponentEnable("EasyMeshController", LOG_LEVEL_INFO);

    double   duration   = DURATION;
    uint32_t scenarioRuns = SCENARIO_RUNS;
    uint32_t numClients = NUM_STA;
    uint32_t numAgents  = NUM_AGENT;
    std::string offlineAgentsStr = std::to_string(OFFLINE_AGENT_ID); // comma-separated, e.g. "1,3,5"
    double   offlineTime = OFFLINE_TIME;
    double   recoverTime = RECOVER_TIME;
    bool     pcap       = false;
    std::string mstMetricStr = "throughput";   // "throughput" | "rssi"
    std::string channelPlanStr = "dynamic";    // "none" | "static" | "dynamic"
    bool     steering   = true;
    bool     roaming    = true;
    bool     loadBalancing = true;
    bool     netOpt     = true;
    double   staSpeed   = STA_MAX_SPEED;
    bool     bhMlo      = true;
    bool     fhMlo      = true;
    uint32_t mldClients = NUM_MLD_STA;
    bool     emlsr      = false;
    uint32_t bhWidth6   = BH6_WIDTH_MHZ;

    CommandLine cmd;
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("scenarios", "Number of randomized scenario runs", scenarioRuns);
    cmd.AddValue("clients",  "Number of STA clients",   numClients);
    cmd.AddValue("agents",   "Number of Agents (APs)",  numAgents);
    cmd.AddValue("offlineAgents", "Comma-separated agent ids to take offline during the run (e.g. 1,3,5)", offlineAgentsStr);
    cmd.AddValue("offlineTime", "Time in seconds when the agent(s) go offline", offlineTime);
    cmd.AddValue("recoverTime", "Time in seconds when the offline agent(s) come back (<0: never)", recoverTime);
    cmd.AddValue("pcap",     "Enable PCAP capture",     pcap);
    cmd.AddValue("mstMetric", "Backhaul MST edge metric: throughput | rssi", mstMetricStr);
    cmd.AddValue("channelPlan", "Fronthaul channel planning: none | static | dynamic", channelPlanStr);
    cmd.AddValue("steering", "Controller client + band steering", steering);
    cmd.AddValue("roaming", "STA-initiated roaming", roaming);
    cmd.AddValue("loadBalancing", "Controller load balancing", loadBalancing);
    cmd.AddValue("netOpt", "Periodic backhaul (network) optimization", netOpt);
    cmd.AddValue("staSpeed", "Max STA speed in m/s (0 = static clients)", staSpeed);
    cmd.AddValue("bhMlo", "Wi-Fi 7 MLO backhaul: 5 GHz + 6 GHz links (false: 5 GHz only)", bhMlo);
    cmd.AddValue("bhWidth6", "Width of the 6 GHz backhaul link in MHz: 20 | 40 | 80 | 160 | 320", bhWidth6);
    cmd.AddValue("fhMlo", "Agents run one tri-band AP MLD and Wi-Fi 7 clients use MLO "
                          "(false: single-link APs and clients)", fhMlo);
    cmd.AddValue("mldClients", "Number of Wi-Fi 7 clients (the others are Wi-Fi 6)", mldClients);
    cmd.AddValue("emlsr", "Experimental: Wi-Fi 7 MLD clients use EMLSR instead of STR "
                          "(aborts once an EMLSR client is steered)", emlsr);
    cmd.Parse(argc, argv);

    if (bhWidth6 != 20 && bhWidth6 != 40 && bhWidth6 != 80 && bhWidth6 != 160 && bhWidth6 != 320)
    {
        std::cerr << "Unknown --bhWidth6 " << bhWidth6 << " (expected 20 | 40 | 80 | 160 | 320); "
                  << "defaulting to " << BH6_WIDTH_MHZ << "\n";
        bhWidth6 = BH6_WIDTH_MHZ;
    }
    if (emlsr)
    {
        // The parking/hop transition retunes STA PHYs directly, which conflicts
        // with the EMLSR manager moving its main PHY between links: ns-3 aborts
        // in DefaultEmlsrManager after the first transition of an EMLSR client.
        std::cerr << "Warning: --emlsr is experimental; the run aborts once an EMLSR client is "
                     "steered, roams or follows a channel switch\n";
    }

    MstMetric mstMetric = MstMetric::THROUGHPUT;
    if (mstMetricStr == "rssi")
    {
        mstMetric = MstMetric::RSSI;
    }
    else if (mstMetricStr != "throughput")
    {
        std::cerr << "Unknown --mstMetric '" << mstMetricStr
                  << "' (expected 'throughput' or 'rssi'); defaulting to 'throughput'\n";
    }

    ChannelPlanMode channelPlan = ChannelPlanMode::DYNAMIC;
    if (channelPlanStr == "none")
    {
        channelPlan = ChannelPlanMode::NONE;
    }
    else if (channelPlanStr == "static")
    {
        channelPlan = ChannelPlanMode::STATIC;
    }
    else if (channelPlanStr != "dynamic")
    {
        std::cerr << "Unknown --channelPlan '" << channelPlanStr
                  << "' (expected none | static | dynamic); defaulting to 'dynamic'\n";
    }

    // Parse "1,3,5" -> {1, 3, 5}, skipping blanks/whitespace.
    std::vector<uint32_t> offlineAgentIds;
    {
        std::stringstream ss(offlineAgentsStr);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            size_t start = token.find_first_not_of(" \t");
            size_t end   = token.find_last_not_of(" \t");
            if (start == std::string::npos)
            {
                continue; // empty token (e.g. trailing comma)
            }
            token = token.substr(start, end - start + 1);
            try
            {
                offlineAgentIds.push_back(static_cast<uint32_t>(std::stoul(token)));
            }
            catch (const std::exception&)
            {
                std::cerr << "Ignoring invalid --offlineAgents entry: '" << token << "'\n";
            }
        }
    }

    RngSeedManager::SetSeed(42);

    for (uint32_t run = 1; run <= scenarioRuns; ++run)
    {
        std::cout << "\n=== Scenario " << run << "/" << scenarioRuns
                  << "  [mstMetric=" << mstMetricStr << ", channelPlan=" << channelPlanStr
                  << ", steering=" << steering << ", roaming=" << roaming
                  << ", loadBalancing=" << loadBalancing << ", netOpt=" << netOpt
                  << ", offlineAgents=" << offlineAgentsStr << ", bhMlo=" << bhMlo
                  << ", bhWidth6=" << bhWidth6 << ", fhMlo=" << fhMlo << ", mldClients=" << mldClients
                  << ", emlsr=" << emlsr << "] ===\n" << std::flush;
        RngSeedManager::SetRun(run);

        EasyMeshSimulation sim;
        sim.SetDuration(duration);
        sim.SetScenarioRun(run);
        sim.SetNumClients(numClients);
        sim.SetNumAgents(numAgents);
        sim.SetOfflineAgentIds(offlineAgentIds);
        sim.SetOfflineTime(offlineTime);
        sim.SetRecoverTime(recoverTime);
        sim.EnablePcap(pcap);
        sim.SetMstMetric(mstMetric);
        sim.SetChannelPlanMode(channelPlan);
        sim.EnableSteering(steering);
        sim.EnableRoaming(roaming);
        sim.EnableLoadBalancing(loadBalancing);
        sim.EnableNetworkOptimization(netOpt);
        sim.SetStaMaxSpeed(staSpeed);
        sim.EnableBackhaulMlo(bhMlo);
        sim.SetBackhaul6Width(static_cast<uint16_t>(bhWidth6));
        sim.EnableFronthaulMlo(fhMlo);
        sim.SetNumMldClients(mldClients);
        sim.EnableEmlsr(emlsr);

        sim.BuildTopology();
        sim.InstallProtocolStack();
        sim.InstallApplications();
        sim.ScheduleEvents();
        sim.Run();
        sim.PrintResults();
    }

    return 0;
}
