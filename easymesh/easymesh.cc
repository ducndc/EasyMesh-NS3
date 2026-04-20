#include "easymesh.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshSim");

const Vector EasyMeshSimulation::CONTROLLER_POS   = Vector(0.0, 0.0, 1.5);
const Vector EasyMeshSimulation::AGENT_POSITIONS[NUM_AGENT] = {
    Vector(20.0, 0.0, 1.5),
    Vector(0.0, 20.0, 1.5),
    Vector(20.0, 20.0, 1.5),
    Vector(10.0, 10.0, 1.5)
};

EasyMeshSimulation::EasyMeshSimulation()  
{
    m_controllerIp = Ipv4Address::GetAny();
    m_controllerBackhaulIp.resize(NUM_AGENT, Ipv4Address::GetAny());
    m_agentIp.resize(NUM_AGENT, Ipv4Address::GetAny()); 
    m_agentFronthaulIp.resize(NUM_AGENT, Ipv4Address::GetAny());
    m_clientIp.resize(NUM_STA, Ipv4Address::GetAny());
    m_clientToAgent.resize(NUM_STA, 0);
    m_agentFronthaulDevices.resize(NUM_AGENT);
    m_clientFronthaulDevices.resize(NUM_STA);
    m_parentIfIndex.resize(NUM_AGENT, 0);
    m_agentBackhaulIfIndex.resize(NUM_AGENT, 0);
    m_agentFronthaulIfIndex.resize(NUM_AGENT, 0);
    m_clientFronthaulIfIndex.resize(NUM_STA, 0);
    m_agentPositions.resize(NUM_AGENT, CONTROLLER_POS);
    m_agentParent.resize(NUM_AGENT, -1);
}

EasyMeshSimulation::~EasyMeshSimulation() {}

double 
EasyMeshSimulation::ComputeDistance(Vector a, Vector b) const
{
    return std::sqrt(std::pow(a.x-b.x,2) +
                     std::pow(a.y-b.y,2) +
                     std::pow(a.z-b.z,2));
}

double
EasyMeshSimulation::EstimateLinkRssi(Vector a, Vector b) const
{
    double dist = ComputeDistance(a, b);
    double pathLoss = 40.0 + 30.0 * std::log10(std::max(1.0, dist));
    return 20.0 + 6.0 - pathLoss;
}

double
EasyMeshSimulation::EstimateLinkThroughput(double rssi) const
{
    uint32_t mcs = rssi >= -65 ? 9 : rssi >= -70 ? 7 : rssi >= -76 ? 5 : 3;
    return (mcs + 1) * 5.5;
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

    for (uint32_t agentId = 0; agentId < NUM_AGENT; ++agentId)
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
        for (uint32_t agentId = 0; agentId < NUM_AGENT; ++agentId)
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

        for (uint32_t left = 0; left < NUM_AGENT; ++left)
        {
            for (uint32_t right = left + 1; right < NUM_AGENT; ++right)
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

    for (uint32_t agentId = 0; agentId < NUM_AGENT; ++agentId)
    {
        NS_LOG_INFO("Optimized Agent" << agentId << " position=("
                    << m_agentPositions[agentId].x << ","
                    << m_agentPositions[agentId].y << ")");
    }
}

void
EasyMeshSimulation::BuildBackhaulTree()
{
    m_agentParent.assign(NUM_AGENT, -1);

    std::vector<uint32_t> order(NUM_AGENT);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [this](uint32_t lhs, uint32_t rhs) {
        return ComputeDistance(CONTROLLER_POS, m_agentPositions[lhs]) <
               ComputeDistance(CONTROLLER_POS, m_agentPositions[rhs]);
    });

    std::vector<double> pathCapacity(NUM_AGENT, 0.0);
    std::vector<uint32_t> connected;

    for (auto agentId : order)
    {
        const auto directRssi = EstimateLinkRssi(CONTROLLER_POS, m_agentPositions[agentId]);
        const auto directCapacity = EstimateLinkThroughput(directRssi);
        const auto directDistance = ComputeDistance(CONTROLLER_POS, m_agentPositions[agentId]);

        int32_t bestParent = -1;
        double bestCapacity = directCapacity;
        double bestScore = directCapacity - 0.5 * std::max(0.0, directDistance - TOPOLOGY_TARGET_RADIUS);

        for (auto candidate : connected)
        {
            const auto candidateDistance = ComputeDistance(CONTROLLER_POS, m_agentPositions[candidate]);
            if (candidateDistance >= directDistance)
            {
                continue;
            }

            const auto relayRssi = EstimateLinkRssi(m_agentPositions[candidate], m_agentPositions[agentId]);
            const auto relayCapacity = EstimateLinkThroughput(relayRssi);
            const auto endToEndCapacity = std::min(pathCapacity[candidate], relayCapacity);
            const auto relayDistance = ComputeDistance(m_agentPositions[candidate], m_agentPositions[agentId]);

            double score = endToEndCapacity - 0.25 * relayDistance;
            if (directDistance > TOPOLOGY_TARGET_RADIUS)
            {
                score += 2.0;
            }

            if (score > bestScore)
            {
                bestScore = score;
                bestParent = static_cast<int32_t>(candidate);
                bestCapacity = endToEndCapacity;
            }
        }

        m_agentParent[agentId] = bestParent;
        pathCapacity[agentId] = bestCapacity;
        connected.push_back(agentId);

        if (bestParent < 0)
        {
            NS_LOG_INFO("Backhaul parent for Agent" << agentId << " is Controller");
        }
        else
        {
            NS_LOG_INFO("Backhaul parent for Agent" << agentId << " is Agent" << bestParent);
        }
    }
}

Ptr<EasyMeshLink>
EasyMeshSimulation::FindBackhaulLinkForAgent(uint32_t agentId) const
{
    for (const auto& link : m_links)
    {
        if (link->GetType() == LinkType::BACKHAUL && link->GetDst() == m_agentNodes.Get(agentId))
        {
            return link;
        }
    }
    return nullptr;
}

Ptr<EasyMeshLink>
EasyMeshSimulation::FindFronthaulLinkForClient(uint32_t clientId) const
{
    for (const auto& link : m_links)
    {
        if (link->GetType() == LinkType::FRONTHAUL && link->GetDst() == m_clientNodes.Get(clientId))
        {
            return link;
        }
    }
    return nullptr;
}

uint32_t
EasyMeshSimulation::FindAgentForFronthaulBssid(Mac48Address bssid) const
{
    for (uint32_t agentId = 0; agentId < NUM_AGENT; ++agentId)
    {
        auto wifiDevice = DynamicCast<WifiNetDevice>(m_agentFronthaulDevices[agentId]);
        if (wifiDevice && wifiDevice->GetMac()->GetAddress() == bssid)
        {
            return agentId;
        }
    }

    return NUM_AGENT;
}

void
EasyMeshSimulation::UpdateClientAssociation(uint32_t clientId, uint32_t agentId)
{
    NS_ABORT_MSG_IF(agentId >= NUM_AGENT, "Invalid fronthaul agent selected for client");

    const uint32_t previousAgent = m_clientToAgent[clientId];
    if (previousAgent != agentId && previousAgent < m_agents.size())
    {
        m_agents[previousAgent]->RemoveClient(clientId);
    }

    if (agentId < m_agents.size())
    {
        m_agents[agentId]->AddClient(clientId);
    }

    auto fronthaulLink = FindFronthaulLinkForClient(clientId);
    if (fronthaulLink)
    {
        if (previousAgent != agentId && previousAgent < m_agents.size())
        {
            m_agents[previousAgent]->RemoveFronthaulLink(fronthaulLink);
        }
        if (agentId < m_agents.size())
        {
            m_agents[agentId]->AddFronthaulLink(fronthaulLink);
        }

        fronthaulLink->SetEndpoints(m_agentNodes.Get(agentId), m_clientNodes.Get(clientId));

        const auto clientPosition = m_clientNodes.Get(clientId)->GetObject<MobilityModel>()->GetPosition();
        const auto agentPosition = GetAgentPosition(agentId);
        const auto rssi = EstimateLinkRssi(agentPosition, clientPosition);
        const auto snr = rssi - (-95.0);
        const auto mcs = rssi >= -65 ? 9 : rssi >= -70 ? 7 : rssi >= -76 ? 5 : 3;
        const auto throughput = EstimateLinkThroughput(rssi);
        fronthaulLink->UpdateMetrics(rssi, snr, mcs, throughput, 0.0);
    }

    m_clientToAgent[clientId] = agentId;
}

void
EasyMeshSimulation::RebuildClientRoutes()
{
    Ipv4StaticRoutingHelper staticRoutingHelper;
    const auto isClientHostRoute = [this](const Ipv4RoutingTableEntry& route) {
        return route.IsHost() &&
               std::find(m_clientIp.begin(), m_clientIp.end(), route.GetDest()) != m_clientIp.end();
    };

    auto clearRoutes = [&](Ptr<Node> node, bool clearDefault) {
        auto routing = staticRoutingHelper.GetStaticRouting(node->GetObject<Ipv4>());
        for (int32_t index = static_cast<int32_t>(routing->GetNRoutes()) - 1; index >= 0; --index)
        {
            const auto route = routing->GetRoute(static_cast<uint32_t>(index));
            if ((clearDefault && route.IsDefault()) || isClientHostRoute(route))
            {
                routing->RemoveRoute(static_cast<uint32_t>(index));
            }
        }
        return routing;
    };

    auto controllerRouting = clearRoutes(m_controllerNode.Get(0), false);
    for (uint32_t agentId = 0; agentId < NUM_AGENT; ++agentId)
    {
        clearRoutes(m_agentNodes.Get(agentId), false);
    }

    for (uint32_t clientId = 0; clientId < m_numClients; ++clientId)
    {
        auto clientRouting = clearRoutes(m_clientNodes.Get(clientId), true);
        const uint32_t agentId = m_clientToAgent[clientId];
        if (agentId >= NUM_AGENT || m_clientIp[clientId] == Ipv4Address::GetAny())
        {
            continue;
        }

        clientRouting->SetDefaultRoute(m_agentFronthaulIp[agentId], m_clientFronthaulIfIndex[clientId]);

        uint32_t current = agentId;
        while (m_agentParent[current] >= 0)
        {
            const uint32_t parent = static_cast<uint32_t>(m_agentParent[current]);
            auto parentRouting = staticRoutingHelper.GetStaticRouting(
                m_agentNodes.Get(parent)->GetObject<Ipv4>());
            parentRouting->AddHostRouteTo(m_clientIp[clientId],
                                          m_agentIp[current],
                                          m_agentBackhaulIfIndex[parent]);
            current = parent;
        }

        controllerRouting->AddHostRouteTo(m_clientIp[clientId],
                                          m_agentIp[current],
                                          m_parentIfIndex[current]);
    }
}

void
EasyMeshSimulation::SyncClientAssociations()
{
    bool updated = false;

    for (uint32_t clientId = 0; clientId < m_numClients; ++clientId)
    {
        auto wifiDevice = DynamicCast<WifiNetDevice>(m_clientFronthaulDevices[clientId]);
        auto staMac = wifiDevice ? DynamicCast<StaWifiMac>(wifiDevice->GetMac()) : nullptr;
        if (!staMac || !staMac->IsAssociated())
        {
            NS_LOG_WARN("STA" << clientId << " is not associated yet; keeping provisional agent A"
                        << m_clientToAgent[clientId]);
            continue;
        }

        const uint32_t selectedAgent = FindAgentForFronthaulBssid(staMac->GetBssid(0));
        if (selectedAgent >= NUM_AGENT)
        {
            NS_LOG_WARN("STA" << clientId << " associated to unknown BSSID " << staMac->GetBssid(0));
            continue;
        }

        if (selectedAgent != m_clientToAgent[clientId])
        {
            NS_LOG_INFO("STA" << clientId << " selected BSS Agent" << selectedAgent
                        << " instead of provisional Agent" << m_clientToAgent[clientId]);
        }

        UpdateClientAssociation(clientId, selectedAgent);
        updated = true;
    }

    if (updated)
    {
        RebuildClientRoutes();
    }
}

uint32_t
EasyMeshSimulation::SelectBestUplinkAgent(uint32_t clientId) const
{
    Ptr<MobilityModel> clientMobility = m_clientNodes.Get(clientId)->GetObject<MobilityModel>();
    Vector clientPosition = clientMobility->GetPosition();

    uint32_t bestAgent = 0;
    double bestScore = -std::numeric_limits<double>::infinity();

    for (uint32_t agentId = 0; agentId < NUM_AGENT; ++agentId)
    {
        const auto fronthaulRssi = EstimateLinkRssi(clientPosition, GetAgentPosition(agentId));
        const auto fronthaulThroughput = EstimateLinkThroughput(fronthaulRssi);

        auto backhaulLink = FindBackhaulLinkForAgent(agentId);
        double backhaulThroughput = backhaulLink ? backhaulLink->GetThroughput() : 0.0;
        double backhaulRssi = backhaulLink ? backhaulLink->GetRssi() : -100.0;

        const double pathCapacity = std::min(fronthaulThroughput, backhaulThroughput);
        const double score = pathCapacity * 10.0 + fronthaulRssi + 0.5 * backhaulRssi;

        if (score > bestScore)
        {
            bestScore = score;
            bestAgent = agentId;
        }
    }

    return bestAgent;
}

void 
EasyMeshSimulation::CreateNodes()
{
    m_controllerNode.Create(NUM_CONTROLLER);
    m_agentNodes.Create(NUM_AGENT);
    m_clientNodes.Create(m_numClients);
    NS_LOG_INFO("Nodes: " << NUM_CONTROLLER << " controller + "<< NUM_AGENT << " agents + " << m_numClients << " clients");
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
    for (int i = 0; i < NUM_AGENT; i++) 
        aa->Add(m_agentPositions[i]);
    mob.SetPositionAllocator(aa);
    mob.Install(m_agentNodes);

    auto clientX = CreateObject<UniformRandomVariable>();
    auto clientY = CreateObject<UniformRandomVariable>();
    clientX->SetAttribute("Min", DoubleValue(TOPOLOGY_AREA_MIN_X));
    clientX->SetAttribute("Max", DoubleValue(TOPOLOGY_AREA_MAX_X));
    clientY->SetAttribute("Min", DoubleValue(TOPOLOGY_AREA_MIN_Y));
    clientY->SetAttribute("Max", DoubleValue(TOPOLOGY_AREA_MAX_Y));

    Ptr<ListPositionAllocator> cla = CreateObject<ListPositionAllocator>();
    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        Vector clientPosition(clientX->GetValue(), clientY->GetValue(), 1.0);
        cla->Add(clientPosition);
        NS_LOG_INFO("Randomized STA" << c << " position=("
                    << clientPosition.x << ","
                    << clientPosition.y << ")");
    }
    MobilityHelper mobClient;
    mobClient.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobClient.SetPositionAllocator(cla);
    mobClient.Install(m_clientNodes);
}

void 
EasyMeshSimulation::CreateWifiChannels()
{
    m_channelHelper = YansWifiChannelHelper::Default();
    m_sharedWifiChannel = m_channelHelper.Create();
}

void 
EasyMeshSimulation::CreateBackhaulLinks()
{
    m_wifiHelper.SetStandard(WIFI_STANDARD_80211ax);
    m_wifiHelper.SetRemoteStationManager("ns3::IdealWifiManager");

    m_phyHelper = YansWifiPhyHelper();
    m_phyHelper.SetChannel(m_sharedWifiChannel);
    m_phyHelper.Set("ChannelSettings", StringValue(WIFI_CHANNEL_SETTINGS));
    m_phyHelper.Set("TxPowerStart", DoubleValue(20.0));
    m_phyHelper.Set("TxPowerEnd",   DoubleValue(20.0));

    Time beaconInterval = MicroSeconds(WIFI_BEACON_INTERVAL_US);

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        Ptr<Node> parentNode = (m_agentParent[a] < 0) ? m_controllerNode.Get(0)
                                                      : m_agentNodes.Get(m_agentParent[a]);
        Vector parentPosition = (m_agentParent[a] < 0) ? CONTROLLER_POS
                                                       : GetAgentPosition(m_agentParent[a]);

        std::ostringstream ssidName;
        ssidName << "EasyMesh-Backhaul-" << a;
        Ssid ssid = Ssid(ssidName.str());

        m_macHelper.SetType("ns3::ApWifiMac",
                            "Ssid", SsidValue(ssid),
                            "BeaconInterval", TimeValue(beaconInterval));
        NetDeviceContainer parentDev = m_wifiHelper.Install(m_phyHelper, m_macHelper, parentNode);

        m_macHelper.SetType("ns3::StaWifiMac",
                            "Ssid", SsidValue(ssid),
                            "ActiveProbing", BooleanValue(false));
        NetDeviceContainer agentDev = m_wifiHelper.Install(m_phyHelper, m_macHelper, m_agentNodes.Get(a));

        m_backhaulDevices.Add(parentDev);
        m_backhaulDevices.Add(agentDev);

        double dist = ComputeDistance(parentPosition, GetAgentPosition(a));
        double pl   = 40.0 + 30.0 * std::log10(std::max(1.0, dist));
        double rssi = 20.0 + 6.0 - pl;
        double snr  = rssi - (-95.0);
        uint32_t mcs = rssi >= -65 ? 9 : rssi >= -70 ? 7 : rssi >= -76 ? 5 : 3;
        double tput  = (mcs + 1) * 5.5;

        Ptr<EasyMeshLink> link = Create<EasyMeshLink>(a,
                                                      parentNode,
                                                      m_agentNodes.Get(a),
                                                      LinkType::BACKHAUL,
                                                      5.0);
        link->UpdateMetrics(rssi, snr, mcs, tput, 0.0);
        m_links.push_back(link);

        std::ostringstream parentName;
        if (m_agentParent[a] < 0)
        {
            parentName << "Controller(AP)";
        }
        else
        {
            parentName << "Agent" << m_agentParent[a] << "(AP)";
        }

        NS_LOG_INFO("BH Wi-Fi: " << parentName.str() << " <-> Agent" << a << "(STA)"
                    << "  dist=" << dist << "m  RSSI=" << rssi
                    << "dBm  MCS=" << mcs << "  Tput=" << tput << "Mbps");
    }
}

void 
EasyMeshSimulation::CreateFronthaulLinks()
{
    WifiHelper fh;
    fh.SetStandard(WIFI_STANDARD_80211ax);
    fh.SetRemoteStationManager("ns3::IdealWifiManager");
    NS_LOG_INFO("=== CreateFronthaulLinks ===");

    YansWifiPhyHelper fhPhy;
    fhPhy.SetChannel(m_sharedWifiChannel);
    fhPhy.Set("ChannelSettings", StringValue(WIFI_CHANNEL_SETTINGS));
    fhPhy.Set("TxPowerStart", DoubleValue(20.0));
    fhPhy.Set("TxPowerEnd",   DoubleValue(20.0));

    WifiMacHelper fhMac;
    Time beaconInterval = MicroSeconds(WIFI_BEACON_INTERVAL_US);
    uint32_t linkId = NUM_AGENT;
    const Ssid fronthaulSsid("EasyMesh-Fronthaul");

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        fhMac.SetType("ns3::ApWifiMac",
                      "Ssid", SsidValue(fronthaulSsid),
                      "BeaconInterval", TimeValue(beaconInterval));
        NetDeviceContainer apDev = fh.Install(fhPhy, fhMac, m_agentNodes.Get(a));
        m_fronthaulDevices.Add(apDev);
        m_agentFronthaulDevices[a] = apDev.Get(0);
    }

    if (m_numClients > 0)
    {
        fhMac.SetType("ns3::StaWifiMac",
                      "Ssid", SsidValue(fronthaulSsid),
                      "ActiveProbing", BooleanValue(true));
        NetDeviceContainer staDevices = fh.Install(fhPhy, fhMac, m_clientNodes);
        m_fronthaulDevices.Add(staDevices);

        for (uint32_t c = 0; c < m_numClients; ++c)
        {
            m_clientFronthaulDevices[c] = staDevices.Get(c);
            const uint32_t provisionalAgent = m_clientToAgent[c];
            const auto rssi = EstimateLinkRssi(
                GetAgentPosition(provisionalAgent),
                m_clientNodes.Get(c)->GetObject<MobilityModel>()->GetPosition());
            const auto snr = rssi - (-95.0);
            const auto mcs = rssi >= -65 ? 9 : rssi >= -70 ? 7 : rssi >= -76 ? 5 : 3;
            const auto throughput = EstimateLinkThroughput(rssi);

            Ptr<EasyMeshLink> link = Create<EasyMeshLink>(linkId++,
                                                          m_agentNodes.Get(provisionalAgent),
                                                          m_clientNodes.Get(c),
                                                          LinkType::FRONTHAUL,
                                                          5.0);
            link->UpdateMetrics(rssi, snr, mcs, throughput, 0.0);
            m_links.push_back(link);
        }
    }
}

void 
EasyMeshSimulation::BuildTopology()
{
    NS_LOG_INFO("=== BuildTopology ===");
    CreateNodes();
    InstallMobility();
    CreateWifiChannels();
    BuildBackhaulTree();
    // Không dùng Ethernet backhaul nữa
    CreateBackhaulLinks();      // Controller ↔ Agent qua Wi-Fi

    // Logical EasyMesh objects
    m_controller = Create<EasyMeshController>(0, m_controllerNode.Get(0));

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        auto agent = Create<EasyMeshAgent>(a,
                                           m_agentNodes.Get(a),
                                           AgentRole::HYBRID,
                                           GetAgentPosition(a));
        m_agents.push_back(agent);
        m_controller->RegisterAgent(agent);
    }

    AssignClientsToAgents();
    CreateFronthaulLinks();

    for (auto& l : m_links)
    {
        m_controller->RegisterLink(l);
        if (l->GetType() == LinkType::BACKHAUL)
        {
            for (auto& agent : m_agents)
            {
                if (l->GetDst() == agent->GetNode())
                {
                    agent->AddBackhaulLink(l);
                    break;
                }
            }
        }
        else if (l->GetType() == LinkType::FRONTHAUL)
        {
            for (auto& agent : m_agents)
            {
                if (l->GetSrc() == agent->GetNode())
                {
                    agent->AddFronthaulLink(l);
                    break;
                }
            }
        }
    }
    NS_LOG_INFO("=== Topology ready ===");
}

void 
EasyMeshSimulation::AssignClientsToAgents()
{
    for (uint32_t c = 0; c < m_numClients; c++)
    {
        const auto clientPosition = m_clientNodes.Get(c)->GetObject<MobilityModel>()->GetPosition();
        const uint32_t bestAgent = SelectBestUplinkAgent(c);
        m_clientToAgent[c] = bestAgent;
        const auto agentPosition = GetAgentPosition(bestAgent);

        NS_LOG_INFO("Best uplink for STA" << c
                    << " at (" << clientPosition.x << "," << clientPosition.y << ")"
                    << " is Agent" << bestAgent
                    << " at (" << agentPosition.x << "," << agentPosition.y << ")");
    }
}

void 
EasyMeshSimulation::InstallProtocolStack()
{
    NS_LOG_INFO("=== Install Protocol Stack (shared-channel EasyMesh) ===");

    InternetStackHelper inet;
    inet.Install(m_controllerNode);
    inet.Install(m_agentNodes);
    inet.Install(m_clientNodes);

    Ipv4AddressHelper addr;
    Ipv4StaticRoutingHelper staticRoutingHelper;

    std::vector<Ipv4Address> parentBhIp(NUM_AGENT, Ipv4Address::GetAny());
    std::vector<std::vector<uint32_t>> children(NUM_AGENT);

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        if (m_agentParent[a] >= 0)
        {
            children[m_agentParent[a]].push_back(a);
        }
    }

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        NetDeviceContainer backhaulPair;
        backhaulPair.Add(m_backhaulDevices.Get(2 * a));
        backhaulPair.Add(m_backhaulDevices.Get(2 * a + 1));

        std::ostringstream subnet;
        subnet << "10.1." << (a + 1) << ".0";
        addr.SetBase(subnet.str().c_str(), "255.255.255.0");
        Ipv4InterfaceContainer ifBh = addr.Assign(backhaulPair);

        parentBhIp[a] = ifBh.GetAddress(0);
        m_agentIp[a] = ifBh.GetAddress(1);
        m_parentIfIndex[a] = ifBh.Get(0).second;
        m_agentBackhaulIfIndex[a] = ifBh.Get(1).second;

        if (m_agentParent[a] < 0 && m_controllerIp == Ipv4Address::GetAny())
        {
            m_controllerIp = ifBh.GetAddress(0);
        }
    }

    if (m_controllerIp == Ipv4Address::GetAny())
    {
        m_controllerIp = parentBhIp[0];
    }

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        m_controllerBackhaulIp[a] = m_controllerIp;
    }

    NetDeviceContainer fronthaulGroup;
    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        fronthaulGroup.Add(m_agentFronthaulDevices[a]);
    }
    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        fronthaulGroup.Add(m_clientFronthaulDevices[c]);
    }

    addr.SetBase("10.2.0.0", "255.255.255.0");
    Ipv4InterfaceContainer ifFh = addr.Assign(fronthaulGroup);
    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        m_agentFronthaulIp[a] = ifFh.GetAddress(a);
        m_agentFronthaulIfIndex[a] = ifFh.Get(a).second;
    }
    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        m_clientIp[c] = ifFh.GetAddress(NUM_AGENT + c);
        m_clientFronthaulIfIndex[c] = ifFh.Get(NUM_AGENT + c).second;
    }

    auto ctrlRouting = staticRoutingHelper.GetStaticRouting(m_controllerNode.Get(0)->GetObject<Ipv4>());
    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        auto agentIpv4 = m_agentNodes.Get(a)->GetObject<Ipv4>();
        agentIpv4->SetForwarding(m_agentBackhaulIfIndex[a], true);
        agentIpv4->SetForwarding(m_agentFronthaulIfIndex[a], true);

        auto agentRouting = staticRoutingHelper.GetStaticRouting(agentIpv4);
        agentRouting->SetDefaultRoute(parentBhIp[a], m_agentBackhaulIfIndex[a]);
    }

    (void) children;
    (void) ctrlRouting;
    RebuildClientRoutes();

    NS_LOG_INFO("=== END Protocol Stack ===");
}

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
        UdpClientHelper client(m_controllerBackhaulIp[m_clientToAgent[c]], PORT_UDP_UL);
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
EasyMeshSimulation::InstallBackhaulStress()
{
    // Không cần backhaul stress trong cấu hình nhỏ này
    NS_LOG_INFO("BackhaulStress disabled (only 2 agents)");
}

void 
EasyMeshSimulation::InstallApplications()
{
    NS_LOG_INFO("=== InstallApplications ===");
    InstallUdpUplink();
    NS_LOG_INFO("Applications installed: " << m_numClients << " UDP uplink flow(s)");
}

void 
EasyMeshSimulation::UpdateAnimationLinks(AnimationInterface* anim)
{
    NS_ASSERT(anim);

    const uint8_t backhaulRed = 231;
    const uint8_t backhaulGreen = 76;
    const uint8_t backhaulBlue = 60;
    const uint8_t fronthaulRed = 52;
    const uint8_t fronthaulGreen = 152;
    const uint8_t fronthaulBlue = 219;

    for (const auto& link : m_links)
    {
        std::ostringstream label;
        uint8_t red = fronthaulRed;
        uint8_t green = fronthaulGreen;
        uint8_t blue = fronthaulBlue;
        if (link->GetType() == LinkType::BACKHAUL)
        {
            red = backhaulRed;
            green = backhaulGreen;
            blue = backhaulBlue;
            uint32_t childAgent = 0;
            for (; childAgent < NUM_AGENT; ++childAgent)
            {
                if (m_agentNodes.Get(childAgent) == link->GetDst())
                {
                    break;
                }
            }

            if (link->GetSrc() == m_controllerNode.Get(0))
            {
                label << "UL A" << childAgent << "->C";
            }
            else
            {
                uint32_t parentAgent = 0;
                for (; parentAgent < NUM_AGENT; ++parentAgent)
                {
                    if (m_agentNodes.Get(parentAgent) == link->GetSrc())
                    {
                        break;
                    }
                }
                label << "UL A" << childAgent << "->A" << parentAgent;
            }
        }
        else
        {
            uint32_t agentId = 0;
            uint32_t clientId = 0;
            for (; agentId < NUM_AGENT; ++agentId)
            {
                if (m_agentNodes.Get(agentId) == link->GetSrc())
                {
                    break;
                }
            }
            for (; clientId < m_numClients; ++clientId)
            {
                if (m_clientNodes.Get(clientId) == link->GetDst())
                {
                    break;
                }
            }
            label << "FH A" << agentId << "-STA" << clientId;
        }

        label << " RSSI=" << std::fixed << std::setprecision(1) << link->GetRssi() << "dBm";
        anim->UpdateLinkDescription(link->GetSrc(), link->GetDst(), label.str(), red, green, blue);
    }
}

void 
EasyMeshSimulation::ConfigureAnimation(AnimationInterface& anim)
{
    anim.SetStartTime(Seconds(UL_START));
    anim.SetStopTime(Seconds(m_duration));
    anim.SetMobilityPollInterval(Seconds(1.0));
    anim.SetMaxPktsPerTraceFile(200000);
    anim.SkipPacketTracing();

    anim.UpdateNodeDescription(m_controllerNode.Get(0), "Controller");
    anim.UpdateNodeColor(m_controllerNode.Get(0), 0, 102, 204);
    anim.UpdateNodeSize(m_controllerNode.Get(0), 2.0, 2.0);

    for (uint32_t a = 0; a < NUM_AGENT; ++a)
    {
        std::ostringstream name;
        name << "Agent " << a << " UL->";
        if (m_agentParent[a] < 0)
        {
            name << "C";
        }
        else
        {
            name << "A" << m_agentParent[a];
        }
        anim.UpdateNodeDescription(m_agentNodes.Get(a), name.str());
        anim.UpdateNodeColor(m_agentNodes.Get(a), 46, 204, 113);
        anim.UpdateNodeSize(m_agentNodes.Get(a), 2.0, 2.0);
    }

    for (uint32_t c = 0; c < m_numClients; ++c)
    {
        std::ostringstream name;
        name << "STA " << c << " -> A" << m_clientToAgent[c];
        anim.UpdateNodeDescription(m_clientNodes.Get(c), name.str());
        anim.UpdateNodeColor(m_clientNodes.Get(c), 231, 126, 35);
        anim.UpdateNodeSize(m_clientNodes.Get(c), 1.0, 1.0);
    }

    Simulator::Schedule(Seconds(UL_START),
                        &EasyMeshSimulation::UpdateAnimationLinks,
                        this,
                        &anim);

}

void 
EasyMeshSimulation::ScheduleEvents()
{
    Simulator::Schedule(Seconds(1.0),
                        &EasyMeshSimulation::SyncClientAssociations,
                        this);
    Simulator::Schedule(Seconds(1.5),
                        &EasyMeshSimulation::SyncClientAssociations,
                        this);
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

void 
EasyMeshSimulation::Run()
{
    NS_LOG_INFO("=== Simulation start  duration=" << m_duration << "s ===");
    m_monitor = m_monHelper.InstallAll();
    std::ostringstream animationFile;
    animationFile << "easymesh-sim-animation-run" << m_scenarioRun << ".xml";
    AnimationInterface anim(animationFile.str());
    ConfigureAnimation(anim);
    if (m_pcap) m_phyHelper.EnablePcapAll("easymesh");
    Simulator::Stop(Seconds(m_duration + 2.0));
    Simulator::Run();

    Ptr<Ipv4FlowClassifier> clf =
        DynamicCast<Ipv4FlowClassifier>(m_monHelper.GetClassifier());
    m_trafficStats.Collect(m_monitor, clf);
    m_controller->ApplyTrafficStats(m_trafficStats);

    Simulator::Destroy();
    std::cout << "[EasyMeshSim] Animation saved to " << animationFile.str() << "\n";
    NS_LOG_INFO("=== Simulation complete ===");
}

void 
EasyMeshSimulation::PrintResults()
{
    m_controller->PrintTopology();
    m_controller->PrintSteeringLog();
    m_trafficStats.PrintPerFlowTable();
    m_controller->PrintAgentTraffic();
    m_trafficStats.PrintNetworkSummary();
}

int main(int argc, char* argv[])
{
    LogComponentEnable("EasyMeshSim",        LOG_LEVEL_INFO);
    LogComponentEnable("EasyMeshController", LOG_LEVEL_INFO);

    double   duration   = DURATION;
    uint32_t scenarioRuns = SCENARIO_RUNS;
    uint32_t numClients = NUM_STA;          
    bool     pcap       = false;

    CommandLine cmd;
    cmd.AddValue("duration", "Simulation duration (s)", duration);
    cmd.AddValue("scenarios", "Number of randomized scenario runs", scenarioRuns);
    cmd.AddValue("clients",  "Number of STA clients",   numClients);
    cmd.AddValue("pcap",     "Enable PCAP capture",     pcap);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(42);

    for (uint32_t run = 1; run <= scenarioRuns; ++run)
    {
        std::cout << "\n=== Scenario " << run << "/" << scenarioRuns << " ===\n" << std::flush;
        RngSeedManager::SetRun(run);

        EasyMeshSimulation sim;
        sim.SetDuration(duration);
        sim.SetScenarioRun(run);
        sim.SetNumClients(numClients);
        sim.EnablePcap(pcap);

        sim.BuildTopology();
        sim.InstallProtocolStack();
        sim.InstallApplications();
        sim.ScheduleEvents();
        sim.Run();
        sim.PrintResults();
    }

    return 0;
}