#include "easymesh-controller.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshController");

EasyMeshController::EasyMeshController(uint32_t id, Ptr<Node> node)
    : m_id(id), m_node(node) {}

void 
EasyMeshController::RegisterAgent(Ptr<EasyMeshAgent> agent)
{
    m_agents.push_back(agent);
}

void 
EasyMeshController::RegisterLink(Ptr<EasyMeshLink> link)
{
    m_links.push_back(link);
}

Ptr<EasyMeshAgent> 
EasyMeshController::GetAgent(uint32_t agentId)
{
    for (auto& a : m_agents)
        if (a->GetId() == agentId) return a;
    return nullptr;
}

double 
EasyMeshController::EstimateRssi(Ptr<EasyMeshAgent> agent) const
{
    Ptr<MobilityModel> mob = agent->GetNode()->GetObject<MobilityModel>();
    if (!mob) return -80.0;
    Vector pos = mob->GetPosition();
    double dist = std::sqrt(pos.x * pos.x + pos.y * pos.y);
    if (dist < 1.0) dist = 1.0;
    // RSSI = TxPow + 2*AntGain - (PL0 + 10*exp*log10(d/d0))
    double pl0 = 40.0;     // ref loss at 1m
    double pl  = pl0 + 10.0 * 3.0 * std::log10(dist);
    return 20.0 + 3.0 + 3.0 - pl;
}

uint32_t 
EasyMeshController::SelectTargetAgent(uint32_t excludeId) const
{
    uint32_t best  = excludeId;
    double   bestL = 1e9;
    for (auto& a : m_agents) 
    {
        if (a->GetId() == excludeId) continue;
        double score = a->GetLoad() + a->GetChanUtil() * 0.5;
        if (score < bestL) { bestL = score; best = a->GetId(); }
    }
    return best;
}

void 
EasyMeshController::CollectApMetrics()
{
    double t = Simulator::Now().GetSeconds();
    NS_LOG_INFO("[Controller t=" << t << "] AP_METRICS_REPORT");

    for (auto& agent : m_agents) 
    {
        // Simulate load: 12% per associated client + random noise
        double jitter = ((rand() % 20) - 10) / 100.0;
        double load   = std::min(1.0, agent->GetClientCount() * 0.12 + jitter);
        agent->SetLoad(load);
        agent->SetChanUtil(std::min(1.0, load * 1.3 + std::abs(jitter) * 0.5));

        NS_LOG_INFO("  Agent[" << agent->GetId()
                    << "] clients=" << agent->GetClientCount()
                    << " load=" << (int)(load * 100) << "%"
                    << " chanUtil=" << (int)(agent->GetChanUtil() * 100) << "%");
    }

    // Refresh backhaul link RSSI via mobility
    for (auto& link : m_links) 
    {
        if (link->GetType() != LinkType::BACKHAUL) continue;
        Ptr<MobilityModel> sm = link->GetSrc()->GetObject<MobilityModel>();
        Ptr<MobilityModel> dm = link->GetDst()->GetObject<MobilityModel>();
        if (!sm || !dm) continue;
        Vector sp = sm->GetPosition(), dp = dm->GetPosition();
        double dist = std::sqrt(std::pow(sp.x - dp.x, 2) +
                                std::pow(sp.y - dp.y, 2));
        double rssi = 20.0 + 6.0 - (40.0 + 10.0 * 3.0 * std::log10(std::max(1.0, dist)));
        double snr  = rssi - (-95.0);        // thermal noise floor ~-95 dBm @ 20MHz
        uint32_t mcs = (rssi >= -65) ? 9 : (rssi >= -70) ? 7 :
                        (rssi >= -76) ? 5 : (rssi >= -82) ? 3 : 0;
        double tput  = (mcs + 1) * 5.5;
        link->UpdateMetrics(rssi, snr, mcs, tput, 0.0);
    }
}

void 
EasyMeshController::RunSteeringEngine()
{
    double t = Simulator::Now().GetSeconds();
    NS_LOG_INFO("[Controller t=" << t << "] STEERING ENGINE TICK");

    for (auto& agent : m_agents) 
    {
        double rssi  = EstimateRssi(agent);
        SteeringReason reason;
        bool doSteer = false;

        if (rssi < RSSI_STEER_THRESHOLD) 
        {
            reason   = SteeringReason::RSSI_LOW;
            doSteer  = true;
        } 
        else if (agent->GetLoad() > LOAD_STEER_THRESHOLD) 
        {
            reason   = SteeringReason::LOAD_BALANCE;
            doSteer  = true;
        } 
        else if (agent->GetChanUtil() > UTIL_STEER_THRESHOLD) 
        {
            reason   = SteeringReason::CHAN_UTIL;
            doSteer  = true;
        }

        if (!doSteer || agent->GetClientCount() == 0) continue;

        uint32_t targetId = SelectTargetAgent(agent->GetId());
        if (targetId == agent->GetId()) continue;

        Ptr<EasyMeshAgent> target = GetAgent(targetId);
        if (!target) continue;

        // Move one client
        uint32_t clientId = agent->GetClients().front();
        agent->RemoveClient(clientId);
        agent->IncrementSteeredOut();
        target->AddClient(clientId);
        target->IncrementSteeredIn();

        double rssiAfter = EstimateRssi(target);

        SteeringEvent ev;
        ev.timeS         = t;
        ev.clientId      = clientId;
        ev.fromAgent     = agent->GetId();
        ev.toAgent       = targetId;
        ev.reason        = reason;
        ev.rssiBeforeDbm = rssi;
        ev.rssiAfterDbm  = rssiAfter;
        ev.success       = true;
        m_steeringLog.push_back(ev);

        NS_LOG_INFO("  STEER client=" << clientId
                    << " A" << agent->GetId() << "->A" << targetId
                    << " reason=" << ev.ReasonStr()
                    << " rssi: " << rssi << "->" << rssiAfter << "dBm");
    }
}

void 
EasyMeshController::OptimizeBackhaulTopology()
{
    NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds()
                << "] BACKHAUL TOPOLOGY OPTIMIZATION");
    for (auto& link : m_links) 
    {
        if (link->GetType() != LinkType::BACKHAUL) continue;
        if (link->GetRssi() < -85.0) 
        {
            link->SetActive(false);
            NS_LOG_INFO("  Deactivated weak link[" << link->GetId()
                        << "] RSSI=" << link->GetRssi() << "dBm");
        } 
        else 
        {
            link->SetActive(true);
        }
    }
}

void 
EasyMeshController::ApplyTrafficStats(const TrafficStats& ts)
{
    // Simple mapping: distribute aggregate UL/DL evenly across agents
    auto& flows = ts.GetFlows();
    std::map<uint32_t, double> agentTput;
    std::map<uint32_t, double> agentDelay;
    std::map<uint32_t, double> agentPdr;
    std::map<uint32_t, int>    agentFlows;

    // Without exact STA↔Agent mapping we round-robin assign
    int idx = 0;
    for (auto& f : flows) 
    {
        uint32_t aid = idx % m_agents.size();
        agentTput[aid]  += f.throughputMbps;
        agentDelay[aid] += f.meanDelayMs;
        agentPdr[aid]   += f.pdr;
        agentFlows[aid]++;
        idx++;
    }

    for (auto& agent : m_agents) 
    {
        uint32_t aid = agent->GetId();
        int      cnt = agentFlows.count(aid) ? agentFlows[aid] : 1;
        agent->AddUlThroughput(agentTput[aid] * 0.5);
        agent->AddDlThroughput(agentTput[aid] * 0.5);
        agent->SetMeanDelay(agentDelay[aid] / cnt);
        agent->SetMeanPdr(agentPdr[aid] / cnt);
    }
}

void 
EasyMeshController::PrintTopology() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                       EasyMesh Network Topology                              ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << "  Controller[" << m_id << "]   Agents: " << m_agents.size()
              << "   Links: " << m_links.size() << "\n\n";

    for (auto& ag : m_agents) 
    {
        std::string roleStr = (ag->GetRole() == AgentRole::HYBRID)         ? "HYBRID" :
                              (ag->GetRole() == AgentRole::FRONTHAUL_ONLY) ? "FH_ONLY" : "BH_ONLY";
        Vector p = ag->GetPosition();
        std::cout << "  ┌─ Agent[" << ag->GetId() << "] " << roleStr
                  << "  pos=(" << p.x << "," << p.y << ")"
                  << "  clients=" << ag->GetClientCount()
                  << "  load=" << (int)(ag->GetLoad() * 100) << "%"
                  << "  chanUtil=" << (int)(ag->GetChanUtil() * 100) << "%"
                  << "  steerOut/In=" << ag->GetSteeredOut() << "/" << ag->GetSteeredIn() << "\n";

        for (auto& l : ag->GetBackhaulLinks()) 
        {
            std::cout << "  │  BH-Link[" << l->GetId() << "]"
                      << "  RSSI=" << std::fixed << std::setprecision(1) << l->GetRssi() << "dBm"
                      << "  MCS=" << l->GetMcs()
                      << "  Tput=" << l->GetThroughput() << "Mbps"
                      << "  " << (l->IsActive() ? "ACTIVE" : "INACTIVE") << "\n";
        }

        for (auto& l : ag->GetFronthaulLinks()) 
        {
            std::cout << "  │  FH-Link[" << l->GetId() << "]"
                      << "  RSSI=" << l->GetRssi() << "dBm"
                      << "  " << (l->IsActive() ? "ACTIVE" : "INACTIVE") << "\n";
        }
        std::cout << "  └─\n";
    }
}

void 
EasyMeshController::PrintSteeringLog() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                          BSS Steering Log                                    ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";

    if (m_steeringLog.empty()) 
    {
        std::cout << "  (No steering events)\n";
        return;
    }
    std::cout << std::left
              << std::setw(8)  << "t(s)"
              << std::setw(8)  << "Client"
              << std::setw(12) << "From→To"
              << std::setw(12) << "Reason"
              << std::setw(14) << "RSSI Before"
              << std::setw(12) << "RSSI After"
              << "OK?\n";
    std::cout << std::string(70, '-') << "\n";

    for (auto& ev : m_steeringLog) 
    {
        std::cout << std::left
                  << std::setw(8)  << std::fixed << std::setprecision(1) << ev.timeS
                  << std::setw(8)  << ev.clientId
                  << "A" << ev.fromAgent << "→A" << ev.toAgent << "       "
                  << std::setw(12) << ev.ReasonStr()
                  << std::setw(14) << std::setprecision(1) << ev.rssiBeforeDbm
                  << std::setw(12) << ev.rssiAfterDbm
                  << (ev.success ? "✓" : "✗") << "\n";
    }
}

void 
EasyMeshController::PrintAgentTraffic() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    Per-Agent Traffic Summary                                 ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::left
              << std::setw(8)  << "Agent"
              << std::setw(10) << "Clients"
              << std::setw(16) << "UL Tput(Mbps)"
              << std::setw(16) << "DL Tput(Mbps)"
              << std::setw(14) << "Delay(ms)"
              << std::setw(8)  << "PDR%"
              << "Load%\n";
    std::cout << std::string(74, '-') << "\n";

    for (auto& ag : m_agents) 
    {
        std::cout << std::left
                  << std::setw(8)  << ag->GetId()
                  << std::setw(10) << ag->GetClientCount()
                  << std::setw(16) << std::fixed << std::setprecision(3) << ag->GetUlThroughput()
                  << std::setw(16) << ag->GetDlThroughput()
                  << std::setw(14) << std::setprecision(2) << ag->GetMeanDelay()
                  << std::setw(8)  << std::setprecision(1) << ag->GetMeanPdr() * 100.0
                  << (int)(ag->GetLoad() * 100) << "%\n";
    }
    std::cout << std::string(74, '-') << "\n";
}
