#include "easymesh-controller.h"
#include "../parameters.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EasyMeshController");

EasyMeshController::EasyMeshController(uint32_t id, Ptr<Node> node)
    : m_id(id),
      m_node(node),
      m_planner(std::vector<uint16_t> FH24_CHANNELS, std::vector<uint16_t> FH5_CHANNELS) {}

void
EasyMeshController::RegisterAgent(Ptr<EasyMeshAgent> agent)
{
    m_agents.push_back(agent);
}

Ptr<EasyMeshAgent>
EasyMeshController::GetAgent(uint32_t agentId) const
{
    for (auto& a : m_agents)
        if (a->GetId() == agentId) return a;
    return nullptr;
}

// ── Topology database ───────────────────────────────────────────

void
EasyMeshController::NotifyClientAssociated(uint32_t clientId, Ptr<EasyMeshRadio> radio)
{
    auto& rec = m_clients.at(clientId);
    if (rec.radio && rec.radio != radio)
    {
        rec.radio->RemoveClient(clientId);
    }
    radio->AddClient(clientId);
    rec.radio      = radio;
    rec.associated = true;
}

void
EasyMeshController::NotifyClientDisassociated(uint32_t clientId)
{
    m_clients.at(clientId).associated = false;
}

void
EasyMeshController::NotifyTransitionStarted(uint32_t clientId)
{
    auto& rec = m_clients.at(clientId);
    rec.pending        = true;
    rec.lastTransition = Simulator::Now().GetSeconds();
}

void
EasyMeshController::RecordTransition(const SteeringEvent& ev)
{
    m_steeringLog.push_back(ev);
    if (ev.isBackhaul)
    {
        return;
    }

    m_clients.at(ev.staId).pending = false;
    if (ev.success)
    {
        if (auto from = (ev.fromAgent >= 0) ? GetAgent(ev.fromAgent) : nullptr)
        {
            from->IncrementSteeredOut();
        }
        if (auto to = GetAgent(ev.toAgent))
        {
            to->IncrementSteeredIn();
        }
    }
}

bool
EasyMeshController::IsTransitionAllowed(uint32_t clientId) const
{
    const auto& rec = m_clients.at(clientId);
    return rec.associated && !rec.pending &&
           Simulator::Now().GetSeconds() - rec.lastTransition >= STEER_COOLDOWN;
}

Ptr<EasyMeshRadio>
EasyMeshController::GetServingRadio(uint32_t clientId) const
{
    return m_clients.at(clientId).radio;
}

// ── Selection helpers ───────────────────────────────────────────

std::vector<Ptr<EasyMeshRadio>>
EasyMeshController::OnlineRadios() const
{
    std::vector<Ptr<EasyMeshRadio>> radios;
    for (const auto& agent : m_agents)
    {
        if (!agent->IsOnline()) continue;
        for (const auto& radio : agent->GetRadios())
        {
            radios.push_back(radio);
        }
    }
    return radios;
}

double
EasyMeshController::RadioScore(uint32_t clientId, Ptr<EasyMeshRadio> radio) const
{
    // Expected client throughput: link rate capped by the Agent's backhaul
    // path capacity, discounted by how busy the channel already is.
    const double rssi = m_ops.clientRssi(clientId, radio);
    const double rate = m_ops.phyRate(rssi, radio->GetChannel().widthMhz);
    const double backhaul = GetAgent(radio->GetAgentId())->GetBackhaulCapacity();
    return std::min(rate, backhaul) * (1.0 - 0.8 * radio->GetChanUtil());
}

Ptr<EasyMeshRadio>
EasyMeshController::SelectBestRadio(uint32_t clientId,
                                    Ptr<EasyMeshRadio> exclude,
                                    std::optional<EmBand> band,
                                    double minRssiDbm) const
{
    Ptr<EasyMeshRadio> best;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (const auto& radio : OnlineRadios())
    {
        if (radio == exclude || (band && radio->GetBand() != *band)) continue;
        if (m_ops.clientRssi(clientId, radio) < minRssiDbm) continue;

        const double score = RadioScore(clientId, radio);
        if (score > bestScore)
        {
            bestScore = score;
            best = radio;
        }
    }
    return best;
}

double
EasyMeshController::Interference(Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b) const
{
    return std::max(0.0, m_ops.radioRssi(a, b) - INTERFERENCE_FLOOR_DBM);
}

bool
EasyMeshController::Steer(uint32_t clientId, Ptr<EasyMeshRadio> target, SteeringReason reason)
{
    if (!target || target == m_clients.at(clientId).radio)
    {
        return false;
    }
    return m_ops.steerClient(clientId, target, reason);
}

// ── Periodic tasks ──────────────────────────────────────────────

void
EasyMeshController::CollectApMetrics()
{
    double maxUtil = 0.0, sumUtil = 0.0;
    uint32_t count = 0;
    Ptr<EasyMeshRadio> busiest;

    for (const auto& radio : OnlineRadios())
    {
        radio->SampleUtilization();
        sumUtil += radio->GetChanUtil();
        count++;
        if (radio->GetChanUtil() >= maxUtil)
        {
            maxUtil = radio->GetChanUtil();
            busiest = radio;
        }
        NS_LOG_DEBUG("  A" << radio->GetAgentId() << "/" << BandStr(radio->GetBand())
                     << " ch" << radio->GetChannel().number
                     << " clients=" << radio->GetClientCount()
                     << " util=" << (int)(radio->GetChanUtil() * 100) << "%");
    }

    if (busiest)
    {
        NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds() << "] AP_METRICS "
                    << count << " radios  meanUtil=" << (int)(100 * sumUtil / count) << "%"
                    << "  busiest=A" << busiest->GetAgentId() << "/" << BandStr(busiest->GetBand())
                    << " ch" << busiest->GetChannel().number << " " << (int)(maxUtil * 100) << "%");
    }
}

void
EasyMeshController::RunSteeringEngine()
{
    if (m_steering)
    {
        for (uint32_t c = 0; c < m_clients.size(); ++c)
        {
            if (!IsTransitionAllowed(c)) continue;
            auto serving = m_clients[c].radio;
            if (!serving) continue;
            const double rssi = m_ops.clientRssi(c, serving);

            // 1. Client steering: sticky client with poor signal
            if (rssi < STEER_RSSI_THRESHOLD)
            {
                auto target = SelectBestRadio(c, serving, std::nullopt, MIN_ASSOC_RSSI_DBM);
                if (target && m_ops.clientRssi(c, target) >= rssi + STEER_RSSI_GAIN_DB)
                {
                    Steer(c, target, SteeringReason::RSSI_LOW);
                    continue;
                }
            }

            // 2. Band steering
            if (serving->GetBand() == EmBand::BAND_2_4GHZ)
            {
                // prefer the 5 GHz radio of the same Agent, else the best 5 GHz BSS
                auto agent  = GetAgent(serving->GetAgentId());
                auto target = agent->GetRadio(EmBand::BAND_5GHZ);
                if (!target || m_ops.clientRssi(c, target) < BAND_STEER_TO_5G_RSSI)
                {
                    target = SelectBestRadio(c, serving, EmBand::BAND_5GHZ, BAND_STEER_TO_5G_RSSI);
                }
                if (target && m_ops.clientRssi(c, target) >= BAND_STEER_TO_5G_RSSI)
                {
                    Steer(c, target, SteeringReason::BAND_STEER);
                }
            }
            else if (rssi < BAND_STEER_TO_24_RSSI)
            {
                auto target = SelectBestRadio(c, serving, EmBand::BAND_2_4GHZ, MIN_ASSOC_RSSI_DBM);
                if (target && m_ops.clientRssi(c, target) >= rssi + STEER_RSSI_GAIN_DB)
                {
                    Steer(c, target, SteeringReason::BAND_STEER);
                }
            }
        }
    }

    if (m_loadBalancing)
    {
        RunLoadBalancing();
    }
}

void
EasyMeshController::RunLoadBalancing()
{
    for (const auto& radio : OnlineRadios())
    {
        const double util = radio->GetChanUtil();
        if (util <= LB_UTIL_THRESHOLD || radio->GetClientCount() == 0) continue;

        // Several own clients → the BSS itself is loaded; a single client on a
        // busy channel → the load comes from neighbouring BSSs.
        const auto reason = (radio->GetClientCount() >= 2) ? SteeringReason::LOAD_BALANCE
                                                           : SteeringReason::CHAN_UTIL;
        uint32_t bestClient = 0;
        Ptr<EasyMeshRadio> bestTarget;
        double bestScore = -std::numeric_limits<double>::infinity();

        for (uint32_t c : radio->GetClients())
        {
            if (!IsTransitionAllowed(c) || m_clients[c].radio != radio) continue;
            for (const auto& target : OnlineRadios())
            {
                if (target == radio) continue;
                if (target->GetChanUtil() > util - LB_UTIL_MARGIN) continue;
                if (m_ops.clientRssi(c, target) < LB_MIN_RSSI_DBM) continue;

                const double score = RadioScore(c, target);
                if (score > bestScore)
                {
                    bestScore  = score;
                    bestClient = c;
                    bestTarget = target;
                }
            }
        }

        if (bestTarget)
        {
            NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds() << "] "
                        << ReasonStr(reason) << " A" << radio->GetAgentId() << "/"
                        << BandStr(radio->GetBand()) << " util=" << (int)(util * 100) << "% → STA"
                        << bestClient << " to A" << bestTarget->GetAgentId() << "/"
                        << BandStr(bestTarget->GetBand()) << " util="
                        << (int)(bestTarget->GetChanUtil() * 100) << "%");
            Steer(bestClient, bestTarget, reason);
        }
    }
}

void
EasyMeshController::PlanChannels(bool initial)
{
    const auto radios = OnlineRadios();
    if (radios.empty()) return;

    std::vector<ChannelPlanner::Radio> input;
    std::map<uint32_t, uint16_t> current;
    for (uint32_t k = 0; k < radios.size(); ++k)
    {
        input.push_back({k, radios[k]->GetBand(), radios[k]->GetChannel().number,
                         radios[k]->GetChanUtil()});
        current[k] = radios[k]->GetChannel().number;
    }
    auto interference = [&](uint32_t a, uint32_t b) { return Interference(radios[a], radios[b]); };

    const double costBefore = m_planner.TotalCost(input, current, interference);
    const double weightedBefore = m_planner.TotalCost(input, current, interference, true);
    const auto plan = initial ? m_planner.Plan(input, interference)
                              : m_planner.Replan(input, interference, CHANNEL_SWITCH_MIN_GAIN,
                                                 CHANNEL_PLAN_MAX_MOVES);
    const double costAfter = m_planner.TotalCost(input, plan, interference);
    const double weightedAfter = m_planner.TotalCost(input, plan, interference, true);

    uint32_t moved = 0;
    for (uint32_t k = 0; k < radios.size(); ++k)
    {
        EmChannel ch = radios[k]->GetChannel();
        if (ch.number == plan.at(k)) continue;
        ch.number = plan.at(k);
        if (initial)
        {
            radios[k]->SetChannel(ch, false);
        }
        else
        {
            NS_LOG_INFO("  CHANNEL_SWITCH A" << radios[k]->GetAgentId() << "/"
                        << BandStr(radios[k]->GetBand()) << " ch" << radios[k]->GetChannel().number
                        << " -> ch" << ch.number << "  (util=" << (int)(radios[k]->GetChanUtil() * 100)
                        << "%, clients=" << radios[k]->GetClientCount() << ")");
            m_ops.switchChannel(radios[k], ch);
        }
        moved++;
    }

    m_planRounds++;
    NS_LOG_INFO("[Controller t=" << Simulator::Now().GetSeconds() << "] CHANNEL_PLAN "
                << (initial ? "initial" : "dynamic") << ": " << moved << " radio(s) retuned,"
                << " co-channel interference " << costBefore << " -> " << costAfter << " dB"
                << " (util-weighted " << weightedBefore << " -> " << weightedAfter << ")");

    if (initial)
    {
        RecordInitialChannels();
    }
}

void
EasyMeshController::RecordInitialChannels()
{
    const auto radios = OnlineRadios();
    std::vector<ChannelPlanner::Radio> input;
    std::map<uint32_t, uint16_t> current;
    for (uint32_t k = 0; k < radios.size(); ++k)
    {
        m_initialChannel[PeekPointer(radios[k])] = radios[k]->GetChannel().number;
        input.push_back({k, radios[k]->GetBand(), radios[k]->GetChannel().number, 0.0});
        current[k] = radios[k]->GetChannel().number;
    }
    m_initialPlanCost = m_planner.TotalCost(
        input, current, [&](uint32_t a, uint32_t b) { return Interference(radios[a], radios[b]); });
}

void
EasyMeshController::HandleAgentOffline(uint32_t agentId)
{
    for (uint32_t c = 0; c < m_clients.size(); ++c)
    {
        auto serving = m_clients[c].radio;
        if (!serving || serving->GetAgentId() != agentId) continue;

        serving->RemoveClient(c);
        auto target = SelectBestRadio(c, nullptr, std::nullopt, MIN_ASSOC_RSSI_DBM);
        if (!target)
        {
            target = SelectBestRadio(c, nullptr, std::nullopt,
                                     -std::numeric_limits<double>::infinity());
        }
        if (target)
        {
            m_ops.steerClient(c, target, SteeringReason::AGENT_OFFLINE);
        }
    }
}

// ── Printing ────────────────────────────────────────────────────

void
EasyMeshController::PrintTopology() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                       EasyMesh Network Topology                              ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << "  Controller[" << m_id << "]   Agents: " << m_agents.size() << "\n\n";

    for (auto& ag : m_agents)
    {
        Vector p = ag->GetPosition();
        std::cout << "  ┌─ Agent[" << ag->GetId() << "] " << (ag->IsOnline() ? "ONLINE " : "OFFLINE")
                  << "  pos=(" << std::fixed << std::setprecision(1) << p.x << "," << p.y << ")"
                  << "  uplink=" << (ag->GetParent() < 0 ? std::string("C")
                                                         : "A" + std::to_string(ag->GetParent()))
                  << "  clients=" << ag->GetClientCount()
                  << "  steerOut/In=" << ag->GetSteeredOut() << "/" << ag->GetSteeredIn() << "\n";

        if (auto l = ag->GetBackhaulLink())
        {
            std::cout << "  │  BH-Link  RSSI=" << l->GetRssi() << "dBm"
                      << "  MCS=" << l->GetMcs()
                      << "  Rate=" << l->GetThroughput() << "Mbps"
                      << "  PathCap=" << ag->GetBackhaulCapacity() << "Mbps"
                      << "  " << (l->IsActive() ? "ACTIVE" : "INACTIVE") << "\n";
        }

        for (auto& r : ag->GetRadios())
        {
            std::cout << "  │  FH " << std::setw(4) << BandStr(r->GetBand())
                      << " ch" << std::setw(3) << r->GetChannel().number
                      << "/" << r->GetChannel().widthMhz << "MHz"
                      << "  meanUtil=" << std::setw(3) << (int)(r->GetMeanChanUtil() * 100) << "%"
                      << "  clients=[";
            for (size_t i = 0; i < r->GetClients().size(); ++i)
            {
                std::cout << (i ? "," : "") << "STA" << r->GetClients()[i];
            }
            std::cout << "]\n";
        }
        std::cout << "  └─\n";
    }
}

void
EasyMeshController::PrintChannelPlan() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                          Channel Plan                                        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::left
              << std::setw(8)  << "Agent"
              << std::setw(7)  << "Band"
              << std::setw(10) << "Initial"
              << std::setw(10) << "Final"
              << std::setw(10) << "Switches"
              << std::setw(11) << "MeanUtil%"
              << "Status\n";
    std::cout << std::string(64, '-') << "\n";

    std::vector<ChannelPlanner::Radio> input;
    std::map<uint32_t, uint16_t> current;
    std::vector<Ptr<EasyMeshRadio>> radios;
    for (auto& ag : m_agents)
    {
        for (auto& r : ag->GetRadios())
        {
            auto it = m_initialChannel.find(PeekPointer(r));
            std::cout << std::left
                      << std::setw(8)  << ("A" + std::to_string(ag->GetId()))
                      << std::setw(7)  << BandStr(r->GetBand())
                      << std::setw(10) << (it != m_initialChannel.end() ? it->second : r->GetChannel().number)
                      << std::setw(10) << r->GetChannel().number
                      << std::setw(10) << r->GetChannelSwitches()
                      << std::setw(11) << std::fixed << std::setprecision(1) << r->GetMeanChanUtil() * 100
                      << (ag->IsOnline() ? "online" : "offline") << "\n";
            if (ag->IsOnline())
            {
                const uint32_t k = radios.size();
                radios.push_back(r);
                input.push_back({k, r->GetBand(), r->GetChannel().number, 0.0});
                current[k] = r->GetChannel().number;
            }
        }
    }
    const double finalCost = m_planner.TotalCost(
        input, current, [&](uint32_t a, uint32_t b) { return Interference(radios[a], radios[b]); });
    std::cout << std::string(64, '-') << "\n";
    std::cout << "  Co-channel interference (sum of dB above " << INTERFERENCE_FLOOR_DBM
              << " dBm): initial " << std::setprecision(1) << m_initialPlanCost
              << "  final " << finalCost << "   re-plan rounds: " << m_planRounds << "\n";
}

void
EasyMeshController::PrintSteeringLog() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                  Steering / Roaming / Backhaul Transition Log                ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";

    if (m_steeringLog.empty())
    {
        std::cout << "  (No transitions)\n";
        return;
    }

    auto endpoint = [](bool backhaul, int32_t agent, EmBand band) {
        if (agent < 0) return std::string(backhaul ? "C" : "-");
        std::string s = "A" + std::to_string(agent);
        if (!backhaul) s += std::string("/") + BandStr(band);
        return s;
    };

    std::cout << std::left
              << std::setw(8)  << "t(s)"
              << std::setw(8)  << "STA"
              << std::setw(24) << "From -> To"
              << std::setw(12) << "Reason"
              << std::setw(9)  << "RSSI0"
              << std::setw(9)  << "RSSI1"
              << std::setw(11) << "Latency"
              << "OK?\n";
    std::cout << std::string(84, '-') << "\n";

    std::map<SteeringReason, std::pair<uint32_t, uint32_t>> perReason;   // total, ok
    std::map<SteeringReason, double> latencySum;
    for (auto& ev : m_steeringLog)
    {
        const std::string sta = ev.isBackhaul ? "bSTA" + std::to_string(ev.staId)
                                              : "STA" + std::to_string(ev.staId);
        const std::string path = endpoint(ev.isBackhaul, ev.fromAgent, ev.fromBand) + " -> " +
                                 endpoint(ev.isBackhaul, ev.toAgent, ev.toBand);
        std::cout << std::left
                  << std::setw(8)  << std::fixed << std::setprecision(2) << ev.timeS
                  << std::setw(8)  << sta
                  << std::setw(24) << path
                  << std::setw(12) << ReasonStr(ev.reason)
                  << std::setw(9)  << std::setprecision(1) << ev.rssiBeforeDbm
                  << std::setw(9)  << ev.rssiAfterDbm
                  << std::setw(11) << (ev.success ? std::to_string((int)ev.latencyMs) + "ms" : "-")
                  << (ev.success ? "✓" : "✗") << "\n";
        perReason[ev.reason].first++;
        if (ev.success)
        {
            perReason[ev.reason].second++;
            latencySum[ev.reason] += ev.latencyMs;
        }
    }

    std::cout << std::string(84, '-') << "\n  Summary:\n";
    for (auto& [reason, counts] : perReason)
    {
        std::cout << "    " << std::setw(12) << ReasonStr(reason)
                  << " total=" << std::setw(4) << counts.first
                  << " ok=" << std::setw(4) << counts.second
                  << " meanLatency=" << std::setprecision(1)
                  << (counts.second ? latencySum[reason] / counts.second : 0.0) << "ms\n";
    }
}

void
EasyMeshController::PrintRadioTraffic(double activeS) const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    Per-Radio Traffic Summary                                 ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::left
              << std::setw(8)  << "Agent"
              << std::setw(7)  << "Band"
              << std::setw(6)  << "Ch"
              << std::setw(10) << "Clients"
              << std::setw(15) << "UL Tput(Mbps)"
              << std::setw(15) << "DL Tput(Mbps)"
              << "MeanUtil%\n";
    std::cout << std::string(70, '-') << "\n";

    const double scale = (activeS > 0) ? 8.0 / activeS / 1e6 : 0.0;
    for (auto& ag : m_agents)
    {
        for (auto& r : ag->GetRadios())
        {
            std::cout << std::left
                      << std::setw(8)  << ("A" + std::to_string(ag->GetId()))
                      << std::setw(7)  << BandStr(r->GetBand())
                      << std::setw(6)  << r->GetChannel().number
                      << std::setw(10) << r->GetClientCount()
                      << std::setw(15) << std::fixed << std::setprecision(3) << r->GetUlBytes() * scale
                      << std::setw(15) << r->GetDlBytes() * scale
                      << std::setprecision(1) << r->GetMeanChanUtil() * 100 << "\n";
        }
    }
    std::cout << std::string(70, '-') << "\n";
}
