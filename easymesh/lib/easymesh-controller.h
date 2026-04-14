#ifndef EASYMESH_CONTROLLER_H
#define EASYMESH_CONTROLLER_H

#include "easymesh-agent.h"
#include "traffic-stats.h"
#include <vector>
#include <string>

// ── Steering event record ────────────────────────────────────────
enum class SteeringReason { RSSI_LOW, LOAD_BALANCE, CHAN_UTIL };

struct SteeringEvent {
    double       timeS;
    uint32_t     clientId;
    uint32_t     fromAgent;
    uint32_t     toAgent;
    SteeringReason reason;
    double       rssiBeforeDbm;
    double       rssiAfterDbm;
    bool         success;

    std::string ReasonStr() const {
        switch (reason) {
            case SteeringReason::RSSI_LOW:      return "RSSI_LOW";
            case SteeringReason::LOAD_BALANCE:  return "LOAD_BAL";
            case SteeringReason::CHAN_UTIL:      return "CHAN_UTIL";
            default:                            return "UNKNOWN";
        }
    }
};

// ── Controller class ─────────────────────────────────────────────
class EasyMeshController : public SimpleRefCount<EasyMeshController> {
public:
    EasyMeshController(uint32_t id, Ptr<Node> node);

    // Agent registry
    void RegisterAgent(Ptr<EasyMeshAgent> agent);
    Ptr<EasyMeshAgent> GetAgent(uint32_t agentId);
    const std::vector<Ptr<EasyMeshAgent>>& GetAgents() const { return m_agents; }

    // Link registry
    void RegisterLink(Ptr<EasyMeshLink> link);
    const std::vector<Ptr<EasyMeshLink>>& GetLinks() const { return m_links; }

    // Periodic MAP tasks (scheduled by Simulation)
    void CollectApMetrics();
    void RunSteeringEngine();
    void OptimizeBackhaulTopology();

    // Called after simulation ends to inject FlowMonitor results
    void ApplyTrafficStats(const TrafficStats& ts);

    // Print helpers
    void PrintTopology()        const;
    void PrintSteeringLog()     const;
    void PrintAgentTraffic()    const;

private:
    uint32_t  m_id;
    Ptr<Node> m_node;

    std::vector<Ptr<EasyMeshAgent>> m_agents;
    std::vector<Ptr<EasyMeshLink>>  m_links;
    std::vector<SteeringEvent>      m_steeringLog;

    // Thresholds
    static constexpr double RSSI_STEER_THRESHOLD  = -75.0;
    static constexpr double LOAD_STEER_THRESHOLD  = 0.75;
    static constexpr double UTIL_STEER_THRESHOLD  = 0.80;

    // Internal helpers
    double  EstimateRssi(Ptr<EasyMeshAgent> agent) const;
    uint32_t SelectTargetAgent(uint32_t excludeId) const;
};

#endif // EASYMESH_CONTROLLER_H