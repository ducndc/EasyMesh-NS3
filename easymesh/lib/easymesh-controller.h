#ifndef EASYMESH_CONTROLLER_H
#define EASYMESH_CONTROLLER_H

#include "easymesh-agent.h"
#include "channel-planner.h"
#include <functional>
#include <map>
#include <optional>
#include <vector>
#include <string>

// ── BSS transition record (client steering / roaming / backhaul) ──
struct SteeringEvent {
    double         timeS;
    bool           isBackhaul;      // true: Agent backhaul STA, false: client STA
    bool           mlo;             // multi-link (re)setup with an AP MLD
    uint32_t       staId;           // client id, or agent id for backhaul
    int32_t        fromAgent;       // -1 = Controller (backhaul) / none
    EmBand         fromBand;
    int32_t        toAgent;
    EmBand         toBand;
    SteeringReason reason;
    double         rssiBeforeDbm;
    double         rssiAfterDbm;
    bool           success;
    double         latencyMs;       // steering command → (re)association complete
};

// ── What a client STA supports ──────────────────────────────────
struct ClientCaps {
    bool                eht = false;    // Wi-Fi 7 (EHT MCS 12/13), else Wi-Fi 6
    bool                mlo = false;    // multi-link device: sets up all its bands at once
    std::vector<EmBand> bands;          // bands it can operate on (MLD: one link per band)
};

// Band restriction for a BSS search: 2.4 GHz (LOW) vs. 5 / 6 GHz (HIGH)
enum class BandFilter { ANY, LOW, HIGH };

// ── Hooks into the simulation (how the controller observes / acts) ──
struct ControllerOps {
    std::function<double(uint32_t clientId, Ptr<EasyMeshRadio> radio)> clientRssi;
    std::function<double(Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b)>  radioRssi;
    std::function<double(double rssiDbm, uint16_t widthMhz, uint32_t maxMcs)> phyRate;
    std::function<bool(uint32_t clientId, Ptr<EasyMeshRadio> target, SteeringReason reason)> steerClient;
    std::function<void(Ptr<EasyMeshRadio> radio, const EmChannel& channel)> switchChannel;
};

// ── Controller class ─────────────────────────────────────────────
class EasyMeshController : public SimpleRefCount<EasyMeshController> {
public:
    EasyMeshController(uint32_t id, Ptr<Node> node);

    void SetOps(const ControllerOps& ops) { m_ops = ops; }
    void SetNumClients(uint32_t n)        { m_clients.resize(n); }
    void EnableSteering(bool e)           { m_steering = e; }
    void EnableLoadBalancing(bool e)      { m_loadBalancing = e; }

    // Agent registry
    void RegisterAgent(Ptr<EasyMeshAgent> agent);
    Ptr<EasyMeshAgent> GetAgent(uint32_t agentId) const;
    const std::vector<Ptr<EasyMeshAgent>>& GetAgents() const { return m_agents; }

    // ── Client capabilities ─────────────────────────────────────
    void SetClientCaps(uint32_t clientId, ClientCaps caps) { m_clients.at(clientId).caps = std::move(caps); }
    const ClientCaps& GetClientCaps(uint32_t clientId) const { return m_clients.at(clientId).caps; }
    bool IsMloClient(uint32_t clientId) const { return m_clients.at(clientId).caps.mlo; }

    // ── Topology database (fed by Agent notifications) ──────────
    // links: the radios the client has set up, serving (anchor) radio first
    void NotifyClientAssociated(uint32_t clientId, std::vector<Ptr<EasyMeshRadio>> links);
    void NotifyClientDisassociated(uint32_t clientId);
    void NotifyTransitionStarted(uint32_t clientId);
    void RecordTransition(const SteeringEvent& ev);
    bool IsTransitionAllowed(uint32_t clientId) const;
    Ptr<EasyMeshRadio> GetServingRadio(uint32_t clientId) const;
    const std::vector<Ptr<EasyMeshRadio>>& GetClientLinks(uint32_t clientId) const;
    bool IsServedBy(uint32_t clientId, Ptr<EasyMeshRadio> radio) const;

    // BSSs a client could be moved to. An MLD client is steered per AP MLD,
    // represented by its anchor radio (the Agent's radio on the client's
    // first band); a single-link client by any radio on a band it supports.
    std::vector<Ptr<EasyMeshRadio>> CandidateRadios(uint32_t clientId) const;
    Ptr<EasyMeshRadio> AnchorRadio(uint32_t clientId, uint32_t agentId) const;

    // Best fronthaul BSS for a client (capacity-, load- and backhaul-aware).
    // exclude: a radio, or for an MLD client the whole AP MLD it belongs to.
    Ptr<EasyMeshRadio> SelectBestRadio(uint32_t clientId,
                                       Ptr<EasyMeshRadio> exclude,
                                       BandFilter filter,
                                       double minRssiDbm) const;

    // ── Periodic MAP tasks (scheduled by Simulation) ────────────
    void CollectApMetrics();
    void RunSteeringEngine();
    // initial: plan before start (applied directly); otherwise re-plan with
    // measured utilization and switch radios through ops.switchChannel
    void PlanChannels(bool initial);
    void RecordInitialChannels();   // snapshot used by PrintChannelPlan
    void HandleAgentOffline(uint32_t agentId);

    // Print helpers
    void PrintTopology()                    const;
    void PrintChannelPlan()                 const;
    void PrintSteeringLog()                 const;
    void PrintRadioTraffic(double activeS)  const;

private:
    struct ClientRecord {
        ClientCaps caps;
        Ptr<EasyMeshRadio> radio;                   // serving (anchor) radio
        std::vector<Ptr<EasyMeshRadio>> links;      // every set-up link (radio first)
        bool   associated     = false;
        bool   pending        = false;
        double lastTransition = -1e9;
    };

    std::vector<Ptr<EasyMeshRadio>> OnlineRadios() const;
    double RadioScore(uint32_t clientId, Ptr<EasyMeshRadio> radio) const;
    double LinkRate(uint32_t clientId, Ptr<EasyMeshRadio> radio) const;
    bool   SameBss(uint32_t clientId, Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b) const;
    void   DetachClient(uint32_t clientId);
    double Interference(Ptr<EasyMeshRadio> a, Ptr<EasyMeshRadio> b) const;
    bool   Steer(uint32_t clientId, Ptr<EasyMeshRadio> target, SteeringReason reason);
    void   RunLoadBalancing();

    uint32_t  m_id;
    Ptr<Node> m_node;
    ControllerOps m_ops;
    bool m_steering      = true;
    bool m_loadBalancing = true;

    std::vector<Ptr<EasyMeshAgent>> m_agents;
    std::vector<ClientRecord>       m_clients;
    std::vector<SteeringEvent>      m_steeringLog;

    ChannelPlanner                       m_planner;
    std::map<const EasyMeshRadio*, uint16_t> m_initialChannel;
    double m_initialPlanCost = 0.0;
    uint32_t m_planRounds    = 0;
};

#endif // EASYMESH_CONTROLLER_H
