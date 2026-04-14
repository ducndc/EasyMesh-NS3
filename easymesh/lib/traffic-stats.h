#ifndef TRAFFIC_STATS_H
#define TRAFFIC_STATS_H

#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include <vector>
#include <map>
#include <string>
#include <iomanip>
#include <sstream>

using namespace ns3;

// ────────────────────────────────────────────────────────────────
// FlowRecord  – statistics for a single monitored flow
// ────────────────────────────────────────────────────────────────
struct FlowRecord {
    uint32_t    flowId;
    std::string srcAddr;
    std::string dstAddr;
    uint16_t    srcPort;
    uint16_t    dstPort;
    uint8_t     proto;          // 6=TCP, 17=UDP

    uint64_t    txPackets;
    uint64_t    rxPackets;
    uint64_t    txBytes;
    uint64_t    rxBytes;
    uint64_t    lostPackets;

    double      throughputMbps;
    double      meanDelayMs;
    double      meanJitterMs;
    double      pdr;            // Packet Delivery Ratio [0..1]
    double      durationSec;

    std::string ProtoStr()   const { return (proto == 6) ? "TCP" : "UDP"; }
    std::string QualityStr() const {
        if (pdr >= 0.98 && meanDelayMs < 20.0) return "EXCELLENT";
        if (pdr >= 0.95 && meanDelayMs < 50.0) return "GOOD";
        if (pdr >= 0.90)                        return "FAIR";
        return "POOR";
    }
};

// ────────────────────────────────────────────────────────────────
// AgentTrafficSummary – aggregate per agent (all flows through it)
// ────────────────────────────────────────────────────────────────
struct AgentTrafficSummary {
    uint32_t agentId;
    uint32_t clientCount;
    double   aggTxThroughputMbps;
    double   aggRxThroughputMbps;
    double   meanDelayMs;
    double   meanPdr;
    uint32_t flowCount;
};

// ────────────────────────────────────────────────────────────────
// TrafficStats – collects, computes and prints all stats
// ────────────────────────────────────────────────────────────────
class TrafficStats {
public:
    TrafficStats();

    // Call after Simulator::Run() with the installed FlowMonitor
    void Collect(Ptr<FlowMonitor>           monitor,
                 Ptr<Ipv4FlowClassifier>    classifier);

    // Add metadata so we can label flows
    void RegisterFlow(uint32_t flowId,
                      const std::string& label,
                      uint32_t agentId);

    // Accessors
    const std::vector<FlowRecord>& GetFlows() const { return m_flows; }

    // Printing
    void PrintPerFlowTable()    const;
    void PrintAgentSummary()    const;
    void PrintNetworkSummary()  const;
    void PrintAll()             const;

private:
    std::vector<FlowRecord>                     m_flows;
    std::map<uint32_t, std::string>             m_flowLabels;   // flowId → label
    std::map<uint32_t, uint32_t>                m_flowAgent;    // flowId → agentId

    // Aggregate helpers
    double TotalThroughputMbps() const;
    double MeanPdr()             const;
    double MeanDelayMs()         const;

    static std::string SepLine(int w = 78) { return std::string(w, '-'); }
};

// ────────────────────────────────────────────────────────────────
// Implementation (header-only for simplicity in NS-3 scratch)
// ────────────────────────────────────────────────────────────────

inline TrafficStats::TrafficStats() {}

inline void TrafficStats::RegisterFlow(uint32_t fid,
                                        const std::string& label,
                                        uint32_t agentId) 
{
    m_flowLabels[fid] = label;
    m_flowAgent[fid]  = agentId;
}

inline void TrafficStats::Collect(Ptr<FlowMonitor>        monitor,
                                   Ptr<Ipv4FlowClassifier> classifier) 
{
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();

    for (auto& kv : stats) {
        auto  fid = kv.first;
        auto& fs  = kv.second;

        FlowRecord rec;
        rec.flowId      = fid;
        rec.txPackets   = fs.txPackets;
        rec.rxPackets   = fs.rxPackets;
        rec.txBytes     = fs.txBytes;
        rec.rxBytes     = fs.rxBytes;
        rec.lostPackets = fs.lostPackets;
        rec.pdr = (fs.txPackets > 0)
                  ? (double)fs.rxPackets / fs.txPackets
                  : 0.0;

        double dur = fs.timeLastRxPacket.GetSeconds()
                   - fs.timeFirstTxPacket.GetSeconds();
        rec.durationSec = (dur > 0) ? dur : 1e-9;

        rec.throughputMbps = (fs.rxBytes * 8.0) / rec.durationSec / 1e6;

        rec.meanDelayMs = (fs.rxPackets > 0)
                          ? fs.delaySum.GetSeconds() / fs.rxPackets * 1000.0
                          : 0.0;

        rec.meanJitterMs = (fs.rxPackets > 1)
                           ? fs.jitterSum.GetSeconds() / (fs.rxPackets - 1) * 1000.0
                           : 0.0;

        // Classify using the 5-tuple
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(fid);
        std::ostringstream src, dst;
        src << t.sourceAddress;
        dst << t.destinationAddress;
        rec.srcAddr = src.str();
        rec.dstAddr = dst.str();
        rec.srcPort = t.sourcePort;
        rec.dstPort = t.destinationPort;
        rec.proto   = t.protocol;

        m_flows.push_back(rec);
    }
}

// ── Aggregates ──────────────────────────────────────────────────
inline double TrafficStats::TotalThroughputMbps() const {
    double sum = 0;
    for (auto& f : m_flows) sum += f.throughputMbps;
    return sum;
}
inline double TrafficStats::MeanPdr() const {
    if (m_flows.empty()) return 0;
    double sum = 0;
    for (auto& f : m_flows) sum += f.pdr;
    return sum / m_flows.size();
}
inline double TrafficStats::MeanDelayMs() const {
    if (m_flows.empty()) return 0;
    double sum = 0;
    for (auto& f : m_flows) sum += f.meanDelayMs;
    return sum / m_flows.size();
}

// ── Printers ────────────────────────────────────────────────────
inline void TrafficStats::PrintPerFlowTable() const {
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                     Per-Flow Traffic Statistics                             ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::left
              << std::setw(6)  << "FID"
              << std::setw(5)  << "PROTO"
              << std::setw(16) << "Src IP"
              << std::setw(16) << "Dst IP"
              << std::setw(10) << "TxPkts"
              << std::setw(10) << "RxPkts"
              << std::setw(12) << "Tput(Mbps)"
              << std::setw(11) << "Delay(ms)"
              << std::setw(11) << "Jitter(ms)"
              << std::setw(8)  << "PDR%"
              << "Quality\n";
    std::cout << SepLine() << "\n";

    for (auto& f : m_flows) {
        std::cout << std::left
                  << std::setw(6)  << f.flowId
                  << std::setw(5)  << f.ProtoStr()
                  << std::setw(16) << f.srcAddr.substr(0, 15)
                  << std::setw(16) << f.dstAddr.substr(0, 15)
                  << std::setw(10) << f.txPackets
                  << std::setw(10) << f.rxPackets
                  << std::setw(12) << std::fixed << std::setprecision(3) << f.throughputMbps
                  << std::setw(11) << std::setprecision(2) << f.meanDelayMs
                  << std::setw(11) << std::setprecision(2) << f.meanJitterMs
                  << std::setw(8)  << std::setprecision(1) << f.pdr * 100.0
                  << f.QualityStr() << "\n";
    }
    std::cout << SepLine() << "\n";
}

inline void TrafficStats::PrintNetworkSummary() const {
    uint64_t totTx = 0, totRx = 0, totLost = 0;
    for (auto& f : m_flows) {
        totTx   += f.txPackets;
        totRx   += f.rxPackets;
        totLost += f.lostPackets;
    }

    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                      Network-Wide Summary                                   ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << "  Monitored flows         : " << m_flows.size()        << "\n";
    std::cout << "  Total Tx packets        : " << totTx                 << "\n";
    std::cout << "  Total Rx packets        : " << totRx                 << "\n";
    std::cout << "  Total lost packets      : " << totLost               << "\n";
    std::cout << "  Aggregate throughput    : " << std::fixed << std::setprecision(3)
                                                << TotalThroughputMbps() << " Mbps\n";
    std::cout << "  Mean end-to-end delay   : " << std::setprecision(2)
                                                << MeanDelayMs()         << " ms\n";
    std::cout << "  Mean PDR                : " << std::setprecision(1)
                                                << MeanPdr() * 100.0     << " %\n";
    std::cout << SepLine() << "\n";
}

inline void TrafficStats::PrintAll() const {
    PrintPerFlowTable();
    PrintNetworkSummary();
}

#endif // TRAFFIC_STATS_H
