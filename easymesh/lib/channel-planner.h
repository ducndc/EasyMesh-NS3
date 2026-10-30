#ifndef CHANNEL_PLANNER_H
#define CHANNEL_PLANNER_H

#include "easymesh-types.h"

#include <functional>
#include <map>
#include <vector>

// ────────────────────────────────────────────────────────────────
// ChannelPlanner – assigns fronthaul channels per band on the inter-AP
// interference graph.
//
// Edge weight w(i,j) = how far the signal between radios i and j is
// above the interference floor (dB, >= 0); radios in different bands
// never interfere. With utilization weighting (dynamic planning) the
// edge becomes w(i,j) * (1 + util_i + util_j): every overlap still
// counts, overlaps between busy radios count more.
//
//  • Plan()   – initial plan: greedy colouring (most constrained radio
//               first) polished by local search.
//  • Replan() – periodic plan: local search starting from the current
//               channels. Each move is the single best per-radio channel
//               change given everybody else's channel, so every move
//               strictly lowers the total (weighted) cost.
// ────────────────────────────────────────────────────────────────
class ChannelPlanner
{
  public:
    struct Radio
    {
        uint32_t key;             // caller-defined identifier
        EmBand   band;
        uint16_t currentChannel;  // 0 if not yet assigned
        double   util;            // measured channel utilization [0..1]
    };

    // dB above the interference floor between two radios (>= 0)
    using InterferenceFn = std::function<double(uint32_t keyA, uint32_t keyB)>;

    ChannelPlanner(std::vector<uint16_t> channels24,
                   std::vector<uint16_t> channels5,
                   std::vector<uint16_t> channels6);

    std::map<uint32_t, uint16_t> Plan(const std::vector<Radio>& radios,
                                      const InterferenceFn& interference) const;

    // stickiness in [0,1): a radio only moves if that cuts its own cost by
    // more than this fraction; at most maxMoves radios are retuned.
    std::map<uint32_t, uint16_t> Replan(const std::vector<Radio>& radios,
                                        const InterferenceFn& interference,
                                        double stickiness,
                                        uint32_t maxMoves) const;

    // Total co-channel interference of an assignment (lower is better);
    // weightByUtil applies the same weighting Replan() optimizes.
    double TotalCost(const std::vector<Radio>& radios,
                     const std::map<uint32_t, uint16_t>& assignment,
                     const InterferenceFn& interference,
                     bool weightByUtil = false) const;

  private:
    using Matrix = std::vector<std::vector<double>>;

    const std::vector<uint16_t>& ChannelsFor(EmBand band) const;
    Matrix Weights(const std::vector<Radio>& radios,
                   const InterferenceFn& interference,
                   bool weightByUtil) const;
    void Improve(const std::vector<Radio>& radios,
                 const Matrix& w,
                 std::vector<uint16_t>& channel,
                 double stickiness,
                 uint32_t maxMoves) const;

    std::vector<uint16_t> m_channels24;
    std::vector<uint16_t> m_channels5;
    std::vector<uint16_t> m_channels6;
};

#endif // CHANNEL_PLANNER_H
