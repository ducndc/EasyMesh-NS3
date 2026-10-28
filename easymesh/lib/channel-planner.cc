#include "channel-planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

ChannelPlanner::ChannelPlanner(std::vector<uint16_t> channels24, std::vector<uint16_t> channels5)
    : m_channels24(std::move(channels24)), m_channels5(std::move(channels5)) {}

const std::vector<uint16_t>&
ChannelPlanner::ChannelsFor(EmBand band) const
{
    return (band == EmBand::BAND_2_4GHZ) ? m_channels24 : m_channels5;
}

ChannelPlanner::Matrix
ChannelPlanner::Weights(const std::vector<Radio>& radios,
                        const InterferenceFn& interference,
                        bool weightByUtil) const
{
    const size_t n = radios.size();
    Matrix w(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = i + 1; j < n; ++j)
        {
            if (radios[i].band != radios[j].band)
            {
                continue;
            }
            double weight = interference(radios[i].key, radios[j].key);
            if (weightByUtil)
            {
                weight *= 1.0 + radios[i].util + radios[j].util;
            }
            w[i][j] = w[j][i] = weight;
        }
    }
    return w;
}

void
ChannelPlanner::Improve(const std::vector<Radio>& radios,
                        const Matrix& w,
                        std::vector<uint16_t>& channel,
                        double stickiness,
                        uint32_t maxMoves) const
{
    const size_t n = radios.size();
    auto costOn = [&](size_t i, uint16_t ch) {
        double cost = 0.0;
        for (size_t j = 0; j < n; ++j)
        {
            if (j != i && channel[j] == ch)
            {
                cost += w[i][j];
            }
        }
        return cost;
    };

    for (uint32_t moves = 0; moves < maxMoves; ++moves)
    {
        // Best single move over all radios: largest cost reduction
        double  bestGain    = 1e-9;
        int64_t bestRadio   = -1;
        uint16_t bestChannel = 0;
        for (size_t i = 0; i < n; ++i)
        {
            const double current = costOn(i, channel[i]);
            for (uint16_t ch : ChannelsFor(radios[i].band))
            {
                if (ch == channel[i]) continue;
                const double cost = costOn(i, ch);
                if (cost >= current * (1.0 - stickiness)) continue;   // not worth a retune
                if (current - cost > bestGain)
                {
                    bestGain    = current - cost;
                    bestRadio   = static_cast<int64_t>(i);
                    bestChannel = ch;
                }
            }
        }
        if (bestRadio < 0)
        {
            return;   // local optimum
        }
        channel[bestRadio] = bestChannel;
    }
}

std::map<uint32_t, uint16_t>
ChannelPlanner::Plan(const std::vector<Radio>& radios, const InterferenceFn& interference) const
{
    const size_t n = radios.size();
    const auto w = Weights(radios, interference, false);

    // Greedy colouring, most-constrained radio first
    std::vector<size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::vector<double> degree(n, 0.0);
    for (size_t i = 0; i < n; ++i)
    {
        degree[i] = std::accumulate(w[i].begin(), w[i].end(), 0.0);
    }
    std::stable_sort(order.begin(), order.end(),
                     [&](size_t a, size_t b) { return degree[a] > degree[b]; });

    std::vector<uint16_t> channel(n, 0);
    for (size_t i : order)
    {
        uint16_t bestChannel = ChannelsFor(radios[i].band).front();
        double   bestCost    = std::numeric_limits<double>::infinity();
        for (uint16_t ch : ChannelsFor(radios[i].band))
        {
            double cost = 0.0;
            for (size_t j = 0; j < n; ++j)
            {
                if (channel[j] == ch) cost += w[i][j];
            }
            if (cost < bestCost - 1e-9)
            {
                bestCost    = cost;
                bestChannel = ch;
            }
        }
        channel[i] = bestChannel;
    }

    // Polish: greedy colouring is order-dependent, local search fixes that
    Improve(radios, w, channel, 0.0, std::numeric_limits<uint32_t>::max());

    std::map<uint32_t, uint16_t> assignment;
    for (size_t i = 0; i < n; ++i)
    {
        assignment[radios[i].key] = channel[i];
    }
    return assignment;
}

std::map<uint32_t, uint16_t>
ChannelPlanner::Replan(const std::vector<Radio>& radios,
                       const InterferenceFn& interference,
                       double stickiness,
                       uint32_t maxMoves) const
{
    const auto w = Weights(radios, interference, true);
    std::vector<uint16_t> channel;
    for (const auto& r : radios)
    {
        channel.push_back(r.currentChannel);
    }
    Improve(radios, w, channel, stickiness, maxMoves);

    std::map<uint32_t, uint16_t> assignment;
    for (size_t i = 0; i < radios.size(); ++i)
    {
        assignment[radios[i].key] = channel[i];
    }
    return assignment;
}

double
ChannelPlanner::TotalCost(const std::vector<Radio>& radios,
                          const std::map<uint32_t, uint16_t>& assignment,
                          const InterferenceFn& interference,
                          bool weightByUtil) const
{
    const auto w = Weights(radios, interference, weightByUtil);
    double total = 0.0;
    for (size_t i = 0; i < radios.size(); ++i)
    {
        for (size_t j = i + 1; j < radios.size(); ++j)
        {
            if (assignment.at(radios[i].key) == assignment.at(radios[j].key))
            {
                total += w[i][j];
            }
        }
    }
    return total;
}
