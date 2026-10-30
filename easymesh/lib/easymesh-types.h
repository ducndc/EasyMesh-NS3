#ifndef EASYMESH_TYPES_H
#define EASYMESH_TYPES_H

#include <cstdint>
#include <sstream>
#include <string>

// ── Radio band of a BSS / affiliated link ────────────────────────
enum class EmBand { BAND_2_4GHZ, BAND_5GHZ, BAND_6GHZ };

inline const char*
BandStr(EmBand band)
{
    switch (band)
    {
    case EmBand::BAND_2_4GHZ: return "2.4G";
    case EmBand::BAND_5GHZ:   return "5G";
    case EmBand::BAND_6GHZ:   return "6G";
    }
    return "?";
}

inline const char*
BandSettingsStr(EmBand band)
{
    switch (band)
    {
    case EmBand::BAND_2_4GHZ: return "BAND_2_4GHZ";
    case EmBand::BAND_5GHZ:   return "BAND_5GHZ";
    case EmBand::BAND_6GHZ:   return "BAND_6GHZ";
    }
    return "BAND_UNSPECIFIED";
}

// ── Operating channel (maps onto WifiPhy "ChannelSettings") ──────
struct EmChannel
{
    uint16_t number   = 0;
    uint16_t widthMhz = 20;
    EmBand   band     = EmBand::BAND_5GHZ;

    std::string ToSettings() const
    {
        std::ostringstream os;
        os << "{" << number << ", " << widthMhz << ", " << BandSettingsStr(band) << ", 0}";
        return os.str();
    }

    bool operator==(const EmChannel& o) const
    {
        return number == o.number && widthMhz == o.widthMhz && band == o.band;
    }
};

// ── Why a STA (client or backhaul STA) changed BSS ───────────────
enum class SteeringReason
{
    RSSI_LOW,       // controller: client signal below steering threshold
    BAND_STEER,     // controller: move between 2.4 GHz and 5 GHz
    LOAD_BALANCE,   // controller: serving radio overloaded by its own clients
    CHAN_UTIL,      // controller: serving channel busy because of neighbours
    ROAMING,        // STA-initiated: signal below roaming threshold
    AGENT_OFFLINE,  // controller: serving agent went offline
    CHANNEL_SWITCH, // controller: serving radio changed channel (CSA follow)
    BACKHAUL_OPT    // controller: backhaul STA re-parented by network optimization
};

inline const char*
ReasonStr(SteeringReason reason)
{
    switch (reason)
    {
    case SteeringReason::RSSI_LOW:       return "RSSI_LOW";
    case SteeringReason::BAND_STEER:     return "BAND_STEER";
    case SteeringReason::LOAD_BALANCE:   return "LOAD_BAL";
    case SteeringReason::CHAN_UTIL:      return "CHAN_UTIL";
    case SteeringReason::ROAMING:        return "ROAMING";
    case SteeringReason::AGENT_OFFLINE:  return "AGENT_OFF";
    case SteeringReason::CHANNEL_SWITCH: return "CH_SWITCH";
    case SteeringReason::BACKHAUL_OPT:   return "BH_OPT";
    }
    return "UNKNOWN";
}

#endif // EASYMESH_TYPES_H
