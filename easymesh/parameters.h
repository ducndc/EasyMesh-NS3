#ifndef PARAMETERS_H
#define PARAMETERS_H

// ── Topology ─────────────────────────────────────────────────
#define DURATION        30.0    // s – total simulation time
#define SCENARIO_RUNS   1
#define NUM_AGENT       15
#define NUM_STA         8
#define NUM_CONTROLLER	1

// ── Traffic – UDP CBR (OnOff) ────────────────────────────────
#define UL_DATA_RATE    "10Mbps"   // per STA uplink rate
#define UL_PKT_SIZE     1472        // bytes
#define UL_START        2.0
#define UL_OFFERED_MBPS 10.0

#define DL_DATA_RATE    "2Mbps"     // per STA downlink rate
#define DL_PKT_SIZE     1024        // bytes
#define DL_START        2.5

// ── Traffic – TCP BulkSend (throughput probe) ────────────────
#define TCP_SEND_SIZE   131072      // bytes per send call
#define TCP_START       3.0

// ── Traffic – Inter-Agent UDP (backhaul stress) ──────────────
#define BH_DATA_RATE    "2Mbps"
#define BH_PKT_SIZE     1024
#define BH_START        1.0

// ── Ports ────────────────────────────────────────────────────
#define PORT_UDP_UL     9001
#define PORT_UDP_DL     9002
#define PORT_TCP_UL     9003
#define PORT_UDP_BH     9004

// ── Wi-Fi configuration ──────────────────────────────────────
// Dedicated 5 GHz backhaul channel (tri-band style), outside the fronthaul set
#define BACKHAUL_CHANNEL_NUMBER 159
#define BACKHAUL_WIDTH_MHZ      40      // 80 MHz is ~2x slower to simulate (per-RU band tracking)
// Idle channel a STA is parked on for an instant to force a re-scan
#define PARKING_CHANNEL_NUMBER  165
#define PARKING_WIDTH_MHZ       20
#define WIFI_BEACON_INTERVAL_US 102400
#define TX_POWER_DBM            20.0
// CCA energy-detect threshold. The 802.11 default is -62 dBm, but ns-3.48
// (PhyEntity/HePhy::SwitchMaybeToCcaBusy) resets CCA to IDLE when an
// undecodable adjacent-channel PPDU arrives while another PPDU's preamble /
// PHY header is being received below the ED threshold, which then trips
// NS_ASSERT(!m_currentEvent) in PhyEntity::StartReceivePreamble. Preamble
// detection triggers at -82 dBm over the whole PPDU width, while CCA-ED looks
// at the primary 20 MHz only (-3 dB for a 40 MHz PPDU), so the threshold sits
// below -85 dBm to keep CCA busy for any PPDU in reception. This value is for
// PHYs up to 40 MHz; wider PHYs lower it by 3 dB per doubling (-91 dBm at 80,
// -97 dBm at 320 MHz), see EasyMeshSimulation::CcaEdThresholdDbm.
#define CCA_ED_THRESHOLD_DBM    -88.0
// Log-distance path loss (exponent 3) at 1 m reference per band
#define PATHLOSS_EXPONENT       3.0
#define REF_LOSS_2_4GHZ_DB      40.05   // 2.437 GHz
#define REF_LOSS_5GHZ_DB        46.68   // 5.15 GHz
#define REF_LOSS_6GHZ_DB        48.17   // 6.105 GHz

// ── Wi-Fi 7 (802.11be) ───────────────────────────────────────
// 6 GHz transmit power is PSD-limited (Low Power Indoor): 5 dBm/MHz,
// capped at 30 dBm EIRP, so wider channels transmit more total power.
// Applied to every 6 GHz AP / bSTA / MLD link (single-radio clients keep
// TX_POWER_DBM on all bands).
#define TX_PSD_6GHZ_DBM_PER_MHZ 5.0
#define TX_POWER_6GHZ_MAX_DBM   30.0
// MLO backhaul: link 0 on the 5 GHz backhaul channel above, link 1 on a
// 6 GHz channel inside 20 MHz channels 1..61 (below the 6 GHz fronthaul set).
// The channel number follows the width: 20→1, 40→3, 80→7, 160→15, 320→31.
// 320 MHz works (--bhWidth6=320) but is ~4x slower to simulate than 80 MHz:
// SpectrumWifiPhy tracks the received power of every RU of the channel.
#define BH6_WIDTH_MHZ           80
#define PARKING6_CHANNEL_NUMBER 233     // idle 6 GHz channel for re-scans of 6 GHz links
#define EHT_MAX_MCS             13      // 4096-QAM; HE (Wi-Fi 6) stops at MCS 11
#define HE_MAX_MCS              11
// Clients [0, NUM_MLD_STA) are Wi-Fi 7 (5 + 6 GHz MLD), the rest Wi-Fi 6 (2.4 / 5 GHz)
#define NUM_MLD_STA             4

// ── Channel planning ─────────────────────────────────────────
#define FH24_CHANNELS           {1, 6, 11}
#define FH24_WIDTH_MHZ          20
#define FH5_CHANNELS            {38, 46, 54, 62, 102, 110, 118, 126, 134, 142}
#define FH5_WIDTH_MHZ           40      // 80 MHz is ~2x slower to simulate (per-RU band tracking)
#define FH6_CHANNELS            {71, 87, 103, 119, 135, 151, 167, 183, 199, 215}
#define FH6_WIDTH_MHZ           80
#define INTERFERENCE_FLOOR_DBM  -85.0   // below this, two APs are not neighbours
#define CHANNEL_PLAN_INTERVAL   10.0    // s – dynamic re-planning period
#define CHANNEL_SWITCH_MIN_GAIN 0.30    // a radio moves only if its cost drops > 30 %
#define CHANNEL_PLAN_MAX_MOVES  3       // radios retuned per dynamic round (each costs a re-association)

// ── Mobility (STA RandomWaypoint) ────────────────────────────
#define STA_MIN_SPEED   0.5     // m/s
#define STA_MAX_SPEED   2.0     // m/s
#define STA_MAX_PAUSE   2.0     // s

// ── Steering / roaming / load balancing ──────────────────────
#define METRICS_INTERVAL        1.0     // s – AP Metrics (channel utilization) reports
#define STEERING_INTERVAL       2.0     // s – controller steering engine tick
#define ROAM_CHECK_INTERVAL     0.5     // s – STA-side roaming check
#define STEER_TIMEOUT           1.5     // s – give up on a BSS transition after this
#define STEER_COOLDOWN          4.0     // s – min time between transitions of one STA
#define MIN_ASSOC_RSSI_DBM      -82.0   // never steer to a BSS weaker than this

#define ROAM_RSSI_THRESHOLD     -78.0   // STA starts looking for a better BSS
#define ROAM_HYSTERESIS_DB      6.0
#define STEER_RSSI_THRESHOLD    -72.0   // controller steers a client below this
#define STEER_RSSI_GAIN_DB      6.0     // ... if the target is at least this much better
#define BAND_STEER_TO_5G_RSSI   -65.0   // 2.4 GHz client with 5 GHz this strong → 5 GHz
#define BAND_STEER_TO_24_RSSI   -77.0   // 5 GHz client weaker than this → 2.4 GHz
#define LB_UTIL_THRESHOLD       0.60    // radio considered overloaded above this
#define LB_UTIL_MARGIN          0.20    // target radio must be this much less utilized
#define LB_MIN_RSSI_DBM         -72.0   // load balancing never degrades a client below this

// ── Agent failure / recovery event ───────────────────────────
#define OFFLINE_AGENT_ID 1
#define OFFLINE_TIME     12.0
#define RECOVER_TIME     -1.0   // < 0: offline agent(s) never come back

// ── Topology optimization (Agent placement) ──────────────────
#define TOPOLOGY_AREA_MIN_X 5.0
#define TOPOLOGY_AREA_MAX_X 95.0
#define TOPOLOGY_AREA_MIN_Y 5.0
#define TOPOLOGY_AREA_MAX_Y 95.0
#define TOPOLOGY_TARGET_RADIUS 70.0
#define TOPOLOGY_MAX_RADIUS 95.0
#define TOPOLOGY_MIN_AGENT_SEPARATION 25.0

// ── Network (backhaul) optimization ──────────────────────────
#define NETOPT_INTERVAL  5.0    // s – re-evaluate backhaul MST
#define NETOPT_MIN_GAIN  0.10   // apply a new tree only if it is > 10 % better

// ── Evaluation ───────────────────────────────────────────────
#define FLOWMON_ENABLED true
#define PRINT_PER_FLOW  true

#endif // PARAMETERS_H
