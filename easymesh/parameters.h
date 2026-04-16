#ifndef PARAMETERS_H
#define PARAMETERS_H

// ── Topology ─────────────────────────────────────────────────
#define DURATION        60.0    // s – total simulation time
#define NUM_AGENT       2
#define NUM_STA         1
#define NUM_CONTROLLER	1

// ── Traffic – UDP CBR (OnOff) ────────────────────────────────
#define UL_DATA_RATE    "512Kbps"   // per STA uplink rate
#define UL_PKT_SIZE     512         // bytes
#define UL_START        2.0

#define DL_DATA_RATE    "1Mbps"     // per STA downlink rate
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

// ── Evaluation ───────────────────────────────────────────────
#define FLOWMON_ENABLED true
#define PRINT_PER_FLOW  true

#endif // PARAMETERS_H