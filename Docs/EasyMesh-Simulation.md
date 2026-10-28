# EasyMesh-NS3 — Simulation Design and Algorithms

This document explains what the EasyMesh-NS3 simulation models, how it is built on ns-3, and how each feature works: backhaul topology, network optimization, client steering, band steering, load balancing, roaming, channel planning and Agent failure handling. All numbers quoted below are the defaults in `easymesh/parameters.h`.

---

## 1. What is simulated

A Wi-Fi EasyMesh network (Wi-Fi Alliance Multi-AP) inside a 90 m × 90 m area:

| Element | Count (default) | Radios |
|---|---|---|
| **Controller** | 1, fixed at (0, 0) | 1 backhaul AP (bBSS), 802.11be |
| **Agent** | 15 (`--agents`) | backhaul AP (bBSS) + backhaul STA (bSTA), 802.11be · 2.4 GHz fronthaul AP + 5 GHz fronthaul AP, 802.11ax |
| **Client (STA)** | 8 (`--clients`) | one 802.11ax radio that can tune to either band |

The Controller is the root of the mesh and the IP sink/source of all traffic. Agents relay traffic over a multi-hop Wi-Fi backhaul tree and serve clients on a dual-band fronthaul (same SSID, `EasyMesh-Fronthaul`, on both bands).

```
                    ┌──────────────┐
                    │  Controller  │  bBSS ch159
                    └──────┬───────┘
              backhaul (bSTA → parent bBSS, ch159/40 MHz, 802.11be)
          ┌────────────────┼────────────────┐
     ┌────┴────┐      ┌────┴────┐      ┌────┴────┐
     │ Agent 0 │      │ Agent 4 │      │ Agent 8 │ ...
     └────┬────┘      └────┬────┘      └────┬────┘
          │ bBSS            │                 │
     ┌────┴────┐            │                 │
     │ Agent 1 │   fronthaul: 2.4 GHz (ch 1/6/11, 20 MHz) + 5 GHz (40 MHz)
     └─────────┘            │
                       (( STA ))  ← mobile client, RandomWaypoint 0.5–2 m/s
```

### What is real and what is modelled

| Aspect | How it is simulated |
|---|---|
| 802.11 MAC/PHY, association, scanning, contention, collisions | **Real** ns-3 Wi-Fi (`SpectrumWifiPhy`, `ApWifiMac`, `StaWifiMac`) |
| Channels and interference | **Real** per-band `MultiModelSpectrumChannel`; PPDUs on different channels do not decode each other |
| Client/backhaul BSS transitions | **Real** disassociation + (re)association to a chosen BSSID |
| Agent failure | **Real**: all four radios of the Agent are switched off (`WifiPhy::SetOffMode`) |
| Channel utilization, UL/DL bytes | **Measured** from PHY state and MAC traces |
| EasyMesh control plane (IEEE 1905 CMDUs, BTM frames, CSA) | **Abstracted**: the Controller reads state and issues decisions through function calls, not over the air |
| RSSI used for decisions | **Estimated** with the same log-distance model the channels use, so estimates match what PHYs receive |
| Routing | Static IPv4 routes rebuilt after every topology/association change |

---

## 2. Code structure

```
easymesh/
├── easymesh.cc / .h          EasyMeshSimulation: builds the network, executes decisions,
│                              BSS transitions, routing, roaming, backhaul optimization, main()
├── parameters.h              every tunable constant
└── lib/
    ├── easymesh-types.h      EmBand, EmChannel, SteeringReason
    ├── easymesh-controller.* Controller: topology DB, steering engine, load balancing,
    │                          channel planning decisions, result printing
    ├── easymesh-agent.*      Agent: online state, backhaul parent/capacity, its radios
    ├── easymesh-radio.*      one fronthaul BSS: channel, clients, measured utilization, UL/DL bytes
    ├── easymesh-link.*       backhaul link record (RSSI, MCS, rate)
    ├── easymesh-assoc-manager.*  association manager that pins a STA to one BSSID
    ├── channel-planner.*     interference-graph channel assignment
    └── traffic-stats.h       FlowMonitor collection and tables
```

**Division of responsibility.** The Controller *decides*; the simulation *acts*. The Controller never touches ns-3 objects directly. It calls hooks (`ControllerOps`) that the simulation provides:

| Hook | Purpose |
|---|---|
| `clientRssi(client, radio)` | estimated RSSI between a client and a fronthaul BSS |
| `radioRssi(a, b)` | estimated RSSI between two APs (interference graph) |
| `phyRate(rssi, width)` | expected PHY rate |
| `steerClient(client, radio, reason)` | start a BSS transition |
| `switchChannel(radio, channel)` | retune an AP radio |

The simulation feeds events back to the Controller: association (`NotifyClientAssociated`), disassociation, transition start and transition result (`RecordTransition`).

### Simulation phases

`main()` runs, per scenario: `BuildTopology` → `InstallProtocolStack` → `InstallApplications` → `ScheduleEvents` → `Run` → `PrintResults`.

### Periodic tasks

| Period | Task | Owner |
|---|---|---|
| 1 s | AP metrics: sample channel utilization of every radio | Controller |
| 2 s (from 4 s) | Steering engine: RSSI steering, band steering, load balancing | Controller |
| 0.5 s (from 2 s) | Client-initiated roaming check | Simulation (STA behaviour) |
| 5 s | Network (backhaul) optimization | Simulation |
| 10 s | Dynamic channel re-planning | Controller |
| `--offlineTime` | Agent failure | Simulation + Controller |
| `--recoverTime` | Agent recovery | Simulation |

---

## 3. Radio model

### Path loss and RSSI

Both bands use a log-distance model with exponent 3 and a per-band reference loss at 1 m:

```
RSSI(d) = P_tx − ( L_ref(band) + 10 · 3 · log10(max(d, 1 m)) )
P_tx = 20 dBm,  L_ref(2.4 GHz) = 40.05 dB,  L_ref(5 GHz) = 46.68 dB
```

2.4 GHz therefore reaches about 6.6 dB further than 5 GHz. The Controller's estimates use exactly this formula, so they agree with the power the ns-3 PHYs receive.

### RSSI → MCS → PHY rate

Used by every decision that needs an expected throughput (MST throughput metric, BSS selection, backhaul capacity). The RSSI is first normalised to a 20 MHz channel, because a wider channel spreads the same power over more noise:

```
RSSI_eff = RSSI − 10 · log10(width / 20)
MCS      = highest m with RSSI_eff ≥ threshold[m]
           thresholds (dBm): −82 −79 −77 −74 −70 −66 −65 −64 −59 −57 −54 −52   (MCS 0…11)
rate     = rate20[MCS] · width / 20         (1 spatial stream, 0.8 µs GI)
           rate20 (Mbps): 8.6 17.2 25.8 34.4 51.6 68.8 77.4 86.0 103.2 114.7 129.0 143.4
rate     = 0 if RSSI_eff < −82 dBm (no link)
```

### Channels

| Use | Band | Channels | Width |
|---|---|---|---|
| Fronthaul 2.4 GHz | 2.4 GHz | 1, 6, 11 | 20 MHz |
| Fronthaul 5 GHz | 5 GHz | 38, 46, 54, 62, 102, 110, 118, 126, 134, 142 | 40 MHz |
| Backhaul (all nodes) | 5 GHz | 159 | 40 MHz |
| Parking (transitions only) | 5 GHz | 165 | 20 MHz |

The backhaul has its own channel outside the fronthaul set (tri-band style), so backhaul and fronthaul do not contend.

---

## 4. BSS transition mechanism (used by every feature that moves a STA)

Steering, roaming, load balancing, channel-switch follow, Agent-failure recovery and backhaul re-parenting all reuse one mechanism, `StartTransition(sta, targetBssid, targetChannel)`. It models an 802.11v BSS Transition Management request (or an EasyMesh Backhaul Steering Request) with real MAC behaviour:

```
1. Pin the STA to the target:   EasyMeshAssocManager.SetTargetBssid(target)
      → scan results from any other BSSID are discarded
2. Park:   retune the STA PHY to the idle parking channel (165)
      → ns-3 disassociates the STA and it starts scanning
3. Hop:    once the PHY is on the parking channel (polled every 1 ms,
           because a channel switch is postponed while the PHY is busy),
           retune to the target channel
4. Scan:   active probing on the target channel finds only the target BSSID
5. Associate: (re)association with the target
6. Assoc trace fires → success: transition latency = now − start;
           routes rebuilt; Controller topology DB updated
   Timeout (1.5 s) without success → failure recorded;
           a client is unpinned and may join any BSS on its channel;
           a backhaul STA stays pinned (it must never join a descendant)
```

The parking step guarantees that the STA really leaves its current BSS, even when the target is on the same channel. A new decision for a STA that is still in transition supersedes the old one (the old one is logged as failed). After a transition, the STA is not moved again for `STEER_COOLDOWN` = 4 s, to avoid ping-pong.

`EasyMeshAssocManager` derives from ns-3's `WifiDefaultAssocManager` and only overrides `CanBeInserted` / `CanBeReturned`. With no target it behaves exactly like the default manager.

---

## 5. Feature algorithms

### 5.1 Backhaul topology — Prim's maximum spanning tree

Every Agent's backhaul STA must choose a parent: the Controller or another Agent. The tree is chosen by Prim's algorithm with **maximised** edge weights over a graph of all online Agents plus a virtual root node for the Controller.

```
weight(a, b) = RSSI(a, b)                    if --mstMetric=rssi
             = rate(RSSI(a, b), 40 MHz)      if --mstMetric=throughput  (default)

inTree = {Controller}
best[v] = weight(Controller, v), parent[v] = Controller   for every online Agent v
repeat |online Agents| times:
    u = online Agent not in tree with the largest best[u]
    add u to tree; its parent is parent[u]
    for every online Agent v not in tree:
        if weight(u, v) > best[v]: best[v] = weight(u, v); parent[v] = u
```

Complexity is O(N²). Each step attaches the outside node reachable by the strongest single edge, which gives the tree with the largest total weight.

**Backhaul path capacity.** All hops share one backhaul channel, so their airtime adds up. An Agent's end-to-end backhaul capacity is

```
C(agent) = 1 / Σ over hops on the path to the Controller ( 1 / rate(hop) )
```

`C` is 0 if the path is broken (an ancestor is offline). The Controller uses it to avoid sending clients to Agents with poor backhaul.

### 5.2 Network (backhaul) optimization

Every 5 s the simulation recomputes the MST and compares it with the tree the backhaul STAs are actually associated with:

```
current  = total weight of the associated tree   (−∞ if any online Agent's parent is offline)
proposed = total weight of the new MST
apply if   current is broken
       or  proposed − current > 10 % · |current|        (NETOPT_MIN_GAIN, hysteresis)
apply  = for each Agent whose parent changes:
             BSS transition of its bSTA to the new parent's bBSS
```

The hysteresis stops the tree from oscillating between near-equal alternatives. Routes follow the *associated* tree, so they always match the real radio links, and a hop limit guards route building against transient cycles while several Agents re-parent at once.

### 5.3 Client BSS selection score

Whenever the Controller chooses a fronthaul BSS for a client (initial association, RSSI steering, Agent failure), it maximises the client's expected throughput:

```
score(client, radio) = min( rate(RSSI, width), C(agent) ) · (1 − 0.8 · util(radio))
candidates: radios of online Agents with RSSI ≥ minimum (default −82 dBm)
```

* `rate(RSSI, width)` — what the fronthaul link can carry
* `C(agent)` — what the Agent's backhaul path can carry
* `util(radio)` — measured channel busy fraction (shared airtime)

### 5.4 Client steering (RSSI)

Every 2 s, for each associated client that is not in transition or cooldown:

```
if RSSI(serving) < −72 dBm:
    target = best-scoring radio (any Agent, any band) except the serving one
    if RSSI(target) ≥ RSSI(serving) + 6 dB:  steer (reason RSSI_LOW)
```

This models a Controller moving a "sticky" client that has not roamed on its own.

### 5.5 Band steering

Evaluated in the same tick, after RSSI steering:

```
client on 2.4 GHz:
    target = 5 GHz radio of the same Agent; if its RSSI < −65 dBm,
             the best-scoring 5 GHz radio with RSSI ≥ −65 dBm
    if target exists: steer to 5 GHz (reason BAND_STEER)

client on 5 GHz with RSSI < −77 dBm:
    target = best-scoring 2.4 GHz radio
    if RSSI(target) ≥ RSSI(serving) + 6 dB: steer to 2.4 GHz (reason BAND_STEER)
```

The 12 dB gap between −65 and −77 dBm is the hysteresis that prevents a client from bouncing between bands.

### 5.6 Load balancing

**Measured channel utilization.** Each fronthaul radio subscribes to its PHY's `State` trace and accumulates the time spent in TX, RX, CCA-busy and channel switching. Every second:

```
util = busy time in window / window length
```

This includes the radio's own traffic and energy from co-channel neighbours, like the channel utilization field of the EasyMesh AP Metrics report.

**Algorithm.** After steering, for each online radio `r` with `util(r) > 60 %` and at least one client:

```
reason = LOAD_BALANCE if r has ≥ 2 clients   (its own clients load it)
         CHAN_UTIL    if r has 1 client      (neighbours load the channel)
choose (client c on r, target radio t ≠ r) maximising score(c, t), subject to
    util(t) < util(r) − 20 %          (target clearly less loaded)
    RSSI(c, t) ≥ −72 dBm              (never degrade the client's link badly)
    c not in transition or cooldown
steer c to t
```

At most one client per overloaded radio is moved per tick; the cooldown and the next metrics window decide whether more need to move.

### 5.7 Roaming (client-initiated)

Clients move with `RandomWaypoint` (0.5–2 m/s, pause up to 2 s). Every 0.5 s each client acts on its own, using signal strength only (it knows nothing about load or backhaul):

```
if RSSI(serving) < −78 dBm:
    best = radio with the strongest RSSI among online Agents (any band)
    if RSSI(best) ≥ RSSI(serving) + 6 dB:  transition (reason ROAMING)
```

The roaming threshold (−78 dBm) is below the Controller's steering threshold (−72 dBm), as for a real client that roams late. With steering enabled the Controller usually acts first; run with `--steering=0` to see pure client roaming.

### 5.8 Channel planning

**Interference graph.** Every pair of fronthaul radios in the same band is an edge, weighted by how far their mutual signal rises above the interference floor:

```
I(a, b) = max(0, RSSI(a, b) − (−85 dBm))        (dB; 0 means "not neighbours")
```

**Initial plan (before start).**

```
1. Greedy colouring: sort radios by total interference Σ_b I(a, b), most constrained first;
   give each radio the channel with the least interference from radios already placed.
2. Local search: repeatedly make the single channel change that lowers total
   interference the most, until no change helps.
```

**Dynamic re-planning (every 10 s, `--channelPlan=dynamic`).** Uses measured utilization and starts from the current channels, so it changes as little as possible:

```
w(a, b) = I(a, b) · (1 + util(a) + util(b))      every overlap counts, busy pairs count more
cost(a, ch) = Σ over radios b on ch of w(a, b)

repeat at most 3 times (CHANNEL_PLAN_MAX_MOVES):
    among all radios a and channels ch, pick the move with the largest
    cost(a, current) − cost(a, ch), allowed only if
    cost(a, ch) < (1 − 30 %) · cost(a, current)        (CHANNEL_SWITCH_MIN_GAIN)
    stop if no move is allowed
```

Each move is the best single move given everyone else's channel, so every move strictly lowers the total weighted cost. A round can therefore never make the network worse by this measure. The move limit and the 30 % threshold keep retuning rare, because each retune forces the radio's clients to re-associate.

**Applying a channel change (CSA follow).** ns-3 has no Channel Switch Announcement. The Controller retunes the AP radio and immediately runs a BSS transition for each of its clients to the same BSSID on the new channel (reason `CH_SWITCH`).

**Modes (`--channelPlan`).**

| Mode | Behaviour |
|---|---|
| `none` | every radio on the first channel of its band (worst-case baseline) |
| `static` | initial plan only |
| `dynamic` | initial plan + periodic re-planning (default) |

### 5.9 Agent failure and recovery

At `--offlineTime` (default 12 s, Agent 1):

```
1. Mark the Agent(s) offline and switch off all their radios (bBSS, bSTA, 2.4 GHz, 5 GHz).
   Children lose their uplink; clients lose their AP — exactly as with a power cut.
2. Network optimization with force = true: recompute the MST over the online Agents,
   re-parent every Agent whose parent changed (reason AGENT_OFF).
3. For every client of a failed Agent: steer to the best-scoring remaining radio
   (reason AGENT_OFF; bypasses cooldown).
4. Rebuild routes.
```

All failed Agents are handled in one batch, so the new tree is computed once against the final set of online Agents.

At `--recoverTime` (disabled by default) the radios are switched back on, the tree is recomputed and the recovered Agent rejoins. Steering and load balancing then move clients onto it over the following ticks.

---

## 6. IP addressing and routing

| Network | Addresses |
|---|---|
| Backhaul `10.1.0.0/16` | Controller bBSS `10.1.0.1`; Agent *a* bBSS `10.1.(a+1).1`, bSTA `10.1.(a+1).2` |
| Fronthaul `10.2.0.0/16` | Agent *a* 2.4 GHz `10.2.1.(a+1)`, 5 GHz `10.2.2.(a+1)`; client *c* `10.2.(10 + c/250).(c%250 + 1)` |

Routing is fully static and rebuilt (`RebuildRoutes`) after every association change:

* **Uplink.** Every online Agent has a default route and a host route to the Controller via its parent's bBSS address, out of its bSTA interface. Each client's default route points at the IP of the radio it is associated with.
* **Downlink.** For each client, a host route on the serving Agent (out of the serving radio), then on every ancestor and on the Controller (next hop: the child's bSTA address on the path).

Several interfaces of a node share a /16, so all forwarding uses explicit host or default routes rather than the connected network routes.

---

## 7. Traffic and measurements

* **Uplink:** UDP CBR, 10 Mbps per client (1472-byte packets), client → Controller, from 2 s.
* **Downlink:** UDP OnOff, 2 Mbps per client (1024-byte packets), Controller → client, from 2.5 s.
* **FlowMonitor** on all nodes: per-flow throughput, delay, jitter, PDR.
* **Per-radio counters:** UL/DL bytes (MAC traces) and mean channel utilization.

---

## 8. Running

```bash
cd ns-3.48
./ns3 build easymesh_sim
./ns3 run easymesh_sim                                     # everything enabled (defaults)
./ns3 run "easymesh_sim --steering=0 --loadBalancing=0"    # client roaming only
./ns3 run "easymesh_sim --channelPlan=none"                # no channel planning (baseline)
./ns3 run "easymesh_sim --offlineAgents=1,3 --offlineTime=10 --recoverTime=20"
./ns3 run "easymesh_sim --staSpeed=0"                      # static clients
```

| Flag | Default | Meaning |
|---|---|---|
| `--duration` | 30 | simulated seconds |
| `--scenarios` | 1 | runs with RNG run number 1…N (seed fixed at 42) |
| `--agents` / `--clients` | 15 / 8 | network size |
| `--offlineAgents` | `1` | comma list of Agents that fail |
| `--offlineTime` / `--recoverTime` | 12 / −1 | failure time / recovery time (< 0 = never) |
| `--mstMetric` | `throughput` | backhaul MST weight: `throughput` or `rssi` |
| `--channelPlan` | `dynamic` | `none`, `static` or `dynamic` |
| `--steering` | 1 | Controller RSSI + band steering |
| `--roaming` | 1 | client-initiated roaming |
| `--loadBalancing` | 1 | Controller load balancing |
| `--netOpt` | 1 | periodic backhaul optimization (failure recovery always runs) |
| `--staSpeed` | 2.0 | maximum client speed in m/s (0 = static) |
| `--pcap` | 0 | write PCAP files |

### Output

1. **Network topology** — every Agent: online state, position, uplink, backhaul RSSI/MCS/rate and path capacity, both radios with channel, mean utilization and client list.
2. **Channel plan** — initial and final channel and number of switches per radio, plus total co-channel interference initially and finally.
3. **Transition log** — every steering, band-steering, load-balancing, roaming, channel-switch, Agent-failure and backhaul re-parenting event: time, from → to, reason, RSSI before/after, latency, success; then a per-reason summary (count, successes, mean latency).
4. **Per-flow table** — FlowMonitor statistics.
5. **Per-radio traffic** — UL/DL throughput and mean utilization.
6. **Network-wide summary** — total packets, aggregate throughput, mean delay and PDR.

A NetAnim trace `easymesh-sim-animation-run<N>.xml` is written to the working directory.

---

## 9. Known limitations and ns-3.48 workarounds

| Item | Detail |
|---|---|
| **CCA bug in ns-3.48** | When a PPDU on an adjacent channel arrives while another PPDU's preamble/PHY header is being received below the CCA energy-detect threshold, ns-3 resets CCA to IDLE during the reception, which later trips `NS_ASSERT(!m_currentEvent)` in `PhyEntity::StartReceivePreamble`. The simulation sets `CcaEdThreshold` to −88 dBm (`CCA_ED_THRESHOLD_DBM`) so CCA stays busy for any PPDU being received. Side effect: radios defer slightly more often to weak energy than with the standard −62 dBm. |
| **Logging crash in ns-3.48** | Enabling `NS_LOG` for `WifiPhy`/`SpectrumWifiPhy`/`PhyEntity` aborts (also in stock ns-3 examples); use trace sources instead. |
| **Simulation speed** | `SpectrumWifiPhy` tracks receive power per RU band for HE/EHT, which is costly for wide channels. 5 GHz fronthaul and backhaul use 40 MHz (80 MHz is about 2× slower). `WifiBandwidthFilter` stops delivery of PPDUs to receivers on non-overlapping channels, and clients only listen on the band they are tuned to. |
| **Control plane** | IEEE 1905 CMDUs, 802.11v BTM, 802.11k reports and CSA are not sent over the air; their effect is modelled through direct calls and the BSS transition mechanism (section 4). |
| **RSSI for decisions** | Computed from positions with the channel's path-loss model (no shadowing or fading), i.e. ideal measurements. |
| **Adjacent-channel interference** | Only signals overlapping a receiver's channel (including the transmit-mask guard band) reach it. |
| **Backhaul** | A single shared backhaul channel; multi-hop capacity is modelled as airtime sharing (section 5.1). |
| **Utilization of idle radios** | Beacons from many co-channel APs, together with the low CCA-ED threshold, give radios without clients a noticeable utilization. |
