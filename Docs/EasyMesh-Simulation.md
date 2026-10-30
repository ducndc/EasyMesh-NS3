# EasyMesh-NS3 — Simulation Design and Algorithms

This document explains what the EasyMesh-NS3 simulation models, how it is built on ns-3, and how each feature works: backhaul topology, network optimization, client steering, band steering, load balancing, roaming, channel planning and Agent failure handling. All numbers quoted below are the defaults in `easymesh/parameters.h`.

---

## 1. What is simulated

A Wi-Fi EasyMesh network (Wi-Fi Alliance Multi-AP) inside a 90 m × 90 m area:

| Element | Count (default) | Radios |
|---|---|---|
| **Controller** | 1, fixed at (0, 0) | backhaul AP MLD (bBSS): 5 GHz + 6 GHz links, 802.11be |
| **Agent** | 15 (`--agents`) | backhaul AP MLD (bBSS) + backhaul STA MLD (bSTA), 5 GHz + 6 GHz links · fronthaul AP MLD with 5, 6 and 2.4 GHz affiliated APs, 802.11be |
| **Wi-Fi 7 client** | 4 (`--mldClients`) | non-AP MLD with a 5 GHz and a 6 GHz link (STR, or EMLSR with `--emlsr`) |
| **Wi-Fi 6 client** | the rest of `--clients` (8) | one 802.11ax radio that can tune to 2.4 or 5 GHz |

The Controller is the root of the mesh and the IP sink/source of all traffic. Agents relay traffic over a multi-hop multi-link Wi-Fi backhaul tree and serve clients on a tri-band fronthaul (same SSID, `EasyMesh-Fronthaul`, on all bands). `--bhMlo=0` reduces the backhaul to its 5 GHz link; `--fhMlo=0` replaces each fronthaul AP MLD by three single-link APs, and Wi-Fi 7 clients then become single-radio tri-band EHT STAs.

```
                    ┌──────────────┐
                    │  Controller  │  bBSS AP MLD: 5G ch159 + 6G ch7
                    └──────┬───────┘
     MLO backhaul (bSTA MLD → parent bBSS: 5G ch159/40 MHz + 6G ch7/80 MHz, 802.11be)
          ┌────────────────┼────────────────┐
     ┌────┴────┐      ┌────┴────┐      ┌────┴────┐
     │ Agent 0 │      │ Agent 4 │      │ Agent 8 │ ...
     └────┬────┘      └────┬────┘      └────┬────┘
          │ bBSS            │                 │
     ┌────┴────┐            │                 │
     │ Agent 1 │   fronthaul AP MLD: 5 GHz (40 MHz) + 6 GHz (80 MHz) + 2.4 GHz (20 MHz)
     └─────────┘            │
             (( Wi-Fi 7 MLD: 5G+6G ))  (( Wi-Fi 6: 2.4G or 5G ))  ← RandomWaypoint 0.5–2 m/s
```

### What is real and what is modelled

| Aspect | How it is simulated |
|---|---|
| 802.11 MAC/PHY, association, scanning, contention, collisions | **Real** ns-3 Wi-Fi (`SpectrumWifiPhy`, `ApWifiMac`, `StaWifiMac`) |
| Channels and interference | **Real** per-band `MultiModelSpectrumChannel`; PPDUs on different channels do not decode each other |
| Multi-link operation | **Real** ns-3 MLDs: multi-link setup, per-link channel access, traffic spread over links |
| Client/backhaul BSS transitions | **Real** disassociation + (multi-link) (re)association to a chosen AP / AP MLD |
| Agent failure | **Real**: every PHY of every Agent radio (all links) is switched off (`WifiPhy::SetOffMode`) |
| Channel utilization, UL/DL bytes | **Measured** per link from PHY state and PHY sniffer traces |
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
    ├── easymesh-radio.*      one fronthaul BSS (single-link AP or affiliated AP of an AP MLD):
    │                          channel, clients, measured utilization, UL/DL bytes
    ├── easymesh-link.*       backhaul link record (RSSI, MCS, rate; per MLO link)
    ├── easymesh-assoc-manager.*  association manager that pins a STA to a set of BSSIDs
    ├── channel-planner.*     interference-graph channel assignment
    └── traffic-stats.h       FlowMonitor collection and tables
```

**Division of responsibility.** The Controller *decides*; the simulation *acts*. The Controller never touches ns-3 objects directly. It calls hooks (`ControllerOps`) that the simulation provides:

| Hook | Purpose |
|---|---|
| `clientRssi(client, radio)` | estimated RSSI between a client and a fronthaul BSS |
| `radioRssi(a, b)` | estimated RSSI between two APs (interference graph) |
| `phyRate(rssi, width, maxMcs)` | expected PHY rate (max MCS 13 for Wi-Fi 7, 11 for Wi-Fi 6) |
| `steerClient(client, radio, reason)` | start a BSS transition |
| `switchChannel(radio, channel)` | retune an AP radio |

The simulation feeds events back to the Controller: client capabilities (`SetClientCaps`: Wi-Fi 7 or 6, MLO, bands), association with the list of set-up links (`NotifyClientAssociated`), disassociation, transition start and transition result (`RecordTransition`).

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

All bands use a log-distance model with exponent 3 and a per-band reference loss at 1 m:

```
RSSI(d) = P_tx − ( L_ref(band) + 10 · 3 · log10(max(d, 1 m)) )
L_ref(2.4 GHz) = 40.05 dB,  L_ref(5 GHz) = 46.68 dB,  L_ref(6 GHz) = 48.17 dB
P_tx = 20 dBm on 2.4 / 5 GHz
P_tx = min(30 dBm, 5 dBm/MHz + 10 · log10(width))  on 6 GHz (LPI PSD limit):
       20 MHz → 18 dBm, 80 MHz → 24 dBm, 160 → 27 dBm, 320 → 30 dBm
```

2.4 GHz therefore reaches about 6.6 dB further than 5 GHz. The 6 GHz rule is applied to every AP, bSTA and MLD link; single-radio clients transmit 20 dBm on every band. The Controller's estimates use exactly this formula, so they agree with the power the ns-3 PHYs receive.

### RSSI → MCS → PHY rate

Used by every decision that needs an expected throughput (MST throughput metric, BSS selection, backhaul capacity). The RSSI is first normalised to a 20 MHz channel, because a wider channel spreads the same power over more noise:

```
RSSI_eff = RSSI − 10 · log10(width / 20)
MCS      = highest m ≤ maxMcs with RSSI_eff ≥ threshold[m]
           thresholds (dBm): −82 −79 −77 −74 −70 −66 −65 −64 −59 −57 −54 −52 −49 −46   (MCS 0…13)
           maxMcs = 13 (EHT, 4096-QAM) for Wi-Fi 7 links, 11 (HE) for Wi-Fi 6 clients
rate     = rate20[MCS] · tones(width) / 234       (1 spatial stream, 0.8 µs GI)
           rate20 (Mbps): 8.6 17.2 25.8 34.4 51.6 68.8 77.4 86.0 103.2 114.7 129.0 143.4 154.9 172.1
           tones: 20 MHz 234, 40 MHz 468, 80 MHz 980, 160 MHz 1960, 320 MHz 3920
rate     = 0 if RSSI_eff < −82 dBm (no link)
```

An MLO backhaul hop is worth the **sum** of its link rates (simultaneous transmit/receive).

### Channels

| Use | Band | Channels | Width |
|---|---|---|---|
| Fronthaul 2.4 GHz | 2.4 GHz | 1, 6, 11 | 20 MHz |
| Fronthaul 5 GHz | 5 GHz | 38, 46, 54, 62, 102, 110, 118, 126, 134, 142 | 40 MHz |
| Fronthaul 6 GHz | 6 GHz | 71, 87, 103, 119, 135, 151, 167, 183, 199, 215 | 80 MHz |
| Backhaul link 0 (all nodes) | 5 GHz | 159 | 40 MHz |
| Backhaul link 1 (MLO) | 6 GHz | 7 (`--bhWidth6`: 20→1, 40→3, 80→7, 160→15, 320→31) | 80 MHz |
| Parking (transitions only) | 5 GHz / 6 GHz | 165 / 233 | 20 MHz |

The backhaul links have their own channels outside the fronthaul sets: the 6 GHz backhaul channel always lies inside 20 MHz channels 1–61, below the 6 GHz fronthaul channels, so backhaul and fronthaul do not contend. The fronthaul AP MLD's links are ordered 5, 6, 2.4 GHz (see section 9).

---

## 4. BSS transition mechanism (used by every feature that moves a STA)

Steering, roaming, load balancing, channel-switch follow, Agent-failure recovery and backhaul re-parenting all reuse one mechanism, `StartTransition(sta, target)`. The target is the address the association trace reports on success (the AP MLD address for a multi-link setup), the set of BSSIDs the STA may join through (every affiliated AP of a target AP MLD) and one channel per STA link. It models an 802.11v BSS Transition Management request (or an EasyMesh Backhaul Steering Request) with real MAC behaviour:

```
1. Pin the STA to the target:   EasyMeshAssocManager.SetTargetBssids(targets)
      → scan results from any other BSSID are discarded
2. Park:   retune every STA PHY (link) to an idle parking channel
           (165 on 5 GHz, 233 for a 6 GHz MLD link)
      → ns-3 disassociates the STA and it starts scanning
3. Hop:    once every PHY is on its parking channel (polled every 1 ms,
           because a channel switch is postponed while the PHY is busy),
           retune each link to the target's channel on that band
4. Scan:   active probing finds only the target BSSIDs
5. Associate: (re)association with the target; an MLD sets up all its
           links with the target AP MLD in one multi-link setup
6. Assoc trace fires → success: transition latency = now − start;
           routes rebuilt; Controller topology DB updated
   Timeout (1.5 s) without success → failure recorded;
           a client is unpinned and may join any BSS on its channel;
           a backhaul STA stays pinned (it must never join a descendant)
```

The parking step guarantees that the STA really leaves its current BSS, even when the target is on the same channel. A new decision for a STA that is still in transition supersedes the old one (the old one is logged as failed). After a transition, the STA is not moved again for `STEER_COOLDOWN` = 4 s, to avoid ping-pong.

`EasyMeshAssocManager` derives from ns-3's `WifiDefaultAssocManager` and only overrides `CanBeInserted` / `CanBeReturned`. With no target it behaves exactly like the default manager.

**Leaving the old AP.** ns-3 STAs leave without a Disassociation frame and nobody tears down Block Ack agreements, so the old AP keeps the STA as associated. When the STA disassociates, the simulation therefore does what a Disassociation + DELBA would (`ForgetStaAtAp`): it resets the old AP's record of the STA on every link and destroys the BA agreements on both sides. Without this, re-associating through another affiliated AP of the same AP MLD (e.g. band steering a Wi-Fi 6 client from 2.4 to 5 GHz on one Agent) reuses a stale AID and receives A-MPDUs under a BA agreement it no longer has, both of which abort ns-3.

**Multi-link setup.** The Assoc trace of an MLD reports the AP MLD address; the simulation reads the links actually set up from `StaWifiMac::GetSetupLinkIds()`. Links left out of a setup are switched off by ns-3; they are switched back on at the next disassociation or transition. The number of partial setups is printed at the end.

---

## 5. Feature algorithms

### 5.1 Backhaul topology — Prim's maximum spanning tree

Every Agent's backhaul STA must choose a parent: the Controller or another Agent. The tree is chosen by Prim's algorithm with **maximised** edge weights over a graph of all online Agents plus a virtual root node for the Controller.

```
weight(a, b) = RSSI(a, b) on the 5 GHz backhaul link       if --mstMetric=rssi
             = Σ over backhaul links rate(RSSI, width)    if --mstMetric=throughput  (default)
               (5G 40 MHz + 6G 80 MHz with MLO)

inTree = {Controller}
best[v] = weight(Controller, v), parent[v] = Controller   for every online Agent v
repeat |online Agents| times:
    u = online Agent not in tree with the largest best[u]
    add u to tree; its parent is parent[u]
    for every online Agent v not in tree:
        if weight(u, v) > best[v]: best[v] = weight(u, v); parent[v] = u
```

Complexity is O(N²). Each step attaches the outside node reachable by the strongest single edge, which gives the tree with the largest total weight.

**Backhaul path capacity.** All hops share the backhaul channels, so their airtime adds up; an MLO hop's rate is the sum over its links. An Agent's end-to-end backhaul capacity is

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
single-link client:
score(client, radio) = min( rate(RSSI, width), C(agent) ) · (1 − 0.8 · util(radio))
candidates: radios of online Agents on a band the client supports, RSSI ≥ minimum (−82 dBm)

MLD client (scored per AP MLD, represented by its 5 GHz "anchor" radio):
score(client, agent) = min( Σ over the client's bands rate(RSSI_link, width) · (1 − 0.8 · util(link)), C(agent) )
candidates: the anchor radio of every online Agent, anchor RSSI ≥ minimum
```

Wi-Fi 6 clients are never offered a 6 GHz radio; their rates stop at MCS 11.

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

This models a Controller moving a "sticky" client that has not roamed on its own. For an MLD client, "radio" means AP MLD: it is only steered to another Agent.

### 5.5 Band steering

Evaluated in the same tick, after RSSI steering:

```
MLD clients: skipped (they use all their bands at once)

single-link client on 2.4 GHz:
    target = best-scoring 5 / 6 GHz radio of the same Agent with RSSI ≥ −65 dBm;
             else the best-scoring 5 / 6 GHz radio of any Agent with RSSI ≥ −65 dBm
    if target exists: steer to 5 / 6 GHz (reason BAND_STEER)

single-link client on 5 / 6 GHz with RSSI < −77 dBm:
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

An MLD client counts on every link it uses; it can only be moved to another AP MLD, and the 20 % margin is checked against that AP MLD's link on the overloaded band. At most one client per overloaded radio is moved per tick; the cooldown and the next metrics window decide whether more need to move.

### 5.7 Roaming (client-initiated)

Clients move with `RandomWaypoint` (0.5–2 m/s, pause up to 2 s). Every 0.5 s each client acts on its own, using signal strength only (it knows nothing about load or backhaul):

```
if RSSI(serving) < −78 dBm:
    best = candidate radio with the strongest RSSI among online Agents
           (an MLD client only considers other Agents' AP MLDs)
    if RSSI(best) ≥ RSSI(serving) + 6 dB:  transition (reason ROAMING)
```

The roaming threshold (−78 dBm) is below the Controller's steering threshold (−72 dBm), as for a real client that roams late. With steering enabled the Controller usually acts first; run with `--steering=0` to see pure client roaming.

### 5.8 Channel planning

**Interference graph.** Planning runs per band (2.4, 5 and 6 GHz); each affiliated AP of an AP MLD is planned like a separate radio. Every pair of fronthaul radios in the same band is an edge, weighted by how far their mutual signal rises above the interference floor:

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

**Applying a channel change (CSA follow).** ns-3 has no Channel Switch Announcement. The Controller retunes the AP radio and immediately runs a BSS transition for each of its clients to the same BSSID on the new channel (reason `CH_SWITCH`); an MLD client using that link re-sets up all its links with the same AP MLD.

**Modes (`--channelPlan`).**

| Mode | Behaviour |
|---|---|
| `none` | every radio on the first channel of its band (worst-case baseline) |
| `static` | initial plan only |
| `dynamic` | initial plan + periodic re-planning (default) |

### 5.9 Agent failure and recovery

At `--offlineTime` (default 12 s, Agent 1):

```
1. Mark the Agent(s) offline and switch off every PHY of their radios (bBSS, bSTA, fronthaul; all links).
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
| Fronthaul `10.2.0.0/16` | Agent *a* AP MLD `10.2.4.(a+1)` (one interface for all links), or with `--fhMlo=0` 2.4 GHz `10.2.1.(a+1)`, 5 GHz `10.2.2.(a+1)`, 6 GHz `10.2.3.(a+1)`; client *c* `10.2.(10 + c/250).(c%250 + 1)` |

Routing is fully static and rebuilt (`RebuildRoutes`) after every association change:

* **Uplink.** Every online Agent has a default route and a host route to the Controller via its parent's bBSS address, out of its bSTA interface. Each client's default route points at the IP of the radio it is associated with.
* **Downlink.** For each client, a host route on the serving Agent (out of the serving radio), then on every ancestor and on the Controller (next hop: the child's bSTA address on the path).

Several interfaces of a node share a /16, so all forwarding uses explicit host or default routes rather than the connected network routes.

---

## 7. Traffic and measurements

* **Uplink:** UDP CBR, 10 Mbps per client (1472-byte packets), client → Controller, from 2 s.
* **Downlink:** UDP OnOff, 2 Mbps per client (1024-byte packets), Controller → client, from 2.5 s.
* **FlowMonitor** on all nodes: per-flow throughput, delay, jitter, PDR.
* **Per-radio counters:** UL/DL bytes and mean channel utilization, per affiliated AP. Bytes come from the link PHY's sniffer traces (unicast data received by / first transmissions sent by that AP), because the MAC traces of an AP MLD are shared by all its links.

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
./ns3 run "easymesh_sim --bhMlo=0 --fhMlo=0"               # Wi-Fi 7 without MLO
./ns3 run "easymesh_sim --bhWidth6=320 --mldClients=8"     # 320 MHz backhaul, all clients MLD
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
| `--bhMlo` | 1 | MLO backhaul (5 GHz + 6 GHz links); 0 = 5 GHz link only |
| `--bhWidth6` | 80 | 6 GHz backhaul link width: 20, 40, 80, 160 or 320 MHz |
| `--fhMlo` | 1 | tri-band AP MLD fronthaul + MLO for Wi-Fi 7 clients; 0 = single-link APs and clients |
| `--mldClients` | 4 | number of Wi-Fi 7 clients (the rest are Wi-Fi 6) |
| `--emlsr` | 0 | experimental: Wi-Fi 7 MLD clients use EMLSR instead of STR (section 9) |
| `--pcap` | 0 | write PCAP files |

### Output

1. **Network topology** — every Agent: online state, position, uplink, backhaul RSSI/MCS/rate (and per MLO link) and path capacity, every fronthaul radio with channel, mean utilization and client list (an MLD client appears on each of its links).
2. **Channel plan** — initial and final channel and number of switches per radio, plus total co-channel interference initially and finally.
3. **Transition log** — every steering, band-steering, load-balancing, roaming, channel-switch, Agent-failure and backhaul re-parenting event: time, from → to, reason, RSSI before/after, latency, success; then a per-reason summary (count, successes, mean latency).
4. **Per-flow table** — FlowMonitor statistics.
5. **Per-radio traffic** — UL/DL throughput and mean utilization, then a line with the Wi-Fi 7 configuration and the number of partial multi-link setups.
6. **Network-wide summary** — total packets, aggregate throughput, mean delay and PDR.

A NetAnim trace `easymesh-sim-animation-run<N>.xml` is written to the working directory.

---

## 9. Known limitations and ns-3.48 workarounds

| Item | Detail |
|---|---|
| **CCA bug in ns-3.48** | When a PPDU on an adjacent channel arrives while another PPDU's preamble/PHY header is being received below the CCA energy-detect threshold, ns-3 resets CCA to IDLE during the reception, which later trips `NS_ASSERT(!m_currentEvent)` in `PhyEntity::StartReceivePreamble`. The simulation sets `CcaEdThreshold` to −88 dBm (`CCA_ED_THRESHOLD_DBM`) for PHYs up to 40 MHz and 3 dB lower per doubling above (−91 dBm at 80 MHz, −97 dBm at 320 MHz), so CCA stays busy for any PPDU being received. Side effect: radios defer more often to weak energy than with the standard −62 dBm. |
| **MLD link IDs in ns-3.48** | At multi-link setup a non-AP MLD takes over the AP MLD's link IDs, but `WifiDefaultAssocManager::DoStartScanning` later rescans assuming link IDs 0..n−1. The fronthaul AP MLD's links are therefore ordered 5, 6, 2.4 GHz, so the Wi-Fi 7 clients' 5 + 6 GHz links keep IDs 0 and 1. |
| **Stale AP state** | ns-3 STAs leave without a Disassociation frame; the simulation resets the old AP's station record and the BA agreements itself (section 4). |
| **NetAnim with MLDs** | `AnimationInterface` calls `GetChannel()` on every device at start-up, which aborts for a multi-band MLD; the animation is therefore created before the Wi-Fi devices exist, and the XML has no node IP addresses. |
| **Logging crash in ns-3.48** | Enabling `NS_LOG` for `WifiPhy`/`SpectrumWifiPhy`/`PhyEntity` aborts (also in stock ns-3 examples); use trace sources instead. |
| **Simulation speed** | `SpectrumWifiPhy` tracks receive power per RU band for HE/EHT, which is costly for wide channels. 5 GHz fronthaul and backhaul use 40 MHz (80 MHz is about 2× slower); the 6 GHz backhaul link defaults to 80 MHz because 320 MHz is about 4× slower again. MLO itself adds little: a 5-Agent run is about 3× slower with 5 + 6 GHz links than single-link. `WifiBandwidthFilter` stops delivery of PPDUs to receivers on non-overlapping channels, and clients only listen on the band they are tuned to. |
| **Control plane** | IEEE 1905 CMDUs, 802.11v BTM, 802.11k reports and CSA are not sent over the air; their effect is modelled through direct calls and the BSS transition mechanism (section 4). |
| **RSSI for decisions** | Computed from positions with the channel's path-loss model (no shadowing or fading), i.e. ideal measurements. |
| **Adjacent-channel interference** | Only signals overlapping a receiver's channel (including the transmit-mask guard band) reach it. |
| **Backhaul** | One shared backhaul channel per band; multi-hop capacity is modelled as airtime sharing, an MLO hop as the sum of its links (section 5.1). |
| **EMLSR (experimental)** | EMLSR clients get PHYs that are not pinned to a band, so the EMLSR manager can move its main PHY between the 5 and 6 GHz links. The BSS transition mechanism (section 4) retunes STA PHYs directly, which leaves the EMLSR manager inconsistent: ns-3 aborts in `DefaultEmlsrManager` ("Main PHY cannot be in state TX") after the first transition of an EMLSR client. Runs without such transitions work; STR clients are unaffected. |
| **Spatial streams** | Every device has one spatial stream; the rate model assumes the same. |
| **Utilization of idle radios** | Beacons from many co-channel APs, together with the low CCA-ED threshold, give radios without clients a noticeable utilization. |
