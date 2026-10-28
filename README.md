# EasyMesh-NS3

A Wi-Fi EasyMesh (Multi-AP) network simulation built on [ns-3](https://www.nsnam.org/). One Controller and a set of dual-band Agents form a multi-hop Wi-Fi backhaul tree and serve mobile clients. The simulation covers client steering, band steering, load balancing, roaming, channel planning, backhaul (network) optimization and Agent failure, all on top of real ns-3 802.11 association, scanning and channel contention.

For the design and the algorithm behind each feature, see **[Docs/EasyMesh-Simulation.md](Docs/EasyMesh-Simulation.md)**.

## Features

- **Tri-band style mesh** — the Controller and every Agent run a backhaul BSS on a dedicated 5 GHz channel (802.11be); each Agent joins its parent through a backhaul STA and serves clients on a 2.4 GHz and a 5 GHz fronthaul BSS (802.11ax, same SSID). Separate spectrum channels per band, so channels and bands really isolate BSSs.
- **Backhaul topology via MST** — each Agent's uplink is chosen by Prim's Maximum Spanning Tree, with edge weight `throughput` (expected PHY rate) or `rssi`.
- **Network optimization** — the backhaul tree is re-evaluated periodically and Agents are re-parented when a new tree is more than 10 % better, or immediately when the tree is broken.
- **Client steering** — the Controller moves clients whose signal drops below a threshold to a better BSS, chosen by expected throughput (link rate, backhaul path capacity and channel load).
- **Band steering** — clients move from 2.4 GHz to 5 GHz when 5 GHz is strong enough, and back when 5 GHz becomes too weak.
- **Load balancing** — channel utilization is measured from each radio's PHY state; clients are moved off overloaded radios to clearly less loaded ones.
- **Roaming** — clients move (RandomWaypoint) and roam on their own when the signal gets weak.
- **Channel planning** — interference-graph channel assignment per band at start-up, with optional periodic re-planning driven by measured utilization; clients follow their AP to the new channel.
- **Agent failure and recovery** — selected Agents lose power mid-run; the backhaul re-forms around them and their clients are moved; optionally they come back later.
- **Real BSS transitions** — every move (steering, roaming, re-parenting) is an actual disassociation and association to a chosen BSSID, modelling an 802.11v BTM request; its latency and success are logged.
- **Reporting** — topology, channel plan, transition log with per-reason statistics, FlowMonitor per-flow table, per-radio traffic and a network-wide summary.

## Prerequisites

- ns-3 **3.48** (tested), built with CMake.
- A C++17-capable compiler (matching your ns-3 toolchain).
- The following ns-3 modules enabled: `core`, `network`, `wifi`, `spectrum`, `propagation`, `mobility`, `internet`, `applications`, `flow-monitor`, `netanim`.

## Project Structure

```
EasyMesh-NS3/                    # mirrors an ns-3 scratch/ directory
├── CMakeLists.txt               # ns-3's stock scratch/CMakeLists.txt
├── Docs/
│   ├── EasyMesh-Simulation.md   # design and algorithms of this simulation
│   └── WiFi_Mesh/               # LaTeX report on EasyMesh concepts (not built)
└── easymesh/
    ├── easymesh.cc / .h         # main(); builds the network, executes decisions, BSS transitions,
    │                            #   routing, roaming, backhaul optimization, Agent failure
    ├── parameters.h             # all tunable constants (topology, channels, thresholds, traffic)
    ├── CMakeLists.txt           # scratch build definition (custom target: easymesh_sim)
    └── lib/
        ├── easymesh-controller.cc/.h      # Controller: steering, band steering, load balancing,
        │                                  #   channel planning decisions, reporting
        ├── easymesh-agent.cc/.h           # Agent: online state, backhaul parent and capacity
        ├── easymesh-radio.cc/.h           # fronthaul BSS: channel, clients, measured utilization
        ├── easymesh-link.cc/.h            # backhaul link record
        ├── easymesh-assoc-manager.cc/.h   # association manager that pins a STA to one BSSID
        ├── channel-planner.cc/.h          # interference-graph channel assignment
        ├── easymesh-types.h               # band, channel and transition-reason types
        └── traffic-stats.h                # FlowMonitor collection & statistics printing
```

## Setup

Copy the `easymesh/` folder into your ns-3 `scratch/` directory (this preserves the `lib/` layout and custom `CMakeLists.txt` that the build depends on). The repo's top-level `CMakeLists.txt` is identical to the one ns-3 already ships in `scratch/`, so it does not need to be copied:

```bash
cp -r EasyMesh-NS3/easymesh ns-3.48/scratch/
```

## Build

```bash
cd ns-3.48
./ns3 configure
./ns3 build easymesh_sim
```

> **Note:** Because `scratch/easymesh/CMakeLists.txt` is a custom build file, ns-3 builds it under the target name **`easymesh_sim`** (as defined by `EXECNAME` in that file) rather than the conventional `scratch/<dirname>` naming used for plain, CMakeLists-less scratch folders. Use `easymesh_sim` as the program name when running.
>
> If you're unsure of the exact target name after building, confirm it with:
> ```bash
> find build -iname "*easymesh*" -type f
> ```

## Run

```bash
# Default run: 15 Agents, 8 mobile clients, 30 s, all features on, Agent 1 fails at 12 s
./ns3 run easymesh_sim

# RSSI-maximizing backhaul MST instead of throughput
./ns3 run "easymesh_sim --mstMetric=rssi"

# Client-driven roaming only (no Controller steering / load balancing)
./ns3 run "easymesh_sim --steering=0 --loadBalancing=0"

# Baseline without channel planning (every radio on one channel per band)
./ns3 run "easymesh_sim --channelPlan=none"

# Agents 1 and 3 fail at 10 s and come back at 20 s
./ns3 run "easymesh_sim --offlineAgents=1,3 --offlineTime=10 --recoverTime=20"

# Static clients, larger mesh
./ns3 run "easymesh_sim --staSpeed=0 --agents=20 --clients=10"
```

Comparing runs that differ in one flag (e.g. `--channelPlan=none` vs `dynamic`, or `--steering=0` vs `1`) shows the effect of each feature. Results are reproducible: the RNG seed is fixed and the run number is the scenario index.

### CLI Parameters

| Flag              | Type   | Default        | Description                                                        |
|-------------------|--------|----------------|--------------------------------------------------------------------|
| `--duration`      | double | `30.0`         | Simulation duration in seconds                                     |
| `--scenarios`     | uint32 | `1`            | Number of randomized scenario runs                                 |
| `--clients`       | uint32 | `8`            | Number of STA clients                                              |
| `--agents`        | uint32 | `15`           | Number of Agents (APs) in the mesh                                 |
| `--offlineAgents` | string | `1`            | Comma-separated Agent IDs that go offline mid-run (e.g. `1,3,5`)   |
| `--offlineTime`   | double | `12.0`         | Time (s) at which the listed Agent(s) go offline                   |
| `--recoverTime`   | double | `-1`           | Time (s) at which they come back (`< 0`: never)                    |
| `--mstMetric`     | string | `throughput`   | Backhaul MST edge weight: `throughput` \| `rssi`                   |
| `--channelPlan`   | string | `dynamic`      | Fronthaul channel planning: `none` \| `static` \| `dynamic`         |
| `--steering`      | bool   | `true`         | Controller client steering and band steering                       |
| `--roaming`       | bool   | `true`         | Client-initiated roaming                                           |
| `--loadBalancing` | bool   | `true`         | Controller load balancing                                          |
| `--netOpt`        | bool   | `true`         | Periodic backhaul optimization (failure recovery always runs)      |
| `--staSpeed`      | double | `2.0`          | Maximum client speed in m/s (`0` = static clients)                  |
| `--pcap`          | bool   | `false`        | Enable PCAP capture                                                |

Thresholds, intervals, channel sets, traffic rates and placement bounds are in `easymesh/parameters.h`.

## Output

Each run prints, in order:

1. **Network topology** — per Agent: online state, position, backhaul parent, backhaul RSSI/MCS/rate and path capacity, and both fronthaul radios with channel, mean utilization and associated clients.
2. **Channel plan** — initial and final channel, number of switches and mean utilization per radio, plus total co-channel interference before and after.
3. **Transition log** — every steering, band-steering, load-balancing, roaming, channel-switch, Agent-failure and backhaul re-parenting event with time, from → to, reason, RSSI before/after, latency and outcome; followed by a per-reason summary (count, successes, mean latency).
4. **Per-flow traffic table** — Tx/Rx packets, throughput, delay, jitter, PDR and quality rating for every monitored flow.
5. **Per-radio traffic summary** — uplink/downlink throughput and mean channel utilization per fronthaul radio.
6. **Network-wide summary** — total flows, packets, aggregate throughput, mean delay and mean PDR.

A NetAnim trace `easymesh-sim-animation-run<N>.xml` is written to the working directory.

## Known Limitations

- **ns-3.48 CCA bug workaround** — ns-3.48 can reset CCA to idle while a frame is still being received when a neighbouring channel is busy, which triggers an assertion. The simulation lowers the CCA energy-detect threshold to −88 dBm (`CCA_ED_THRESHOLD_DBM`) to avoid it; radios therefore defer slightly more often to weak energy.
- **Simulation speed** — 5 GHz fronthaul and backhaul use 40 MHz channels; 80 MHz roughly halves simulation speed.
- **Control plane** — IEEE 1905 messages, 802.11v/k frames and channel switch announcements are not sent over the air; their effect is modelled directly.
- **Debug logging** — enabling `NS_LOG` for the ns-3 Wi-Fi PHY components aborts in ns-3.48 (also in stock examples).

## License

This project is licensed under the **GNU General Public License version 2 (GPLv2)**, consistent with ns-3's own licensing, since it is built as an ns-3 scratch program linked directly against ns-3's libraries.

```
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation;

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
```

See the [full GPLv2 license text](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html) for complete terms.