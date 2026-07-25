# EasyMesh-NS3

A Wi-Fi EasyMesh network simulation built on [ns-3](https://www.nsnam.org/), modeling a Controller/Agent backhaul-and-fronthaul mesh with dynamic topology optimization, BSS client steering, and per-flow traffic analytics.

## Features

- **Multi-AP backhaul mesh** — one Controller and multiple Agents (APs), interconnected over Wi-Fi (802.11be) or Ethernet.
- **Backhaul topology selection via MST** — each Agent's uplink AP is chosen by a Prim's Maximum-Spanning-Tree algorithm, selectable at runtime between two edge-weight strategies:
  - `throughput` — maximizes aggregate achievable Wi-Fi throughput across the mesh.
  - `rssi` — maximizes aggregate/strongest signal quality across the mesh.
- **Client (STA) association & BSS steering** — clients associate with the best available Agent and can be steered based on RSSI, load, or channel utilization.
- **Agent failure handling** — simulate an Agent going offline mid-run and observe the backhaul tree and client associations re-converge.
- **Mixed traffic model** — UDP CBR (uplink/downlink), TCP bulk transfer, and inter-Agent backhaul stress traffic.
- **FlowMonitor-based reporting** — per-flow, per-Agent, and network-wide throughput/delay/PDR statistics printed at the end of each run.

## Prerequisites

- ns-3 **3.46.1** (or compatible), built with CMake.
- A C++17-capable compiler (matching your ns-3 toolchain).
- The following ns-3 modules enabled: `core`, `network`, `wifi`, `mobility`, `internet`, `applications`, `flow-monitor`, `netanim`, `csma`, `bridge`.

## Project Structure

```
EasyMesh-NS3/
├── easymesh.cc              # Simulation entry point (main()), topology/build orchestration
├── easymesh.h
├── parameters.h             # Tunable simulation constants (topology, traffic, thresholds)
├── CMakeLists.txt           # Scratch build definition (custom target: easymesh_sim)
└── lib/
    ├── easymesh-agent.cc/.h        # Agent (AP) model
    ├── easymesh-controller.cc/.h   # Controller: steering engine, topology optimization, reporting
    ├── easymesh-link.cc/.h         # Backhaul/fronthaul link model
    └── traffic-stats.h             # FlowMonitor collection & statistics printing
```

## Setup

Copy the project into your ns-3 `scratch/` directory **as a subdirectory** (this preserves the `lib/` layout and custom `CMakeLists.txt` that the build depends on):

```bash
cp -r EasyMesh-NS3 ns-3.46.1/scratch/easymesh
```

## Build

```bash
cd ns-3.46.1
./ns3 configure
./ns3 build
```

> **Note:** Because `scratch/easymesh/CMakeLists.txt` is a custom build file, ns-3 builds it under the target name **`easymesh_sim`** (as defined by `EXECNAME` in that file) rather than the conventional `scratch/<dirname>` naming used for plain, CMakeLists-less scratch folders. Use `easymesh_sim` as the program name when running.
>
> If you're unsure of the exact target name after building, confirm it with:
> ```bash
> find build -iname "*easymesh*" -type f
> ```

## Run

```bash
# Default run (throughput-maximizing MST)
./ns3 run easymesh_sim

# Case 1: throughput-maximizing MST
./ns3 run "easymesh_sim --mstMetric=throughput"

# Case 2: RSSI-maximizing MST
./ns3 run "easymesh_sim --mstMetric=rssi"
```

### CLI Parameters

| Flag             | Type   | Default        | Description                                              |
|-------------------|--------|----------------|------------------------------------------------------------|
| `--duration`      | double | `15.0`         | Simulation duration in seconds                              |
| `--scenarios`     | uint32 | `1`            | Number of randomized scenario runs                          |
| `--clients`       | uint32 | `1`            | Number of STA clients                                       |
| `--offlineAgent`  | uint32 | `1`            | Agent ID to take offline mid-run                             |
| `--offlineTime`   | double | `6.0`          | Simulation time (s) at which the Agent goes offline           |
| `--pcap`          | bool   | `false`        | Enable PCAP capture                                          |
| `--mstMetric`     | string | `throughput`   | Backhaul MST edge-weight metric: `throughput` \| `rssi`       |

Example — a longer run with 5 clients and RSSI-based MST:

```bash
./ns3 run "easymesh_sim --duration=30 --clients=5 --mstMetric=rssi"
```

## Output

Each run prints, in order:

1. **Network topology** — Agent roles, positions, load, channel utilization, and per-Agent link status.
2. **BSS steering log** — every client-steering event with reason (RSSI/load/channel-utilization), before/after RSSI, and outcome.
3. **Per-flow traffic table** — Tx/Rx packets, throughput, delay, jitter, PDR, and quality rating for every monitored flow.
4. **Per-Agent traffic summary** — aggregate uplink/downlink throughput, delay, PDR, and load per Agent.
5. **Network-wide summary** — total flows, packets, aggregate throughput, mean delay, and mean PDR.

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