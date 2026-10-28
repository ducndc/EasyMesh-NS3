# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An ns-3 scratch program that simulates a Wi-Fi EasyMesh network: one Controller plus N Agents (APs) linked by an 802.11be Wi-Fi backhaul tree, with STA clients on a shared fronthaul. It covers backhaul tree selection (Prim's MST), client association and steering, Agents going offline mid-run, and FlowMonitor traffic reports. There is no standalone build, no test suite and no linter. The code only compiles inside an ns-3 source tree.

## Repo layout vs. ns-3 tree

The repo root mirrors an ns-3 `scratch/` directory:
- `CMakeLists.txt` at the root is ns-3's stock `scratch/CMakeLists.txt`. It finds subdirectories and calls `add_subdirectory` on any that have their own CMakeLists.
- `easymesh/CMakeLists.txt` is the real build file. It builds `easymesh-lib` from `lib/*.cc` and an executable named **`easymesh_sim`** (not `scratch_easymesh_easymesh`).
- `Docs/WiFi_Mesh/` is a LaTeX report (TU Delft template) describing EasyMesh concepts (onboarding, topology, steering, channel planning). It is not part of the build.

On this machine the ns-3 tree is at `/home/ducndc/Workspaces/ns-3.48`, and its `scratch/` holds a **copy** of this repo (not a symlink). Edits made here must be copied to `ns-3.48/scratch/easymesh/` before building, for example `cp -r easymesh/. ../ns-3.48/scratch/easymesh/`. Check which copy the user means before you edit.

## Build & run (from the ns-3 root)

```bash
./ns3 configure            # once; needs core network wifi mobility internet applications flow-monitor netanim csma bridge
./ns3 build easymesh_sim
./ns3 run "easymesh_sim --mstMetric=rssi --agents=20 --clients=5 --duration=30"
./ns3 run "easymesh_sim --offlineAgents=1,3,5 --offlineTime=8.0"
```

CLI flags (parsed in `main()` in `easymesh/easymesh.cc`): `--duration`, `--scenarios`, `--clients`, `--agents`, `--offlineAgents` (comma list), `--offlineTime`, `--pcap`, `--mstMetric=throughput|rssi`. Defaults come from `easymesh/parameters.h`. Each run writes `easymesh-sim-animation-run<N>.xml` (NetAnim) to the working directory. The RNG seed is fixed at 42, and the run number is the scenario index, so results are reproducible.

## Architecture

**`EasyMeshSimulation` (`easymesh.h` / `easymesh.cc`) does almost all the work.** For each scenario, `main()` runs these phases in order: `BuildTopology` → `InstallProtocolStack` → `InstallApplications` → `ScheduleEvents` → `Run` → `PrintResults`.

- **Topology:** Agent positions are random within the `TOPOLOGY_*` bounds. The Controller is fixed at `CONTROLLER_POS` (0,0). `BuildBackhaulTree()` runs Prim's *maximum* spanning tree over agents plus a virtual Controller root node (index `m_numAgents`) and fills `m_agentParent` (-1 means the parent is the Controller). Edge weights come from an analytic model, not from ns-3 PHY measurements: `EstimateLinkRssi` (log-distance path loss) and `EstimateLinkThroughput` (RSSI→MCS→Mbps). `--mstMetric` only changes which of these two the `edgeWeight` lambda returns.
- **Backhaul devices:** `CreateBackhaulLinks()` installs one AP/STA Wi-Fi pair per agent, with SSID `EasyMesh-Backhaul-<a>`, on a single shared `YansWifiChannel`. `m_backhaulDevices` is indexed as `[2a]` = parent side and `[2a+1]` = agent side. `InstallProtocolStack` depends on that order.
- **Addressing/routing:** Backhaul pair `a` gets subnet `10.1.(a+1).0/24`. The whole fronthaul (all agent APs plus all STAs) is one `10.2.0.0/24` subnet. Routing is all static: each agent's default route points to its parent, and `RebuildClientRoutes()` installs per-client host routes along the tree path. After any change to topology or association, call `RebuildClientRoutes()`.
- **Agent failure:** `HandleAgentOffline()` marks every listed agent offline at once, then rebuilds the MST once, then calls `RefreshBackhaulLinks()` (which reuses the existing `EasyMeshLink` objects and changes their endpoints), re-associates clients via `SelectBestUplinkAgent`, and rebuilds routes. The event is only scheduled when `UL_START < offlineTime < duration`.
- **Logical model (`easymesh/lib/`):** `EasyMeshController`, `EasyMeshAgent` and `EasyMeshLink` are plain `SimpleRefCount` bookkeeping objects. They are not ns-3 `Application`s and send no IEEE 1905 CMDUs. The controller's `CollectApMetrics`, `RunSteeringEngine` and `OptimizeBackhaulTopology` run on fixed timers (every 5 s, 10 s and 20 s) and log `SteeringEvent`s. The steering thresholds are `static constexpr` values in `easymesh-controller.h`.
- **Traffic & stats:** UDP uplink/downlink CBR, TCP BulkSend and Agent→Agent backhaul stress traffic, configured by the ports, rates and start times in `parameters.h`. `lib/traffic-stats.h` is header-only (all functions `inline`). It collects FlowMonitor results after `Simulator::Run()`, and `ApplyTrafficStats` passes them to the controller for the per-Agent summary.

## Conventions

- When you add vectors indexed by agent or client, resize them in **both** the constructor and `SetNumAgents`/`SetNumClients`. The constructor sizes them from the `NUM_*` macros, and the setters resize them again for the CLI values.
- New `.cc` files under `lib/` must be added to `add_library(easymesh-lib ...)` in `easymesh/CMakeLists.txt`.
- Code style is ns-3 style: `m_` members, return type on its own line before `Class::Method`, and `NS_LOG_INFO` logging (the `EasyMeshSim` and `EasyMeshController` components are enabled at INFO level in `main`). Some comments are in Vietnamese.
