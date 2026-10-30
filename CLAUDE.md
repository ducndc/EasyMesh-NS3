# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An ns-3 scratch program that simulates a Wi-Fi 7 (802.11be) EasyMesh network: one Controller plus N Agents linked by a multi-link (5 + 6 GHz MLO) Wi-Fi backhaul tree, each Agent serving clients through a tri-band (5/6/2.4 GHz) AP MLD. Clients are a mix of Wi-Fi 7 MLDs (5 + 6 GHz, STR or EMLSR) and single-radio Wi-Fi 6 STAs. It covers backhaul tree selection (Prim's MST), client association and steering, Agents going offline mid-run, and FlowMonitor traffic reports. There is no standalone build, no test suite and no linter. The code only compiles inside an ns-3 source tree.

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

CLI flags (parsed in `main()` in `easymesh/easymesh.cc`): `--duration`, `--scenarios`, `--clients`, `--agents`, `--offlineAgents` (comma list), `--offlineTime`, `--recoverTime`, `--pcap`, `--mstMetric=throughput|rssi`, `--channelPlan`, `--steering`, `--roaming`, `--loadBalancing`, `--netOpt`, `--staSpeed`, and the Wi-Fi 7 flags `--bhMlo`, `--bhWidth6`, `--fhMlo`, `--mldClients`, `--emlsr` (experimental: aborts after the first transition of an EMLSR client). Defaults come from `easymesh/parameters.h`. Each run writes `easymesh-sim-animation-run<N>.xml` (NetAnim) to the working directory. The RNG seed is fixed at 42, and the run number is the scenario index, so results are reproducible.

## Architecture

**`EasyMeshSimulation` (`easymesh.h` / `easymesh.cc`) does almost all the work.** For each scenario, `main()` runs these phases in order: `BuildTopology` → `InstallProtocolStack` → `InstallApplications` → `ScheduleEvents` → `Run` → `PrintResults`.

- **Topology & backhaul tree:** Agent positions are random within the `TOPOLOGY_*` bounds, then spread apart; the Controller is fixed at `CONTROLLER_POS` (0,0). `ComputeBackhaulTree()` runs Prim's *maximum* spanning tree over online agents plus a virtual Controller root (index `m_numAgents`); `m_plannedParent` is the MST, `m_agentParent` the parent each bSTA is actually associated with (-1 = Controller). Edge weights are analytic, not measured: `EstimateRssi` (log-distance, per band, 6 GHz power from a PSD limit) and `PhyRateMbps` (RSSI→MCS 0–13→Mbps). With MLO the throughput weight is the sum over `m_bhChannels` (`BackhaulRate`). `OptimizeBackhaul` re-parents bSTAs when the tree improves by `NETOPT_MIN_GAIN` or is broken.
- **Wi-Fi devices:** one `MultiModelSpectrumChannel` per band (`m_channel24/5/6`). Backhaul: every node has a bBSS AP (MLD with links `m_bhChannels` = 5 GHz [+ 6 GHz]); every agent a bSTA (`m_stas[BstaIndex(a)]`). Fronthaul: per agent one tri-band AP MLD (links ordered 5, 6, 2.4 GHz — required, see the comment in `InstallFronthaulDevices`) or three single-link APs with `--fhMlo=0`; each link is one `EasyMeshRadio` (device + link ID). Clients `[0, m_numMldClients)` are Wi-Fi 7 (MLD with 5 + 6 GHz links when `m_fhMlo`), the rest single-radio Wi-Fi 6. `ConfigureLinks` sets per-link channel, power, CCA and band pinning. `m_bssMap` maps both MLD and per-link addresses to their owner.
- **BSS transitions:** every move (steering, roaming, channel-switch follow, re-parenting, agent failure) goes through `StartTransition(staIdx, TransitionTarget, ev)`: pin the STA to the target BSSIDs (`EasyMeshAssocManager`), park every PHY on an idle channel, hop to the target channels, wait for the `Assoc` trace (`OnStaAssoc`). `OnStaDeAssoc` calls `ForgetStaAtAp`, which resets the old AP's station record and BA agreements — ns-3 STAs send no Disassociation and nothing tears down BA, which otherwise aborts on re-association through another link of the same AP MLD. STA link IDs change at multi-link setup (they take the AP's IDs), so iterate `GetLinkIds()`, never `0..n-1`; device PHY indices do not change.
- **Addressing/routing:** backhaul `10.1.0.0/16` (Controller `10.1.0.1`, agent bBSS `10.1.(a+1).1`, bSTA `.2`); fronthaul `10.2.0.0/16` (AP MLD `10.2.4.(a+1)`, or per band `10.2.{1,2,3}.(a+1)`; clients `10.2.10+.x`). All routing is static; `RebuildRoutes()` must run after any association or topology change.
- **Logical model (`easymesh/lib/`):** `EasyMeshController`, `EasyMeshAgent`, `EasyMeshRadio`, `EasyMeshLink` are `SimpleRefCount` bookkeeping objects, not ns-3 Applications, and send no IEEE 1905 CMDUs. The controller decides (steering, band steering, load balancing, channel planning via `ChannelPlanner`) and acts only through `ControllerOps` callbacks the simulation provides. It knows each client's `ClientCaps` (Wi-Fi 7/6, MLO, bands): an MLO client is handled per AP MLD via its anchor (5 GHz) radio and skips band steering. Timers and thresholds are in `parameters.h`.
- **Traffic & stats:** UDP uplink CBR and downlink OnOff (TCP BulkSend exists but is not installed). `lib/traffic-stats.h` is header-only and collects FlowMonitor results after `Simulator::Run()`. Per-radio UL/DL bytes come from each link PHY's sniffer traces.
- **NetAnim:** `m_animation` is created in `BuildTopology` *before* any Wi-Fi device, because `AnimationInterface`'s constructor calls `GetChannel()` on every device, which aborts for a multi-band MLD.
- **Speed:** wide channels are slow in ns-3 (per-RU power tracking); 6 GHz backhaul defaults to 80 MHz, 320 MHz is ~4× slower. A default 15-agent 30 s run takes tens of minutes; use `--agents=5 --clients=6 --duration=20` for quick checks.

## Conventions

- When you add vectors indexed by agent or client, resize them in **both** the constructor and `SetNumAgents`/`SetNumClients`. The constructor sizes them from the `NUM_*` macros, and the setters resize them again for the CLI values.
- New `.cc` files under `lib/` must be added to `add_library(easymesh-lib ...)` in `easymesh/CMakeLists.txt`.
- Code style is ns-3 style: `m_` members, return type on its own line before `Class::Method`, and `NS_LOG_INFO` logging (the `EasyMeshSim` and `EasyMeshController` components are enabled at INFO level in `main`). Some comments are in Vietnamese.
