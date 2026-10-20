# AutoHaul

Autonomous job runner for **Euro Truck Simulator 2, single-player only** (target build 1.53.2.1). After you accept a job, AutoHaul drives to the trailer, couples it, drives to the destination and delivers. It plans fuel and rest stops, and weighs deadlines against fines and profit.

- Design: [ARCHITECTURE.md](ARCHITECTURE.md)
- **Phase 1 complete:** telemetry, control output and kill switch, verified in-game on 1.53.2.1. See [docs/PHASE1.md](docs/PHASE1.md).
- **Phase 2 complete:** lane keeping, speed, following distance, tolls and closed roads, driven in-game. See [docs/PHASE2.md](docs/PHASE2.md).
- **Phase 3 in progress:** routing to the job's depot, following the route through exits and interchanges, and automatic lane changes all work in-game. Junction rules (traffic lights, give way) and tolls come next. See [docs/PHASE3.md](docs/PHASE3.md).

## Layout
| Path | |
|---|---|
| `native/` | `autohaul_bridge.dll` (SCS SDK telemetry + virtual input device), fake-game test harness, layout spec, vendored SCS SDK 1.15 headers |
| `autohaul/` | Python package: `io` (shared memory, recorder), `safety` (kill switch, override, multiplayer guard), `world` (lane map, tracker, lane graph, routing, junctions, places), `control` (steering, speed, lane changes), `drive` (autopilot), `sim` (truck simulator), `infra` |
| `tools/` | build, fetch, install, dashboard, self-test, Phase 1 checks, replay, map extraction; `drive.py` (AutoHaul drives), `shadow_drive.py`, `sim_drive.py`, `steer_ident.py`, `pedal_check.py`, `validate_lanes.py` |
| `config/autohaul.toml` | every tunable; machine overrides go in `config/local.toml` |
| `deps.lock.toml` | pinned 1.53-era third-party dependencies |
| `%USERPROFILE%\AutoHaul\` | large generated data kept out of OneDrive: third-party checkouts, builds, map data, recordings, logs |

## Setup (once)

```bash
py -3.12 -m venv .venv
```

```bash
.venv/Scripts/python.exe -m pip install -e ".[dev]"
```

```bash
.venv/Scripts/python.exe tools/fetch_deps.py
```

```bash
.venv/Scripts/python.exe tools/build_native.py
```

```bash
.venv/Scripts/python.exe -m pytest
```

Requires Visual Studio Build Tools (C++ workload), Python 3.12, Node.js (map extraction only) and git.

AutoHaul never supports TruckersMP or Convoy, and refuses to engage when it detects either one.
