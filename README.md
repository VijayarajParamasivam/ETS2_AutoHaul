# AutoHaul

Autonomous job runner for **Euro Truck Simulator 2, single-player only** (target build 1.53.2.1). After you accept a job, AutoHaul drives to the trailer, couples it, drives to the destination and delivers. It plans fuel and rest stops, and weighs deadlines against fines and profit.

- Design: [ARCHITECTURE.md](ARCHITECTURE.md)
- **Phase 1 complete:** telemetry, control output and kill switch, verified in-game on 1.53.2.1. See [docs/PHASE1.md](docs/PHASE1.md).
- **Phase 2 complete:** lane keeping, speed, following distance, tolls and closed roads, driven in-game. See [docs/PHASE2.md](docs/PHASE2.md).
- **Phase 3 in progress:** routing to the job's depot, following the route through exits and interchanges, and automatic lane changes all work in-game. Junction rules from the map (give way, right of way, stop signs, turn signals) are built and simulated on the real map; traffic lights and tolls come next. See [docs/PHASE3.md](docs/PHASE3.md).
- **Phase 4 built, simulated:** the mission manager runs the active job (to the trailer, then to the destination), plans fuel and rest stops (per-country fuel prices, deadline vs cost), refuels and sleeps by itself, and logs every decision with its reasons. Coupling and parking stay with the driver until Phase 5. See [docs/PHASE4.md](docs/PHASE4.md).

## Layout
| Path | |
|---|---|
| `native/` | `autohaul_bridge.dll` (SCS SDK telemetry + virtual input device), fake-game test harness, layout spec, vendored SCS SDK 1.15 headers |
| `autohaul/` | Python package: `io` (shared memory, recorder), `safety` (kill switch, override, multiplayer guard), `world` (lane map, tracker, lane graph, routing, junctions, junction rules, places), `control` (steering, speed, lane changes, give way), `drive` (autopilot), `mission` (job, stop planning, economics, refuel/sleep scripts, decision log), `sim` (truck and game simulator), `infra` |
| `tools/` | build, fetch, install, dashboard, self-test, Phase 1 checks, replay, map extraction; `drive.py` (AutoHaul drives; `--mission` runs the job), `shadow_drive.py`, `sim_drive.py`, `sim_mission.py` (a whole job in the simulator), `steer_ident.py`, `pedal_check.py`, `validate_lanes.py` |
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

## Run

Double-click **`AutoHaul.bat`** (or a desktop shortcut to it). The launcher window offers these modes:

| Mode | |
|---|---|
| Job (full mission) | runs the active job: to the trailer, couples it, to the destination, quick park; fuel and rest stops |
| Drive to a place | follows the route to a `CITY` or `CITY:COMPANY` you type |
| Follow the road | lane keeping and speed, no route |
| Steer only / Speed only | AutoHaul does one axis, you do the other |

Press **Start**, click the game, and press `/` to engage. Press `/` again (or any driving key) to take over. **Stop** hands the truck back and quits. The launcher runs `tools/drive.py` (`--mission`, `--to`, `--no-route`, `--mode`), which also works from a terminal.

**Overlay** (`tools/overlay.py`, "Show overlay"): a see-through panel on the game showing what AutoHaul reads, what it's doing and why. Clicks go through it, it never takes the focus, and the traffic-light reader doesn't see it.

**Learning** (`autohaul/learn`, "Learn while driving"): AutoHaul adjusts its curve speed, steering balance, correction strength, braking and pedal values to the truck, per truck state, in small steps inside hard bounds. It slows down at places where you've had to take over, corrects its routes where the game's GPS disagrees, and goes back to earlier values if drives get clearly worse. Every change and its reason is in `learned/learning.jsonl` under the cache folder; `tools/learn_replay.py --report` shows what it knows; "Reset learning" forgets it. See [docs/LEARNING.md](docs/LEARNING.md).

**Wipers and headlights**: while AutoHaul drives, the wipers follow the rain the game records in its autosaves (so up to 5 minutes late), and the low beam is on from 20:00 to 07:00 game time and off by day. After you switch either yourself, AutoHaul leaves it alone for a while.
