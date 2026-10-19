# AutoHaul — Architecture (Step 0)

Status: **approved** · 2026-10-19 (rev. 3: retargeted to game **1.53.2.1**; Phase 1 implemented, see [docs/PHASE1.md](docs/PHASE1.md))
Scope: Euro Truck Simulator 2 **1.53.2.1**, **single-player only**, Windows. AutoHaul never supports TruckersMP or Convoy, and it refuses to engage if it detects either one (§5.11).

---

## 0. Decisions

1. ~~Which game build to target~~ → **Decided: 1.53.2.1** (your current install). Every tool is pinned to a version from the 1.53 era (§1.2). Map and recording caches are keyed by game version, so moving to a newer build later means swapping the pins, not redesigning.
2. **Build approach (needs your OK).** I recommend a **standalone Python system that reuses ETS2LA's open low-level components**, not a plugin or fork of ETS2LA (§3.6).
3. **Approve the phase plan (needs your OK)** (§7). Phase 1 starts after your OK. It opens with 1.53 compatibility tests, so the riskiest assumptions are checked first.

---

## 1. Your installation (verified 2026-10-19)

| Item | Found |
|---|---|
| Game binary | `C:\Games\Euro Truck Simulator 2\bin\win_x64\eurotrucks2.exe` |
| Version | `1.53.2.1` (`game.log.txt`: "Loaded pack set version 1.53.2.1 created at 1733317869" = 2024-12-04) |
| Profile/home dir | `%USERPROFILE%\OneDrive\Documents\Euro Truck Simulator 2` (one local, offline profile) |
| Map DLC packs mounted | `dlc_east`, `dlc_north`, `dlc_fr`, `dlc_it`, `dlc_balt`, `dlc_iberia`, `dlc_balkan_w`, `dlc_balkan_e`, `dlc_greece` (+ trailer/truck/paint DLCs) |
| Mods | none (0 local, 0 workshop) |
| SDK plugins folder | `bin\win_x64\plugins\` **does not exist yet** (no telemetry plugin installed) |
| Input devices | Keyboard, mouse, `Controller (Xbox360 For Windows)` (XInput) |
| Display | 1920×1080, **exclusive fullscreen** (`r_fullscreen 1`) |
| PC | i5-13420H (8C/16T), 16 GB RAM, **RTX 4050 Laptop 6 GB** (driver 592.82) |
| Toolchain | Visual Studio Build Tools 2026 (MSVC 14.51, Windows SDK 10.0.26100) ✓, Python 3.12.10 + 3.14.6 ✓, Node.js 24.18 / npm 11.16 ✓, git ✓, GitHub CLI ✓, CMake + Ninja installed in the project venv (`.venv`), **.NET SDK missing** (only needed for the optional TruckLib cross-check) |

### 1.1 Scope
AutoHaul only uses the official, documented SCS SDK plus the pinned tools below; nothing in it depends on how the game was installed.

### 1.2 The 1.53 toolchain (pinned versions)
Current versions of several tools only support 1.59–1.61. Each one has a version from the 1.53 era that we pin instead. Pins live in `deps.lock.toml` (repo + commit SHA). `tools/fetch_deps.py` fetches them into `%USERPROFILE%\AutoHaul\third_party\`.

| Component | Current version (won't work on 1.53) | **1.53 pin** | What it gives us on 1.53 | Status |
|---|---|---|---|---|
| SCS SDK headers | 1.15 | **1.15** (backward compatible; the game negotiates down) | Telemetry API 1.01 with ETS2 telemetry game version **1.18**, input API 1.00 | ✓ by design |
| `mandatory.break` channel | patch 1.60+ | **not needed**: your 1.53 profile has only `g_fatigue` and no mandatory-break setting | fatigue = `rest.stop` only | n/a |
| Input device (semantical) | — | own DLL against SDK input 1.00 | all needed mixes confirmed in your `controls.sii` (§2.4) | ✓ verified in your files |
| **ETS2LA game-memory plugin** | `main` (patterns/structs for 1.61) | **`ETS2LA/plugin@c328dc38`** (2025-03-12), the last commit before "update data structures for 1.54" (2025-03-27) | 20 nearest traffic vehicles (pos, quat, size, speed, accel, up to 2 trailers), 20 nearest traffic lights/gates (state, time remaining, id), GPS route (≤5000 node UIDs with distance/time to end), camera (FOV, pos, quat) | ⚠ developed while 1.53 was current. **Tested 2026-10-19:** unmodified pin crashes the game (traffic-light reader, access violation). With patch 0002 (that reader off, others SEH-guarded): camera ✓, AI traffic ✓ (speeds match movement), route pending, traffic lights ✗ → vision |
| Map extraction | TruckLib 0.5.x (format 907) | **TruckSim Maps parser `truckermudgeon/maps@eca70d70`** (2025-02-17). It includes "parser,generator: support 1.53 release" (`79f43437`, 2024-11-30) and is the last parser change before "support 1.55 release" (`b7481735`, 2025-07-10) | roads + road looks (lane lists, shoulders), prefab descriptions with **AI nav curves** (start/end poses, next/prev links, per-node input/output lanes), spawn points, trigger points, POIs (gas, service, parking, weigh station, garages, dealers), companies, ferries/trains, country truck speed limits | ✓ **verified 2026-10-19**: full parse of your files in 193 s (516,930 nodes, 204,887 roads, 60,556 prefabs / 61,468 nav curves, 18,848 POIs) |
| Map cross-check (optional) | TruckLib 0.5.x | **TruckLib v0.1.7** (2025-05-21; map format **901** = game 1.51–1.54) | second, independent reader for `.mbd` and prefab `.ppd` | optional; needs .NET 8 SDK |

**Gaps the 1.53 pins leave, and how we fill them:**
- **Nav-curve traffic rules and semaphore links** (stop / give way / which light controls a lane) aren't in the parser's output. → A small patch to its `prefab-ppd-parser.ts` exports the `.ppd` nav-curve flags and semaphore IDs. The SCS prefab format documents those fields; Phase 2 confirms they're present in 1.53's `.ppd` version. Fallback: infer give-way from junction geometry and road class.
- **Fuel prices** aren't exported at that commit. → Extend the def parsing (country `.sii` files hold fuel prices). Fallback: a fixed per-country table in config.
- **Parked-vehicle data and the raise to 40 objects** only arrived in later plugin versions. On 1.53 we get **20** vehicles and **20** semaphores, which is enough at highway and urban ranges. Parked cars at the roadside are handled by vision in tight streets.
- **The plugin's steering hook** (MinHook on `physics_vehicle_u::steering_advance`) is the only code in it that changes the game. We build it with that feature **disabled** (one-line patch), so it is read-only. All control goes through the SDK input device.
- **Game updates:** this build doesn't update, so once the memory offsets pass the self-test they stay valid.

**If a pin fails its self-test (fallback ladder):**
| Fails | Fallback |
|---|---|
| Traffic block | Vision vehicle detection (YOLO) + conservative ACC (larger gaps, lower speed cap) |
| Semaphore block (**this happened**: reader crashes 1.53.2.1, disabled) | Vision traffic-light detector. Training crops come from projecting semaphore positions in the map data into the image, with the state labelled by colour. If confidence is low, hand control back at signalled junctions |
| Route block | Our own A* over the extracted lane graph to the job destination, cross-checked against `truck.navigation.distance` every second |
| Camera block | Camera pose from the truck pose + head offset + configured FOV (good enough for projecting labels) |
| Whole plugin crashes on load | Remove it. AutoHaul runs at `DEGRADED` (highway-only, vision) until we port the struct offsets ourselves: diff `prism/` headers between `c328dc38` and the 1.54 commit, then fix them against your binary |
| Map parser at `eca70d70` | TruckLib v0.1.7 exporter (C#), then our own exporter for `.mbd` + `.ppd` |

### 1.3 Profile settings that affect the design (from `profiles/<profile>/config.cfg`)
| Setting | Now | Plan |
|---|---|---|
| `r_fullscreen` | 1 (exclusive) | → **borderless window**, needed for the overlay and screen capture |
| `g_developer` / `g_console` | 0 / 0 | → 1 during development (enables `sdk reload` for hot-reloading the DLL) |
| `g_fatigue` | 1 | keep (rest planning depends on it) |
| `g_fuel_simulation` | 0 | verify in Phase 1 how consumption behaves; probably set to 1 for realistic fuel planning |
| `g_trailer_advanced_coupling` | 0 (simple) | keep (simple coupling is far more forgiving) |
| `g_parking_difficulty` | 0 | keep (easiest) |
| `g_lane_assistant` / `g_acc` | 0 / 0 | keep off (the game's own ADAS would fight our controller) |
| `g_automatic_headlights` | 0 | → 1 as a backstop against `no_lights` fines; AutoHaul still checks and controls lights |
| `g_use_speed_limiter` | 1 | keep |
| Steering options | — | non-linearity **0**, speed-sensitive steering **off**, sensitivity calibrated in Phase 1 |
| Transmission | — | **automatic** (AutoHaul only selects D/R/N; the game shifts) |

---

## 2. Research: SCS SDK 1.15 (official, released with patch 1.61)

Source: `https://download.eurotrucksimulator2.com/scs_sdk_1_15.zip` (MIT-style license, so we can vendor the headers). The headers are backward compatible: a plugin built against 1.15 runs on 1.53, which reports telemetry game version 1.18. The DLL checks that version at init and simply skips channels added later (`mandatory.break`, bus-job configs).

### 2.1 How plugins work
- The game loads every DLL in `bin\win_x64\plugins\` (or DLLs listed under `HKLM\SOFTWARE\SCS Software\Euro Truck Simulator 2\Plugins`).
- All SDK callbacks run on the **game main thread**. Calls back into the game are only legal inside those callbacks. → The DLL must only copy data into shared memory and return. No locks, no I/O.
- **Timing:** each physics step is one telemetry frame (`frame_start` … channel callbacks … `frame_end`). Input is processed at the **start of each render frame**. The physics runs a fixed step and stays at most one step ahead of rendering. → Telemetry runs at physics rate (≥60 Hz). Control latency is about one render frame.
- Console `sdk reload` / `sdk unload` hot-swaps plugins (needs developer console).
- Version negotiation: telemetry API 1.01, input API 1.00. Game-specific version: ETS2 telemetry 1.20 on 1.61, 1.18 on 1.53.

### 2.2 Telemetry fields AutoHaul needs → exact SDK names

| Need | Channel / config (exact id) | Units / notes |
|---|---|---|
| Position + heading | `truck.world.placement` (dplacement) | world metres (double). Heading in [0,1), 0 = north, CCW (0.25 = west). Pitch in ±0.25 |
| Velocity / accel | `truck.local.velocity.linear/angular`, `truck.local.acceleration.linear/angular` | vehicle frame |
| Speed | `truck.speed` | m/s, negative when reversing |
| Speed limit | `truck.navigation.speed.limit` | m/s. Follows the "Route Advisor speed limit" option; 0 = unknown |
| Gear / RPM | `truck.engine.gear`, `truck.displayed.gear`, `truck.engine.rpm`; config `gears.forward/reverse`, `rpm.limit` | |
| Fuel | `truck.fuel.amount`, `truck.fuel.range`, `truck.fuel.consumption.average`, `truck.fuel.warning`; config `fuel.capacity`, `fuel.warning.factor` | litres / km / l/km |
| AdBlue | `truck.adblue`, `truck.adblue.warning` | |
| **Fatigue** | `rest.stop` | **in-game minutes** until the next rest is needed |
| Mandatory break | `mandatory.break` | **not on 1.53** (channel added in telemetry 1.19 = patch 1.60; no matching setting in your profile). Registration fails gracefully and the code path stays dormant |
| Game time | `game.time` | in-game minutes since day 0 00:00 |
| Time/distance scale | `local.scale` | changes in cities and the UK. Needed to convert real↔game time |
| Job info | config `job`: `cargo`, `cargo.id`, `cargo.mass`, `source.city/company`, `destination.city/company`, `income`, `delivery.time`, `planned_distance.km`, `job.market`, `is.special.job`, `cargo.loaded` | income in EUR |
| **Deadline** | `delivery.time − game.time` | in-game minutes remaining |
| Trailer attached | `trailer.connected` (per trailer index, up to 10); config `hook.position` | |
| Trailer pose | `trailer.world.placement`, `trailer.velocity.*` | for articulation angle |
| Cargo damage | `job.cargo.damage`, `trailer.cargo.damage` | 0..1 |
| Navigation | `truck.navigation.distance` (m), `truck.navigation.time` (s) | game GPS values |
| Driver inputs (raw / effective) | `truck.input.steering/throttle/brake/clutch`, `truck.effective.*` | used for manual-override cross-check |
| Cruise control | `truck.cruise_control` | m/s, 0 = off |
| Brakes | `truck.brake.parking`, `truck.brake.motor`, `truck.brake.retarder`; config `retarder.steps`; `truck.brake.air.pressure(.warning/.emergency)`, `truck.brake.temperature` | |
| Lights / signals | `truck.lblinker`, `truck.rblinker`, `truck.hazard.warning`, `truck.light.beam.low/high`, `truck.light.parking`, `truck.light.brake/reverse`, `truck.light.beacon` | |
| Wipers | `truck.wipers` | |
| Engine / electrics | `truck.engine.enabled`, `truck.electric.enabled` | |
| Wheel steering angle | `truck.wheel.steering` | for steering calibration |
| Health | `truck.wear.*`, `trailer.wear.*`, `truck.oil.*`, `truck.water.temperature`, `truck.battery.voltage` | |
| Odometer | `truck.odometer` | km |

**Events:** `frame_start`, `frame_end`, `paused`, `started`, `configuration` (truck/trailer/job/controls changed), `gameplay`:
- `job.delivered` (revenue, earned.xp, cargo.damage, distance.km, delivery.time, **auto.park.used**, **auto.load.used**)
- `job.cancelled` (cancel.penalty)
- `player.fined` (`fine.offence` ∈ crash, avoid_sleeping, wrong_way, speeding_camera, no_lights, red_signal, speeding, avoid_weighing, illegal_trailer, avoid_inspection, illegal_border_crossing, hard_shoulder_violation, damaged_vehicle_usage, generic; `fine.amount`)
- `player.tollgate.paid`, `player.use.ferry`, `player.use.train`

These events are how we measure "zero fines" and €/km automatically.

### 2.3 What the SDK does *not* provide
Traffic vehicles · traffic-light states · road/lane geometry · the route's geometry (only distance/time) · POIs (fuel, rest, parking, tolls, weigh stations) · **weather/rain** · UI/dialog state · the position of an *uncoupled* trailer · camera pose. The sections below fill each of these gaps.

### 2.4 Input SDK (virtual device)
- Device types: `generic` (bindable in the game UI) or **`semantical`**: inputs map directly to game "mixes" by name, with no binding needed. Up to 400 inputs. Values are `bool` or `float`.
- Your `controls.sii` already has `semantical.<mix>?0` entries for every control we need, including:
  `steering`, `aforward` (throttle), `abackward` (brake), `clutch`, `gearup`, `geardown`, `geardrive`, `gearreverse`, `gear0..16`, `lblinker`, `rblinker`, `flasher4way`, `light`, `hblight`, `wipers`, `wipers0..4`, `parkingbrake`, `engine`, `engineelect`, `ignitionon/off/strt`, `cruiectrl(+inc/dec/res)`, `retarderup/down/0..5`, `motorbrake`, `engbrake*`, `horn`, `airhorn`, `beacon`, `activate` (interact/Enter), `attach` (couple), `quickpark`, `navmap`, `menu`, `pause`, `quicksave`, `selectfcs`/`back`/`j_up/down/left/right` (UI navigation), `embrake`, `diflock`, `liftaxle`, `cam1..8`.
- **So no vJoy/ViGEm is needed.** The SDK device is the primary and only output path. vJoy is kept only as a documented fallback.

---

## 3. Research: ETS2LA (Euro Truck Simulator 2 Lane Assist)

### 3.1 Current state (Oct 2026)
- The main app was **rewritten in C# (.NET 10)** and lives in `github.com/ETS2LA/ETS2LA` (GPL-3.0). The older Python V2 was superseded.
- The rewrite's devblog (Apr 2026) says it targets release in H2 2026. **The lane-keeping plugin is closed source.** The plugin API is "still in development and subject to change".
- The app is a host process: everything happens in plugins. The host provides telemetry, map access (via TruckLib), a controls abstraction (`ApplicationState`: SET / + / − / Assist), and an ImGui overlay. Its control model is driver assistance with a supervising human.

### 3.2 Components (the useful, open parts)
| Repo | What it is | Lang | License | Fragility |
|---|---|---|---|---|
| `ETS2LA/telemetry` | SDK telemetry → shared memory (ring of 4 snapshots, blocking `get_telemetry`), fork of truckermudgeon/scs-sdk-plugin | C++ | GPL-3.0 | low (official SDK) |
| `ETS2LA/controller` | **Semantical input device** + shared memory (`Local\ETS2LA_Controller`, 287 inputs, per-frame events, `commit()` heartbeat; releases all inputs after **3 s** without a commit) + Python client | C++/Py | GPL-3.0 | low (official SDK) |
| `ETS2LA/plugin` | **Game-memory reader** (derived from ts-extra-utilities). Current `main`: 40 nearest traffic vehicles (pos, rot, size, speed, accel, trailers), 40 parked vehicles, 40 nearest **traffic lights/gates with state + time remaining**, **navigation route** (node UIDs, ≤6000 items), camera matrices, plus a steering/accel override input. Exposed as `Local\ETS2LA*` shared memory. **1.53 pin `c328dc38`:** 20 vehicles, 20 semaphores, ≤5000 route items, camera, no parked vehicles (§1.2) | C++ | **MIT** | **high**: byte-pattern scans of engine structs, tied to game version |
| `ETS2LA/capture` | In-process frame capture | C++ | GPL-3.0 | medium |
| TruckLib (sk-zk) | Reads `.scs` HashFS + `.mbd` map + prefab `.ppd` | C# | GPL-2.0 | current: format 907 = 1.59–1.61. **1.53 pin v0.1.7** (format 901 = 1.51–1.54) |
| TruckSim Maps parser (truckermudgeon) | CLI: game dir → JSON (nodes, roads, prefabs with AI nav curves, companies, POIs…) | TS/Node + native | GPL-3.0 | tracks current releases. **1.53 pin `eca70d70`** |

### 3.3 How ETS2LA gets map/road data
Originally: pre-extracted JSON from a ts-map fork, modified to export **prefab AI navigation curves** (the lanes AI traffic drives on), then turned into a road graph. Now: TruckLib reads the game files directly for the installed version and mods.

### 3.4 How it controls the truck
Through the SDK **semantical input device** (the controller DLL). Steering and pedals are float axes, buttons are pulses or holds, with a heartbeat watchdog. The memory plugin can also write steering/accel directly into game memory. AutoHaul will **not** do that: SDK-only output keeps the game in charge of physics and stays stable across updates.

### 3.5 How it detects traffic and traffic lights
It reads **ground truth from game memory** (traffic actors and semaphore instances), not vision. Earlier versions used screen capture with colour filtering and a small CNN/YOLO for traffic lights. Memory data is exact (positions, sizes, light state, time to change) but fragile across updates.

### 3.6 Build on top of ETS2LA vs. reuse its ideas

| | A. ETS2LA C# plugin | B. Fork Python V2 | **C. Standalone AutoHaul (Python) + reuse ETS2LA low-level parts** |
|---|---|---|---|
| Language fit (you asked for Python) | ✗ C#/.NET 10 | ✓ | ✓ |
| Stability of the base | ✗ API "subject to change", beta | ✗ superseded, unmaintained | ✓ we own everything above shared memory |
| Lane keeping | closed-source core | old code | our own (map-based) |
| Mission logic (fuel, rest, deadline, coupling, parking) | not in scope for ETS2LA | not present | ✓ designed in from day 1 |
| Safety control (watchdog, override, kill) | their model ("SET" toggles, 3 s watchdog) | old | ✓ ours: ≤250 ms watchdog, in-DLL kill key |
| Game data (traffic/lights/route) | ✓ built in | ✓ | ✓ reuse `ETS2LA/plugin` (MIT) as-is |
| Map extraction | ✓ TruckLib | old data pipeline | ✓ reuse TruckSim Maps parser / TruckLib as an offline tool |
| **Works on 1.53** | ✗ the C# app depends on current TruckLib (1.59–1.61 only) | ⚠ only with old data | ✓ every dependency pinned to a 1.53-era commit (§1.2) |
| Effort | lowest at first, highest coupling | high (big legacy codebase) | medium |

**Recommendation: C.** On 1.53 it's also the only option that works, because ETS2LA's current app can't load a 1.53 map. We reuse the exact, fragile, hard-won parts (game-memory reader, map parsing) as separate binaries or tools, and we build the driving stack and mission stack in Python where we control safety. Telemetry and the input device are small enough (~600 lines of C++) that **we write our own combined `autohaul_bridge.dll`**. That gives us one layout, a fast watchdog, a kill key handled inside the game process, and no GPL linkage. GPL components (map parser) run as separate tools, so licensing stays simple. For personal use, GPL has no practical effect either way.

---

## 4. System overview

```
┌──────────────────────────── eurotrucks2.exe (game process) ────────────────────────────┐
│  bin/win_x64/plugins/                                                                  │
│   ├─ autohaul_bridge.dll  (ours, C++, SDK only)                                        │
│   │    • telemetry → SHM "Local\AutoHaul_Telemetry" (seqlock ring, frame event)        │
│   │    • semantical input device ← SHM "Local\AutoHaul_Controls"                       │
│   │    • watchdog: no heartbeat 250 ms → release all inputs                            │
│   │    • kill key polled every input frame → release all + latch DISABLED              │
│   └─ ets2la_plugin.dll    (ETS2LA @c328dc38, MIT, read-only build): traffic, lights,   │
│                            route, camera                                               │
└──────────────▲───────────────────────────────▲─────────────────────────────────────────┘
               │ shared memory (zero-copy)     │
┌──────────────┴───────────────────────────────┴───── autohaul (Python 3.12) ────────────┐
│  io/        telemetry reader · gamedata reader · controls writer · recorder            │
│  world/     map DB (offline-extracted) · localization · route · tracked objects        │
│  perception/ (fallback) screen capture → YOLO (lights, vehicles) · rain/night · UI OCR  │
│  planning/  behaviour FSM (lane follow/change/junction/stop) · speed planner · path    │
│  control/   lateral (pure pursuit → MPC) · longitudinal (ACC) · reverse/coupling       │
│  mission/   job FSM · fatigue · fuel · economics/decision log · UI action scripts      │
│  safety/    supervisor · override detector · kill switch · confidence gate             │
│  ui/        overlay (transparent click-through window)                                 │
└────────────────────────────────────────────────────────────────────────────────────────┘
        ▲ offline: tools/extract_map (TruckSim Maps parser @eca70d70, patched)
                   → data/map/1.53.2.1/*.json → *.sqlite
```

### 4.1 Process and loop rates
| Loop | Rate | Process |
|---|---|---|
| Telemetry ingest, control, safety supervisor | every physics frame (~60 Hz), woken by the DLL's named event | main |
| Behaviour + path planning | 20 Hz | main (deadline-bounded; on overrun the last plan is reused) |
| Mission manager, economics | 1 Hz + on events | main |
| Vision (fallback) | 20–30 Hz | separate process (GIL), results via SHM |
| Overlay | 30 Hz | separate process |
| Recorder | every frame, async writer thread | main |

Python 3.12 is the default: it has the widest CUDA/ML wheel support. You have both 3.12 and 3.14 installed.

---

## 5. Module design

### 5.1 Game bridge (`native/bridge`, C++/MSVC/CMake)
- Telemetry: register channels at `configuration` events. Fill a fixed-layout `#pragma pack` struct during the frame. At `frame_end`, publish it to a **seqlock ring (8 slots)** and `SetEvent("Local\AutoHaul_Frame")`. Configs (job/truck/trailer) and gameplay events go to a separate versioned block with an event queue (so fines/deliveries are never missed).
- Input: one semantical device, `autohaul`, with the inputs listed in §2.4. The Python side writes a command block (axes as float, buttons as hold/pulse counters, `seq`, `heartbeat_qpc`). The device's event callback emits only changed values (and pulses).
- **Fail-safes inside the game process:** heartbeat older than 250 ms → all axes 0, all buttons released. Kill key (default `Pause/Break`, configurable) checked with `GetAsyncKeyState` on every input frame → release all and latch `disabled` until Python re-arms it. Telemetry `paused` → hold everything released.
- Struct layout is generated from one spec file (`native/spec/layout.toml`) into both the C++ header and the Python `ctypes` definition, so the two can't drift apart.

### 5.2 Game-data reader (`io/gamedata.py`)
Reads the 1.53-pin blocks: `Local\ETS2LATraffic` (2600 B: 20 × vehicle + 2 trailers), `Local\ETS2LASemaphore` (1040 B: 20 × lights/gates), `Local\ETS2LARoute` (80 000 B: 5000 × `node_uid, distance_to_end, time_to_end`), `Local\ETS2LACameraProps` (36 B). The layouts are defined once in `native/spec/ets2la_c328dc38.toml`, so a later pin bump only changes that file. Each block is validated: change detection (these blocks carry no timestamp, so "stale" means unchanged for N frames while the truck moves), and sanity checks (positions near the truck, quaternions normalized, route distance decreasing). If the plugin is missing or stale, `world` marks those sources **unavailable** and the confidence gate (§5.11) limits what AutoHaul may do: highway ACC only with vision, no urban driving.

### 5.3 Map database (`world/mapdb.py`, `tools/extract_map.py`)
- Offline: run the **pinned TruckSim Maps parser (`eca70d70` + our small patch for nav-curve flags, semaphore IDs and fuel prices)** on `C:\Games\Euro Truck Simulator 2` → JSON → our **SQLite + numpy** cache in `data/map/1.53.2.1/`. Node 24 is installed. If the parser's native addon won't build on Node 24, `tools/extract_map.py` uses a portable Node 22 LTS (no system install). Contents: road segments with lane centerlines, prefab **AI nav curves** with connectivity (in-node → out-node, lane index, traffic-rule flags), semaphores, signs where available, POIs (fuel stations, rest areas/truck stops with sleep, parking, company depots with parking spots, tolls, weigh stations, ferries/trains, borders).
- Runtime: R-tree spatial index, lane graph (`networkx`, or a compact custom graph for speed).

### 5.4 Localization and route (`world/localization.py`, `world/route.py`)
- Pose comes straight from telemetry (exact), so we need **map matching**, not SLAM: find the lane polyline that best fits position + heading + history (HMM-style over candidate lanes). Output: lane id, lateral offset, along-track distance, confidence.
- Route: the game GPS route as node UIDs from the memory plugin → a sequence of map edges → a **lane-level route** (which lanes allow the required exit/turn). Fallback when route data is missing: our own A* over the lane graph to the job destination, checked against `truck.navigation.distance`.

### 5.5 Objects and perception (`world/objects.py`, `perception/`)
- **Primary: ground truth** from the memory plugin. Tracked vehicles get lane assignment, gap, and relative speed. Traffic lights are matched to the prefab nav curve they control (state + time to change).
- **Fallback / complement (vision):** `dxcam` capture (borderless window) → YOLO-nano (Ultralytics), trained on auto-labelled frames. Labels are projected from ground truth through the camera matrices the plugin exposes, so no manual labelling is needed. Uses:
  - vehicles and lights when memory data is unavailable;
  - **rain detection** for wipers (not in telemetry or memory data);
  - **UI-dialog verification** (template match / OCR) for refuel, sleep, and delivery screens.
- Pedestrians/animals: ETS2 spawns few on roads; vision covers them later as a safety layer.

### 5.6 Behaviour planning (`planning/behavior.py`)
A hierarchical state machine per tick:
`LANE_FOLLOW → PREPARE_LANE_CHANGE → LANE_CHANGE → JUNCTION_APPROACH → JUNCTION_TRAVERSE → STOP_LINE_WAIT → TOLL → WEIGH_STATION → MERGE`.
- **Rules, as hard constraints:**
  - speed ≤ limit − margin (default max(3 km/h, 3 %));
  - stop at red/amber when stoppable at ≤ 2.5 m/s²;
  - signal ≥ 3 s before a lane change and ≥ 50 m before a turn;
  - no lane change across a solid line (lane-boundary type from road look data; unknown counts as solid);
  - never use the hard shoulder;
  - lights on at night or in rain, wipers in rain.
- Give-way and priority from prefab nav-curve rule flags. Yield to any tracked vehicle whose predicted path conflicts within a time window.
- Roundabouts are a prefab type: the lane-level route picks the exit lane; give way on entry.
- Tolls: the map POI says whether the booth needs a stop and interaction (`activate`) or is pass-through. Weigh stations: follow signage lanes, stop on the scale, continue when cleared (verified via fine events during testing).

### 5.7 Speed planning (`planning/speed.py`)
`v_target = min(limit − margin, sqrt(a_lat_max / κ(s)) over the preview horizon, ACC(lead), stop-line profile, user cap, mission cap)`.
- `a_lat_max` depends on load: 1.5 m/s² loaded, 2.0 m/s² empty (tunable).
- Jerk-limited profiles to protect cargo.
- On descents (pitch from placement): use the retarder first, then the engine brake, keeping service brakes for stopping. Watch `truck.brake.temperature` and air pressure.

### 5.8 Control (`control/`)
- **Lateral:** pure pursuit on the tractor rear-axle reference, with speed-scheduled lookahead `L = clamp(k·v + L0)`. Feed-forward from path curvature. Trailer off-tracking compensation at junctions (the path is shifted outward using a kinematic tractor–semitrailer model so the *trailer* clears kerbs). Phase 3+: linear MPC on the kinematic model if pure pursuit isn't precise enough in tight urban turns.
- **Steering calibration** (Phase 1 tool): map command → `truck.wheel.steering` at several speeds, then fit an inverse curve.
- **Longitudinal:**
  - PI speed controller with feed-forward from grade and mass.
  - ACC: time gap 2.5 s (3.0 s wet), standstill gap 8 m, TTC guard → emergency braking.
  - Brake/throttle are never both applied.
  - Automatic gearbox: AutoHaul only commands D/R/N and the parking brake.
- **Reverse and coupling (Phase 5, separate modules):** a cascaded controller: outer loop = path tracking for the **trailer axle**, inner loop = hitch-angle control (LQR on the linearized reversing model), with jack-knife limits. Coupling = align with the trailer axis 15–25 m ahead of the kingpin, reverse along the axis, hit `attach` when inside the coupling window, confirm with `trailer.connected`. All of this is developed against `sim/` first.

### 5.9 Mission manager (`mission/fsm.py`)

```
            job config event (cargo set, trailer not connected)
IDLE ──────────────────────────────────────────────► ARMING (5 s cancellable countdown)
                                                          │
                                                          ▼
TO_TRAILER ──(within 40 m of trailer)──► COUPLING ──(trailer.connected)──► TO_DEST
     ▲                                                                      │
     │                                     (within delivery area / company) ▼
     │                                                         PARKING / DELIVERY
     │                                                                      │
     └──────────── job.delivered / job.cancelled ◄────── DONE ◄─────────────┘

Interrupts (from any driving state, resume afterwards):
  REFUEL  – planned stop at fuel POI → stop at pump → activate → fill → confirm → resume
  REST    – planned stop at sleep POI → park → activate → sleep → confirm → resume
  RECOVERY– stuck / off-route / blocked → slow → re-plan → (fail) SAFE_STOP + alert
  HANDOVER– manual override, kill switch, low confidence, ferry/train terminals (initially)
```
Every transition is logged with the reason and a telemetry snapshot.

### 5.10 Fatigue, fuel, deadline and profit (`mission/fatigue.py`, `fuel.py`, `economics.py`)
- All time is **in-game minutes**. Real↔game conversion uses `local.scale`.
- **Fatigue:** when `rest.stop` drops below the planning horizon (on 1.53 that's the only fatigue clock; `mandatory.break` support stays dormant) (default 180 game-min), list sleep POIs along the remaining route, estimate the ETA to each, and keep those with `ETA < rest.stop − buffer (60 min)`. Driving past `rest.stop` is never an option: it risks `avoid_sleeping` fines and in-game microsleep, so it is a hard constraint and not a trade-off.
- **Fuel:** refuel needed if `truck.fuel.range` < (distance to the next reachable station beyond the current one) + reserve (default 150 km). If a cheaper station (by country fuel price from def data) is within a detour budget, prefer it. Combine fuel with rest at truck stops when both are within the horizon.
- **Profit model, per option** (rest at A/B/C, refuel at X/Y, combined):
  `EV = income·f_late(lateness) − fuel_cost − tolls − E[fine] − damage_loss − time_value·duration`
  - `f_late` is configurable and **calibrated from your `job.delivered` events** (revenue vs income vs arrival time). I won't hard-code a penalty formula I can't verify.
  - `time_value` = rolling €/game-hour from past jobs.
  - Options that break a rule are discarded before scoring.
- **Decision log:** `logs/decisions.jsonl`, one record per decision: inputs, every candidate with its numbers, the choice, and a one-line rationale.

### 5.11 Safety (`safety/`)
- **Kill switch, two independent paths:**
  1. In-DLL key poll, inside the game, which works even if Python hangs.
  2. Python global hotkey (`RegisterHotKey`), which also stops the mission.
- **Manual override, three detectors, any one of which hands control back in the same frame:**
  1. XInput poll of the physical Xbox controller (stick/trigger beyond a deadzone);
  2. low-level keyboard hook (driving keys);
  3. telemetry cross-check: `truck.input.*` deviates from our last command beyond tolerance (catches wheels/pedals and any other device).
- **Supervisor checks every tick:**
  - telemetry age < 50 ms;
  - game not paused;
  - **not multiplayer**: `multiplayer.time.offset` ≠ 0 or a Convoy session or `core_ets2mp.dll` loaded → refuse to engage;
  - lateral acceleration, speed vs limit, gap vs minimum, TTC.
  - A violation triggers braking or a safe stop, regardless of what the planner wants.
- **Confidence gate:** localization, route, object-data and perception confidence feed a level:
  - `FULL` → normal driving;
  - `DEGRADED` → speed cap, no lane changes, larger gaps;
  - `LOW` → controlled stop in lane, hazards on, parking brake, HANDOVER + alert (sound + Windows toast + overlay).
  - AutoHaul never guesses at an unknown junction.

### 5.12 Overlay (`ui/overlay.py`)
A transparent, click-through, always-on-top window (needs borderless mode). Shows: state, mission phase, target vs actual speed, limit, next action + distance, fuel / range, `rest.stop`, ETA vs deadline, confidence level, override/kill status. Phase 6 option: move it in-process (DX11) if it ever conflicts with the game.

### 5.13 Engineering
- **Config:** `config/autohaul.toml` (stdlib `tomllib`), validated with pydantic. Every tunable lives there with its units in the key name (`acc_time_gap_s`, `speed_margin_kmh`).
- **Logs:** structlog JSON lines, one file per run (`logs/<run_id>/`).
- **Recordings:** every telemetry frame, game-data block, command and planner output → `recordings/<run_id>.arrow` (Apache Arrow IPC, zstd).
  - `tools/replay.py` feeds a recording back through the stack (open loop) to debug planners offline, deterministically.
  - `tools/inspect_recording.py` plots runs.
- **Simulation:** `sim/` holds a kinematic tractor–semitrailer model + scenario runner for unit-testing the lateral, reverse and coupling controllers without the game.
- **Tests:** pytest. Unit tests per module; golden-file tests on recorded runs; property tests for the reversing controller (no jack-knife over randomized start poses).

---

## 6. Repository layout
```
ETS2_AutoHaul/                             (OneDrive: source only)
  ARCHITECTURE.md  README.md  pyproject.toml
  deps.lock.toml                           (pinned 1.53-era deps: repo + SHA + local patches)
  config/autohaul.toml                     (local overrides: config/local.toml, git-ignored)
  docs/PHASE1.md ...                       (install + test guide per phase)
  native/
    CMakeLists.txt
    third_party/scs_sdk_1_15/              (vendored SDK headers, MIT-style)
    spec/layout.toml  spec/inputs.toml     (shared-memory layout + input device: source of truth)
    spec/ets2la_c328dc38.toml              (ETS2LA plugin block layouts for the 1.53 pin)
    bridge/src/bridge.cpp                  (autohaul_bridge.dll)
    bridge/generated/autohaul_layout.h     (generated by tools/gen_layout.py)
    tests/fake_game.cpp                    (plays the game's side of the SDK for tests and dry runs)
  patches/<dep>/*.patch                    (local patches applied by fetch_deps.py)
  autohaul/
    io/ safety/ infra/                     (Phase 1)
    world/ perception/ planning/ control/ mission/ ui/ sim/   (Phases 2-6)
  tools/  fetch_deps  gen_layout  build_native  install_plugin  extract_map
          dashboard  selftest_gamedata  phase1_check  replay
  tests/

%USERPROFILE%\AutoHaul\                     (cache_dir: large generated data, not synced by OneDrive)
  third_party/ets2la_plugin  third_party/trucksim_maps  (pinned checkouts)
  build/out/                               (autohaul_bridge.dll, ets2_la_plugin.dll, fake_game.exe)
  map/1.53.2.1/raw/                        (parser output, 408 MB)
  recordings/  logs/  calibration/
```
The cache is deliberately not under `AppData`. Packaged apps (such as the Claude desktop app this was built from) get `AppData` writes redirected into private storage that your own terminal can't see.

---

## 7. Phases (each ends with a demo you can run)

| # | Deliverable | Demo / acceptance |
|---|---|---|
| **1** | **1.53 compatibility tests first:** build the pinned `ets2la_plugin@c328dc38` (read-only) and run `selftest_gamedata.py` in-game (traffic, lights, route, camera each PASS/FAIL), plus a smoke run of the pinned map parser on your game files. Then: `autohaul_bridge.dll` (telemetry + semantical input + watchdog + in-DLL kill), Python reader/writer, recorder/replay, kill switch, override detectors, steering calibration tool, multiplayer guard | Self-test report for each 1.53 pin; any FAIL switches to its fallback in §1.2 before Phase 2. DLL loads (visible in `game.log.txt`). Live telemetry dashboard ≥60 Hz with jitter stats. Parked "wiggle test": steering sine, blinkers, lights, wipers, horn. Kill key releases within 1 frame. Killing Python releases within 250 ms. Touching the controller hands back control. A recorded session replays offline. |
| **2** | Map extraction + map DB, localization, highway lane keeping, ACC, speed-limit compliance, lights/wipers | 50 km highway drive with traffic and you supervising: lateral error RMS < 0.3 m, speed ≤ limit − margin, safe gaps, no fines |
| **3** | Lane-level routing from game GPS, lane changes with signalling, junctions, roundabouts, traffic lights, give-way, tolls, weigh stations | City-to-city drive (no job) to a GPS waypoint, including urban junctions. Zero fines, zero collisions over N runs |
| **4** | Mission manager, fatigue/fuel planning, economics + decision log, UI scripts for refuel and sleep | Long job with at least one planned refuel and one sleep, decisions logged with reasoning |
| **5** | Coupling and reversing modules (sim-tested first), delivery parking using the easiest option / auto-park | Full job: accept → couple → drive → deliver, repeated across several company yards |
| **6** | Unattended long runs, tuning, metrics dashboard (fines, damage, €/km, €/game-day), recovery hardening | Multi-job sessions with metrics report |

After each phase you'll get: exact install steps, test steps, and a list of what's still missing.

---

## 8. Risks and open questions
1. **1.53 pins are best-effort** (§1.2). `ets2la_plugin@c328dc38` and `trucksim_maps@eca70d70` were developed while 1.53 was the current release. Nobody has confirmed them against your exact build, 1.53.2.1, and I can't test them without the game running. Phase 1 tests them before anything is built on top, and each has a fallback.
2. **Memory-reader fragility:** struct offsets are version-specific. On a build that doesn't update, offsets that pass the self-test stay valid. AutoHaul still re-runs the self-test at every start and drops to `DEGRADED` (vision) if a block goes bad.
3. **Unmaintained pins:** no one fixes bugs in 1.53-era commits any more. We carry small local patches (`patches/<dep>/`). Fixes from later upstream commits, e.g. `ETS2LA/plugin` "Fix crashes and freezes" (2025-12-13), get reviewed and backported only if they don't depend on post-1.53 structs.
4. **UI automation** (refuel/sleep/delivery dialogs, ferry menus) is the most brittle part. Mitigation: minimal key sequences, screen verification, hand back control on mismatch. Ferries/trains start as HANDOVER.
5. **Weather isn't exposed:** rain comes from vision (or a later memory-read extension).
6. **Exact behaviour of `quickpark`, auto-park and auto-load** in your version must be confirmed in-game (Phase 5). `auto.park.used` / `auto.load.used` in `job.delivered` let us verify it.
7. **"Zero crashes" is the target, not a guarantee.** ETS2 AI traffic sometimes behaves badly (e.g. AI rear-ending you). We measure it with `player.fined`/collision events and drive defensively (bigger gaps, low urban speeds).
8. **How the game mixes inputs** when both the physical controller and our device are active needs testing in Phase 1. It affects override detection #3.
9. **Missing build tools:** CMake + Ninja go into the project venv (`pip install cmake ninja`), so nothing is installed system-wide. The .NET 8 SDK is needed only if the TruckLib cross-check is used.

---

## 9. Sources
- SCS telemetry SDK wiki and SDK 1.15 archive (headers read directly): https://modding.scssoft.com/wiki/Documentation/Engine/SDK/Telemetry · https://download.eurotrucksimulator2.com/scs_sdk_1_15.zip
- ETS2 update history: https://simulationdaily.com/news/euro-truck-simulator-2-update-1-61/ · https://www.pcgameshardware.de/Euro-Truck-Simulator-2-Spiel-22781/News/Update-1-60-Open-Beta-Release-1544528/
- ETS2LA: https://github.com/ETS2LA · https://github.com/ETS2LA/ETS2LA · https://github.com/ETS2LA/plugin · https://github.com/ETS2LA/controller · https://github.com/ETS2LA/telemetry · https://github.com/ETS2LA/official-plugins · https://docs.ets2la.com/blog/devblog-april-2026 · https://github.com/ETS2LA/website (FAQ, VTC policy)
- Map tooling: https://github.com/sk-zk/TruckLib · https://github.com/truckermudgeon/maps · https://github.com/dariowouters/ts-extra-utilities
- 1.53 pins:
  - https://github.com/ETS2LA/plugin/commit/c328dc38 (last commit before https://github.com/ETS2LA/plugin/commit/d2421468 "update data structures for 1.54")
  - https://github.com/truckermudgeon/maps/commit/eca70d70 (after https://github.com/truckermudgeon/maps/commit/79f43437 "support 1.53 release")
  - https://github.com/sk-zk/TruckLib/releases/tag/v0.1.7 (README: map format 901, game 1.51–1.54)
- 1.54 release timing (early Apr 2025, after open beta): https://traxion.gg/euro-truck-simulator-2s-v1-54-update-released-implements-new-physics-and-overhauled-cargo-system/
