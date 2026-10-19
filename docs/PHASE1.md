# Phase 1: telemetry, control output, kill switch

Target: ETS2 **1.53.2.1**, single-player. Status 2026-10-19: built and tested without the game; waiting for in-game verification.

## What Phase 1 delivers

| Piece | Where | Purpose |
|---|---|---|
| `autohaul_bridge.dll` | `native/bridge/` | Runs inside the game. Telemetry → shared memory at physics rate (183 channels, job/truck/trailer config, fines/deliveries). A virtual SDK input device with 8 axes + 77 buttons. Fail-safes: **250 ms watchdog**, **kill key handled inside the game**, everything released while paused. |
| `ets2_la_plugin.dll` (pinned `c328dc38`, read-only build) | fetched into `%USERPROFILE%\AutoHaul\third_party` | Traffic, traffic lights, GPS route and camera from game memory. Steering hook removed. |
| Python I/O | `autohaul/io/` | Telemetry reader (seqlock ring, no torn frames), controls writer (heartbeat), ETS2LA block reader, recorder/replay. |
| Safety | `autohaul/safety/` | Kill switch (keyboard hook + Xbox BACK+START + the DLL's own latch), manual override (driving keys, stick/triggers, telemetry cross-check), multiplayer guard (Convoy offset, TruckersMP DLL). |
| Tools | `tools/` | `dashboard.py`, `selftest_gamedata.py`, `phase1_check.py`, `install_plugin.py`, `replay.py`, `extract_map.py`, build tools. |
| Map data | `%USERPROFILE%\AutoHaul\map\1.53.2.1\raw` | Your 1.53 map, parsed with the pinned TruckSim Maps parser (used from Phase 2 on). |

### Already verified on your PC (no game needed)
- **Map parser on 1.53: PASS.** The full parse took 193 s: 516,930 nodes, 204,887 roads, 60,556 prefabs (61,468 lane curves), 18,848 POIs (3,543 fuel stations, 10,041 parking spots, 270 service areas, 342 tolls), 1,571 companies, 73 ferries.
- **Pinned ETS2LA plugin builds** with MSVC 14.51 (read-only patch applied).
- **Bridge DLL: 28/28 automated tests pass.** The DLL runs inside `fake_game.exe`, which plays the game's side of the SDK. Covered: telemetry values, config, events, a missing `mandatory.break` (as on 1.53), axes/holds/pulses, watchdog release in 20–45 frames at 120 Hz (≈250 ms), kill latch until re-arm, release while paused, frame-rate/latency.
- **Guided checks dry-run** against a simulated parked truck: wiggle, watchdog (released 278 ms after the last heartbeat) and calibrate all pass.

### Needs your game (what I could not test)
1. That 1.53.2.1 loads the bridge and polls its input device: telemetry, steering, lights, blinkers and pedals in the real game.
2. Kill key, watchdog and manual override against the real game.
3. The pinned ETS2LA memory plugin against your exact build: traffic, traffic lights, route, camera.

---

## Before you start
- **Save your game manually** (and before step 3 especially). If the memory plugin's offsets don't match 1.53.2.1, it can crash the game.
- Every command below runs in **PowerShell from the project folder** (the folder you cloned AutoHaul into), using the project's Python environment `.venv`.
- **Required:** Options → Gameplay → Transmission type → **Automatic**, not "Simple automatic". In Simple automatic (telemetry shows `shifter arcade`), holding the brake at a standstill shifts into reverse. AutoHaul holds the brake at every stop, so `phase1_check.py` refuses to run in that mode.
- Recommended game settings (Options):
  - Controls → steering: non-linearity **0**, speed-sensitive steering **off**.
  - Display: **borderless window**. Switching between the console and the game is much easier, and the overlay needs it later.
- Kill keys: **`\` (backslash)** or **Pause/Break** (Pause also pauses the game), or **BACK+START** on the Xbox controller.

## Step 1: install the bridge only
Close the game, then:

```bash
.venv/Scripts/python.exe tools/install_plugin.py
```

Start ETS2. If it warns that SDK plugins / advanced SDK features are in use, confirm with OK. Load your profile and get into the truck.

Then watch live telemetry (read-only, sends nothing):

```bash
.venv/Scripts/python.exe tools/dashboard.py
```

**Expected:**
- About 60 Hz, `bridge flags: TEL INPUT`, `telemetry game v1.18`, `shifter automatic`.
- `safety now: OK`. If you press a driving key (arrows, W/A/S/D, Space), it shows `(last trip ... override: driving key ...)`. That's expected: the dashboard only reports, it never drives.
- Speed, gear, fuel, `rest needed in`, job and trailer all match the game.
- `mandatory break n/a (1.53)`.
- The ETS2LA line shows `-` (not installed yet).

Drive a little, then Ctrl+C.

## Step 2: guided control checks (truck parked, parking brake on)
Stop somewhere quiet and engage the parking brake. Run each check. Each one waits for Enter, then counts down 5 s while you switch to the game window.

```bash
.venv/Scripts/python.exe tools/phase1_check.py preflight
```

```bash
.venv/Scripts/python.exe tools/phase1_check.py wiggle
```

Wiggle: the wheel sweeps left/right for 12 s, then blinkers, hazards, lights, wipers, horn, brake 0.3 and throttle 0.2. Expect `steering follows the command: input gain ≈ -1.0`. If the gain is ~+1.0 or 0, tell me (it means the game mixes the axis differently than its `controls.sii` says).

```bash
.venv/Scripts/python.exe tools/phase1_check.py kill
```

Kill: press `\` while the wheel moves. Expect steering back to 0 within ~100 ms, and "the DLL latched the kill itself".

```bash
.venv/Scripts/python.exe tools/phase1_check.py watchdog
```

Watchdog: AutoHaul stops its heartbeat on purpose. Expect release ≤ ~310 ms.

```bash
.venv/Scripts/python.exe tools/phase1_check.py override
```

Override: while the wheel moves, nudge the left stick, a trigger, or press W/A/S/D. Expect "control handed back".

```bash
.venv/Scripts/python.exe tools/phase1_check.py calibrate
```

Calibrate: holds 9 steering positions. Saves `%USERPROFILE%\AutoHaul\calibration\steering.json`, the input/effective gains and front-wheel degrees per unit.

(`phase1_check.py all` runs the five checks in a row.) Each run writes a JSON report and a recording. The paths are printed at the end.

## Step 3: the ETS2LA memory plugin (save the game first!)
Close the game, then:

```bash
.venv/Scripts/python.exe tools/install_plugin.py --with-ets2la
```

Start the game.
- **If it crashes** on load or when you start driving: remove the plugin and tell me. AutoHaul then uses the fallbacks in ARCHITECTURE.md §1.2.

```bash
.venv/Scripts/python.exe tools/install_plugin.py --remove ets2la
```

- **If it loads:** take a job (or set a GPS destination), drive towards traffic and a junction with traffic lights, and run the 60-second self-test while driving:

```bash
.venv/Scripts/python.exe tools/selftest_gamedata.py --seconds 60
```

**Expected PASS lines:**
- `ets2la: camera`
- `ets2la: traffic` (vehicles, plausible sizes)
- `ets2la: traffic lights` (states like red/green)
- `ets2la: route` (agrees with the GPS distance)

WARN only means it saw no traffic, lights or route during the sample; drive closer and re-run. Also watch your FPS: a big drop after installing the plugin means one of its memory patterns failed (it then rescans every frame). Tell me if that happens.

## Recording and replay
`dashboard.py --record` and every `phase1_check.py` test record to `%USERPROFILE%\AutoHaul\recordings\`. To inspect the newest recording or export it to CSV:

```bash
.venv/Scripts/python.exe tools/replay.py
```

```bash
.venv/Scripts/python.exe tools/replay.py --csv run.csv
```

## Uninstall

```bash
.venv/Scripts/python.exe tools/install_plugin.py --remove all
```

## Please send me back
The console output of `phase1_check.py all` and `selftest_gamedata.py --seconds 60`, or the report JSON files under `%USERPROFILE%\AutoHaul\logs\`. Also mention anything odd you saw (steering direction, FPS, crashes).

---

## Rebuilding from source
Needed only after changing native code or the layout spec:

```bash
.venv/Scripts/python.exe tools/fetch_deps.py
```

```bash
.venv/Scripts/python.exe tools/gen_layout.py
```

```bash
.venv/Scripts/python.exe tools/build_native.py
```

```bash
.venv/Scripts/python.exe -m pytest
```

Dry run without the game: start the simulated parked truck in one terminal, then run any tool in a second terminal with the same `$env:AUTOHAUL_SHM_SUFFIX = "_dry"` set:

```bash
$env:AUTOHAUL_SHM_SUFFIX = "_dry"; & "$env:USERPROFILE/AutoHaul/build/out/fake_game.exe" "$env:USERPROFILE/AutoHaul/build/out/autohaul_bridge.dll" --frames 100000 --hz 60 --parked
```

## Troubleshooting
| Symptom | Likely cause |
|---|---|
| Tools print "Waiting for the game" forever | The DLL isn't loaded. Check `install_plugin.py --status`, and `game.log.txt` for `[autohaul]` lines. |
| `LayoutMismatch` | The DLL and Python come from different builds. Run `gen_layout.py`, then `build_native.py bridge`, and reinstall. |
| Steering doesn't move but telemetry works | The input device isn't polled. Check `bridge flags` shows INPUT and `input frames` increases while driving. Tell me. |
| "telemetry stale" aborts | The game froze for > 1 s (autosave / loading). Re-run. |
| FPS drop with the ETS2LA plugin | A memory pattern failed on 1.53.2.1. Remove the plugin (`--remove ets2la`) and report. |

## Still missing after Phase 1
- **No driving logic yet:** lane keeping, ACC, routing and mission logic start in Phase 2.
- **No on-screen overlay** (Phase 2) and no engage hotkey: the Phase 1 tools engage via Enter in the console.
- **Map data is only parsed to JSON.** The SQLite/numpy map DB, localization and lane graph come in Phase 2. The parser patch for nav-curve traffic rules, semaphore links and fuel prices is also Phase 2.
- **Unverified in the real game:** the pinned ETS2LA plugin, and the exact behaviour of `quickpark`, auto-park and auto-load (Phase 5).
- **No vision / rain detection yet.**
- **Steering wheels and pedals (DirectInput)** are only detected through the telemetry cross-check. Xbox controller and keyboard are detected directly.
