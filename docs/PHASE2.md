# Phase 2: highway lane keeping and speed / following-distance control

Target: ETS2 **1.53.2.1**, single-player, your normal **relative** steering and keyboard driving unchanged.
Status 2026-10-19: built and tested in simulation and in shadow mode (AutoHaul watching you drive); supervised
in-game driving is next.

## What Phase 2 delivers

| Piece | Where | Purpose |
|---|---|---|
| Lane map | `autohaul/world/lanemap.py` | Every drivable lane in Europe as short segments (1.24 M lanes), built once from the parsed map and cached. Road lanes come from road curves, junction/roundabout lanes from the prefabs' AI nav curves. Where each road look's lanes sit is **calibrated from the junction lanes that connect to it**. |
| Lane tracker | `autohaul/world/localizer.py` | Which lane the truck is in (map matching on telemetry position), with hysteresis and the planned branch preferred at splits; the path ahead. |
| Steering | `autohaul/control/steering.py` | A servo that works **with relative steering**: the game turns the wheel at `c_rsteersens` (0.42 /s per unit in your profile) × AutoHaul's axis, and the servo closes the loop on telemetry's wheel position. A lane-keeping law (curvature feedforward + lateral/heading feedback, latency prediction, learned curve response). |
| Speed | `autohaul/control/speed.py` | Target speed = min(max speed, posted limit − 2 km/h, curve speed with planned braking to every curve ahead, stop before a mapped lane ends). ACC: constant time gap (10 m + 2.2 s × speed) to the nearest vehicle or trailer in your lane, emergency braking when closing too fast. Pedals: jerk-limited reference with a PI loop (no overshoot with a 40 t load). |
| Autopilot | `autohaul/drive/autopilot.py` | Ties it together per telemetry frame; reports a `fault` for anything it can't handle (lane lost, uncertain, off centre, someone else steering, no path). |
| Simulator | `autohaul/sim/` | Truck model with your truck's geometry and the game's relative steering, automatic gear shifts, loads, slopes; synthetic courses; closed-loop runner. |
| Tools | `tools/drive.py`, `tools/shadow_drive.py`, `tools/steer_ident.py`, `tools/validate_lanes.py` | Drive; watch you drive (sends nothing); measure the steering axis; check the lane map against AI traffic. |

### Verified without driving
- **Lane map vs. live AI traffic** (AI vehicles drive on lane centres): road lanes median error 0.12 m, junction lanes 0.23–0.31 m; path prediction 80 m ahead median 0.13 m.
- **Your driving in shadow mode** (89 km/h, trailer attached): the game truck turns exactly as the kinematic model predicts (learned ratio 0.995); compute time p99 5.5 ms per frame.
- **Lane map fix from shadow mode:** one-way roads (highway carriageways, exits) had their lanes one lane-width off, so lanes didn't connect at junctions. Fixed by calibrating every road look from the junction lanes that join it (265 of 364 looks; the rest by the rule measured from those).
- **Simulated truck on the real map** (`tools/sim_drive.py`, started where your highway drive started): 8.8 km of
  French motorway and interchanges in 8 min, **0 faults**, lane-centre error median 0.06 m, max 0.59 m, compute p99
  1.6 ms. Found and fixed on the way: a sub-metre join between lanes read as a 2 m hairpin; tight corners need the
  wheel turned early (relative steering turns at most 0.42 /s), so the steering looks ahead and the speed planner
  slows down where the wheel couldn't keep up.
- **Simulation (67 tests in total):** lane keeping at 78 km/h max error < 0.4 m (typically 0.15–0.25 m) with wheel rate 0.25–0.8 /s, 3 frames of extra lag, strong understeer, 40 t uphill, 30 fps; stays under the limit with a 40 t load; slows to the curve speed before a 120 m radius curve; follows a lead vehicle at the set gap and stops behind it when it brakes hard to 0 (never closer than 3 m); stops before a mapped lane ends; refuses to engage off-centre / reversing / parking brake on.

---

## Before you start
Everything from [PHASE1.md](PHASE1.md) still applies (Automatic transmission, not elevated, game window focused), **except** steering: keep your normal **relative** steering (`c_relatsteer 1`).
- The ETS2LA plugin must be installed (lead-vehicle detection). `drive.py` refuses speed control without it.
- **Save your game manually** before each drive.
- Engage / hand back: **`/`** in the game. Take over any time: steering keys (A, D, ←, →) when AutoHaul steers, pedal keys (W, S, ↑, ↓, Space) when it controls speed, controller stick / triggers, or the kill key **`\`** / Pause / BACK+START.
- Beeps: rising = engaged, falling = handed back, rapid = fault (AutoHaul let go by itself).

## Test steps (in order)

### 1. Steering axis check (parked, ~1 min)
Truck parked, parking brake on, game window focused.
```powershell
.venv\Scripts\python.exe tools\steer_ident.py
```
Press ENTER, switch to the game within 5 s. The wheel turns left/right at several speeds. It prints whether the game applies a deadzone to AutoHaul's steering axis (your profile has `c_steer_dz 0.16`) and the exact config line if one is needed.

### 2. Speed only, on a quiet highway
```powershell
.venv\Scripts\python.exe tools\drive.py --mode speed --max-kmh 70
```
Drive onto the highway, centre in your lane, press `/`. **You steer**, AutoHaul does throttle and brake: holds 70 km/h (or limit − 2), slows for curves, keeps distance to the vehicle ahead. Brake (S / Space) to take over.

### 3. Steering only, at moderate speed
```powershell
.venv\Scripts\python.exe tools\drive.py --mode steer
```
Hold a steady speed yourself (W), press `/`. AutoHaul keeps the lane; hands on A/D to take over.

### 4. Both: a highway stretch
```powershell
.venv\Scripts\python.exe tools\drive.py --mode full --max-kmh 80
```
After each run the console prints engaged time, distance and faults, plus the recording path. Send me the console output.

## What Phase 2 does not do yet
- **No lane changes, overtaking, junction turns, traffic lights, tolls** (Phase 3). At a split it follows the straightest branch; supervise exits.
  When the path ahead turns sharper than a 40 m radius (junctions, roundabouts), it **hands back about 6 s early** with the fault beep.
- **No routing**: it doesn't know your GPS route (the ETS2LA route reader is empty on 1.53).
- **Speed limits only as posted now** (from telemetry); no look-ahead to the next sign.
- **Trailer swing** isn't modelled; fine on highways, matters for tight turns (Phase 3/5).
- **Left-hand traffic (UK)** untested.
