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
- **In-game steering axis test** (`tools/steer_ident.py`, parked, 2026-10-19): wheel rate = **0.419 × axis per second**,
  linear from 0.04 to 0.50 and the same in both directions (rms error 0.0002), **no deadzone**. It matches
  `c_rsteersens 0.42`, so `c_steer_dz 0.16` doesn't apply to AutoHaul's axis and nothing needs configuring.
- **In-game speed-only drive** (`drive.py --mode speed --max-kmh 70`, trailer attached, 2026-10-19): engaged once for
  10.9 min / 7.5 km, no faults; you handed back with the brake trigger at a toll. Felt good overall, but it braked
  "here and there" and was slow: the recording shows 24% of the time limited by a curve *at the truck*. Cause:
  the steering-rate check (meant for when AutoHaul steers) read your normal 3-degree keyboard corrections as a
  wheel that must swing, e.g. 11 km/h demanded on an 800 m curve. Fixed: no steering-rate limit in speed-only
  mode, and a 3.4-degree dead band when AutoHaul steers. Replayed through the fix: target mean 62 -> 67 km/h
  (cap 68), curve-at-truck limits 24% -> 6% (real tight bends).
- **In-game steering-only drives** (`drive.py --mode steer`, trailer attached, 2026-10-19): 3 engagements, 2.1 km at
  30–87 km/h, **no faults**; every hand-back was you moving the stick. Lane-centre error median 0.13 m, p95 0.52 m,
  max 0.94 m (running a little wide at 85 km/h at the end of a 670 m right-hander). The learned curve response
  stayed at 1.0 and the learned wheel rate at 0.41 (measured 0.419). To tune: above 70 km/h the error is larger
  (median 0.48 m in one run) than in simulation. Engaging was refused 7 times (lane match uncertain inside
  junction areas, or more than 1 m off centre): engage on plain road. As designed, it follows the lane, not your
  GPS route: at splits it keeps to the straightest branch (route following is Phase 3).
- **First full-mode drive: too slow, because the game ignored AutoHaul's pedals.** AutoHaul sent full
  throttle and the game read 0, so the truck coasted. Parked checks (`tools/pedal_check.py`,
  `tools/pedal_pulse_check.py`) pinned it down. While AutoHaul's steering axis stays non-zero over
  consecutive frames, the game ignores AutoHaul's throttle **and brake** (input 0.30 -> 0.00). Steering in
  single-frame pulses, with a zero frame between, leaves the pedals fully working (100% of frames) and still
  turns the wheel (0.0021 per pulse, as predicted).
  - Fixed: relative steering is now pulse-modulated (average axis up to 0.5 = 0.21 wheel units/s, plenty for
    lane keeping).
  - New guard: AutoHaul hands back if the game doesn't show its throttle or brake for 0.5 s.
  - The simulator models the masking, so tests catch it from now on.
- **The real cause (2026-10-20): the game takes ONE input event per frame.** New bridge counters showed the
  game calls AutoHaul's input callback exactly once per frame (215 calls in 215 frames), so it fetches one
  changed input per frame. The old bridge marked every queued change as delivered, so whenever steering also
  changed, throttle and brake changes were lost for good. That's what looked like "pedals masked while steering".
  - Fixed in the bridge: a change counts as delivered only when the game fetches it, and the queue takes turns
    starting after the last delivered input.
  - The fake game can now fetch one event per frame, and an end-to-end test covers it.
  - The simulator models the same rule, and pulsed steering is off again.
  - Refined: changed values always go before resyncs, and after the game reactivates the device (focus
    change) only the 8 axes and pressed buttons are re-sent. Re-sending all 85 inputs had delayed AutoHaul's
    first commands by ~1.3 s and could have delayed a kill-switch release. Fake-game tests cover both.
  - Verified in-game, parked (`tools/pedal_check.py`, 2026-10-20): throttle/brake 0.30 reached the game in
    every phase, with steering at 0, held, or changing every frame (before the fix: 0.00). The game took one
    event per frame (863 calls in 863 frames); 6 changes it skipped were re-offered and delivered.
- **Map: hidden roads are closed roads.** A teleport onto a test stretch landed on a road the game shows as
  closed (X markers). In the parsed map, `hidden` roads and junctions aren't drivable: 0 of 3,799 samples on
  roads you actually drove were hidden. The lane map now leaves them out (1.24M -> 802k lanes).
- **First full-mode drive** (motorway near Magdeburg, no trailer, 2026-10-20): 2.5 min, 3.2 km, 49 -> 78 km/h.
  - **No faults.** It handed back only when the game window lost focus.
  - **Pedals reached the game:** game throttle within 0.05 of AutoHaul's command in 99.4% of frames, within 0.15
    in 100%. Speed held 77.3 km/h at a 78 km/h target.
  - **Correct with traffic:** a car cut in 30 m ahead at 118 km/h and pulled away, and AutoHaul didn't brake hard.
  - **Lane keeping:** no bias (-0.03 m mean), median 0.22 m, a slow +-0.3 m sway over ~40 s (to tighten at speed).
- **Toll plazas:** AutoHaul now hands back (fault beep) when the path ahead passes within 60 m of one of the map's
  342 toll plazas within 8 s / 120 m, and won't engage there. Replayed: it would have handed back 120 m before
  the toll you stopped at.
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
