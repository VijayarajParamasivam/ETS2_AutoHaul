# Phase 3: routes, junctions, traffic rules

Target: ETS2 **1.53.2.1**. Status 2026-10-21: route following and automatic lane changes work in-game. Junction rules
(give way, right of way, stop signs) are built and simulated on the real map; traffic lights still hand back.

## Plan
1. **Routing** (done): AutoHaul's own route over the lane map to the job's depot (the game's GPS route can't be
   read on 1.53), checked against the game's GPS while driving.
2. **Following the route through exits and interchanges** (done): the route's branch at every split, planned
   turns down to a 10 m radius, re-routing when off the route.
3. **Lane changes** (done, simulated): indicator 3 s before, a gap check against the AI traffic beside you,
   a smooth move across.
4. **Junction rules** (give way / right of way / stop signs done, simulated; traffic lights next): who gives way
   comes from the map's junction data (map parser patch); traffic lights from their map positions plus a colour
   check in the camera image (the memory reader for light states crashes 1.53).
5. **Tolls** (stop, pay, wait for the barrier), roundabouts, weigh stations.

## What exists now
| Piece | Where | Purpose |
|---|---|---|
| Lane graph | `autohaul/world/graph.py` | For every lane: the lanes that continue it, and the parallel lanes it can change into. Built in ~2 s, cached (`lanes_graph.npz`). 991k links, 137k lane-change pairs. |
| Router | `autohaul/world/route.py` | A* to a destination point; lane changes cost 80 m, sharp joins a little more. Routes across a region in well under 0.1 s. Remaining distance along the route. |
| Junctions | `autohaul/world/junctions.py` | Whether a junction has crossing paths (crossroads, roundabouts: ~21% of junctions, nearly all with 10+ paths). Paths that only split or merge (exits, interchanges) have none. |
| Places | `autohaul/world/places.py` | Cities and company depots from the map; the job's target (source depot until the cargo is loaded, then the destination depot). |
| Autopilot | `autohaul/drive/autopilot.py` | With a router and a destination: keeps the tracker on the route, takes the route's branch at splits, re-routes when off it, hands back before a traffic light or barrier (8 s / 100 m; without junction rules or AI traffic data, before any intersection) and 60 m before the destination. Refuses to engage there too. |
| Drive | `tools/drive.py --to CITY[:COMPANY]` | Routes to the active job's depot by default. The status line shows the remaining route and pending lane changes. |
| Lane changes | `autohaul/control/lanechange.py`, autopilot | When the route needs another lane, as early as possible: into the neighbouring lane of whichever road piece the truck is on, then the route is re-planned from there. The planner itself puts changes at the last moment, often on a 100-300 m piece that is too short.<br>**Signal:** indicator for at least 3 s.<br>**Gap check** on the AI traffic in the target lane, in the truck's frame: ahead at least max(10 m, 0.8 s x speed) plus the speed difference; behind at least max(10 m, 0.5 s x speed + 3 s x closing speed), measured from the truck's rear (3.5 m, or 17 m with a trailer); nobody alongside.<br>**Move:** the target lane with a sideways offset easing to 0 over max(90 m, 5 s); about 1 m/s^2 of lateral acceleration.<br>Hands back if no gap opens before the latest point. While straddling, following distance watches both lanes. |
| Indicators | `tools/drive.py` | The game's indicator buttons are toggles, so AutoHaul compares with telemetry and presses only on a mismatch. It only switches off what it switched on, and switches its indicator off when handing back. |
| GPS check | `tools/drive.py` | The game's GPS distance falls at exactly the map scale x speed while you follow its route: 19.0 on open roads, measured. A jump up means the game re-routed; it's logged and shown. |
| Map parser patch | `patches/trucksim_maps/0001-*.patch` | The pinned TruckSim Maps parser read these fields and dropped them; the patch exports them: every junction path's flags (priority, blinker), traffic rule and traffic-light id, and every traffic light / barrier (position, type, light profile, timings). `tools/fetch_deps.py` applies it; the extract report records the patch hash. |
| Junction rules | `autohaul/world/rules.py` | Per junction path: priority (0-15), the indicator the game's AI uses, traffic rule (`stop`, `give_way`, ...), and the lights or barriers that control it, with 47k light heads / barrier arms placed in world coordinates. Built from the extracted map in ~3 s, cached (`lanes_rules.npz`). |
| Conflicts | `autohaul/world/junctions.py` | Where two paths of a junction cross at the same height (flyovers don't count) or merge into the same lane (turns joining a road, roundabout entries). Each has a conflict zone: from where the two paths come within 3 m of each other. |
| Give way | `autohaul/control/giveway.py`, autopilot | **Who gives way:** at each conflict the lower map priority does (higher number has right of way; a main road is 12, a left turn across it 6). AutoHaul also gives way at equal priority.<br>**Clear?** AI vehicles on the other path or the lanes leading to it (150 m back) are timed to the conflict zone. Crossing, each must arrive only after the truck has cleared it plus 3 s; merging and past the line, less (see the fixes of 2026-10-30). Vehicles standing still more than 15 m out are waiting themselves.<br>**Waiting:** at the line (1 m before the junction), or inside the junction just short of a conflict zone (like a left turn waiting in the middle).<br>**Right of way:** still stops for a vehicle too fast to stop at its own line.<br>**Stop signs:** a full stop first.<br>**Speeds:** 15 km/h at a give-way line, 20 km/h inside intersections, turns at 0.7 m/s^2.<br>Indicates turns from the map's blinker flags. Hands back after waiting 45 s, before traffic lights and barriers, and before any intersection when no AI traffic data comes. |

### Verified without driving
- **Routes:** in the simulator AutoHaul followed a route from the Magdeburg motorway towards a depot in Berlin.
  - 8.8 km through motorway interchanges and onto the approach roads, slowing for every ramp curve.
  - Lane-centre median 0.05 m, max 0.62 m.
  - Handed back 100 m before the first town intersection, as designed.
- **Lane changes on the real map** (simulated, Magdeburg towards Poznań): signalled 3 s, moved left in 4.7 s
  with 542 m to spare, then 14.9 km of route at up to 78 km/h through interchanges. It handed back 145 m
  before a toll plaza on the Polish motorway. Offset from the followed line max 0.37 m, lane change included.
- **Tests:** 90 pass, including lane changes in the simulator and the gap check. In simulation it changes
  lanes (also from walking pace), waits behind a neighbour until it drops back, and hands back when the
  neighbour never does.
- **Earlier tests:** 85 pass, including route choice at a split, routes needing a lane change, unreachable
  destinations, closed-loop following through a junction to arrival, and hand-backs before lane changes and
  intersections.

### First in-game route following (2026-10-20, no trailer, destination: nbfc in Berlin)
- Two engagements, 5 min 10 s, about 4.8 km of route on the map (about 67 GPS km).
  - On AutoHaul's route 100% of the time.
  - Lane-centre median 0.07 m and 0.11 m (max 0.73 m).
  - 75 km/h on the motorway, 46 km/h on the curvy approach roads.
- The game's GPS (set to a different destination, 388 km away) fell by 49.4 km while AutoHaul's route ran
  along it, against 50.9 km expected from speed x scale. Where the routes split, the game re-routed once and
  the monitor logged it.
- Ended by you pressing D, then by the intersection hand-back 100 m before the first town junction (as designed).

### First in-game lane change and long route (2026-10-20, no trailer, Magdeburg towards Poznań)
- **Lane change:**
  - The **left indicator came on in the game** at once.
  - AutoHaul **waited** because a vehicle was 6 m behind in the left lane, then started moving over.
  - Pulling away from a standstill, the move was slow (under 38 km/h). The "between two lanes" moment then
    lowered the tracker's confidence for over a second, so AutoHaul handed back mid-move (simulation at
    60 km/h hadn't shown it).
  - **Fixed:** during a deliberate change the lane is known, so the tracker reports high confidence. The check
    against the eased line still guards the position. A low-speed lane-change test now covers it.
- **Long route:** 13 minutes and 15.1 km of map (about 287 GPS km).
  - 100% on the route, offset median 0.09 m (max 1.0 m), up to 79 km/h.
  - Handed back 120 m before the toll plaza on the Polish motorway.
- **After the toll:** 1.65 km more, then handed back 172 m before an intersection. Both hand-backs were as
  designed.

### Junction rules, verified without driving (2026-10-21)
- **What the map says:** 588k junction paths in use.
  - 70% have priority 0: short connectors that cross nothing.
  - Where paths cross without a light, 95% of pairs have different priorities.
  - Explicit `stop` / `give_way` rules are rare (55 paths); right of way comes almost entirely from priorities.
  - At lit junctions the light sits on a short entry piece at the stop line.
- **Kiel roundabout entry, real map:** cars circling the roundabout, timed to meet the truck.
  - The truck stopped 1 m before the line and waited 22 s while they passed (each one twice, as they
    circled), then went when clear.
  - Closest car to the truck: 11 m.
- **Synthetic junctions (tests):**
  - waits for a stream of crossing cars, then goes; nobody is in the crossing while the truck is;
  - drives through when it has right of way and the other car waits;
  - still stops for a car that ignores its give-way line;
  - stops fully at a stop sign;
  - gives way to main-road traffic when turning right onto it (a merge);
  - signals junction turns;
  - hands back before a traffic light, and before intersections when there's no traffic data.
- **54 random town routes on the real map** (no traffic, start 0.5-1.5 km from a depot, up to 4 min each):
  - **Before the fixes below:** 6 of the first 24 ended off the lane or with the lane match lost.
  - **After:** none of the 54 did.
    - 31 intersections driven through (Naples: 10 in 4 minutes).
    - Worst lateral error per run: median 0.30-0.52 m, worst 1.09 m.
  - **How the 54 ended:**
    - 26: traffic light ahead (hand-back);
    - 13: time up, still driving;
    - 5: engage refused (a light or toll right at the start);
    - 4: toll plaza;
    - 3: turn under 10 m radius;
    - 2: lane change needed too close to a split;
    - 1: destination reached.
- **Fixes the town routes found** (they also help outside junctions):
  - **Steering preview:** looking ahead for the wheel's turn rate no longer cancels the correction. In a junction
    S-bend it had pinned the wheel to the next curve and the truck drifted 1.6 m.
  - **Understeer:** the truck turns less than the simple model at speed (tyre slip). That is now learned as a
    coefficient that grows with speed squared, not one ratio. A ratio learned on the motorway made the truck
    oversteer in slow town turns.
  - **Low-speed lateral response:** stiffer below 26 km/h (`d_min` 12 → 7 m).
  - **Path behind the truck at low speed:** the rear axle is measured against the real lane. Before, it was
    checked against a straight extension of the path, which misleads in tight junction curves.
  - **Tracker:** overlapping junction curves don't count as ambiguous when the tracked lane is the planned one.
  - **Lane changes:** a move that runs past the end of its target lane follows that lane's continuation.
  - **Speed:**
    - stop points are reached within a metre (no creeping past them);
    - letting off the brake at low speed is quicker (no stalls in slow turns);
    - the wheel-rate speed limit sees short junction curves (6 m curvature window).
- **Motorway regression check:** the Magdeburg → Poznań run is unchanged: 14.9 km, the lane change, and the
  hand-back 145 m before the toll plaza. Worst lateral error 0.66 m (0.68 m before these changes).
- **Tests:** 104 pass, 14 of them new junction tests.

### First in-game junction run (2026-10-21, Kiel, no trailer)
- **Teleport:** to `goto 10.6;-3.38;-21045.9;-0.5655;0`, then Ctrl+F9. Ctrl is also your gear-down key, so the
  truck ended up in reverse; shift up twice to Drive.
- **Roundabout entry (give way):** AutoHaul queued behind a car, waited for a car on a crossing path, then
  entered when it was clear. Inside the roundabout it gave way to another car.
- **Rest of the run:** it drove through a right-of-way junction and handed back 100 m before the traffic light.
  Lateral error stayed within ±0.4 m.
- **Fixed afterwards:**
  - The first engage at standstill had handed back with "the game isn't taking AutoHaul's throttle". The pedal
    check now waits 1.5 s after engaging.
  - In the queue it stopped 5 m behind the car ahead. A standing vehicle is now a stop point at the 10 m
    standstill gap.
  - AutoHaul now refuses to engage in reverse gear.

## Where development stopped (2026-10-21) and what's next
- **Done:**
  - steps 1-3: routing, route following, automatic lane changes (in-game);
  - step 4 without traffic lights: give way, right of way, stop signs, turn signals (simulated, and driven
    in-game once at Kiel, above).
- **Next, in order:**
  1. **More in-game junction runs** with AI traffic, and the low-speed lane change (the confidence fix).
  2. **Traffic lights (rest of step 4):**
     - the lights' world positions are in `lanes_rules.npz`;
     - colour check of the camera image at each light head (the camera reader works on 1.53; the light-state
       reader crashes the game).
     Until then AutoHaul hands back 100 m before every light. In town that's often within the first kilometre.
  3. **Step 5:** tolls (stop at the booth, pay with `activate`, wait for the barrier), weigh stations.
  4. **Tuning:** steadier lane keeping at motorway speed (slow +-0.3 m sway); overtaking slow traffic; trailer
     swing in junction turns (Phase 5).
- **Game setup left in place:** developer console and the `goto` / Ctrl+F9 teleport (`config.cfg`:
  `g_developer 1`, `g_console 1`, plus `g_save_format 2`, which has no effect on this build). The original
  is backed up in `%USERPROFILE%\AutoHaul\backups\20261019-235044-savefix`.

## Test it (in the game)
1. Get onto a motorway, then start AutoHaul with a destination: your job's depot (no argument needed while a
   job is active), or a city or depot:
   ```powershell
   .venv\Scripts\python.exe tools\drive.py --mode full --max-kmh 80 --to berlin:nbfc
   ```
2. Press `/` on a plain stretch. AutoHaul follows the route through exits, interchanges and junctions without
   lights. It gives way where the map says so, and the status line shows what it's doing at a junction. With fast
   beeps it hands back:
   - before a traffic light or barrier;
   - after 45 s of waiting at one junction;
   - at a toll and at the destination.
   `--no-rules` restores the old behaviour: hand back before every intersection.
3. Junction tunables are in `config/autohaul.toml` `[drive.give_way]`.

## Not yet
- Traffic lights: AutoHaul hands back 100 m before each one (step 4, next).
- Lane changes are automatic only for the route (no overtaking slow traffic yet). Lane markings (solid lines)
  aren't in the map data, so supervise. If the move is cut short, take over.
- Give way trusts the AI traffic reader. Vehicles it doesn't report are invisible to AutoHaul. Supervise
  junctions, especially the first in-game ones.
- Merging onto a motorway (a merge outside an intersection) doesn't check traffic in the target lane yet:
  supervise merges.
- Turns tighter than 10 m radius hand back. Trailer swing in tight turns isn't modelled (Phase 5): with a
  trailer, take over for tight ramps and town turns.

## Traffic lights (2026-10-24)
The ETS2LA plugin's traffic-light reader crashes 1.53 (Phase 1), so AutoHaul reads the lights from the screen.
- **Which light:** the map says which semaphore group controls each junction path, and where its heads stand
  (`autohaul/world/rules.py`). A head's map position is the pole base. The lamps can be anywhere above it or out
  on an arm over the road, so a box around it is searched: 9 m either side across the lane, 2-8 m up.
- **Where on the screen:** the plugin's camera block (position, rotation, horizontal field of view) projects the
  box onto the game's picture (`autohaul/vision/camera.py`). Checked against lane markings in the game.
- **Which colour:** lit lamps are tiny, bright and fully saturated. Measured on 1.53: green about (40, 248, 35),
  red about (250, 30, 27). Pixels of each colour are counted; a colour needs about a quarter of a lamp's area at
  that distance. Red wins over amber over green. Two reads in a row must agree (`autohaul/vision/lights.py`).
- **The screen:** copied with plain Windows calls (`autohaul/vision/screen.py`), only while the game is the front
  window. With another window in front, the light counts as unseen.
- **Driving** (`autohaul/control/giveway.py`):
  - **Red:** stop.
  - **Amber:** stop if that takes no more than 3 m/s^2, else go through.
  - **Green:** go. Paths of the junction on another light group count as stopped, so the truck has right of way
    over them.
  - **Unseen:** approach slowly, stop, and hand back after 6 s.
  - **Long reds:** it waits up to 150 s.
  - **Where to stop:** lamps above the windscreen can't be seen from the cab. The truck stops far enough before
    the line that one head of the light stays within 12 deg above eye level (at most 20 m back).
  - **Virtual lights** (only for AI traffic) are plain give-way junctions. Barriers still hand back.
- **Checked in the game (2026-10-24, Rotterdam, free camera 41 m before the junction):** the reader followed the
  light's full cycle (red -> green -> amber -> red) on every read.
- **Simulator:** `tools/sim_drive.py` cycles every junction's lights (`--lights none` hands back as before), and
  reports the colour each time the truck enters a lit junction.
  - Tests: red then green; the stop point keeping the light in view; green past cross traffic waiting at its
    red; amber far out (stops) and 4 m out (goes); a light it can't see (stops, then hands back).
- **Turn it off:** `traffic_lights = false` in `config/autohaul.toml` `[drive]` (it then hands back before
  lights again).
- **Not yet:**
  - It hasn't driven through a light in the game.
  - Night, rain and low sun are untested.
  - Two light groups in one search box (a red turn arrow next to our green) read as red: it waits, then hands
    back after 150 s.
  - Lights hidden behind a truck ahead count as unseen.

## Wrong routes (2026-10-24)
In your run from Rotterdam, AutoHaul's route to the Paris depot was 41 world km (about 760 game km) where the
game's GPS said 435 km. It made a U-turn at the first roundabout to go back west, where you took over. From
Amsterdam it planned 692-829 game km against the game's 514. The lane graph was missing connections:
- **A road style laid out mirrored:** "Highway 3 lanes 2m offset" was calibrated with its forward lanes on the left
  (264 road ends point that way, 30 the other). Where the junctions disagree, the road's lanes ran the wrong way
  relative to the junction, which cut a motorway just east of you off the graph.
- **Lanes a few metres off their junctions:** around Rotterdam, 3,039 lanes dead-ended, 1,981 of them with another
  lane starting 2.5-12 m away.
- **Seams between road styles:** "Highway 3 lanes 2m offset" running straight into "... 5m offset" (lanes 3 m
  apart, not linked).

Fixes (`autohaul/world/lanemap.py`, lane map version 9; it rebuilds by itself):
- **Junction evidence first:** every road end that meets a junction puts its lanes where the junction's lanes
  start and end (`snap_offsets`).
  - Up to 6 m from the road style's layout: the lanes blend into that position over the last 40 m. Lanes do move
    in near junctions; "5m offset" motorways by about 3 m.
  - Further off: the layout is wrong for that road, and the junction's positions run its whole length. They're
    carried along chains of roads that join without a junction.
- **Seams:** two roads joining without a junction meet halfway, blended in the same way.
- **City distances like the game:** the router counts a metre inside a city area as 3/19 of an open-road metre,
  as the game's GPS does (city scale 3, open roads 19; checked against the game's own scale: 6,415 of 6,415
  samples inside city areas read 3).

Results:

| | Before | After |
|---|---|---|
| Dead ends, all of Europe | 23,570 | 18,707 |
| Dead ends, around Rotterdam | 3,039 | 1,607 |
| Your spot near Rotterdam -> Paris depot | about 760 game km | about 465 (game GPS: 435) |
| Amsterdam depot -> Paris depot | 692-829 game km | about 551 (game: 514) |
| Your manual driving, distance to the nearest lane | median 0.74 m | median 0.75 m |

Routing stays fast: Rotterdam to Munich takes 0.2 s.

## Toll gates (2026-10-24)
- **Which gates:** the map's semaphore profiles say what a junction path's semaphore controls (`profile_kind` in
  `autohaul/world/rules.py`). Measured on 1.53:
  - `*_toll` and `tol_*`: toll booths (2,224 paths).
  - `bd_*`, `border_*`, `ab_*bd*`: border checks (674).
  - `br_*`, `bar_*`, `b_gen*`: barriers (805).
  - `weigh`: weigh stations (3).
  - `gas_*`: fuel stations' own pump signals; nothing to stop for.
  - Everything else is a traffic light. Before this, every profile counted as a traffic light.
- **At a toll booth** (`autohaul/control/giveway.py`):
  1. Slow to 12 km/h for the last 40 m.
  2. Stop the front bumper 2 m before the booth's semaphore.
  3. Press `activate` and wait for the game's `player.tollgate.paid` event, which `drive.py` passes on.
  4. Wait 2.5 s for the barrier, then drive through at 10 km/h.
  5. No payment after 3 presses: hand back ("pay yourself, then press /").
- **Simulator:** on your Paris route's first French toll plaza it stopped at the booth, paid, waited and drove
  on. Tests cover pay-and-go and the hand-back.
- **Turn it off:** `tolls = false` in `config/autohaul.toml` `[drive]`.
- **Not yet:**
  - Where exactly the booth's payment spot is relative to its semaphore hasn't been checked in the game. If the
    press doesn't pay there, it hands back after 3 tries.
  - Wide plazas are crossed at the 20 km/h junction speed.
  - Border checks, barriers and weigh stations still hand back.

## Fixes from the drives of 2026-10-29
- **Roundabouts and tight turns with a trailer:** the cab followed the lane centre and the trailer cut in over
  the kerb. A semi-trailer cuts in by about R - sqrt(R^2 - L^2), about 2 m on a 15 m roundabout.
  - In curves tighter than 45 m with a trailer, the cab now runs outside the centre by half of that, at most
    1.2 m, and back on the straight.
  - The lane checks allow for the swing.
- **Lamp colours:** a lit lamp must be near full brightness and saturation (green about (40, 248, 35), red about
  (250, 30, 27)). Sunlit leaves, red roofs and orange signs in the search box could pass the looser limits, so a
  false green could send the truck through a red.
  - A patch of one colour over 25 lamps' areas counts as a sign, not lamps.
- **A light out of view** (it stood at a light that had gone out of sight, then handed back): the game's AI
  obeys the lights, so AutoHaul reads them from the traffic.
  - Vehicles moving on another signal group's paths: red.
  - Vehicles moving on our group's paths: green.
  - With no lamp and no telling traffic, it gives way to everyone (a signal for the AI only, a head beside the
    cab, or lights off).
- **Toll booths with nowhere to pay:** the map doesn't say which booths have a payment machine.
  - The router no longer uses `ab_toll_no` booth lanes in plazas that also have paying booths (111 paths).
  - A booth where 3 presses of activate took no payment is handed back as before, and saved in
    `logs/toll_booths_no_payment.json`. The router avoids it from then on.

## Fixes from the drive of 2026-10-30

- **Slowing at interchanges.** `inside_kmh` applied to every path of any junction with a crossing somewhere in it, for the path's whole length (up to 599 m): straight over a bridge or under one at 20 km/h, a third of the drive. It now applies within 30 m before to 10 m past a path's own conflicts, or along a path under a signal.
- **Taking the side lane.** From an exit-only lane, a lane change (80 m) lost to going down a ramp, through the junction at the bottom and back up, hardly longer. Height change inside a junction now costs 12 m per metre, so it changes lanes and goes straight over. With no room to change, the ramp is still a way on.
- **Sharp turns.** 6 % of the map's junction joins start behind the last path's end, 7 % to its side. Stitching added that point, drawing a hairpin the steering and curve speeds took for a tight turn the other way (curvature -0.21, +0.11, -0.08 on a radius of 12 m), and "sharp turn ahead" fired. Stitching now bridges only real gaps ahead. The radius check uses 6 m of road and allows 5 m, about full lock.
- **"1.6 m off the lane centre"** needs 1 s of it below 29 km/h (0.2 s above; 3.5 m at once): a jump where two paths don't line up isn't the truck leaving the lane.
- **A tight turn lost.** Three faults together, on a turn of radius 10 m, ran the truck off with "lane match uncertain":
  - `PathRef` left segments under 0.5 m out of its curvature. Tight junction curves are made of those (19 % of the map's junction pieces), so the curve read as straight and the steering's feedforward fell to 0 mid-turn. Only slivers under 0.15 m are left out now.
  - A sideways step between two paths (7 % of joins, here 0.96 m) drew a sharp turn the wrong way (curvature +0.25, the wheel steered right). It's now eased across, at least 8 m and 16 m per metre of step, smoothly.
  - The eased line now stays one fixed line while the truck crosses the join, and lateral and lane-match confidence are measured against it, so the map's step isn't counted as the truck being off.
- **Waiting too early.** At a merge the truck stood at the line for a car 153 m out, and stopped again halfway round the turn for one 142 m out.
  - Its pull-away was assumed at 0.5 m/s^2. Over 56 real pull-aways the median is 1.2, or 0.75 with a trailer.
  - **Merging:** the car must arrive 3 s after the truck enters its lane, or 1 s after it has cleared, whichever comes first. **Crossing:** still 3 s after it has cleared.
  - **Past the line** (2 m or more): the truck stops in the junction only for a vehicle arriving within 1 s of the truck reaching the zone.
