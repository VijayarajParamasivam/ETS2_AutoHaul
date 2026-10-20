# Phase 3: routes, junctions, traffic rules

Target: ETS2 **1.53.2.1**. Status 2026-10-20: route following works in-game; automatic lane changes built and simulated on the
real map; junction rules are next.

## Plan
1. **Routing** (done): AutoHaul's own route over the lane map to the job's depot (the game's GPS route can't be
   read on 1.53), checked against the game's GPS while driving.
2. **Following the route through exits and interchanges** (done): the route's branch at every split, planned
   turns down to a 10 m radius, re-routing when off the route.
3. **Lane changes** (done, simulated): indicator 3 s before, a gap check against the AI traffic beside you,
   a smooth move across.
4. **Junction rules**: give way / stop from the map's junction data (needs the map parser patch that exports
   nav-curve flags and traffic-light IDs); traffic lights from their map positions plus a colour check in the
   camera image (the memory reader for light states crashes 1.53).
5. **Tolls** (stop, pay, wait for the barrier), roundabouts, weigh stations.

## What exists now
| Piece | Where | Purpose |
|---|---|---|
| Lane graph | `autohaul/world/graph.py` | For every lane: the lanes that continue it, and the parallel lanes it can change into. Built in ~2 s, cached (`lanes_graph.npz`). 991k links, 137k lane-change pairs. |
| Router | `autohaul/world/route.py` | A* to a destination point; lane changes cost 80 m, sharp joins a little more. Routes across a region in well under 0.1 s. Remaining distance along the route. |
| Junctions | `autohaul/world/junctions.py` | Whether a junction has crossing paths (crossroads, roundabouts: ~21% of junctions, nearly all with 10+ paths). Paths that only split or merge (exits, interchanges) have none. |
| Places | `autohaul/world/places.py` | Cities and company depots from the map; the job's target (source depot until the cargo is loaded, then the destination depot). |
| Autopilot | `autohaul/drive/autopilot.py` | With a router and a destination: keeps the tracker on the route, takes the route's branch at splits, re-routes when off it, hands back for a needed lane change (8 s / 150 m before), before any intersection (8 s / 100 m), and 60 m before the destination. Refuses to engage there too. |
| Drive | `tools/drive.py --to CITY[:COMPANY]` | Routes to the active job's depot by default. The status line shows the remaining route and pending lane changes. |
| Lane changes | `autohaul/control/lanechange.py`, autopilot | When the route needs another lane, as early as possible: into the neighbouring lane of whichever road piece the truck is on, then the route is re-planned from there. The planner itself puts changes at the last moment, often on a 100-300 m piece that is too short.<br>**Signal:** indicator for at least 3 s.<br>**Gap check** on the AI traffic in the target lane, in the truck's frame: ahead at least max(10 m, 0.8 s x speed) plus the speed difference; behind at least max(10 m, 0.5 s x speed + 3 s x closing speed), measured from the truck's rear (3.5 m, or 17 m with a trailer); nobody alongside.<br>**Move:** the target lane with a sideways offset easing to 0 over max(90 m, 5 s); about 1 m/s^2 of lateral acceleration.<br>Hands back if no gap opens before the latest point. While straddling, following distance watches both lanes. |
| Indicators | `tools/drive.py` | The game's indicator buttons are toggles, so AutoHaul compares with telemetry and presses only on a mismatch. It only switches off what it switched on, and switches its indicator off when handing back. |
| GPS check | `tools/drive.py` | The game's GPS distance falls at exactly the map scale x speed while you follow its route: 19.0 on open roads, measured. A jump up means the game re-routed; it's logged and shown. |

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

## Where development stopped (2026-10-20) and what's next
- **Done:** steps 1-3 (routing, route following through exits/interchanges, automatic lane changes). The lane
  change waited for a gap and signalled in-game. The confidence fix above is tested in simulation, not yet
  in-game.
- **Next, in order:**
  1. **In-game check of the confidence fix:** a lane change from a standstill or at low speed.
  2. **Step 4, junction rules:**
     - patch the map parser (`prefab-ppd-parser.ts`) to export nav-curve flags (priority / give way / stop)
       and traffic-light IDs and positions, re-extract, rebuild the lane map;
     - give way and stop at junctions using AI traffic from the memory reader;
     - traffic-light state from a colour check of the camera image at each light's map position (the
       camera reader works on 1.53; the light-state reader crashes the game).
  3. **Step 5:** tolls (stop at the booth, pay with `activate`, wait for the barrier), roundabouts, weigh stations.
  4. **Tuning:** steadier lane keeping at motorway speed (slow +-0.3 m sway); overtaking slow traffic.
- **Game setup left in place:** developer console and the `goto` / Ctrl+F9 teleport (`config.cfg`:
  `g_developer 1`, `g_console 1`, plus `g_save_format 2`, which has no effect on this build). The original
  is backed up in `%USERPROFILE%\AutoHaul\backups\20261019-235044-savefix`.

## Test it (in the game)
1. Get onto a motorway, then start AutoHaul with a destination: your job's depot (no argument needed while a
   job is active), or a city or depot:
   ```powershell
   .venv\Scripts\python.exe tools\drive.py --mode full --max-kmh 80 --to berlin:nbfc
   ```
2. Press `/` on a plain stretch. AutoHaul follows the route through exits and interchanges. With fast beeps it
   hands back for a lane change it needs, before an intersection, at a toll and at the destination.

## Not yet
- Lane changes are automatic only for the route (no overtaking slow traffic yet). Lane markings (solid lines)
  aren't in the map data, so supervise. If the move is cut short, take over.
- No traffic lights, give way or stop signs (step 4): it never enters an intersection by itself.
- Merging onto a motorway doesn't check traffic in the target lane yet: supervise merges.
- Trailer swing in tight turns isn't modelled (Phase 5): with a trailer, take over for tight ramps.
