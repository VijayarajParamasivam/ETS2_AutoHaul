# Phase 3: routes, junctions, traffic rules

Target: ETS2 **1.53.2.1**. Status 2026-10-20: route following built and simulated on the real map; lane changes
and junction rules are next.

## Plan
1. **Routing** (done): AutoHaul's own route over the lane map to the job's depot (the game's GPS route can't be
   read on 1.53), checked against the game's GPS while driving.
2. **Following the route through exits and interchanges** (done): the route's branch at every split, planned
   turns down to a 10 m radius, re-routing when off the route.
3. **Lane changes**: indicator 3 s before, a gap check against the AI traffic beside you, a smooth move across.
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
| GPS check | `tools/drive.py` | The game's GPS distance falls at exactly the map scale x speed while you follow its route: 19.0 on open roads, measured. A jump up means the game re-routed; it's logged and shown. |

### Verified without driving
- **Routes:** in the simulator AutoHaul followed a route from the Magdeburg motorway towards a depot in Berlin.
  - 8.8 km through motorway interchanges and onto the approach roads, slowing for every ramp curve.
  - Lane-centre median 0.05 m, max 0.62 m.
  - Handed back 100 m before the first town intersection, as designed.
- **Tests:** 85 pass, including route choice at a split, routes needing a lane change, unreachable
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

## Test it (in the game)
1. Get onto a motorway, then start AutoHaul with a destination: your job's depot (no argument needed while a
   job is active), or a city or depot:
   ```powershell
   .venv\Scripts\python.exe tools\drive.py --mode full --max-kmh 80 --to berlin:nbfc
   ```
2. Press `/` on a plain stretch. AutoHaul follows the route through exits and interchanges. With fast beeps it
   hands back for a lane change it needs, before an intersection, at a toll and at the destination.

## Not yet
- Lane changes are not automated (step 3): it hands back early enough to do them yourself.
- No traffic lights, give way or stop signs (step 4): it never enters an intersection by itself.
- Merging onto a motorway doesn't check traffic in the target lane yet: supervise merges.
- Trailer swing in tight turns isn't modelled (Phase 5): with a trailer, take over for tight ramps.
