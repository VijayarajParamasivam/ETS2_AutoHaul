# Phase 4: the mission manager

Target: ETS2 **1.53.2.1**. Status 2026-10-22: built and run in the simulator on the real map; the refuel and sleep
scripts still need checking in the game (the game's own screens can't be simulated).

## What it does
`tools/drive.py --mission` runs the active job on top of Phase 3's driving:
- **Job legs:** it drives to the trailer's depot, then (once the cargo is loaded) to the destination depot.
  Coupling and parking are Phase 5, so it hands back about 60 m before each depot with a message saying what to
  do. Press `/` again after coupling and it carries on to the destination.
- **Stops:** on the way it plans fuel and rest stops, drives into the station or rest area, stops at the pump or
  in the parking place, refuels or sleeps by itself, and drives on.
- **Decision log:** every decision goes to `logs/decisions.jsonl` with its inputs, every candidate and option
  with its numbers, the choice, and a one-line reason. Covered: job accepted, stop plan, stop done, hand-back,
  delivery.
- **Job history:** finished jobs (`job.delivered` / `job.cancelled`) go to `logs/jobs.jsonl`. The economics refit
  themselves from it.

### How stops are planned (`autohaul/mission/planner.py`)
- **Needs:**
  - **Fuel:** the litres to the destination plus a reserve (150 game km) are more than in the tank.
  - **Rest:** with the game's fatigue on, the drive to the destination ends less than 60 game minutes before the
    rest timer runs out. Whether fatigue is on comes from your profile's `g_fatigue` setting (read when
    `drive.py` starts). If that can't be read, from watching the rest timer. It can also be forced in the config.
    **Your profile has fatigue off (`g_fatigue 0`), so AutoHaul plans no sleeps until you switch it on.**
- **Where:** fuel stations and places to sleep within 400 world metres (about 8 game km) of the route.
  - Stations must be reachable keeping half the reserve.
  - Places to sleep must be reached an hour before the rest timer runs out.
  - The farthest few, the cheapest stations, and sites with both get a detour measured on the lane graph: off the
    route to the pump and back onto it 2.5 km further on.
- **Which:** every combination that covers the needs is scored:
  `value = income x late_factor(lateness) - extra fuel cost - time value x hours`.
  - **Extra fuel cost:** the detour's fuel, plus the price difference against the average on the litres bought.
    Cheap fuel is a saving.
  - **Late factor:** 1 on time; when late, 1 - 20% - 5% per hour, never below 30%. These are settings, refitted
    from your delivered jobs once there are three late ones. The game's formula isn't documented.
  - **Time value:** €600 per game hour, refitted from your jobs' income per game hour.
  - The best option wins. Plans are refreshed every ~28 game km and kept unless a new one is clearly better.
- **Units:** the map is measured in world metres. Fuel per world metre and game minutes per world metre are
  learned while driving (they include the map scale: 19 on open roads, 3 in towns). The logs show game km (×19).

### Where the stops come from (`autohaul/world/services.py`)
- **Fuel:** the map's `gas_ico` points are the pumps. 2,934 have a truck lane within 8 m (1,475 stations); car-only
  pumps drop out.
- **Sleep:** `parking_ico` points are the game's sleep triggers (rest areas, fuel stations, hotels). 7,831 have a
  lane within 30 m (2,337 sites).
- **Fuel prices:** per country, from the game's country definitions, from €0.61/l (Russia) to €2.18/l (Sweden).
  The map parser patch now exports them.
- **Car-only lanes:** the map marks some junction paths as car-only (e.g. car pumps). Routing never uses them now.

### The stop scripts (`autohaul/mission/actions.py`)
Each runs while the truck stands with the brake held. It presses only AutoHaul's virtual buttons and checks its
effect in telemetry.
- **Refuel:** press `activate` at the pump. If no fuel flows within 3 s, hold `activate` instead (both ways have
  existed in ETS2 versions). It's done when the tank is full or the flow stops.
- **Sleep:** press `activate` (opens the sleep screen), then `selectfcs` (confirm). It's done when the game clock
  jumps and the rest timer resets.
- **On failure:** if nothing happens, it hands back with "refuel yourself / sleep yourself, then press / to go on".
- **Settings:** buttons and timings are in `config/autohaul.toml` `[mission.actions]`.

## Verified without the game (2026-10-22)
- **A whole job in the simulator on the real map:** Magdeburg motorway → itcc in Szczecin, 17.9 world km (≈340
  game km). It started with 140 l in a 900 l tank and the rest timer due in 3h20.
  - **Plan at the start:** sleep at a German rest area 109 game km ahead (the last place before the rest limit),
    then refuel ~846 l in Poland at €1.39/l (the cheapest of 5 reachable stations) 314 game km ahead. Arrival
    estimate Mon 22:00, 2 h before the deadline.
  - **The sleep:** it drove into the rest area, stopped, pressed activate and confirm, and slept 9 game hours.
  - **The refuel:** it drove on into Poland, into the station, stopped at the pump and filled up (846 l).
  - **The end:** it drove on to Szczecin and handed back 100 m before a traffic light (Phase 3). The run
    covered 17.6 of 17.9 world km.
- **The "hold to refuel" variant:** same plan; holding `activate` filled the same 846 l.
- **Fatigue off, 120 l:** refuel only, at a German station (€1.79/l) 105 game km ahead with a 1 game-km detour.
  The cheap Polish stations were out of reach with that fuel. No sleep. The decision record lists all 6
  candidate stations with measured detours. Two were on the other carriageway (171 and 308 game km round trips)
  and lost on the score.
- **Tests:** 113 pass (8 new mission tests). They cover:
  - the learned rates and fatigue detection;
  - the late factor and refitting it from history;
  - refuel by press, by hold, and the failure path; sleep and its failure path;
  - the planner's needs and choices on a road with a fuel station loop;
  - a whole job in the simulator that refuels on the way and is delivered, with the decision log and the job
    history checked;
  - re-engaging during a stop script drops it and re-plans (so it never sleeps twice after you took over).
- **Phase 3 fixes from the first in-game junction run:**
  - **Pedal check:** it waits 1.5 s after engaging (the first engage at standstill had handed back with "the game
    isn't taking AutoHaul's throttle").
  - **Queueing:** a standing vehicle ahead is a stop point at the 10 m standstill gap. AutoHaul had crept to 5 m.
  - **Reverse gear:** AutoHaul refuses to engage while the truck is in reverse.

## Test it (in the game)
1. Accept a job (freight market or quick job) and drive onto a mapped road.
2. Start AutoHaul in mission mode:
   ```powershell
   .venv\Scripts\python.exe tools\drive.py --mission
   ```
3. Press `/`. The status line shows the mission state and the next planned stop. To see a fuel stop sooner, start
   with little fuel. To see sleeps, switch fatigue on in the game's settings.
4. At the trailer's depot it hands back: couple the trailer, drive out onto the road, press `/` again. At the
   destination it hands back: park and deliver.
5. Afterwards, `logs\decisions.jsonl` (in `%USERPROFILE%\AutoHaul`) holds the reasons for everything it did.

## Not yet
- **Refuel and sleep screens:** the scripts are untested in the game. If the game wants a different key or
  timing, they hand back and say so; adjust `[mission.actions]`.
- **Still the driver's:** coupling, parking and delivery (Phase 5); ferries and trains; traffic lights and tolls
  (they still hand back, Phase 3).
- **Single legs:** stops are planned per leg (to the trailer, then to the destination). A stop the next leg will
  need isn't planned on the current leg.
- **One stop at a time:** a job needing two refuels plans the second one after the first.

## Fixes after the first full in-game test (2026-10-23, Kiel → Amsterdam)
- **"No route":** the lane map has seams of up to 1.6 m (around Amsterdam), which cut the depot off the lane
  graph. Lanes now join across gaps of up to 2.5 m (a lane is 4.5 m wide).
- **Hand-backs at lane changes (about 50 times):** when there's no room left for the change the route needs,
  AutoHaul now re-routes from the lane it's in (past the split, the other branch, back round). It hands back
  only if no such route exists.
- **Crash during a lane change (+4.2% damage at 69 km/h):** the map's lanes on that road were about 0.8 m off the
  real ones.
  - AutoHaul now keeps the lateral position you engaged at, up to 1.5 m off the mapped centre, fading back over
    about 1 km.
  - While that offset is over 0.4 m, it makes no automatic lane changes (it waits, or re-routes in lane).
- **Towns:** it engages up to 1.5 m off the mapped centre and at lower lane-match confidence. Planned turns may
  be down to 7 m radius without a trailer (12 m with one).
- **Company yards:** standing off the lanes, AutoHaul now drives out along a smooth curve to a lane within 60 m
  that has a route to the destination. That takes 5-10 s at up to 10 km/h. It then makes no lane change for
  200 m. It doesn't know about buildings or parked trailers: supervise.
  - In the simulator it drove out at all 7 places where it had refused to engage in your run.
- **Headlights:** low beam on from 20:00 to 07:00 game time while AutoHaul drives (`auto_headlights` in
  `config/autohaul.toml`). It switches off only what it switched on.
  - Wipers stay yours: the game's telemetry has no rain signal.
  - The horn isn't used.
- **Tests:** 118 pass, 5 new for these fixes.

## Fixes after the crash run (2026-10-23 evening)
- **Runaway after a focus loss (the crash, about 40% damage):** another window took the focus while AutoHaul had
  full throttle. The game reads no input while it isn't the front window and keeps the last pedals and steering.
  So the hand-back never reached it, and the truck drove on at full throttle for about a minute.
  - AutoHaul now brings the game straight back to the front. That took under 30 ms against the real game window.
  - If that hasn't worked within 0.5 s, it hands back and keeps trying for 10 s, so the released pedals arrive.
  - Settings: `refocus_game`, `refocus_grace_s`.
- **Slow in curves and turns:** you take curves at up to about 2.6 m/s^2 of sideways force (measured from your
  manual driving). AutoHaul never went past about 1.3.
  - Curve limit 1.4 -> 2.0 m/s^2. Junction turns 0.7 -> 1.4 m/s^2 (still at most 20 km/h inside a junction).
  - In the simulator the average speed went from 55 to 59 km/h, and the worst tracking error from 0.39 to
    0.58 m.
  - In your run AutoHaul was at full throttle 73% of the time below the planned speed: the loaded truck itself
    accelerates slowly (the automatic gearbox holds about 1,200 rpm).
- **Braking for cars that aren't in the way:** the car ahead is now chosen by heading.
  - Crossing traffic counts only when standing or crawling in the lane. Cars crossing at 40-50 km/h had made it
    brake at junctions.
  - Oncoming traffic counts only in the middle of the lane. One oncoming car had caused a full emergency brake at
    54 km/h.
  - Cars beside the truck don't count.
- **Engine, parking brake, gears** (`autohaul/drive/keeper.py`): on engaging, AutoHaul starts the engine,
  releases the parking brake, and shifts from neutral or reverse into drive. It holds the brake meanwhile, and
  hands back if 4 presses don't do it. It no longer refuses to engage with the parking brake on or in reverse.
- **Turns with a trailer:** planned turns down to 8 m radius (was 12 m, which handed back at most town turns).
- **Roads it refused (lane map):** seven road types were placed 4.5 m off: "Highway 2 lanes 5m offset", "Highway
  3 lanes 5m offset", and others with 5-25 m medians.
  - Their inner lane was missing, so "not on a mapped lane" and "lane match uncertain" were the top refusals.
  - The lanes now follow the offset in the road type's name; every road type with prefab evidence agrees with
    it.
  - Your manual driving on those roads is now a median 0.65 m from a mapped lane.
  - The lane map rebuilds by itself (cache version 4).
- **Driving without a job:** `drive.py --mission --to CITY` (or `CITY:COMPANY`, `X,Y`) drives there with fuel and
  rest stops whenever no job is active.
  - A city centre more than 60 m from a road now routes to the nearest road within 2 km. "paris" used to give
    "no route".
- **Simulator:** `tools/sim_drive.py --set FIELD=VALUE` overrides any speed or give-way setting.
- What full autonomy still needs: [AUTONOMY.md](AUTONOMY.md).
- **Engine start (2026-10-24):** in your run the keeper pressed `engine` 4 times and nothing happened. The next
  engage then failed at once, because the failed count carried over.
  - In 1.53 the engine starts only while `engine` is held (a 1.5 s hold started it); a press does nothing.
  - `engineelect` switches the electrics. The keeper switches them on first if needed.
  - The count of failed tries now resets on every engage.

## Depots: quick park (2026-10-29, first step of Phase 5)
At a depot AutoHaul no longer just hands back. It first tries the game's own quick park (the `quickpark` input, Q
in your controls), which parks the trailer, or hooks it up, by itself (`QuickPark` in
`autohaul/mission/actions.py`).
- **At the destination:**
  1. Stop where the route ends (about 60 m before the depot point).
  2. Press `quickpark`, and wait up to 8 s for the game's delivery event; then press once more.
  3. Delivered: AutoHaul hands back with "delivered by quick park (pick the next job)". Picking a job stays
     yours.
- **At the trailer's depot:** the same, until the trailer is attached (telemetry) or the job shows the cargo
  loaded. Then it drives out of the yard and on to the destination.
- **No result after 2 presses:** it hands back as before ("park and deliver yourself" / "couple the trailer
  yourself").
- **Simulator:** the simulated game answers `quickpark` at the depots. Tests cover delivery and the hand-back
  when nothing happens.
- **Turn it off:** `quick_park = false` in `config/autohaul.toml` `[mission]`.
- **Not checked in the game:**
  - Whether quick park is offered at the point where AutoHaul stops.
  - Whether it hooks up a trailer at the source depot (1.53 may offer it only for delivery).
  - Reversing under a trailer and into a slot without quick park is not built.
- **Also:** planned turns down to 6 m radius, with or without a trailer. Two 7-8 m town turns in your last run
  handed back.
