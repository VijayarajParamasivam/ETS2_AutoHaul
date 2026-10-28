# What full autonomy still needs

Status 2026-10-23. The goal: accept a job, or name a place, and AutoHaul does the whole trip with nobody at the
keyboard. This lists what it can do now, and every place a run still stops for a human, most frequent first.
The counts come from the 16 recorded drives so far (136 engagements).

## Done
- **Driving:** lane keeping, speed limits, curves, following traffic, lane changes for the route, re-routing.
- **Junctions:** give way, right of way, stop signs, indicators, and traffic lights read from the screen.
- **Trips:** routes to the job's depots, or to any place without a job (`--mission --to CITY`). Fuel and rest
  stops are planned on the way and done by themselves.
- **Truck:** starts the engine, releases the parking brake, shifts into drive. Headlights at night.
- **Safety:** brings the game back to the front if another window takes the focus.

## Still stops for a human

| # | What | How often now | What it takes |
|---|------|---------------|---------------|
| 1 | **Traffic lights** | Built 2026-10-24 (see docs/PHASE3.md); not yet driven in the game. | Colours are read from the screen. Lights it can't see still stop it, then hand back after 6 s. |
| 2 | **Coupling and delivery (Phase 5)** | Every job, twice: at the trailer and at the destination. | Reversing to the trailer, coupling, reversing into a parking slot, and the delivery screens. Needs a reversing controller for an articulated truck. With the game's easier parking options, the slot can be simpler. |
| 3 | **Toll gates and border barriers** | Toll gates built 2026-10-24 (stop, pay with activate, drive through); not yet tried in the game. Borders, barriers and weigh stations still hand back. | Border checks: stop and wait for the barrier, like a toll without paying. |
| 4 | **Roads the lane map gets wrong** | Was the top refusal: "not on a mapped lane" (104) and "lane match uncertain" (46). | Fixed on 2026-10-23 for the 7 road types that were placed 4.5 m off ("Highway 2 lanes 5m offset" and similar). Your manual driving is now a median 0.65 m from a mapped lane, with 2.4% of samples more than 4 m off. The rest: lane seams, and town roads whose lanes the map doesn't place exactly. Checking more recorded drives will find more. |
| 5 | **Taking a job** | Every job. | The freight market is a game menu with no API. Either the screen is driven by clicks and keys (fragile), or you pick the job and AutoHaul does the rest. |
| 6 | **Ferries and trains** | Routes that need them. | The router doesn't use ferries. Needs: ferry links in the route, drive into the terminal, confirm the crossing screen. |
| 7 | **Rain and wipers** | When it rains. | The game's telemetry has no rain signal. Options: detect raindrops on the windscreen from screenshots, or switch the wipers on at a fixed level. Not built. |
| 8 | **Obstacles the game doesn't report** | Yards and car parks. | Parked trailers, walls and props aren't in the traffic data. AutoHaul only sees moving AI vehicles. Needs a camera-based check, or slow driving plus your supervision in yards. |
| 9 | **Damage and repair** | After crashes and long runs. | It never drives to a service station. Needs: a damage threshold, then plan a repair stop like a fuel stop. |
| 10 | **Long unattended runs (Phase 6)** | Hours of driving. | A watchdog that restarts after a game pause or freeze. Recovery after a hand-back. Daily logs to review. |

## Smaller gaps
- **Speed on hills and after curves:** the automatic gearbox holds about 1,200 rpm, so a loaded truck picks up
  speed slowly. Shifting down early (the game's shift buttons) could help. Not built.
- **The game's own GPS destination:** the 1.53 plugin exports no GPS route, so AutoHaul can't read where you set
  the GPS. Use `--to CITY`, `--to CITY:COMPANY` or `--to X,Y` instead.
- **Trailer swing:** turns are planned for the truck's path. The trailer cuts corners. AutoHaul now takes turns
  down to 8 m radius with a trailer (the game's junction curves are made for trucks with trailers); tight town
  corners still need supervising.

## Suggested order
1. Traffic lights in the game: drive a town and check the light log.
2. Toll gates and barriers. Same machinery as the stop scripts.
3. Coupling and delivery (Phase 5). The largest piece.
4. Damage-based repair stops, then the long-run watchdog (Phase 6).
5. Wipers from screenshots, ferries, picking jobs.
