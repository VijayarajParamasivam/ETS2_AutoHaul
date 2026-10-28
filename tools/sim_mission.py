"""Run a whole job in the simulator on the REAL map (no game needed): the mission manager plans fuel and rest
stops, drives to them, refuels and sleeps through the simulated game, and logs every decision.

Example (Magdeburg motorway -> itcc in Szczecin, low fuel, rest due soon):
    python tools/sim_mission.py --x 1618.13 --y -9160.99 --rot 17.435 --to szczecin:itcc --fuel 140 --rest-min 200

Hand-backs a driver would handle (traffic lights, tolls, the depot) end the run, except the arrival at the
destination depot, which is "delivered" so the job.delivered path runs too. Decisions go to
logs/sim-mission-decisions.jsonl.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.control.speed import SpeedParams  # noqa: E402
from autohaul.control.vehicle import TruckGeometry  # noqa: E402
from autohaul.drive.autopilot import Autopilot  # noqa: E402
from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import logs_dir, map_dir  # noqa: E402
from autohaul.mission.decisions import DecisionLog  # noqa: E402
from autohaul.mission.job import JobHistory, clock, duration  # noqa: E402
from autohaul.mission.manager import Mission, MissionDriver  # noqa: E402
from autohaul.sim.game import SimGame  # noqa: E402
from autohaul.sim.truck import SimParams, TruckSim  # noqa: E402
from autohaul.world.bundle import load_world  # noqa: E402
from autohaul.world.services import Services  # noqa: E402


def main() -> int:
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--x", type=float, required=True)
    a.add_argument("--y", type=float, required=True)
    a.add_argument("--rot", type=float, required=True, help="degrees, map rotation")
    a.add_argument("--to", required=True, help="destination CITY:COMPANY (tokens)")
    a.add_argument("--fuel", type=float, default=140.0, help="litres in the tank at the start")
    a.add_argument("--capacity", type=float, default=900.0)
    a.add_argument("--rest-min", type=float, default=200.0, help="game minutes until a rest is due")
    a.add_argument("--no-fatigue", action="store_true")
    a.add_argument("--clock", type=float, default=8 * 60, help="start time, game minutes (Monday 08:00)")
    a.add_argument("--deadline-h", type=float, default=16.0, help="deadline, game hours after the start")
    a.add_argument("--income", type=int, default=18000)
    a.add_argument("--refuel-mode", choices=["press", "hold"], default="hold")
    a.add_argument("--limit-kmh", type=float, default=80.0)
    a.add_argument("--minutes", type=float, default=40.0, help="simulated real minutes at most")
    args = a.parse_args()

    cfg = load_config()
    md = map_dir(cfg)
    t0 = time.time()
    world = load_world(md, log=lambda *_: None)
    services = Services(world.lm, md, no_go=world.rules.cars_only() if world.rules else None)
    print(f"world loaded in {time.time() - t0:.1f} s")
    city, company = args.to.split(":")
    dest = world.places.company(city, company)
    if dest is None:
        print(f"unknown destination {args.to!r}")
        return 1
    job = {"cargo": "machinery", "loaded": True, "deadline": args.clock + args.deadline_h * 60,
           "income": args.income, "source_city": "magdeburg", "source_company": "sim",
           "destination_city": city, "destination_company": company}
    geom = TruckGeometry()
    sim = TruckSim(geom, SimParams(mass=20000.0), x=args.x, y=args.y, rotation=math.radians(args.rot), speed=60 / 3.6)
    game = SimGame(sim, job=job, fuel_l=args.fuel, capacity_l=args.capacity, game_time=args.clock,
                   next_rest_min=args.rest_min, fatigue=not args.no_fatigue, refuel_mode=args.refuel_mode,
                   pumps=[(p.x, p.y) for p in services.fuel], parkings=[(p.x, p.y) for p in services.sleep])
    log_path = logs_dir(cfg) / "sim-mission-decisions.jsonl"
    log_path.unlink(missing_ok=True)
    decisions = DecisionLog(log_path)
    mission = Mission(world, services, decisions, JobHistory(logs_dir(cfg) / "sim-jobs.jsonl"))
    mission.profile_fatigue = not args.no_fatigue        # as drive.py reads it from the profile (g_fatigue)
    ap = Autopilot(world.lm, geom, "relative", speed_params=SpeedParams(max_speed_kmh=args.limit_kmh),
                   hazards=world.hazards, router=world.router, junctions=world.junctions, rules=world.rules)
    md_ = MissionDriver(ap, mission)
    limit = args.limit_kmh / 3.6
    mission.update(game.frame(limit), game.config(), [])
    reason = md_.engage(game.frame(limit))
    if reason:
        print(f"engage refused: {reason}")
        return 1
    print(f"job: {mission.status().job}; fuel {args.fuel:.0f} l, rest due in {duration(args.rest_min)}")
    seen = 0
    last_state, last_print = "", 0.0
    dist = 0.0
    end = "time up"
    for _ in range(int(args.minutes * 60 / sim.p.dt)):
        frame = game.frame(limit)
        out = md_.step(frame, game.config(), game.events(), [], sim.p.dt)
        for rec in decisions.records[seen:]:
            print(f"  [{clock(game.game_time)}] {rec['decision']}: " +
                  str(rec.get("rationale") or rec.get("result") or rec.get("reason") or
                      {k: v for k, v in rec.items() if k in ("cargo", "from", "to", "income", "deadline",
                                                              "time_left", "revenue", "late_min")}))
        seen = len(decisions.records)
        ms = out.mission
        if ms.state != last_state:
            print(f"  t={sim.t:6.0f}s {clock(game.game_time)} state: {ms.state}  {ms.next_stop or ''}")
            last_state = ms.state
        if sim.t - last_print > 60:
            last_print = sim.t
            print(f"  t={sim.t:6.0f}s {clock(game.game_time)} {sim.s.speed * 3.6:4.0f} km/h fuel {game.fuel:4.0f} l "
                  f"rest {duration(game.next_rest)} | {out.status.target_reason} | {out.status.lane_desc}")
        if out.fault:
            if mission.state == "at destination":
                print(f"  t={sim.t:6.0f}s {out.fault} -> the driver parks; the game reports the delivery")
                game.deliver()
                mission.update(game.frame(limit), game.config(), game.events())
                end = "delivered"
            else:
                end = f"handed back: {out.fault}"
            break
        game.buttons(out.press, out.hold)
        game.step(out.cmd.steer, out.cmd.throttle, out.cmd.brake)
        dist += abs(sim.s.speed) * sim.p.dt
    for rec in decisions.records[seen:]:
        print(f"  [{clock(game.game_time)}] {rec['decision']}: {rec.get('rationale') or rec.get('revenue') or ''}")
    print(f"\n{end}: {dist / 1000:.1f} world km (~{dist * 19 / 1000:.0f} game km) in {sim.t / 60:.1f} real min; "
          f"game clock {clock(game.game_time)}; fuel {game.fuel:.0f} l; rest timer {duration(game.next_rest)}")
    print("game:", "; ".join(f"t={t:.0f}s {what}" for t, what in game.log))
    print(f"decisions: {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
