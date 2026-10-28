"""Phase 4: the mission manager, stop planning, economics, the stop scripts, and a whole job in the simulator."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.mission import economics
from autohaul.mission.actions import ActionParams, Refuel, Sleep
from autohaul.mission.decisions import DecisionLog
from autohaul.mission.economics import EconParams
from autohaul.mission.job import JobHistory, clock, duration
from autohaul.mission.manager import Mission, MissionDriver, MissionParams
from autohaul.mission.planner import PlanState, StopPlanner
from autohaul.mission.rates import Rates
from autohaul.sim.course import arc_polyline
from autohaul.sim.game import SimGame
from autohaul.sim.truck import TruckSim
from autohaul.world.bundle import World
from autohaul.world.graph import LaneGraph
from autohaul.world.junctions import Junctions
from autohaul.world.lanemap import KIND_ROAD
from autohaul.world.places import Place
from autohaul.world.route import Router
from autohaul.world.services import ServicePoint, Services
from tests.test_route import build


def test_clock_and_duration():
    assert clock(0) == "Mon 00:00"
    assert clock(24 * 60 + 5 * 60 + 46) == "Tue 05:46"
    assert clock(7 * 24 * 60) == "Mon 00:00"
    assert duration(434) == "7h 14min"
    assert duration(-65) == "-1h 05min"


def test_rates_learn_fuel_time_and_fatigue():
    r = Rates()
    r.seed(0.35)                                     # litres per game km on the dashboard
    assert r.fuel_per_m() == pytest.approx(0.35 * 19 / 1000)
    fuel, t, rest = 500.0, 600.0, 400.0
    for i in range(1, 2001):                         # 2 km at 20 m/s, 10 Hz
        fuel -= 2.0 * 0.004                          # 0.004 l per world metre
        t += 0.1 * 19 / 60                           # game minutes at scale 19
        rest -= 0.1 * 19 / 60
        r.update(i * 2.0, 0.0, 20.0, fuel, t, rest)
    assert r.fuel_per_m() == pytest.approx(0.004, rel=0.05)
    assert r.min_per_m == pytest.approx(19 / (20 * 60), rel=0.05)
    assert r.fatigue_on is True
    off = Rates()
    for i in range(1, 2001):
        off.update(i * 2.0, 0.0, 20.0, 500.0, 600 + i * 0.1 * 19 / 60, 400.0)
    assert off.fatigue_on is False


def test_late_factor_and_calibration():
    p = EconParams()
    assert economics.late_factor(-10, p) == 1.0
    assert economics.late_factor(60, p) == pytest.approx(1 - p.late_fixed - p.late_per_hour)
    assert economics.late_factor(10_000, p) == p.late_floor
    hist = [{"event": "delivered", "job": {"income": 10000}, "revenue": int(10000 * (1 - 0.1 - 0.02 * h)),
             "late_min": h * 60, "job_minutes": 600} for h in (1, 3, 5, 8)]
    fitted, note = economics.calibrate(hist, p)
    assert fitted.late_fixed == pytest.approx(0.1, abs=0.01) and fitted.late_per_hour == pytest.approx(0.02, abs=0.005)
    assert "late penalty" in note and "time value" in note


def _frame(**kw):
    base = dict(speed=0.0, fuel=100.0, game_time=600, next_rest_stop=300)
    base.update(kw)
    return SimpleNamespace(**base)


def test_refuel_by_press_and_by_hold_and_failure():
    p = ActionParams(settle_s=0.2, flow_wait_s=1.0, stall_s=0.5)
    # press mode: one press starts the flow
    r, fuel, pressed = Refuel(500.0, p), 100.0, []
    for _ in range(400):
        out = r.step(_frame(fuel=fuel), 0.05)
        pressed += out.press
        if pressed:
            fuel = min(500.0, fuel + 2.0)
        if out.done or out.failed:
            break
    assert out.done and pressed == ["activate"] and r.litres >= 385          # filled to 98 %
    # hold mode: a press does nothing; holding fills
    r, fuel, holding = Refuel(500.0, p), 100.0, False
    for _ in range(800):
        out = r.step(_frame(fuel=fuel), 0.05)
        holding = out.hold.get("activate", holding)
        if holding:
            fuel = min(500.0, fuel + 2.0)
        if out.done or out.failed:
            break
    assert out.done and r.litres >= 385 and out.hold.get("activate") is False
    # nothing happens at all
    r = Refuel(500.0, p)
    for _ in range(400):
        out = r.step(_frame(), 0.05)
        if out.done or out.failed:
            break
    assert out.failed and "refuel yourself" in out.failed


def test_sleep_script_and_failure():
    p = ActionParams(settle_s=0.2, sleep_open_s=0.5, sleep_wait_s=2.0)
    s, t, rest, keys = Sleep(p), 600.0, 30.0, []
    for _ in range(200):
        out = s.step(_frame(game_time=t, next_rest_stop=rest), 0.05)
        keys += out.press
        if "selectfcs" in out.press:
            t, rest = t + 540, 660.0
        if out.done or out.failed:
            break
    assert out.done and keys == ["activate", "selectfcs"] and s.slept_min == pytest.approx(540)
    s = Sleep(p)
    for _ in range(400):
        out = s.step(_frame(), 0.05)
        if out.done or out.failed:
            break
    assert out.failed and "sleep yourself" in out.failed


# ---------------------------------------------------------------- a road with a fuel station beside it

def make_station_world():
    """Main road east: A (0-1000 m), B1 (1000-1300), B2 (1300-4000). A fuel station loop: exit E off the end of
    A, pump lane P (parallel, ~13 m north... in map coordinates +y), return R onto B2. Pump in the middle of P."""
    a = arc_polyline([(1000, 0.0)])
    b1 = arc_polyline([(300, 0.0)], x=1000)
    b2 = arc_polyline([(2700, 0.0)], x=1300)
    k, ln = 1 / 120.0, 40.0
    e = arc_polyline([(ln, k), (ln, -k)], x=1000, y=0.0, step=2.0)
    dx, dy = e[-1, 0] - 1000, e[-1, 1]
    r = arc_polyline([(ln, -k), (ln, k)], x=1300 - dx, y=dy, step=2.0)
    p = arc_polyline([(300 - 2 * dx, 0.0)], x=1000 + dx, y=dy, step=4.0)
    lanes = [(a, KIND_ROAD, 1, 1), (b1, KIND_ROAD, 2, 1), (b2, KIND_ROAD, 3, 1), (e, KIND_ROAD, 4, 1),
             (p, KIND_ROAD, 5, 1), (r, KIND_ROAD, 6, 1)]
    lm = build(lanes)
    g = LaneGraph.build(lm, log=lambda *_: None)
    pump_s = lm.lane_length(4) / 2
    pump = ServicePoint("fuel", float(p[0, 0] + pump_s), float(dy), 4, pump_s, 0.0, "st1", "germany", 1.40)
    bed = ServicePoint("sleep", float(p[0, 0] + pump_s + 20), float(dy + 8), 4, pump_s + 20, 8.0, "st1", "germany",
                       None)
    places = SimpleNamespace(company=lambda city, company: Place(3800.0, 0.0, f"{company} in {city}"))
    world = World(Path("."), lm, g, Router(lm, g), Junctions(lm), None, places, np.zeros((0, 2)))
    return world, Services.from_points(lm, [pump, bed], default_price=1.6), pump, bed


def test_planner_needs_and_choices():
    world, services, pump, bed = make_station_world()
    rates = Rates()
    rates.seed(0.33)
    planner = StopPlanner(world.router, services, rates)
    rt = world.router.route(0, 100.0, (3800.0, 0.0))
    base = dict(capacity_l=500.0, game_time=600.0, income=10000.0, deadline=600.0 + 20 * 60)
    plan = planner.plan(PlanState(rt, fuel_l=300.0, next_rest=None, **base), 100.0)
    assert plan.stops == [] and plan.record["choice"] == "no stop"
    plan = planner.plan(PlanState(rt, fuel_l=50.0, next_rest=None, **base), 100.0)
    assert [s.kind for s in plan.stops] == ["fuel"] and plan.stops[0].point is pump
    assert 0 < plan.stops[0].detour_m < 120 and plan.stops[0].litres > 400
    assert "refuel" in plan.record["rationale"] and plan.record["options"]
    plan = planner.plan(PlanState(rt, fuel_l=50.0, next_rest=100.0, **base), 100.0)   # 40 min to spare
    assert sorted(s.kind for s in plan.stops) == ["fuel", "sleep"]
    plan = planner.plan(PlanState(rt, fuel_l=10.0, next_rest=None, **base), 100.0)
    assert plan.problem and "no fuel station reachable" in plan.problem


def test_whole_job_refuels_on_the_way_and_delivers(tmp_path):
    world, services, pump, bed = make_station_world()
    geom = TruckGeometry()
    sim = TruckSim(geom, x=150.0, y=0.0, rotation=0.0, speed=15.0)
    job = {"cargo": "tests", "loaded": True, "deadline": 600 + 10 * 60, "income": 9000, "source_city": "a",
           "source_company": "s", "destination_city": "b", "destination_company": "d"}
    game = SimGame(sim, job=job, fuel_l=60.0, capacity_l=500.0, game_time=600, fatigue=False,
                   pumps=[(pump.x, pump.y)], parkings=[(bed.x, bed.y)], quick_park=False)
    log = DecisionLog(tmp_path / "decisions.jsonl")
    history = JobHistory(tmp_path / "jobs.jsonl")
    mission = Mission(world, services, log, history, MissionParams(fatigue="off"),
                      action_params=ActionParams(settle_s=0.3))
    ap = Autopilot(world.lm, geom, "relative", router=world.router, junctions=world.junctions)
    md = MissionDriver(ap, mission)
    mission.update(game.frame(80 / 3.6), game.config(), [])
    assert md.engage(game.frame(80 / 3.6)) is None
    states, fault = [], None
    for _ in range(int(400 / sim.p.dt)):
        out = md.step(game.frame(80 / 3.6), game.config(), game.events(), [], sim.p.dt)
        if not states or states[-1] != out.mission.state:
            states.append(out.mission.state)
        if out.fault:
            fault = out.fault
            break
        game.buttons(out.press, out.hold)
        game.step(out.cmd.steer, out.cmd.throttle, out.cmd.brake)
    assert states[:3] == ["to stop", "at stop", "to destination"], states
    assert game.fuel > 400                                         # filled up at the pump
    assert any("refuel started" in what for _, what in game.log)
    assert fault and fault.startswith("quick park didn't deliver")      # quick park off in this game: hand back
    game.deliver()
    mission.update(game.frame(), game.config(), game.events())
    kinds = [r["decision"] for r in log.records]
    assert kinds[:2] == ["job", "plan"] and "stop" in kinds and kinds[-1] == "delivered"
    assert (tmp_path / "decisions.jsonl").read_text().count("\n") == len(log.records)
    assert history.load()[-1]["revenue"] == 9000 and history.load()[-1]["late_min"] < 0


def test_reengaging_during_a_stop_script_drops_it_and_replans(tmp_path):
    world, services, pump, bed = make_station_world()
    geom = TruckGeometry()
    sim = TruckSim(geom, x=150.0, y=0.0, rotation=0.0, speed=15.0)
    job = {"cargo": "tests", "loaded": True, "deadline": 600 + 10 * 60, "income": 9000, "source_city": "a",
           "source_company": "s", "destination_city": "b", "destination_company": "d"}
    game = SimGame(sim, job=job, fuel_l=60.0, capacity_l=500.0, game_time=600, fatigue=False,
                   pumps=[(pump.x, pump.y)], parkings=[(bed.x, bed.y)])
    log = DecisionLog(None)
    mission = Mission(world, services, log, JobHistory(tmp_path / "jobs.jsonl"), MissionParams(fatigue="off"),
                      action_params=ActionParams(settle_s=5.0))
    md = MissionDriver(Autopilot(world.lm, geom, "relative", router=world.router, junctions=world.junctions), mission)
    mission.update(game.frame(80 / 3.6), game.config(), [])
    assert md.engage(game.frame(80 / 3.6)) is None
    for _ in range(int(200 / sim.p.dt)):
        out = md.step(game.frame(80 / 3.6), game.config(), game.events(), [], sim.p.dt)
        if mission.state == "at stop":
            break
        game.step(out.cmd.steer, out.cmd.throttle, out.cmd.brake)
    assert mission.state == "at stop" and mission.action is not None
    game.fuel = 480.0                                # the driver took over and filled up
    md.engage(game.frame(80 / 3.6))
    assert mission.action is None and mission.state == "to destination"
    assert log.records[-1]["result"] == "interrupted by the driver"
    out = md.step(game.frame(80 / 3.6), game.config(), [], [], sim.p.dt)
    assert mission.plan is not None and mission.plan.stops == []   # re-planned: no stop needed any more


def test_without_a_job_it_drives_to_a_chosen_place_refuelling_on_the_way(tmp_path):
    world, services, pump, bed = make_station_world()
    geom = TruckGeometry()
    sim = TruckSim(geom, x=150.0, y=0.0, rotation=0.0, speed=15.0)
    game = SimGame(sim, job=None, fuel_l=60.0, capacity_l=500.0, game_time=600, fatigue=False,
                   pumps=[(pump.x, pump.y)], parkings=[(bed.x, bed.y)])
    mission = Mission(world, services, DecisionLog(tmp_path / "decisions.jsonl"), JobHistory(tmp_path / "jobs.jsonl"),
                      MissionParams(fatigue="off"), action_params=ActionParams(settle_s=0.3))
    md = MissionDriver(Autopilot(world.lm, geom, "relative", router=world.router, junctions=world.junctions), mission)
    mission.update(game.frame(80 / 3.6), game.config(), [])
    assert mission.target() is None                                 # no job, no place: nothing to drive to
    mission.set_free_target(Place(3800.0, 0.0, "the car park"))
    assert mission.state == "to place"
    assert md.engage(game.frame(80 / 3.6)) is None
    states, fault = [], None
    for _ in range(int(400 / sim.p.dt)):
        out = md.step(game.frame(80 / 3.6), game.config(), game.events(), [], sim.p.dt)
        if not states or states[-1] != out.mission.state:
            states.append(out.mission.state)
        if out.fault:
            fault = out.fault
            break
        game.buttons(out.press, out.hold)
        game.step(out.cmd.steer, out.cmd.throttle, out.cmd.brake)
    assert states[:3] == ["to stop", "at stop", "to place"], states
    assert game.fuel > 400
    assert fault == "at the destination: the car park" and mission.state == "arrived"


def _park_run(tmp_path, loaded: bool, quick_park: bool):
    world, services, pump, bed = make_station_world()
    geom = TruckGeometry()
    sim = TruckSim(geom, x=2600.0, y=0.0, rotation=0.0, speed=15.0)       # past the station, 1.2 km out
    job = {"cargo": "tests", "loaded": loaded, "deadline": 600 + 600, "income": 9000, "source_city": "a",
           "source_company": "s", "destination_city": "b", "destination_company": "d"}
    game = SimGame(sim, job=job, fuel_l=400.0, capacity_l=500.0, game_time=600, fatigue=False, quick_park=quick_park)
    log = DecisionLog(tmp_path / "decisions.jsonl")
    mission = Mission(world, services, log, JobHistory(tmp_path / "jobs.jsonl"), MissionParams(fatigue="off"),
                      action_params=ActionParams(settle_s=0.3, park_wait_s=2.0))
    md = MissionDriver(Autopilot(world.lm, geom, "relative", router=world.router, junctions=world.junctions), mission)
    mission.update(game.frame(80 / 3.6), game.config(), [])
    assert md.engage(game.frame(80 / 3.6)) is None
    fault, pressed = None, []
    for _ in range(int(200 / sim.p.dt)):
        out = md.step(game.frame(80 / 3.6), game.config(), game.events(), [], sim.p.dt)
        pressed += [b for b in out.press if b == "quickpark"]
        if out.fault or out.status.fault:              # as drive.py: either hands back
            fault = out.fault or out.status.fault
            break
        game.buttons(out.press, out.hold)
        game.step(out.cmd.steer, out.cmd.throttle, out.cmd.brake)
    return fault, pressed, game, log


def test_quick_park_delivers_at_the_destination(tmp_path):
    fault, pressed, game, log = _park_run(tmp_path, loaded=True, quick_park=True)
    assert fault == "at the destination: delivered by quick park (pick the next job)"
    assert pressed == ["quickpark"] and game.job is None
    assert any(r["decision"] == "delivered" for r in log.records)


def test_quick_park_that_does_nothing_hands_back(tmp_path):
    fault, pressed, game, log = _park_run(tmp_path, loaded=True, quick_park=False)
    assert fault and fault.startswith("quick park didn't deliver") and len(pressed) == 2
