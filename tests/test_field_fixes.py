"""What the drives of 2026-10-29/30 showed: detours, fuel planning and indicators."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.drive.indicators import IndicatorControl
from autohaul.mission.planner import PlanState, StopPlanner
from autohaul.mission.rates import Rates
from autohaul.sim.course import arc_polyline, offset
from autohaul.sim.truck import SimVehicle, TruckSim
from autohaul.world.graph import LaneGraph
from autohaul.world.lanemap import KIND_PREFAB, KIND_ROAD
from autohaul.world.route import Router
from tests.test_mission import make_station_world
from tests.test_route import build


def _exit_map(road_m: float = 500.0):
    """Two lanes (0 left, 1 right), an exit from the right lane only (3), which leads to a two-lane road (4 right,
    5 left); the left lane goes straight on (2 -> 6)."""
    left = arc_polyline([(road_m, 0.0)])
    right = offset(left, 4.5)
    straight = arc_polyline([(60, 0.0)], x=road_m, y=0)
    exit_ = arc_polyline([(60, 1 / 80)], x=road_m, y=4.5)
    e, r = exit_[-1], math.atan2(exit_[-1, 1] - exit_[-2, 1], exit_[-1, 0] - exit_[-2, 0])
    after_right = arc_polyline([(1500, 0.0)], x=e[0], y=e[1], rot=r)
    after_left = offset(after_right, -4.5)
    on = arc_polyline([(1500, 0.0)], x=road_m + 60, y=0)
    lm = build([(left, KIND_ROAD, 1, 1), (right, KIND_ROAD, 1, 2), (straight, KIND_PREFAB, 7, 0),
                (exit_, KIND_PREFAB, 7, 1), (after_right, KIND_ROAD, 9, 2), (after_left, KIND_ROAD, 9, 1),
                (on, KIND_ROAD, 8, 1)])
    return lm, LaneGraph.build(lm, log=lambda *_: None)


def _run(lm, g, start, goal, seconds=60.0, traffic=None, speed=60 / 3.6, **kw):
    geom = TruckGeometry()
    pts = lm.lane_points(start)
    rot = math.atan2(pts[1, 1] - pts[0, 1], pts[1, 0] - pts[0, 0])
    x0, y0 = pts[0] + 20 * np.array([math.cos(rot), math.sin(rot)])
    sim = TruckSim(geom, x=x0, y=y0, rotation=rot, speed=speed)
    ap = Autopilot(lm, geom, "relative", router=Router(lm, g), **kw)
    xy = lm.lane_points(goal[0])[-1]
    ap.set_destination(float(xy[0]), float(xy[1]), goal=goal)
    assert ap.engage(sim.frame(80 / 3.6)) is None
    log = []
    for _ in range(int(seconds / sim.p.dt)):
        cmd, st = ap.step(sim.frame(80 / 3.6), [v.record() for v in (traffic or [])], sim.p.dt)
        for v in traffic or []:
            v.step(sim.t, sim.p.dt)
        log.append((sim.t, st.lane, st.indicator, st.lane_change, st.fault))
        if st.fault:
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    return log, ap


def test_no_early_lane_change_away_from_an_exit_the_route_takes():
    """2026-10-29, near Paris: the route needed the left lane 3 km on, past an exit on the right. AutoHaul moved
    left as early as it could, before the exit, and missed it (23 game km)."""
    lm, g = _exit_map()
    log, _ = _run(lm, g, 1, (5, 1000.0), seconds=70.0)
    lanes = []
    for _, lane, *_rest in log:
        if lane is not None and (not lanes or lanes[-1] != lane):
            lanes.append(lane)
    assert 0 not in lanes and 2 not in lanes, lanes       # never into the lane that goes straight on
    assert 3 in lanes and lanes[-1] == 5, lanes           # took the exit, then changed lanes


def test_an_early_change_still_happens_where_the_lanes_run_on_together():
    lm, g = _exit_map()
    log, _ = _run(lm, g, 0, (3, 30.0), seconds=25.0)      # left lane, the exit needs the right one
    assert any(lane == 1 for _, lane, *_ in log)


def test_no_indicator_while_waiting_for_a_gap_with_time_to_spare():
    """2026-10-30: indicators on for up to 76 s, waiting for gaps, a long way before the change was due."""
    lm, g = _exit_map(road_m=1600.0)
    from autohaul.control.path import PathRef
    right = PathRef(lm.lane_points(1))
    blocker = SimVehicle(1, right, s=20.0, speed=60 / 3.6, length=16.0, width=2.5)   # alongside, same speed
    log, _ = _run(lm, g, 0, (3, 30.0), seconds=20.0, traffic=[blocker])
    waiting = [row for row in log if row[3] and "target lane" in row[3] and row[0] < 18.0]
    assert waiting, "expected it to be waiting for a gap"
    assert all(row[2] is None for row in waiting)


def test_indicator_on_while_waiting_when_the_change_is_close():
    lm, g = _exit_map(road_m=250.0)
    from autohaul.control.path import PathRef
    right = PathRef(lm.lane_points(1))
    blocker = SimVehicle(1, right, s=20.0, speed=60 / 3.6, length=16.0, width=2.5)
    log, _ = _run(lm, g, 0, (3, 30.0), seconds=3.0, traffic=[blocker])
    assert any(row[2] == "right" for row in log)


class _Blinkers:
    def __init__(self):
        self.pressed = []

    def press(self, name):
        self.pressed.append(name)


def test_indicator_switches_on_and_off_only_on_a_mismatch():
    b = _Blinkers()
    c = IndicatorControl(b.press)
    c.step(False, False, "left", 1.0)
    assert b.pressed == ["lblinker"] and c.mine == "left"
    c.step(False, False, "left", 1.2)                     # too soon: telemetry may not show it yet
    assert b.pressed == ["lblinker"]
    c.step(True, False, "left", 2.0)
    assert b.pressed == ["lblinker"]
    c.step(True, False, None, 3.0)
    assert b.pressed == ["lblinker", "lblinker"] and c.mine is None


def test_a_stray_indicator_is_cancelled_after_the_drivers_grace_time():
    """It used to stay on for the rest of the drive: only one AutoHaul remembered switching on was switched off."""
    b = _Blinkers()
    c = IndicatorControl(b.press)
    c.step(False, True, None, 10.0)                       # on, not ours: the driver's, for now
    c.step(False, True, None, 15.0)
    assert b.pressed == []
    c.step(False, True, None, 18.5)
    assert b.pressed == ["rblinker"]


def test_an_indicator_already_on_when_wanted_is_taken_over():
    b = _Blinkers()
    c = IndicatorControl(b.press)
    c.step(True, False, "left", 1.0)
    assert b.pressed == [] and c.mine == "left"
    c.step(True, False, None, 2.0)
    assert b.pressed == ["lblinker"]


def test_cars_only_lanes_cost_extra_but_arent_impassable():
    """2026-10-29, Amsterdam -> Paris: 3 short lanes flagged for small vehicles made the route 65 game km longer
    than the game's GPS, which sends trucks through them."""
    a = arc_polyline([(100, 0.0)])
    short = arc_polyline([(40, 0.0)], x=100, y=0)
    b = arc_polyline([(100, 0.0)], x=140, y=0)
    round1 = arc_polyline([(60, 0.0), (1200, 1 / 150), (60, 0.0)], x=100, y=0)
    lm = build([(a, KIND_ROAD, 1, 1), (short, KIND_PREFAB, 2, 0), (b, KIND_ROAD, 3, 1), (round1, KIND_ROAD, 4, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    soft = np.array([False, True, False, False])
    r = Router(lm, g, avoid=soft)
    rt = r.route(0, 10.0, tuple(lm.lane_points(2)[-1]), goal_radius=5.0)
    assert rt is not None and rt.lanes == [0, 1, 2]
    hard = Router(lm, g, no_go=soft)
    assert hard.route(0, 10.0, tuple(lm.lane_points(2)[-1]), goal_radius=5.0) is None
    assert 1 not in r.goal_lanes(*lm.lane_points(1)[10], 3.0)   # never a destination


def test_fuel_rate_isnt_skewed_by_a_town():
    """2026-10-30: the first 400 m in a town (scale 3) gave 0.13 l/game km against the real 0.49."""
    rates = Rates()
    rates.seed(0.5)
    x, fuel, t = 0.0, 500.0, 600.0
    for scale, metres in ((3.0, 2000.0), (19.0, 2000.0)):
        for _ in range(int(metres / 10)):
            x += 10.0
            fuel -= 10.0 * scale / 1000.0 * 0.49
            t += 10.0 * scale / (20.0 * 60.0)
            rates.update(x, 0.0, 20.0, fuel, t, None, 0.5, scale)
    assert rates.fuel_per_m() * 1000.0 / 19.0 == pytest.approx(0.49, abs=0.02)


def test_a_stop_is_planned_when_the_tank_would_arrive_nearly_empty():
    world, services, pump, bed = make_station_world()
    rates = Rates()
    rates.seed(0.33)
    planner = StopPlanner(world.router, services, rates)
    rt = world.router.route(0, 100.0, (3800.0, 0.0))
    base = dict(game_time=600.0, income=10000.0, deadline=600.0 + 20 * 60, next_rest=None)
    assert planner.plan(PlanState(rt, fuel_l=300.0, capacity_l=500.0, **base), 100.0).stops == []
    # the same fuel in a big tank: it covers the job, but arrives with under 20 %
    plan = planner.plan(PlanState(rt, fuel_l=300.0, capacity_l=2000.0, **base), 100.0)
    assert [s.kind for s in plan.stops] == ["fuel"]



def test_a_big_interchange_doesnt_block_the_driving_loop():
    """2026-10-30: is_intersection took 2-4 s the first time a junction of 100-140 paths came into range: the truck
    drove blind for 66 m at 79 km/h. A fan of paths like that, none crossing."""
    import time

    from autohaul.world.junctions import Junctions
    lanes = []
    for k in range(150):
        pts = arc_polyline([(60, 0.0)], x=0.0, y=k * 12.0)
        lanes.append((pts, KIND_PREFAB, 7, k))
    j = Junctions(build(lanes))
    start = time.perf_counter()
    assert j.is_intersection(7) is False
    assert time.perf_counter() - start < 0.2


def test_is_intersection_finds_crossings_and_ignores_flyovers():
    from autohaul.world.junctions import Junctions
    a = arc_polyline([(60, 0.0)], x=0.0, y=0.0)
    b = arc_polyline([(60, 0.0)], x=30.0, y=-30.0, rot=math.pi / 2)
    apart = arc_polyline([(60, 0.0)], x=0.0, y=100.0)
    j = Junctions(build([(a, KIND_PREFAB, 1, 0), (b, KIND_PREFAB, 1, 1), (apart, KIND_PREFAB, 2, 0), (apart + [0, 9], KIND_PREFAB, 2, 1)]))
    assert j.is_intersection(1) is True
    assert j.is_intersection(2) is False
    assert j.crossings(0) == [1] and j.crossings(1) == [0]
    lm = build([(a, KIND_PREFAB, 1, 0), (b, KIND_PREFAB, 1, 1)])
    lm.seg[lm.lane_first[1]:lm.lane_last[1] + 1, 4:6] = 8.0   # the second path passes 8 m above the first
    j = Junctions(lm)
    assert j.is_intersection(1) is False and j.crossings(0) == []


def test_fuel_looks_further_off_the_route_when_no_pump_is_in_reach():
    """2026-10-30: 1 pump within 400 m on 465 game km, and out of reach: "none possible" from 165 l down to 67 l, with
    more within 1600 m."""
    world, services, pump, bed = make_station_world()
    rates = Rates()
    rates.seed(0.33)
    rt = world.router.route(0, 100.0, (3800.0, 0.0))
    base = dict(capacity_l=500.0, game_time=600.0, income=10000.0, deadline=600.0 + 20 * 60, next_rest=None)
    calls = []
    real_along = services.along

    def along(kind, pts, corridor):
        calls.append(corridor)
        return real_along(kind, pts, corridor) if corridor >= 800.0 else []    # nothing within the narrow corridor
    services.along = along
    plan = StopPlanner(world.router, services, rates).plan(PlanState(rt, fuel_l=50.0, **base), 100.0)
    assert [s.kind for s in plan.stops] == ["fuel"] and calls[:2] == [400.0, 800.0]



class _FakeServices:
    default_price = 1.6

    def __init__(self, pumps):
        self.pumps = pumps

    def along(self, kind, pts, corridor):
        return [(sp, along, off) for sp, along, off in self.pumps if off <= corridor] if kind == "fuel" else []

    def price_at(self, sp):
        return 1.6


def test_with_no_sane_detour_the_smallest_one_is_taken():
    """2026-10-30, the same 465 km: the first pump found was a 306 km detour, while one further on was 113 km."""
    from types import SimpleNamespace
    world, *_ = make_station_world()
    rates = Rates()
    rates.seed(0.33)
    rt = world.router.route(0, 100.0, (3800.0, 0.0))
    near_bad = SimpleNamespace(site="bad", x=500.0, y=700.0, lane=0, s=0.0, country="x", kind="fuel")
    less_bad = SimpleNamespace(site="less", x=1500.0, y=900.0, lane=0, s=0.0, country="x", kind="fuel")
    planner = StopPlanner(world.router, _FakeServices([(near_bad, 500.0, 700.0), (less_bad, 1500.0, 900.0)]), rates)
    detours = {"bad": 16000.0, "less": 6000.0}
    calls = []

    def detour(sp, along, poly, starts, total):
        calls.append(sp.site)
        return detours[sp.site]

    planner.detour = detour
    base = dict(capacity_l=500.0, game_time=600.0, income=10000.0, deadline=600.0 + 20 * 60, next_rest=None)
    plan = planner.plan(PlanState(rt, fuel_l=50.0, **base), 100.0)
    assert [s.point.site for s in plan.stops] == ["less"]
    assert calls.count("bad") == 1 and calls.count("less") == 1    # each worked out once: they take a while
    detours["less"] = 500.0                                   # a fair detour: taken without looking wider
    plan = planner.plan(PlanState(rt, fuel_l=50.0, **base), 100.0)
    assert [s.point.site for s in plan.stops] == ["less"]


def _plan_mission(tmp_path):
    from types import SimpleNamespace

    from autohaul.mission.decisions import DecisionLog
    from autohaul.mission.job import JobHistory
    from autohaul.mission.manager import Mission, MissionParams
    from autohaul.world.places import Place
    world, services, pump, bed = make_station_world()
    world.places = SimpleNamespace(company=lambda city, company: Place(3800.0, 0.0, company))
    m = Mission(world, services, DecisionLog(tmp_path / "d.jsonl"), JobHistory(tmp_path / "j.jsonl"),
                MissionParams(fatigue="off", plan_in_background=True, plan_every_s=0.0))
    m.job = SimpleNamespace(source_city="a", source_company="s", destination_city="b", destination_company="d",
                            cargo_loaded=True, deadline=600.0 + 1200.0, income=10000.0, key="k")
    m.trailer_on = True
    m.state = "to destination"
    frame = SimpleNamespace(fuel=50.0, game_time=600.0, next_rest_stop=0.0)
    return m, frame


def test_planning_in_the_background_doesnt_block_and_is_applied_on_the_driving_thread(tmp_path):
    """A plan can take 0.5 s: 11 m blind at 80 km/h if it's made in the driving loop."""
    import threading
    import time

    m, frame = _plan_mission(tmp_path)
    release = threading.Event()
    real = m._make_plan

    def slow(req):
        release.wait(5.0)
        return real(req)

    m._make_plan = slow
    start = time.perf_counter()
    m.replan_async(frame, 0, 100.0, 500.0, None)
    assert time.perf_counter() - start < 0.05                 # back at once, with the search still going
    assert m.plan is None and not m.replan_poll()
    m.replan_async(frame, 0, 100.0, 500.0, None)              # only one at a time
    first = m._plan_thread
    release.set()
    first.join(10.0)
    assert m.replan_poll() and m.plan is not None and [s.kind for s in m.plan.stops] == ["fuel"]
    assert not m.replan_poll()


def test_a_plan_for_the_old_job_is_thrown_away(tmp_path):
    m, frame = _plan_mission(tmp_path)
    m.replan_async(frame, 0, 100.0, 500.0, None)
    m._plan_gen += 1                                          # the job changed while it was searching
    m._plan_thread.join(10.0)
    assert m.replan_poll() and m.plan is None
