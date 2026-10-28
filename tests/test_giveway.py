"""Junction rules in the simulator: give way, right of way, stop signs, traffic lights, merges."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autohaul.control.giveway import time_to
from autohaul.control.path import PathRef
from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.sim.course import arc_polyline
from autohaul.sim.truck import SimVehicle, TruckSim
from autohaul.world.graph import LaneGraph
from autohaul.world.junctions import Junctions
from autohaul.world.lanemap import KIND_PREFAB, KIND_ROAD
from autohaul.world.route import Router
from autohaul.world.rules import PRIORITY_SHIFT, JunctionRules
from tests.test_route import build

LINE_X = 400.0          # our road enters the junction here; the main road crosses at x = 420


def make_crossroads():
    """Our road (lane 0, east) into junction 7: through path (lane 1, x 400-440) crossing the main road's path
    (lane 2, north, x = 420); main road before (lane 3) and after (lane 4) the junction; our road after (lane 5)."""
    ours = arc_polyline([(LINE_X, 0.0)])
    through = arc_polyline([(40, 0.0)], x=LINE_X, y=0)
    crossing = arc_polyline([(40, 0.0)], x=420, y=-20, rot=math.pi / 2)
    main_in = arc_polyline([(500, 0.0)], x=420, y=-520, rot=math.pi / 2)
    main_out = arc_polyline([(280, 0.0)], x=420, y=20, rot=math.pi / 2)
    after = arc_polyline([(400, 0.0)], x=440, y=0)
    lm = build([(ours, KIND_ROAD, 1, 1), (through, KIND_PREFAB, 7, 0), (crossing, KIND_PREFAB, 7, 1),
                (main_in, KIND_ROAD, 3, 1), (main_out, KIND_ROAD, 4, 1), (after, KIND_ROAD, 2, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    main = PathRef(np.vstack([main_in, crossing[1:], main_out[1:]]))
    return lm, g, main, after


@pytest.fixture
def crossroads():
    return make_crossroads()


def _with_lights(r: JunctionRules, sids=(0,), profile: str = "cr_2x1", item: int = 7) -> JunctionRules:
    """Give junction `item` a light head per semaphore group in `sids` (a path with a signal id but no semaphore
    in its junction has no light: signals for the AI only)."""
    n = len(sids)
    r.sem_item = np.full(n, item, dtype=np.uint64)
    r.sem_sid = np.array(sids, dtype=np.int16)
    r.sem_xyz = np.array([[LINE_X - 2.0, 5.0 + 3 * k, 0.0] for k in range(n)])
    r.sem_rot = np.zeros(n)
    r.sem_type = np.zeros(n, dtype=np.int8)
    r.sem_profile = np.zeros(n, dtype=np.int16)
    r.profile_names = [profile]
    r.sem_cycle = np.zeros(n)
    r.sem_intervals = np.zeros((n, 4))
    return r


def _rules(lm, ours: int, theirs: int) -> JunctionRules:
    r = JunctionRules.empty(lm)
    r.flags[1] = ours << PRIORITY_SHIFT
    r.flags[2] = theirs << PRIORITY_SHIFT
    return r


def _run(lm, g, target, rules, traffic=None, seconds=80.0, x0=200.0, speed_kmh=50.0, traffic_none=False,
         lights=None, clock=None):
    geom = TruckGeometry()
    sim = TruckSim(geom, x=x0, y=0.0, rotation=0.0, speed=speed_kmh / 3.6)
    ap = Autopilot(lm, geom, "relative", router=Router(lm, g), junctions=Junctions(lm), rules=rules)
    ap.set_destination(*target)
    if lights is not None:
        ap.set_lights(lights)
    reason = ap.engage(sim.frame(60 / 3.6))
    log = {"t": [], "x": [], "v": [], "note": [], "fault": None, "refused": reason, "veh": []}
    if reason:
        return log
    traffic = traffic or []
    for _ in range(int(seconds / sim.p.dt)):
        if clock is not None:
            clock["t"] = sim.t
        cmd, st = ap.step(sim.frame(60 / 3.6), None if traffic_none else [v.record() for v in traffic], sim.p.dt)
        log["t"].append(sim.t)
        log["x"].append(sim.s.x + geom.front_bumper)          # front bumper (the truck heads east)
        log["v"].append(sim.s.speed)
        log["note"].append(st.junction)
        log["veh"].append([tuple(v.path.point(v.s)) for v in traffic])
        if st.fault:
            log["fault"] = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
        for v in traffic:
            v.step(sim.t, sim.p.dt)
    return log


def test_time_to():
    assert time_to(0.0, 5.0, 1.0, 10.0) == 0.0
    assert time_to(100.0, 10.0, 1.0, 10.0) == pytest.approx(10.0)
    assert time_to(8.0, 0.0, 1.0, 10.0) == pytest.approx(4.0)            # d = a t^2 / 2
    assert time_to(150.0, 0.0, 1.0, 10.0) == pytest.approx(10.0 + 10.0)  # 50 m to reach 10 m/s, then 100 m


def test_conflicts_cross_and_merge():
    # a T-junction: our right turn merges into the main road's through path
    main_in = arc_polyline([(30, 0.0)], x=0, y=0)
    through = arc_polyline([(30, 0.0)], x=30, y=0)
    right_turn = arc_polyline([(math.pi / 2 * 15, 1 / 15)], x=45, y=15, rot=-math.pi / 2)    # turns to head east
    rt_end = right_turn[-1]
    shift = np.array([60.0, 0.0]) - rt_end                   # make the turn end where the through path ends
    right_turn = right_turn + shift
    out = arc_polyline([(100, 0.0)], x=60, y=0)
    lm = build([(main_in, KIND_ROAD, 1, 1), (through, KIND_PREFAB, 7, 0), (right_turn, KIND_PREFAB, 7, 1),
                (out, KIND_ROAD, 2, 1)])
    j = Junctions(lm)
    kinds = {(c.other, c.kind) for c in j.conflicts(2)}
    assert (1, "merge") in kinds
    assert j.crossings(2) == [] and not j.is_intersection(7)


def test_conflicts_ignore_flyovers(crossroads):
    lm, g, *_ = crossroads
    assert [c.other for c in Junctions(lm).conflicts(1)] == [2]
    lm.seg[lm.lane_first[2]:lm.lane_last[2] + 1, 4:6] = 7.0      # the main road passes 7 m above
    assert Junctions(lm).conflicts(1) == []


def test_gives_way_to_crossing_traffic_then_goes(crossroads):
    lm, g, main, after = crossroads
    # a stream of cars on the main road, reaching the crossing (s = 520) between 20 s and 30.5 s
    cars = [SimVehicle(100 + i, main, s=520 - 12.0 * (20 + 3.5 * i), speed=12.0) for i in range(4)]
    log = _run(lm, g, tuple(after[-20]), _rules(lm, ours=6, theirs=12), cars)
    assert log["fault"] == "destination reached: take over"
    x, v = np.array(log["x"]), np.array(log["v"])
    waiting = (v < 0.3) & (x < LINE_X)
    assert waiting.any(), "the truck should have stopped for the cars"
    assert x[waiting].max() <= LINE_X - 0.5                        # at the line, not in the junction
    assert any("giving way" in n for n in log["note"])
    # no car was in the crossing while any part of the truck was across the main road's lane
    length = TruckGeometry().front_bumper + 3.5
    for k in range(len(x)):
        if x[k] > 418 and x[k] - length < 422:
            for vx, vy in log["veh"][k]:
                assert not (abs(vx - 420) < 3 and abs(vy) < 6), "a car crossed while the truck was in the way"


def test_right_of_way_through_waiting_car(crossroads):
    lm, g, main, after = crossroads
    waiting = SimVehicle(200, main, s=520 - 22.0, speed=0.0)        # stopped at its line, 22 m from the crossing
    log = _run(lm, g, tuple(after[-20]), _rules(lm, ours=12, theirs=6), [waiting])
    assert log["fault"] == "destination reached: take over"
    x, v = np.array(log["x"]), np.array(log["v"])
    assert v[(x > LINE_X - 60) & (x < LINE_X + 40)].min() > 3.5     # slowed for the intersection, no stop
    assert "right of way" in log["note"]


def test_right_of_way_still_stops_for_a_car_that_doesnt_yield(crossroads):
    lm, g, main, after = crossroads
    rogue = SimVehicle(201, main, s=520 - 12.0 * 21.5, speed=12.0)  # would meet us; ignores its give-way line
    log = _run(lm, g, tuple(after[-20]), _rules(lm, ours=12, theirs=6), [rogue], speed_kmh=40.0, x0=230.0)
    assert log["fault"] == "destination reached: take over"
    assert any("without giving way" in n and "committed" not in n for n in log["note"])
    x = np.array(log["x"])
    length = TruckGeometry().front_bumper + 3.5
    for k in range(len(x)):
        if x[k] > 418 and x[k] - length < 422:
            for vx, vy in log["veh"][k]:
                assert not (abs(vx - 420) < 3 and abs(vy) < 6), "the rogue car crossed while the truck was in the way"


def test_stop_sign_full_stop_then_go(crossroads):
    lm, g, main, after = crossroads
    r = _rules(lm, ours=6, theirs=12)
    r.rule[1] = 0
    r.rule_names = ["stop"]
    log = _run(lm, g, tuple(after[-20]), r, [])
    assert log["fault"] == "destination reached: take over"
    x, v = np.array(log["x"]), np.array(log["v"])
    stopped = (v < 0.3) & (x < LINE_X) & (x > LINE_X - 4)
    dt = float(np.diff(log["t"]).mean())
    assert stopped.sum() * dt >= 0.9                               # stood at the line for ~1 s


def test_traffic_light_hands_back(crossroads):
    lm, g, main, after = crossroads
    r = _rules(lm, ours=6, theirs=12)
    r.sem_id[1] = 0
    _with_lights(r)
    log = _run(lm, g, tuple(after[-20]), r, [])
    assert log["fault"] and log["fault"].startswith("traffic light in")
    assert max(log["x"]) < LINE_X - 50


def test_no_traffic_data_hands_back(crossroads):
    lm, g, main, after = crossroads
    log = _run(lm, g, tuple(after[-20]), _rules(lm, ours=12, theirs=6), traffic_none=True)
    assert log["fault"] and "no AI traffic data" in log["fault"]


def test_engage_allowed_near_a_give_way_junction(crossroads):
    lm, g, main, after = crossroads
    log = _run(lm, g, tuple(after[-20]), _rules(lm, ours=6, theirs=12), [], x0=370.0, speed_kmh=10.0)
    assert log["refused"] is None and log["fault"] == "destination reached: take over"


def test_conflict_zone_starts_where_the_paths_come_close(crossroads):
    lm, *_ = crossroads
    (c,) = Junctions(lm).conflicts(1)
    assert c.s_mine == pytest.approx(20.0, abs=0.5) and c.s_other == pytest.approx(20.0, abs=0.5)
    # crossing at right angles: the zone starts ~3 m (the clearance) before the crossing point, on both paths
    assert 16.0 <= c.zone_mine <= 17.5 and 16.0 <= c.zone_other <= 17.5


def make_t_junction():
    """Main road east (lane 0) -> through path (lane 1, prio 12) -> on east (lane 2); our side road north
    (lane 3) -> right turn (lane 4) merging into the through path's end."""
    main_in = arc_polyline([(330, 0.0)], x=-300, y=0)
    through = arc_polyline([(30, 0.0)], x=30, y=0)
    after = arc_polyline([(340, 0.0)], x=60, y=0)
    side = arc_polyline([(285, 0.0)], x=45, y=300, rot=-math.pi / 2)
    turn = arc_polyline([(math.pi / 2 * 15, 1 / 15)], x=45, y=15, rot=-math.pi / 2, step=1.0)
    lm = build([(main_in, KIND_ROAD, 1, 1), (through, KIND_PREFAB, 7, 0), (after, KIND_ROAD, 2, 1),
                (side, KIND_ROAD, 3, 1), (turn, KIND_PREFAB, 7, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    r = JunctionRules.empty(lm)
    r.flags[1] = 12 << PRIORITY_SHIFT
    r.flags[4] = 6 << PRIORITY_SHIFT
    return lm, g, r, PathRef(np.vstack([main_in, through[1:], after[1:]])), after


def test_right_turn_gives_way_to_main_road_traffic():
    lm, g, r, main, after = make_t_junction()
    j = Junctions(lm)
    assert [(c.other, c.kind) for c in j.conflicts(4)] == [(1, "merge")]
    assert j.conflicts(4)[0].zone_mine > 3.0                   # the paths converge: wait well before the merge
    # a merge alone doesn't make an intersection; give the junction a crossing so the rules apply
    cross = arc_polyline([(30, 0.0)], x=40, y=20, rot=-math.pi / 2)
    lm2 = build([(lm.lane_points(i), int(lm.lane_kind[i]), int(lm.lane_item[i]), int(lm.lane_sub[i]))
                 for i in range(5)] + [(cross, KIND_PREFAB, 7, 2)])
    g2 = LaneGraph.build(lm2, log=lambda *_: None)
    r2 = JunctionRules.empty(lm2)
    r2.flags[:5] = r.flags[:5]
    geom = TruckGeometry()
    sim = TruckSim(geom, x=45.0, y=200.0, rotation=-math.pi / 2, speed=30 / 3.6)
    ap = Autopilot(lm2, geom, "relative", router=Router(lm2, g2), junctions=Junctions(lm2), rules=r2)
    ap.set_destination(*after[-20])
    assert ap.engage(sim.frame(50 / 3.6)) is None
    # five cars, 2.5 s apart, reaching the merge point (s = 360) between 19 s and 29 s
    cars = [SimVehicle(400 + i, main, s=360 - 11.0 * (19 + 2.5 * i), speed=11.0) for i in range(5)]
    v_line, fault, entered = 99.0, None, None
    for _ in range(int(80 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(50 / 3.6), [v.record() for v in cars], sim.p.dt)
        front_y = sim.s.y - geom.front_bumper                       # heading north: front is at smaller y
        if 15.0 < front_y < 25.0:
            v_line = min(v_line, sim.s.speed)
        if entered is None and front_y < 3.0:
            entered = sim.t
            assert all(v.s > 360 + 3 or v.s < 360 - 60 for v in cars), "pulled out in front of a car"
        if st.fault:
            fault = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
        for v in cars:
            v.step(sim.t, sim.p.dt)
    assert fault == "destination reached: take over"
    assert v_line < 3.0                                              # (nearly) stopped at the line for the stream
    assert entered is not None and entered > 29.0                    # after the last car passed


def test_planner_stop_point_is_reached_within_a_metre():
    from autohaul.control.speed import SpeedParams, SpeedPlanner
    path = PathRef(arc_polyline([(100, 0.0)]))
    sp = SpeedPlanner(SpeedParams())
    v_now, *_ = sp.ahead(path, 0.0, 0.0, points=[(0.6, 0.0, "junction")])
    assert v_now == 0.0                                            # standing 0.6 m short: don't creep
    v_far, *_ = sp.ahead(path, 0.0, 0.0, points=[(20.0, 0.0, "junction")])
    assert v_far > 3.0


def test_indicates_for_a_junction_turn_from_the_map():
    from autohaul.world.rules import RIGHT_BLINKER
    lm, g, r, main, after = make_t_junction()
    cross = arc_polyline([(30, 0.0)], x=40, y=20, rot=-math.pi / 2)
    lm2 = build([(lm.lane_points(i), int(lm.lane_kind[i]), int(lm.lane_item[i]), int(lm.lane_sub[i]))
                 for i in range(5)] + [(cross, KIND_PREFAB, 7, 2)])
    g2 = LaneGraph.build(lm2, log=lambda *_: None)
    r2 = JunctionRules.empty(lm2)
    r2.flags[:5] = r.flags[:5]
    r2.flags[4] |= RIGHT_BLINKER
    geom = TruckGeometry()
    sim = TruckSim(geom, x=45.0, y=200.0, rotation=-math.pi / 2, speed=30 / 3.6)
    ap = Autopilot(lm2, geom, "relative", router=Router(lm2, g2), junctions=Junctions(lm2), rules=r2)
    ap.set_destination(*after[-20])
    assert ap.engage(sim.frame(50 / 3.6)) is None
    on = []
    for _ in range(int(60 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(50 / 3.6), [], sim.p.dt)
        on.append((sim.s.y - geom.front_bumper, sim.s.x, st.indicator))
        if st.fault:
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    signalled = [(y, x) for y, x, ind in on if ind == "right"]
    assert signalled and max(y for y, _ in signalled) < 15 + 60      # on from ~40-60 m before the turn, not earlier
    assert all(ind is None for y, x, ind in on if x > 75)             # off once the turn is done
    assert {ind for *_, ind in on} <= {None, "right"}


def _light_run(crossroads, colour_at, traffic=None, other_sid=1, stop_back=0.0, **kw):
    from autohaul.sim.lights import ScriptedLights
    lm, g, main, after = crossroads
    r = _rules(lm, ours=6, theirs=12)
    r.sem_id[1] = 0                                     # our path: light group 0
    r.sem_id[2] = other_sid                             # the main road: another group
    _with_lights(r, sids=(0, other_sid))
    clock = {"t": 0.0}
    lights = ScriptedLights(lambda lane, item, t: colour_at(t), lambda: clock["t"], stop_back)
    return _run(lm, g, tuple(after[-20]), r, traffic, lights=lights, clock=clock, **kw)


def test_traffic_light_red_stop_then_green_go(crossroads):
    log = _light_run(crossroads, lambda t: "red" if t < 40 else "green")
    assert log["fault"] == "destination reached: take over"
    x, v, t = np.array(log["x"]), np.array(log["v"]), np.array(log["t"])
    waiting = (v < 0.3) & (x < LINE_X)
    assert waiting.any() and x[waiting].max() <= LINE_X - 0.5 and x[waiting].min() > LINE_X - 3.0
    assert t[x > LINE_X].min() > 40                                     # not into the junction before green
    assert any("red light" in n for n in log["note"])


def test_traffic_light_stop_point_keeps_the_light_in_view(crossroads):
    log = _light_run(crossroads, lambda t: "red", stop_back=8.0, seconds=40)
    x, v = np.array(log["x"]), np.array(log["v"])
    waiting = (v < 0.3) & (x < LINE_X)
    assert waiting.any() and abs(x[waiting].max() - (LINE_X - 1.0 - 8.0)) < 1.0


def test_green_light_drives_through_past_cross_traffic_waiting_at_red(crossroads):
    lm, g, main, after = crossroads
    waiting = SimVehicle(200, main, s=520 - 12.0, speed=0.0)          # stopped at its red, 12 m from the crossing
    log = _light_run(crossroads, lambda t: "green", [waiting])
    assert log["fault"] == "destination reached: take over"
    x, v = np.array(log["x"]), np.array(log["v"])
    assert v[(x > LINE_X - 60) & (x < LINE_X + 40)].min() > 3.5        # no stop
    assert not any("giving way" in n for n in log["note"])


def test_amber_stops_when_it_can_and_goes_when_too_close(crossroads):
    far = _light_run(crossroads, lambda t: "green" if t < 2 else "amber" if t < 6 else "red", seconds=30)
    x, v = np.array(far["x"]), np.array(far["v"])
    assert ((v < 0.3) & (x < LINE_X)).any()                             # 200 m out at amber: it stops
    # amber with the truck 4 m from the line (slowing for the junction, ~22 km/h): going through is safer
    green = _light_run(crossroads, lambda t: "green", seconds=40)
    x, v, ts = np.array(green["x"]), np.array(green["v"]), np.array(green["t"])
    k = int(np.argmax(x > LINE_X - 1.0 - 4.0))
    t_amber, v_amber = float(ts[k]), float(v[k])
    assert v_amber * v_amber / (2 * 4.0) > 3.0                          # too hard a stop from that speed
    near = _light_run(crossroads, lambda t: "green" if t < t_amber else "amber" if t < t_amber + 4 else "red",
                      seconds=80)
    xs, vs = np.array(near["x"]), np.array(near["v"])
    assert near["fault"] == "destination reached: take over" and vs[xs < LINE_X + 5].min() > 2.0
    assert any("committed (amber" in n for n in near["note"])


def test_a_light_it_cant_see_stops_at_the_line_then_hands_back(crossroads):
    log = _light_run(crossroads, lambda t: None, seconds=80)
    assert log["fault"] and log["fault"].startswith("waited") and "can't see" in log["fault"]
    x = np.array(log["x"])
    assert x.max() <= LINE_X - 0.5


def _toll_rules(lm):
    r = _rules(lm, ours=12, theirs=6)
    r.sem_id[1] = 0
    r.sem_item = np.array([7], dtype=np.uint64)
    r.sem_sid = np.array([0], dtype=np.int16)
    r.sem_xyz = np.array([[LINE_X + 10.0, -3.0, 0.0]])            # the booth: 10 m into the junction, beside the lane
    r.sem_rot = np.zeros(1)
    r.sem_type = np.array([0], dtype=np.int8)                    # a profile semaphore ...
    r.sem_profile = np.array([0], dtype=np.int16)
    r.profile_names = ["ab_toll"]                                # ... with a toll profile
    r.sem_cycle = np.zeros(1)
    r.sem_intervals = np.zeros((1, 4))
    return r


def _toll_run(crossroads, pay: bool, seconds=90.0):
    lm, g, main, after = crossroads
    rules = _toll_rules(lm)
    assert rules.control(1, 7) == "toll gate"
    geom = TruckGeometry()
    sim = TruckSim(geom, x=200.0, y=0.0, rotation=0.0, speed=50 / 3.6)
    ap = Autopilot(lm, geom, "relative", router=Router(lm, g), junctions=Junctions(lm), rules=rules)
    ap.set_destination(*tuple(after[-20]))
    ap.set_tolls(True)
    assert ap.engage(sim.frame(60 / 3.6)) is None
    log = {"x": [], "v": [], "note": [], "press": [], "fault": None}
    pressed_at = None
    for _ in range(int(seconds / sim.p.dt)):
        cmd, st = ap.step(sim.frame(60 / 3.6), [], sim.p.dt)
        log["x"].append(sim.s.x + geom.front_bumper)
        log["v"].append(sim.s.speed)
        log["note"].append(st.junction)
        if st.press:
            log["press"].append((sim.t, sim.s.x + geom.front_bumper, sim.s.speed, list(st.press)))
            pressed_at = sim.t
        if pay and pressed_at is not None and sim.t - pressed_at > 1.0:
            ap.toll_paid()                                       # the game's player.tollgate.paid event
            pressed_at = None
        if st.fault:
            log["fault"] = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    return log


def test_toll_gate_stop_pay_and_drive_through(crossroads):
    log = _toll_run(crossroads, pay=True)
    assert log["fault"] == "destination reached: take over"
    assert len(log["press"]) == 1
    t, x, v, buttons = log["press"][0]
    assert buttons == ["activate"] and v < 0.3 and abs(x - (LINE_X + 10.0 - 2.0)) < 1.5     # at the booth
    assert any("waiting for the barrier" in n for n in log["note"])


def test_toll_gate_without_payment_hands_back(crossroads):
    log = _toll_run(crossroads, pay=False)
    assert log["fault"] and log["fault"].startswith("toll gate: no payment")
    assert len(log["press"]) == 3 and max(log["x"]) < LINE_X + 9.0



def test_toll_gate_on_a_plaza_without_crossing_paths():
    """2026-10-29: a toll plaza whose lanes only fan out isn't an intersection; it must still stop and pay."""
    road = arc_polyline([(LINE_X, 0.0)])
    booth = arc_polyline([(40, 0.0)], x=LINE_X, y=0)
    after = arc_polyline([(400, 0.0)], x=LINE_X + 40, y=0)
    lm = build([(road, KIND_ROAD, 1, 1), (booth, KIND_PREFAB, 7, 0), (after, KIND_ROAD, 2, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    assert not Junctions(lm).is_intersection(7)
    log = _toll_run((lm, g, None, after), pay=True)
    assert log["fault"] == "destination reached: take over"
    assert len(log["press"]) == 1 and abs(log["press"][0][1] - (LINE_X + 10.0 - 2.0)) < 1.5



def test_a_signal_id_without_a_semaphore_is_no_light(crossroads):
    """2026-10-29: country junctions whose paths carry a signal id for the AI but no semaphore (no lights there):
    plain give way, not "can't see the light"."""
    lm, g, main, after = crossroads
    r = _rules(lm, ours=12, theirs=6)
    r.sem_id[1] = 0
    assert r.control(1, 7) is None
    from autohaul.sim.lights import ScriptedLights
    log = _run(lm, g, tuple(after[-20]), r, [], lights=ScriptedLights(lambda lane, item, t: None, lambda: 0.0))
    assert log["fault"] == "destination reached: take over"


def test_a_light_in_view_with_no_lamp_lit_is_a_give_way_junction(crossroads):
    """2026-10-29: a junction 1 km before the Paris depot has a `cr_1x1` signal but no lamps on screen (AI only).
    Stop, look, then go as at an unsigned junction (giving way to everyone)."""
    from autohaul.sim.lights import ScriptedLights
    lm, g, main, after = crossroads
    r = _with_lights(_rules(lm, ours=12, theirs=6))
    r.sem_id[1] = 0
    clock = {"t": 0.0}
    lights = ScriptedLights(lambda lane, item, t: None, lambda: clock["t"], dark=True)
    log = _run(lm, g, tuple(after[-20]), r, [], lights=lights, clock=clock)
    assert log["fault"] == "destination reached: take over"
    x, v = np.array(log["x"]), np.array(log["v"])
    assert ((v < 0.3) & (x < LINE_X)).any()                        # it did stop and look first
