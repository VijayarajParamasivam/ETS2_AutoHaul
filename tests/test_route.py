"""Lane graph and A* routing on small synthetic maps."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autohaul.sim.course import arc_polyline, offset
from autohaul.world.graph import LaneGraph
from autohaul.world.lanemap import CACHE_VERSION, KIND_PREFAB, KIND_ROAD, LaneMap
from autohaul.world.route import Router


def build(lanes: list[tuple[np.ndarray, int, int, int]]) -> LaneMap:
    """lanes: (polyline, kind, item uid, sub)."""
    rows, ids = [], []
    for i, (pts, *_rest) in enumerate(lanes):
        n = len(pts) - 1
        arr = np.zeros((n, 6))
        arr[:, 0:2], arr[:, 2:4] = pts[:-1], pts[1:]
        rows.append(arr)
        ids.append(np.full(n, i, dtype=np.int32))
    return LaneMap(np.concatenate(rows), np.concatenate(ids),
                   np.array([k for _, k, _, _ in lanes], np.uint8), np.array([u for _, _, u, _ in lanes], np.uint64),
                   np.array([s for *_, s in lanes], np.int16), {"cache_version": CACHE_VERSION, "built": 1.0})


@pytest.fixture
def junction():
    """A two-lane road (item 1: sub 1 left, sub 2 right) ending at a junction whose straight-on curve
    leaves from the left lane (lane 2) and whose right exit leaves only from the right lane (lane 3);
    the exit leads on to road 9 (lane 4), the straight curve to road 8 (lane 5)."""
    left = arc_polyline([(300, 0.0)])
    right = offset(left, 4.5)
    straight = arc_polyline([(60, 0.0)], x=300, y=0)
    exit_ = arc_polyline([(60, 1 / 80)], x=300, y=4.5)
    after_straight = arc_polyline([(500, 0.0)], x=360, y=0)
    e_end, r_end = exit_[-1], math.atan2(exit_[-1, 1] - exit_[-2, 1], exit_[-1, 0] - exit_[-2, 0])
    after_exit = arc_polyline([(500, 0.0)], x=e_end[0], y=e_end[1], rot=r_end)
    lm = build([(left, KIND_ROAD, 1, 1), (right, KIND_ROAD, 1, 2), (straight, KIND_PREFAB, 7, 0),
                (exit_, KIND_PREFAB, 7, 1), (after_exit, KIND_ROAD, 9, 1), (after_straight, KIND_ROAD, 8, 1)])
    return lm, LaneGraph.build(lm, log=lambda *_: None)


def test_graph_links_and_lane_change_neighbours(junction):
    lm, g = junction
    assert list(g.successors(0)) == [2]
    assert list(g.successors(1)) == [3]
    assert list(g.successors(2)) == [5] and list(g.successors(3)) == [4]
    assert list(g.neighbours(0)) == [1] and list(g.neighbours(1)) == [0]
    assert len(g.neighbours(2)) == 0                                  # no lane changes inside junctions
    assert g.length[0] == pytest.approx(300.0, abs=0.5)


def test_route_takes_the_exit_with_a_lane_change(junction):
    lm, g = junction
    after_exit = lm.lane_points(4)
    target = tuple(after_exit[len(after_exit) // 2])
    rt = Router(lm, g, lane_change_m=80.0).route(0, 50.0, target)
    assert rt is not None
    assert rt.lanes == [0, 1, 3, 4] and rt.changes == [False, True, False, False]
    assert rt.next_after(1) == 3 and rt.next_after(4) is None
    # driving distance: 250 m left of the start lane + the exit curve + half of the road after it
    expected = 250.0 + g.length[3] + g.length[4] / 2
    assert rt.length_m == pytest.approx(expected, abs=8.0)


def test_route_straight_on_needs_no_lane_change(junction):
    lm, g = junction
    after = lm.lane_points(5)
    rt = Router(lm, g).route(0, 0.0, tuple(after[-5]))
    assert rt.lanes == [0, 2, 5] and not any(rt.changes)


def test_unreachable_destination(junction):
    lm, g = junction
    assert Router(lm, g).route(0, 0.0, (5000.0, 5000.0)) is None          # no lane near it
    assert Router(lm, g).route(4, 0.0, tuple(lm.lane_points(5)[-5])) is None   # no way back from the exit road


def test_graph_cache_roundtrip(junction, tmp_path):
    lm, g = junction
    g.save(tmp_path / "lanes_graph.npz")
    g2 = LaneGraph.load_or_build(lm, tmp_path, log=lambda *_: None)
    assert np.array_equal(g2.succ_idx, g.succ_idx) and np.array_equal(g2.nbr_idx, g.nbr_idx)


# ---------------------------------------------------------------- autopilot following a route (simulator)

def _drive(lm, g, start_lane, target, seconds=60.0, junctions=None, speed_kmh=60.0, traffic=None,
           **ap_kwargs):
    from autohaul.control.vehicle import TruckGeometry
    from autohaul.drive.autopilot import Autopilot
    from autohaul.sim.truck import TruckSim
    geom = TruckGeometry()
    pts = lm.lane_points(start_lane)
    rot = math.atan2(pts[1, 1] - pts[0, 1], pts[1, 0] - pts[0, 0])
    x0, y0 = pts[0] + 20 * np.array([math.cos(rot), math.sin(rot)])
    sim = TruckSim(geom, x=x0, y=y0, rotation=rot, speed=speed_kmh / 3.6)
    ap = Autopilot(lm, geom, "relative", router=Router(lm, g), junctions=junctions, **ap_kwargs)
    ap.set_destination(*target)
    assert ap.engage(sim.frame(80 / 3.6)) is None
    lanes, fault = [], None
    for _ in range(int(seconds / sim.p.dt)):
        cmd, st = ap.step(sim.frame(80 / 3.6), [v.record() for v in (traffic or [])], sim.p.dt)
        for v in traffic or []:
            v.step(sim.t, sim.p.dt)
        if st.lane is not None and (not lanes or lanes[-1] != st.lane):
            lanes.append(st.lane)
        if st.fault:
            fault = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    return lanes, fault, ap


def test_autopilot_follows_the_route_branch_and_arrives(junction):
    lm, g = junction
    lanes, fault, ap = _drive(lm, g, 0, tuple(lm.lane_points(5)[-30]))
    assert lanes[:3] == [0, 2, 5]                       # through the junction's straight-on curve
    assert fault == "destination reached: take over"


def test_autopilot_changes_lanes_for_the_exit(junction):
    lm, g = junction
    target = tuple(lm.lane_points(4)[-40])
    lanes, fault, ap = _drive(lm, g, 0, target)
    assert lanes[:4] == [0, 1, 3, 4]                    # signalled, moved right, took the exit
    assert fault == "destination reached: take over"


def test_autopilot_hands_back_for_a_lane_change_when_not_automated(junction):
    lm, g = junction
    target = tuple(lm.lane_points(4)[len(lm.lane_points(4)) // 2])
    lanes, fault, ap = _drive(lm, g, 0, target, auto_lane_change=False)
    assert fault and "lane change to the right" in fault
    assert lanes == [0]                                  # handed back on the approach, before the junction


def test_autopilot_hands_back_before_an_intersection():
    from autohaul.world.junctions import Junctions
    a = arc_polyline([(400, 0.0)])
    through = arc_polyline([(40, 0.0)], x=400, y=0)
    crossing = arc_polyline([(40, 0.0)], x=420, y=-20, rot=math.pi / 2)    # a crossing path, same junction
    after = arc_polyline([(400, 0.0)], x=440, y=0)
    lm = build([(a, KIND_ROAD, 1, 1), (through, KIND_PREFAB, 7, 0), (crossing, KIND_PREFAB, 7, 1),
                (after, KIND_ROAD, 2, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    lanes, fault, _ = _drive(lm, g, 0, tuple(after[-20]), junctions=Junctions(lm))
    assert fault and fault.startswith("intersection in")
    assert lanes == [0]


def test_engage_refused_right_before_an_intersection():
    from autohaul.control.vehicle import TruckGeometry
    from autohaul.drive.autopilot import Autopilot
    from autohaul.sim.truck import TruckSim
    from autohaul.world.junctions import Junctions
    a = arc_polyline([(400, 0.0)])
    through = arc_polyline([(40, 0.0)], x=400, y=0)
    crossing = arc_polyline([(40, 0.0)], x=420, y=-20, rot=math.pi / 2)
    lm = build([(a, KIND_ROAD, 1, 1), (through, KIND_PREFAB, 7, 0), (crossing, KIND_PREFAB, 7, 1)])
    ap = Autopilot(lm, TruckGeometry(), "relative", junctions=Junctions(lm))
    assert "intersection in" in ap.engage(TruckSim(x=350, y=0, rotation=0.0, speed=10).frame())
    assert ap.engage(TruckSim(x=100, y=0, rotation=0.0, speed=10).frame()) is None


# ---------------------------------------------------------------- lane changes

def _veh(vid, x, y, speed, rot=0.0, length=4.5):
    th = -(rot + math.pi / 2)
    return {"wx": x, "wz": y, "qw": math.cos(th / 2), "qx": 0.0, "qy": math.sin(th / 2), "qz": 0.0,
            "length": length, "width": 1.9, "speed": speed, "acceleration": 0.0, "id": vid,
            "trailer_count": 0, "trailers": []}


def test_gap_check_sides_and_distances():
    from autohaul.control.lanechange import LaneChangeParams, gap_check
    from autohaul.control.vehicle import Pose
    p, pose = LaneChangeParams(), Pose(0.0, 0.0, 0.0, 20.0, 0.0)          # heading east, right = +y
    check = lambda veh, side=1: gap_check(pose, side, [veh], 3.2, 3.5, p)   # noqa: E731
    assert "alongside" in check(_veh(1, 0.0, 4.5, 20.0))
    assert check(_veh(1, 0.0, -4.5, 20.0)) is None                          # that's the left lane
    assert "alongside" in check(_veh(1, 0.0, -4.5, 20.0), side=-1)
    assert check(_veh(1, 80.0, 4.5, 20.0)) is None                          # well ahead, same speed
    assert "ahead" in check(_veh(1, 15.0, 4.5, 15.0))                       # close and slower
    assert check(_veh(1, -60.0, 4.5, 20.0)) is None                         # behind, same speed
    assert "behind" in check(_veh(1, -40.0, 4.5, 30.0))                     # behind and 10 m/s faster
    assert check(_veh(1, 0.0, 12.0, 20.0)) is None                          # two lanes over
    # 2026-10-29: a wreck standing in the target lane 30 m ahead at 40 km/h: AutoHaul moved over towards it
    slow = lambda gap: gap_check(Pose(0.0, 0.0, 0.0, 11.1, 0.0), -1, [_veh(1, gap, -4.5, 0.0)], 3.2, 3.5, p)  # noqa: E731
    assert "stopped vehicle ahead" in slow(35.0) and "stopped vehicle ahead" in slow(140.0)
    assert slow(200.0) is None
    crawl = gap_check(Pose(0.0, 0.0, 0.0, 1.0, 0.0), -1, [_veh(1, 20.0, -4.5, 0.0)], 3.2, 3.5, p)
    assert crawl is None                                                    # in a queue ourselves: fine


def test_lane_change_waits_for_a_gap_then_makes_a_short_change(junction):
    """A vehicle alongside at the same speed: the truck waits; slowing for the split it lets the vehicle go ahead,
    then changes in the room left (2026-10-29: a short change instead of a long way round)."""
    from autohaul.control.path import PathRef
    from autohaul.sim.truck import SimVehicle
    lm, g = junction
    right = PathRef(lm.lane_points(1))
    blocker = SimVehicle(1, right, s=20.0, speed=60 / 3.6, length=16.0, width=2.5)   # alongside, same speed
    lanes, fault, ap = _drive(lm, g, 0, tuple(lm.lane_points(4)[-40]), traffic=[blocker])
    assert fault == "destination reached: take over" and lanes[:2] == [0, 1]


def test_lane_change_hands_back_if_no_gap_ever_opens(junction):
    from autohaul.control.path import PathRef
    from autohaul.sim.truck import SimVehicle
    lm, g = junction
    right = PathRef(lm.lane_points(1))
    parked = SimVehicle(1, right, s=right.length / 2, speed=0.0, length=right.length - 10.0, width=2.5)  # a queue
    lanes, fault, ap = _drive(lm, g, 0, tuple(lm.lane_points(4)[-40]), traffic=[parked], seconds=90.0)
    assert fault and fault.startswith("no room left to change lanes to the right")
    assert lanes == [0]


def test_lane_change_after_the_neighbour_drops_back(junction):
    from autohaul.control.path import PathRef
    from autohaul.sim.truck import SimVehicle
    lm, g = junction
    right = PathRef(lm.lane_points(1))
    slower = SimVehicle(1, right, s=24.0, speed=45 / 3.6, length=16.0, width=2.5)    # alongside, slower
    lanes, fault, ap = _drive(lm, g, 0, tuple(lm.lane_points(4)[-40]), traffic=[slower])
    assert lanes[:4] == [0, 1, 3, 4] and fault == "destination reached: take over"


def test_slow_lane_change_completes(junction):
    """Regression (2026-10-20 in-game): pulling away from a standstill, the move took long enough that the
    'between two lanes' moment lowered the tracker confidence for > 1 s and AutoHaul handed back."""
    lm, g = junction
    lanes, fault, ap = _drive(lm, g, 0, tuple(lm.lane_points(4)[-40]), speed_kmh=20.0, seconds=200.0,
                              speed_params=__import__("autohaul.control.speed", fromlist=["SpeedParams"]).SpeedParams(max_speed_kmh=25))
    assert lanes[:4] == [0, 1, 3, 4] and fault == "destination reached: take over"
