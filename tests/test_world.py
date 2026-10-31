"""Lane map and lane tracker on small synthetic maps (no game data needed)."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from autohaul.world.lanemap import (CACHE_VERSION, KIND_PREFAB, KIND_ROAD, LANE_WIDTH, LaneMap, calibrate_looks, fit_lanes,
                                    heading_to_rotation, hermite, offset_polyline, quat_forward_rotation, rule_layout)
from autohaul.world.localizer import LaneTracker
from autohaul.control.path import PathRef
from autohaul.sim.course import arc_polyline


def lanes_to_map(lanes: list[np.ndarray]) -> LaneMap:
    """LaneMap from polylines (one lane each, in order)."""
    rows, lane_ids = [], []
    for i, pts in enumerate(lanes):
        n = len(pts) - 1
        arr = np.zeros((n, 6))
        arr[:, 0:2], arr[:, 2:4] = pts[:-1], pts[1:]
        rows.append(arr)
        lane_ids.append(np.full(n, i, dtype=np.int32))
    k = len(lanes)
    return LaneMap(np.concatenate(rows), np.concatenate(lane_ids), np.zeros(k, np.uint8),
                   np.arange(k, dtype=np.uint64), np.ones(k, np.int16), {"cache_version": CACHE_VERSION})


def line(x0, y0, x1, y1, n=20):
    return np.stack([np.linspace(x0, x1, n + 1), np.linspace(y0, y1, n + 1)], axis=1)


@pytest.fixture
def highway() -> LaneMap:
    """Eastbound lanes at y=0 (lane 0) and y=4.5 (lane 1, right of it), westbound at y=-4.5 (lane 2);
    lane 0 continues straight (lane 3) and also splits 30 degrees to the right (lane 4)."""
    a = math.radians(30)
    return lanes_to_map([
        line(0, 0, 100, 0), line(0, 4.5, 100, 4.5), line(100, -4.5, 0, -4.5),
        line(100, 0, 200, 0), line(100, 0, 100 + 80 * math.cos(a), 80 * math.sin(a)),
    ])


def test_heading_and_quaternion_conventions():
    # SDK heading 0 = north (-Z) = rotation -pi/2; heading 0.25 (CCW, a left turn) = west (-X) = pi
    assert heading_to_rotation(0.0) == pytest.approx(-math.pi / 2)
    assert math.cos(heading_to_rotation(0.25)) == pytest.approx(-1.0)
    assert quat_forward_rotation(1, 0, 0, 0) == pytest.approx(-math.pi / 2)        # identity faces -Z
    q = (math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0)                          # +90 deg about +Y
    assert abs(quat_forward_rotation(*q)) == pytest.approx(math.pi)                 # faces -X, same as heading 0.25


def test_hermite_endpoints_and_offset_side():
    pts = hermite((0, 0), (100, 0), 0.0, 0.0, 10)
    assert pts[0] == pytest.approx([0, 0]) and pts[-1] == pytest.approx([100, 0])
    assert np.allclose(pts[:, 1], 0)
    # quarter circle-ish: start east, end south (rotation +pi/2 = a right turn in map coordinates)
    q = hermite((0, 0), (50, 50), 0.0, math.pi / 2, 20)
    assert q[1, 1] > 0 and q[-2, 0] < 50
    # right of eastbound travel is +y (game +Z = south)
    off = offset_polyline(line(0, 0, 10, 0), LANE_WIDTH)
    assert np.allclose(off[:, 1], LANE_WIDTH)


def test_nearest_lateral_sign_and_direction_filter(highway):
    h = highway.nearest(50, 1.0, rotation=0.0)
    assert h[0].lane == 0 and h[0].lateral == pytest.approx(1.0)        # 1 m right of lane 0
    assert h[1].lane == 1 and h[1].lateral == pytest.approx(-3.5)
    w = highway.nearest(50, -4.0, rotation=math.pi)
    assert [x.lane for x in w] == [2]                                    # eastbound lanes filtered out
    assert w[0].lateral == pytest.approx(-0.5)                           # westbound: right is -y
    assert highway.nearest(50, 30.0, rotation=0.0) == []


def test_successors_sorted_by_turn(highway):
    succ = highway.successors(0)
    assert [lane for lane, _ in succ] == [3, 4]
    assert succ[1][1] == pytest.approx(math.radians(30))
    assert highway.successors(1) == []
    assert highway.lane_length(0) == pytest.approx(100)


def test_save_load_roundtrip(highway, tmp_path):
    p = tmp_path / "lanes.npz"
    highway.save(p)
    lm = LaneMap.load(p)
    assert np.array_equal(lm.seg, highway.seg) and np.array_equal(lm.seg_lane, highway.seg_lane)
    assert lm.nearest(50, 1.0, 0.0)[0].lane == 0


def test_tracker_hysteresis_and_switch(highway):
    tr = LaneTracker(highway, hysteresis_m=1.2)
    assert tr.update(10, 4.5, 0.0).lane == 1
    st = tr.update(20, 2.0, 0.0)                  # lane 0 is 0.5 m closer: within hysteresis, stay
    assert st.lane == 1 and not st.switched
    st = tr.update(30, 0.5, 0.0)                  # clearly in lane 0 now
    assert st.lane == 0 and st.switched
    st = tr.update(110, 0.2, 0.0)                 # onto the successor: not a "switch"
    assert st.lane == 3 and not st.switched
    assert tr.update(50, 40, 0.0) is None


def test_path_ahead_follows_straightest_or_chosen_branch(highway):
    tr = LaneTracker(highway)
    tr.update(60, 0.3, 0.0)
    p = tr.path_ahead(80)
    assert p[0] == pytest.approx([60, 0])
    assert np.allclose(p[:, 1], 0) and p[-1, 0] >= 140
    branch = tr.path_ahead(80, choose=lambda lane, succ: succ[-1][0])
    assert branch[-1, 1] > 10                     # took the right-hand split


def test_build_from_parser_json(tmp_path):
    """A road with 2 forward + 1 backward lanes and a rotated prefab with one nav curve."""
    nodes = [{"uid": "a", "x": 0.0, "y": 0.0, "z": 10.0, "rotation": 0.0},
             {"uid": "b", "x": 100.0, "y": 0.0, "z": 12.0, "rotation": 0.0},
             {"uid": "c", "x": 500.0, "y": 0.0, "z": 0.0, "rotation": math.pi / 2}]
    files = {
        "nodes": nodes,
        "roadLooks": [{"token": "look", "offset": 2.0, "lanesRight": ["l", "l"], "lanesLeft": ["l"]}],
        "roads": [{"uid": "1f", "startNodeUid": "a", "endNodeUid": "b", "roadLookToken": "look", "length": 100.0},
                  {"uid": "3a", "startNodeUid": "a", "endNodeUid": "c", "roadLookToken": "look", "length": 500.0,
                   "hidden": True}],                                            # closed in the game: left out
        "prefabDescriptions": [{"token": "pf", "nodes": [{"x": 0.0, "y": 0.0, "rotation": 0.0}],
                                "navCurves": [{"start": {"x": 0.0, "y": 0.0, "rotation": 0.0, "z": 1.0},
                                               "end": {"x": 20.0, "y": 0.0, "rotation": 0.0, "z": 1.0}}]}],
        "prefabs": [{"uid": "2e", "token": "pf", "nodeUids": ["c"], "originNodeIndex": 0},
                    {"uid": "4b", "token": "pf", "nodeUids": ["a"], "originNodeIndex": 0, "hidden": True}],
    }
    for name, data in files.items():
        (tmp_path / f"europe-{name}.json").write_text(json.dumps(data))
    lm = LaneMap.build(tmp_path, log=lambda *_: None)
    assert list(lm.lane_kind) == [KIND_ROAD, KIND_ROAD, KIND_ROAD, KIND_PREFAB]
    assert list(lm.lane_sub) == [1, 2, -1, 0]
    assert lm.nearest(50, 2.0 + 2.25, 0.0)[0].lane == 0       # rule: lanes start `offset` (2 m) out
    assert lm.nearest(50, 2.0 + 6.75, 0.0)[0].lane == 1
    back = lm.nearest(50, -4.25, math.pi)[0]
    assert back.lane == 2 and abs(back.lateral) < 1e-6
    assert lm.seg[lm.lane_first[2], 4] == pytest.approx(12.0)  # backward lane starts at the far node's height
    # prefab rotated by +90 deg about its origin node: the curve runs from (500, 0) to (500, 20)
    pre = lm.lane_points(3)
    assert pre[0] == pytest.approx([500, 0]) and pre[-1] == pytest.approx([500, 20])
    assert lm.seg[lm.lane_first[3], 4] == pytest.approx(1.0)


def test_rule_layout_one_way_and_two_way():
    # one-way roads (measured on 1.53): lanes split around the road line, ceil(n/2) of them on the left
    assert rule_layout({"lanesRight": ["l"]}) == ([-2.25], [])
    assert rule_layout({"lanesRight": ["l"] * 2}) == ([-2.25, 2.25], [])
    assert rule_layout({"lanesRight": ["l"] * 3}) == ([-6.75, -2.25, 2.25], [])
    assert rule_layout({"lanesRight": ["l"] * 4})[0] == [-6.75, -2.25, 2.25, 6.75]
    # two-way: no median -> first lane centre 2.25 m out; a small `offset` moves lanes out by that much
    assert rule_layout({"lanesLeft": ["l"], "lanesRight": ["l"]}) == ([2.25], [-2.25])
    assert rule_layout({"lanesLeft": ["l"] * 2, "lanesRight": ["l"] * 2, "offset": 1.0}) == ([3.25, 7.75], [-3.25, -7.75])


def test_fit_lanes_finds_offset_and_narrow_width():
    obs = np.array([3.25] * 40 + [7.75] * 35 + [12.0] * 3)          # two lanes seen, plus a little noise
    offs, support = fit_lanes(obs, 2)
    assert offs == pytest.approx([3.25, 7.75]) and support == 75
    narrow, _ = fit_lanes(np.array([1.25] * 30), 1)                   # single narrow lane
    assert narrow == pytest.approx([1.25])
    three, _ = fit_lanes(np.array([-6.75] * 10 + [2.25] * 10), 3)    # middle lane never seen: still placed
    assert three == pytest.approx([-6.75, -2.25, 2.25])


def test_calibrate_looks_from_prefab_connections():
    """A one-lane road look whose lane really sits 3.25 m right of the line (wide median the look
    doesn't describe): a prefab curve ending at its start node shows it."""
    nodes = {"a": (0.0, 0.0, 0.0, 0.0), "b": (100.0, 0.0, 0.0, 0.0)}
    looks = {"odd": {"lanesLeft": ["l"], "lanesRight": ["l"]}, "plain": {"lanesLeft": ["l"], "lanesRight": ["l"]}}
    roads = [{"roadLookToken": "odd", "startNodeUid": "a", "endNodeUid": "b"} for _ in range(3)]
    into_a = (line(-20, 3.25, 0, 3.25), 0, 0, 1, 0)          # forward lane: ends at A, same direction
    out_of_a = (line(0, -3.25, -20, -3.25), 0, 0, 1, 1)      # backward lane: starts at A, opposite direction
    out = calibrate_looks(roads, nodes, looks, [into_a, out_of_a])
    fwd, bwd, source = out["odd"]
    assert source == "prefab evidence" and fwd == pytest.approx([3.25]) and bwd == pytest.approx([-3.25])
    assert out["plain"] == ([2.25], [-2.25], "rule")


def test_snap_offsets_trusts_the_junction_even_against_the_layout():
    from autohaul.world.lanemap import snap_offsets
    mirrored = [-13.25, -8.75, -4.25]                      # "Highway 3 lanes 2m offset": majority vote said left
    obs = np.array([4.25, 4.3, 8.75, 13.25])               # two curves start from the first lane
    assert snap_offsets(obs, mirrored) == pytest.approx([4.275, 8.75, 13.25])
    assert snap_offsets(np.array([-4.25, -8.75]), [-4.25, -8.75]) == pytest.approx([-4.25, -8.75])   # innermost first
    assert snap_offsets(np.array([2.0, 4.3, 8.7, 20.0]), [4.25, 8.75]) == pytest.approx([4.3, 8.7])   # extra lanes
    assert snap_offsets(np.array([4.3]), [4.25, 8.75]) is None                                         # too few


def test_lane_offsets_blend_small_corrections_and_replace_wrong_layouts():
    from autohaul.world.lanemap import lane_offsets
    frac = np.linspace(0, 1, 101)[:, None]
    small = lane_offsets(7.25, 4.25, None, True, False, frac, 200.0).ravel()   # lanes move in 3 m at the junction
    assert small[0] == pytest.approx(4.25) and small[20] == pytest.approx(7.25) and small[-1] == pytest.approx(7.25)
    wrong = lane_offsets(-13.25, 4.25, 4.25, True, False, frac, 200.0).ravel()  # layout on the wrong side
    assert np.allclose(wrong, 4.25)
    carried = lane_offsets(7.25, 4.25, None, False, False, frac, 200.0).ravel()  # small and only carried: layout
    assert np.allclose(carried, 7.25)


def test_router_counts_city_metres_like_the_game():
    """From A to B: 600 m of open road (19 game m per m: 11,400) or ~940 m through a city area (3 per m: ~2,800).
    The game's GPS takes the city; with city weights so does AutoHaul, without them the open road."""
    from autohaul.sim.course import arc_polyline
    from autohaul.world.graph import LaneGraph
    from autohaul.world.lanemap import hermite
    from autohaul.world.route import Router
    from tests.test_route import build
    start = arc_polyline([(100, 0.0)])
    straight = arc_polyline([(600, 0.0)], x=100, y=0)
    up = np.array(hermite((100.0, 0.0), (400.0, 300.0), 0.0, 0.0, 60))
    down = np.array(hermite((400.0, 300.0), (700.0, 0.0), 0.0, 0.0, 60))
    end = arc_polyline([(100, 0.0)], x=700, y=0)
    lm = build([(start, KIND_ROAD, 1, 1), (straight, KIND_ROAD, 2, 1), (up, KIND_ROAD, 3, 1), (down, KIND_ROAD, 4, 1),
                (end, KIND_ROAD, 5, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    plain = Router(lm, g).route(0, 0.0, (790.0, 0.0))
    w = np.ones(len(g.length))
    w[[2, 3]] = 3 / 19
    city = Router(lm, g, weight=w).route(0, 0.0, (790.0, 0.0))
    assert 1 in plain.lanes and 2 not in plain.lanes
    assert 2 in city.lanes and 3 in city.lanes and city.length_m > plain.length_m + 250   # lengths stay metres


def _stepped_join():
    """Two junction paths; the second starts 0.96 m to the left of the first one's end (as 322379 -> 322389 does)."""
    a = arc_polyline([(40, 0.0)], step=1.0)
    b = arc_polyline([(40, 0.0)], x=40.07, y=-0.96, step=1.0)
    from tests.test_route import build
    return build([(a, KIND_PREFAB, 7, 0), (b, KIND_PREFAB, 7, 1)])


def test_a_sideways_step_between_paths_is_eased_not_jumped():
    tr = LaneTracker(_stepped_join())
    tr.update(30.0, 0.0, 0.0)
    path = PathRef(tr.path_ahead(45.0))
    s = np.arange(0.0, path.length, 1.0)
    assert np.abs(np.asarray(path.curvature(s, 3.0))).max() < 0.04      # a jump reads as a sharp turn the wrong way
    ends = path.project(69.0, -0.96)
    assert abs(ends.lateral) < 0.02                                     # and it does end up on the second path


def test_the_eased_line_stays_put_as_the_truck_crosses_the_join():
    tr = LaneTracker(_stepped_join())
    tr.update(30.0, 0.0, 0.0)
    line = PathRef(tr.path_ahead(45.0))
    i = np.searchsorted(line.pts[:, 0], 45.0)
    y = float(line.pts[i, 1])
    st = tr.update(float(line.pts[i, 0]), y, 0.0)
    assert st.lane == 1 and abs(st.lateral) < 0.05 and st.confidence > 0.7   # against the line, not the map's step
    after = tr.path_ahead(10.0, behind=8.0)
    assert np.hypot(*(after[0] - after[1])) < 2.0
    assert max(abs(line.project(float(x), float(z)).lateral) for x, z in after if x > 30.0) < 0.05
    st = tr.update(70.0, -0.96, 0.0)
    assert abs(st.lateral) < 0.02                                       # eased out: the path's own centre again


def test_a_tight_curve_of_short_pieces_keeps_its_curvature():
    path = PathRef(arc_polyline([(20.0, -1 / 10)], step=0.45))
    assert abs(float(path.curvature(10.0, 3.0)) + 0.1) < 0.005
    assert abs(path.mean_curvature(8.0, 12.0) + 0.1) < 0.005
