"""Lane map and lane tracker on small synthetic maps (no game data needed)."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from autohaul.world.lanemap import (CACHE_VERSION, KIND_PREFAB, KIND_ROAD, LANE_WIDTH, LaneMap, calibrate_looks, fit_lanes,
                                    heading_to_rotation, hermite, offset_polyline, quat_forward_rotation, rule_layout)
from autohaul.world.localizer import LaneTracker


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
        "roads": [{"uid": "1f", "startNodeUid": "a", "endNodeUid": "b", "roadLookToken": "look", "length": 100.0}],
        "prefabDescriptions": [{"token": "pf", "nodes": [{"x": 0.0, "y": 0.0, "rotation": 0.0}],
                                "navCurves": [{"start": {"x": 0.0, "y": 0.0, "rotation": 0.0, "z": 1.0},
                                               "end": {"x": 20.0, "y": 0.0, "rotation": 0.0, "z": 1.0}}]}],
        "prefabs": [{"uid": "2e", "token": "pf", "nodeUids": ["c"], "originNodeIndex": 0}],
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
