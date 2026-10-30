"""Fixes from the first full in-game mission test (2026-10-23): lane changes without room re-route in lane, the
driver's lane position is kept (map lanes can be ~1 m off), driving out of a yard onto the lanes, headlights."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.drive.lights import Headlights
from autohaul.sim.course import HIGHWAY, Course, arc_polyline, offset
from autohaul.sim.truck import TruckSim
from autohaul.world.graph import LaneGraph
from autohaul.world.lanemap import KIND_PREFAB, KIND_ROAD, hermite
from autohaul.world.route import Router
from tests.test_route import build

GEOM = TruckGeometry()


def make_split_with_a_way_round(far: float = 200.0):
    """Two-lane road (lane 0 left, lane 1 right) to a split at x=300: straight on from the left lane (2) onto
    road S1 (5) and S2 (6, from x=1000); the exit from the right lane (3) runs onto a longer way round (4, 7)
    that comes back into S2."""
    left = arc_polyline([(300, 0.0)])
    right = offset(left, 4.5)
    straight = arc_polyline([(60, 0.0)], x=300, y=0)
    exit_ = arc_polyline([(60, 1 / 80)], x=300, y=4.5)
    s1 = arc_polyline([(640, 0.0)], x=360, y=0)
    s2 = arc_polyline([(800, 0.0)], x=1000, y=0)
    e_rot = math.atan2(exit_[-1, 1] - exit_[-2, 1], exit_[-1, 0] - exit_[-2, 0])
    round_a = np.array(hermite(tuple(exit_[-1]), (700.0, far), e_rot, 0.0, 80))
    round_b = np.array(hermite((700.0, far), (1000.0, 0.0), 0.0, 0.0, 80))
    lm = build([(left, KIND_ROAD, 1, 1), (right, KIND_ROAD, 1, 2), (straight, KIND_PREFAB, 7, 0),
                (exit_, KIND_PREFAB, 7, 1), (round_a, KIND_ROAD, 9, 1), (s1, KIND_ROAD, 8, 1),
                (s2, KIND_ROAD, 10, 1), (round_b, KIND_ROAD, 11, 1)])
    return lm, LaneGraph.build(lm, log=lambda *_: None)


def test_router_can_forbid_lane_changes_near_the_start():
    lm, g = make_split_with_a_way_round()
    r = Router(lm, g)
    target = (1700.0, 0.0)
    normal = r.route(1, 250.0, target)                       # right lane, 50 m before the split
    assert normal.changes[1] and normal.lanes[:2] == [1, 0]   # the cheap way: change left, straight on
    stay = r.route(1, 250.0, target, no_change_m=200.0)
    assert not any(stay.changes) and 3 in stay.lanes and 4 in stay.lanes   # the exit and back round


def test_no_room_for_a_lane_change_re_routes_instead_of_handing_back():
    lm, g = make_split_with_a_way_round()
    sim = TruckSim(GEOM, x=250.0, y=4.5, rotation=0.0, speed=60 / 3.6)  # right lane, 50 m before the split
    ap = Autopilot(lm, GEOM, "relative", router=Router(lm, g))
    ap.set_destination(1700.0, 0.0)
    assert ap.engage(sim.frame(80 / 3.6)) is None
    lanes, notes, fault = [], set(), None
    for _ in range(int(60 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(80 / 3.6), [], sim.p.dt)
        if st.lane is not None and (not lanes or lanes[-1] != st.lane):
            lanes.append(st.lane)
        notes.add(st.route_note)
        if st.fault:
            fault = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert any("re-routed in lane" in n for n in notes)
    assert fault is None or fault.startswith("destination reached")
    assert 3 in lanes and 4 in lanes                         # took the exit and came back round


def test_the_engage_offset_is_kept_and_blocks_lane_changes():
    hw = Course.from_pieces(HIGHWAY)
    r = hw.path.rotation_at(20.0)
    x, y = hw.path.point(20.0)
    sim = TruckSim(GEOM, x=x - 0.8 * math.sin(r), y=y + 0.8 * math.cos(r), rotation=r, speed=70 / 3.6)
    ap = Autopilot(hw.lm, GEOM, "relative")
    assert ap.engage(sim.frame(80 / 3.6)) is None
    assert abs(ap.bias) > 0.7 and not ap._changes_allowed()
    lat = []
    for _ in range(int(20 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(80 / 3.6), [], sim.p.dt)
        lat.append(hw.path.project(sim.s.x, sim.s.y).lateral)
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert abs(np.mean(lat[-120:]) - ap.bias) < 0.25         # stays where the driver put it, fading slowly
    assert 0.5 < abs(ap.bias) < 0.8


def test_drives_out_of_a_yard_onto_the_lane():
    road = arc_polyline([(1200, 0.0)])
    lm = build([(road, KIND_ROAD, 1, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    rot = math.atan2(25.0, 30.0)                            # parked 25 m off the road, nose towards it
    sim = TruckSim(GEOM, x=100.0, y=-25.0, rotation=rot, speed=0.0)
    ap = Autopilot(lm, GEOM, "relative", router=Router(lm, g))
    ap.set_destination(1100.0, 0.0)
    assert ap.engage(sim.frame(80 / 3.6)) is None and ap.join is not None
    fault, joined_at = None, None
    for _ in range(int(80 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(80 / 3.6), [], sim.p.dt)
        if ap.join is None and joined_at is None:
            joined_at = sim.t
        if st.fault:
            fault = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert joined_at is not None and joined_at < 30
    assert fault is None or fault.startswith("destination reached")
    assert abs(sim.s.y) < 0.6 and sim.s.x > 250              # on the lane and driving on


def test_headlights_follow_the_clock_whoever_switched_them_on():
    """2026-10-30: they only switched off lights AutoHaul had switched on; they'd been on all along, so it did nothing."""
    hl = Headlights()
    night, day = 22 * 60, 10 * 60
    assert hl.step(night, low_beam=False, parking=False, now=0.0) == "light"
    assert hl.step(night, low_beam=False, parking=True, now=0.1) is None          # wait for telemetry
    assert hl.step(night, low_beam=False, parking=True, now=1.0) == "light"
    assert hl.step(night, low_beam=True, parking=False, now=2.0) is None
    assert hl.step(day, low_beam=True, parking=False, now=3.0) == "light"         # morning: off
    assert hl.step(day, low_beam=False, parking=False, now=4.0) is None
    left_on = Headlights()
    assert left_on.step(day, low_beam=True, parking=False, now=0.0) == "light"    # on from before, by day: off too
    high = Headlights()
    assert high.step(night, low_beam=True, parking=False, now=0.0, high_beam=True) == "hblight"
    assert high.step(night, low_beam=True, parking=False, now=1.0, high_beam=False) is None


def test_headlights_leave_alone_what_you_just_changed():
    hl = Headlights()
    day = 10 * 60
    assert hl.step(day, low_beam=False, parking=False, now=0.0) is None
    assert hl.step(day, low_beam=True, parking=False, now=50.0) is None           # you switched them on
    assert hl.step(day, low_beam=True, parking=False, now=300.0) is None          # left alone for a while
    assert hl.step(day, low_beam=True, parking=False, now=700.0) == "light"       # ... then the clock again


def load_drive_tool():
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "tools" / "drive.py"
    spec = importlib.util.spec_from_file_location("drive_tool", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_lost_focus_brings_the_game_back_or_hands_back(monkeypatch):
    """2026-10-23: a click on another window left the game at full throttle for a minute (it reads no input
    while unfocused). Now AutoHaul brings the game back to the front, and hands back only if that fails."""
    from types import SimpleNamespace
    drive = load_drive_tool()
    GAME, OTHER = 100, 200
    fg = {"pid": OTHER}
    tries = []

    def bring(pid):
        tries.append(pid)
        return False

    monkeypatch.setattr(drive, "foreground_pid", lambda: fg["pid"])
    monkeypatch.setattr(drive, "bring_to_front", bring)
    d = drive.Driver.__new__(drive.Driver)
    d.s = SimpleNamespace(reader=SimpleNamespace(header=lambda: SimpleNamespace(game_pid=GAME)), recorder=None)
    d.refocus, d.refocus_grace_s = True, 0.5
    d._unfocused_at, d._refocus_until, d._refocus_tried = None, 0.0, -1e9
    d.message = ""
    assert d.keep_focus(0.0) and tries == [GAME]                 # within the grace time: keep driving
    fg["pid"] = GAME
    assert d.keep_focus(0.2) and d._unfocused_at is None and "brought the game back" in d.message
    fg["pid"] = OTHER
    assert d.keep_focus(1.0)
    assert not d.keep_focus(1.6)                                 # still not in front: hand back
    d._refocus_until, n = 11.6, len(tries)
    d.refocus_after_handback(1.9)
    assert len(tries) == n + 1                                   # keeps trying so the released controls arrive
    fg["pid"] = GAME
    d.refocus_after_handback(2.2)
    assert d._refocus_until == 0.0


def _vehicle(vid, x, z, heading_rot, speed, length=4.5, width=1.8):
    """A traffic entry facing map rotation `heading_rot` (inverse of lanemap.quat_forward_rotation)."""
    # forward (-Z) = (cos r, sin r) in map x/y; a yaw-only quaternion about +Y
    yaw = math.atan2(-math.cos(heading_rot), -math.sin(heading_rot))
    return {"id": vid, "wx": x, "wz": z, "qw": math.cos(yaw / 2), "qx": 0.0, "qy": math.sin(yaw / 2), "qz": 0.0,
            "length": length, "width": width, "speed": speed, "trailer_count": 0, "trailers": []}


def test_lead_ignores_crossing_oncoming_and_beside_but_not_real_leads():
    from autohaul.control.speed import SpeedParams, find_lead
    from autohaul.world.lanemap import quat_forward_rotation
    hw = Course.from_pieces(HIGHWAY)
    path = hw.path
    sp = SpeedParams()
    x0, y0 = path.point(100.0)
    r = path.rotation_at(100.0)
    v = _vehicle(1, x0, y0, r, 10.0)
    assert abs(math.remainder(quat_forward_rotation(v["qw"], v["qx"], v["qy"], v["qz"]) - r, 2 * math.pi)) < 1e-6
    ahead = find_lead(path, 50.0, [v], sp)
    assert ahead is not None and 40 < ahead.gap < 50 and ahead.speed > 9
    beside = find_lead(path, 103.0, [_vehicle(2, x0, y0, r, 25.0)], sp)            # its rear behind our bumper
    assert beside is None
    crossing = find_lead(path, 50.0, [_vehicle(3, x0, y0, r + math.pi / 2, 12.0)], sp)
    assert crossing is None                                                          # moving: clears the lane
    stalled = find_lead(path, 50.0, [_vehicle(4, x0, y0, r + math.pi / 2, 0.3)], sp)
    assert stalled is not None and stalled.gap > 40                                  # standing across our lane
    nx, ny = -math.sin(r), math.cos(r)
    next_lane = find_lead(path, 50.0, [_vehicle(5, x0 + 2.0 * nx, y0 + 2.0 * ny, r + math.pi, 20.0)], sp)
    assert next_lane is None                                                         # oncoming, 2 m off centre
    head_on = find_lead(path, 50.0, [_vehicle(6, x0, y0, r + math.pi, 20.0)], sp)
    assert head_on is not None and head_on.speed < 0


def test_keeper_starts_the_engine_releases_the_brake_and_shifts_to_drive():
    """1.53: the engine starts only while `engine` is held (~1.5 s); a press does nothing."""
    from types import SimpleNamespace
    from autohaul.drive.keeper import Keeper
    k = Keeper()
    f = SimpleNamespace(engine_enabled=False, electric_enabled=False, parking_brake=True, displayed_gear=0, speed=0.0)
    assert k.step(f, True, 0.02, 0.0).press == "engineelect"
    f.electric_enabled = True
    assert k.step(f, True, 0.02, 0.5).problem == "starting the engine"            # wait for telemetry
    s = k.step(f, True, 0.02, 2.1)
    assert s.hold == "engine" and s.press is None
    assert k.step(f, True, 0.02, 3.0).hold == "engine"                          # still holding (1.5 s)
    assert k.step(f, True, 0.02, 3.7).hold is None                              # released, waiting
    f.engine_enabled = True
    assert k.step(f, True, 0.02, 5.8).press == "parkingbrake"
    assert k.step(f, False, 0.02, 8.0).problem is None                          # not moving: leave the brake
    f.parking_brake = False
    assert k.step(f, True, 0.3, 8.1).problem is None                            # neutral, not long yet
    assert k.step(f, True, 0.3, 8.4).press == "geardrive"
    f.displayed_gear, f.speed = -1, -1.0
    assert k.step(f, True, 0.02, 11.0).problem == "stopping before shifting out of reverse"
    f.speed = 0.0
    assert k.step(f, True, 0.02, 11.1).press == "geardrive"
    for t_ in (13.2, 15.3, 17.4):
        k.step(f, True, 0.02, t_)
    assert "didn't work" in (k.step(f, True, 0.02, 19.5).failure or "")
    k.reset()                                                                   # a new engage starts fresh
    assert k.step(f, True, 0.02, 20.0).press == "geardrive"


def test_the_driver_can_be_built_in_every_mode():
    """2026-10-23: drive.py crashed on start (the keeper read `longitudinal` before it was set)."""
    from types import SimpleNamespace
    drive = load_drive_tool()
    hw = Course.from_pieces(HIGHWAY)
    s = SimpleNamespace(cfg={"drive": {"keeper": {"press_every_s": 1.5}}},
                        keyboard=SimpleNamespace(state=SimpleNamespace(press_count=lambda vk: 0)), recorder=None)
    for lateral, longitudinal in ((True, True), (True, False), (False, True)):
        ap = Autopilot(hw.lm, GEOM, "relative")
        d = drive.Driver(s, ap, lateral, longitudinal)
        assert (d.keeper is not None) == longitudinal and d.lateral == lateral
        assert ap.p.ready_check == (not longitudinal)
        assert d.keeper is None or d.keeper.p.press_every_s == 1.5


def test_a_destination_away_from_the_roads_routes_to_the_nearest_road():
    """`--to paris` had "no route": the city centre point is more than 60 m from any lane."""
    road = arc_polyline([(1200, 0.0)])
    lm = build([(road, KIND_ROAD, 1, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    r = Router(lm, g)
    rt = r.route(0, 10.0, (800.0, 400.0))                    # 400 m off the road
    assert rt is not None and 780 < 10.0 + rt.length_m < 820
    assert Router(lm, g, snap_m=0.0).route(0, 10.0, (800.0, 400.0)) is None



def test_a_long_way_round_is_not_taken_a_short_lane_change_is():
    """2026-10-29: 98 m left to merge off a slip lane; AutoHaul re-routed from its lane instead, 15 km longer.
    Now a way on from the lane may be only a little longer; otherwise the change is squeezed into the room left."""
    lm, g = make_split_with_a_way_round(far=2500.0)            # the way round: ~5 km longer
    sim = TruckSim(GEOM, x=225.0, y=4.5, rotation=0.0, speed=40 / 3.6)  # right lane, 75 m before the split
    ap = Autopilot(lm, GEOM, "relative", router=Router(lm, g))
    ap.set_destination(1700.0, 0.0)
    assert ap.engage(sim.frame(80 / 3.6)) is None
    lanes, notes, fault = [], set(), None
    for _ in range(int(80 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(80 / 3.6), [], sim.p.dt)
        if st.lane is not None and (not lanes or lanes[-1] != st.lane):
            lanes.append(st.lane)
        notes.add(st.route_note)
        if st.fault:
            fault = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert not any("re-routed in lane" in n for n in notes)
    assert 0 in lanes and 2 in lanes and 4 not in lanes                # changed left, straight on
    assert fault is None or fault.startswith("destination reached")


def test_with_a_trailer_the_cab_swings_out_in_a_tight_curve():
    """2026-10-29: in roundabouts the trailer cut in over the kerb. With a trailer the cab now follows a line
    outside the lane centre in tight curves (a share of the trailer's cut-in), and back on the straight."""
    from types import SimpleNamespace
    from autohaul.control.path import PathRef
    pts = arc_polyline([(60, 0.0), (40, 1 / 15), (80, 0.0)])           # a 15 m right-hand curve
    lm = build([(pts, KIND_ROAD, 1, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    ap = Autopilot(lm, GEOM, "relative", router=Router(lm, g))
    ap.set_destination(*pts[-5])
    assert ap._swing(1 / 15) < -0.8 and ap._swing(-1 / 15) > 0.8          # out to the left / to the right
    assert ap._swing(1 / 100) == 0.0
    path = PathRef(pts)
    off = ap._with_trailer_swing(path, 0.0, None)
    assert abs(off(30.0)[0]) < 0.05 and off(80.0)[0] < -0.8                # straight: centre; in the curve: out
    sim = TruckSim(GEOM, x=5.0, y=0.0, rotation=0.0, speed=12 / 3.6)
    assert ap.engage(sim.frame(15 / 3.6)) is None
    lat, fault = [], None
    for _ in range(int(40 / sim.p.dt)):
        f = sim.frame(15 / 3.6)
        f.trailers = [SimpleNamespace(connected=True)]
        cmd, st = ap.step(f, [], sim.p.dt)
        if st.fault:
            fault = st.fault
            break
        pr = path.project(sim.s.x, sim.s.y)
        if 72 < pr.s < 92:
            lat.append(pr.lateral)
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert fault is None or fault.startswith("destination reached"), fault
    assert lat and np.mean(lat) < -0.4                                      # the cab ran outside the centre



def _paused_driver(moved_to=None):
    """A Driver engaged at (100, 200), with just what the pause handling touches."""
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import drive
    d = drive.Driver.__new__(drive.Driver)
    d.s = MagicMock()
    d.s.safety.check.return_value = SimpleNamespace(ok=False, kind="paused", reason="game paused")
    d.md = None
    d.ap = MagicMock()
    d.engaged = True
    d.message = "ENGAGED"
    d.notes = []
    d.note = lambda **kw: d.notes.append(kw)
    d.disengage = MagicMock()
    d.feed_overlay = MagicMock()
    d.print_status = MagicMock()
    d._last_status = 1e18
    d.status_period = 1.0
    d.last_cmd = "the last command"
    return d, SimpleNamespace(truck_placement=SimpleNamespace(x=100.0, z=200.0), paused=1)


def test_a_game_pause_doesnt_hand_back():
    """2026-10-30: paused, AutoHaul handed back; after the pause nobody steered, and at 78 km/h the truck hit the rail."""
    d, f = _paused_driver()
    d.hold_paused(f, None, [], 10.0)
    d.disengage.assert_not_called()
    assert d.engaged and d._paused_at == (100.0, 200.0) and "paused" in d.message
    d.s.controls.commit.assert_called()                       # the heartbeat goes on
    assert d.resume_after_pause(f) is True
    d.disengage.assert_not_called()
    assert d._paused_at is None and d.message == "ENGAGED"
    d.s.safety.reset.assert_called()                          # keys pressed in the menus don't count
    d.ap.servo.reset.assert_called()
    assert d.last_cmd != "the last command"


def test_a_truck_moved_during_a_pause_is_handed_back():
    d, f = _paused_driver()
    d.hold_paused(f, None, [], 10.0)
    f.truck_placement.x = 5000.0                              # a save was loaded
    assert d.resume_after_pause(f) is False
    d.disengage.assert_called_once()


def test_the_kill_key_works_during_a_pause():
    from types import SimpleNamespace
    d, f = _paused_driver()
    d.s.safety.check.return_value = SimpleNamespace(ok=False, kind="kill", reason="kill key")
    d.hold_paused(f, None, [], 10.0)
    d.disengage.assert_called_once()
