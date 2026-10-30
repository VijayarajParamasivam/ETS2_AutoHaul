"""Handing over at speed: holding the lane until the driver takes it. And learning from how the driver then drives."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.learn.learner import Learner, lane_key
from autohaul.sim.course import arc_polyline
from autohaul.sim.truck import TruckSim
from autohaul.world.graph import LaneGraph
from autohaul.world.lanemap import KIND_PREFAB, KIND_ROAD
from autohaul.world.route import Router
from tests.test_route import build

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


def _course(radius=60.0, curve_m=60.0):
    """A straight road (0), a curved junction path (1), a straight road after it (2)."""
    a = arc_polyline([(300, 0.0)])
    curve = arc_polyline([(curve_m, 1 / radius)], x=300, y=0)
    end, rot = curve[-1], math.atan2(curve[-1, 1] - curve[-2, 1], curve[-1, 0] - curve[-2, 0])
    b = arc_polyline([(400, 0.0)], x=end[0], y=end[1], rot=rot)
    return build([(a, KIND_ROAD, 0xA1, 1), (curve, KIND_PREFAB, 0xA2, 0), (b, KIND_ROAD, 0xA3, 1)])


def _ap(lm, x=20.0, speed=15.0, router=True, **kw):
    geom = TruckGeometry()
    sim = TruckSim(geom, x=x, y=0.0, rotation=0.0, speed=speed)
    args = {}
    if router:
        args["router"] = Router(lm, LaneGraph.build(lm, log=lambda *_: None))
    ap = Autopilot(lm, geom, "relative", **args, **kw)
    if router:
        ap.set_destination(*lm.lane_points(2)[-1])
    assert ap.engage(sim.frame(25.0)) is None
    return ap, sim


# ------------------------------------------------------------------ the safe stop
def test_a_safe_stop_holds_the_lane_and_brakes_to_a_standstill():
    """2026-10-30: unsteered after a hand-over, the truck ran into a rail at 78 km/h."""
    lm = _course()
    ap, sim = _ap(lm, x=235.0, speed=22.0)                    # close enough to reach the curve while braking
    for _ in range(60):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    ap.safe_stop = True
    worst = 0.0
    went_through_curve = False
    for _ in range(int(40 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        assert not st.fault, st.fault
        assert cmd.throttle == 0.0
        worst = max(worst, abs(st.lateral))
        went_through_curve |= st.lane == 1
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert sim.s.speed == pytest.approx(0.0, abs=0.05)
    assert went_through_curve and worst < 1.0                 # it steered through the curve while slowing
    assert cmd.brake >= 0.5                                   # and it stays there
    assert st.target_reason == "stopping: take over"


def test_a_safe_stop_doesnt_hand_back_again_for_what_it_was_handing_back_for():
    lm = _course(radius=15.0, curve_m=30.0)                   # sharper than it would take alone
    ap, sim = _ap(lm, x=200.0, speed=12.0, router=False)
    fault = None
    for _ in range(int(10 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        if st.fault:
            fault = st.fault
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert fault and fault.startswith("sharp turn ahead")
    ap.safe_stop = True
    for _ in range(int(20 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        assert not st.fault, st.fault
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert sim.s.speed == pytest.approx(0.0, abs=0.05)


def test_engaging_again_ends_a_safe_stop():
    lm = _course()
    ap, sim = _ap(lm)
    ap.safe_stop = True
    assert ap.engage(sim.frame(25.0)) is None and ap.safe_stop is False


def _driver():
    import drive
    d = drive.Driver.__new__(drive.Driver)
    d.s = MagicMock()
    d.ap = MagicMock(safe_stop=False)
    d.ind = MagicMock(mine=None)
    d.learner = MagicMock()
    d.feed = None
    d.beeps = False
    d.lateral = d.longitudinal = True
    d.engaged = True
    d.message = "ENGAGED"
    d.stats = {"faults": {}}
    d.notes = []
    d.note = lambda **kw: d.notes.append(kw)
    d._unfocused_at = None
    d._last_lane = 7
    d._kept_hold = None
    d.last_cmd = None
    return d


def test_which_hand_overs_can_stop_safely():
    d = _driver()
    moving, parked = SimpleNamespace(speed=20.0), SimpleNamespace(speed=1.0)
    for ok in ("no room left to change lanes to the right (too close to the split): take over", "1.6 m off the lane centre",
               "sharp turn ahead (radius 6 m in 27 m): take over", "lane match uncertain for 2.0 s",
               "waited 6 s at the junction (red light: stopping): take over"):
        assert d.can_stop_safely(moving, ok), ok
    for no in ("the wheel is moving against AutoHaul's steering (someone else steering?)", "lost the lane (no mapped lane under the truck)",
               "the game isn't taking AutoHaul's brake (sent 0.50, game shows 0.00)", "no path ahead on the lane map",
               "at the destination: delivered by quick park"):
        assert not d.can_stop_safely(moving, no), no
    assert not d.can_stop_safely(parked, "1.6 m off the lane centre")   # standing already
    d.longitudinal = False                                    # it doesn't have the pedals: can't brake
    assert not d.can_stop_safely(moving, "1.6 m off the lane centre")


def test_a_safe_stop_warns_and_gives_the_truck_back_at_a_key():
    d = _driver()
    f = SimpleNamespace(speed=20.0, hazard_warning=0)
    d.begin_safe_stop(f, "1.6 m off the lane centre", 10.0)
    assert d.stopping == "1.6 m off the lane centre" and d.ap.safe_stop is True
    d.s.controls.press.assert_called_with("flasher4way")
    d.learner.on_handback.assert_called_once_with("1.6 m off the lane centre", 20.0, 7)
    d.safe_stop_alarm(f, 11.0)
    assert d.message.startswith("TAKE OVER: 1.6 m off the lane centre (slowing down")
    # the driver presses a key
    d.s.reader.read_latest.return_value = SimpleNamespace(speed=9.0, hazard_warning=1, lblinker=0, rblinker=0)
    d.disengage("driving key A", fault=False)
    assert d.stopping is None and d.ap.safe_stop is False and d.engaged is False
    d.learner.on_handback.assert_called_once()                # not twice: it was reported when it began
    assert d.learner.start_lesson.call_args[0][2] == "1.6 m off the lane centre"   # the lesson knows why
    assert d.s.controls.press.call_args_list[-1][0][0] == "flasher4way"            # hazards back off


def test_taking_control_yourself_starts_a_lesson_too():
    d = _driver()
    d.stopping = None
    d._hazards_mine = False
    d.s.reader.read_latest.return_value = SimpleNamespace(speed=9.0, hazard_warning=0, lblinker=0, rblinker=0)
    d.disengage("SLASH pressed", fault=False)
    assert d.learner.start_lesson.call_args[0][2] == "SLASH pressed"


# ------------------------------------------------------------------ lessons
def _frame_at(lm, lane, s, speed, lateral=0.0, throttle=0.0, wear=0.0):
    pts = lm.lane_points(lane)
    d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    i = min(int(np.searchsorted(d, s, "right")) - 1, len(pts) - 2)
    t = (s - d[i]) / max(d[i + 1] - d[i], 1e-9)
    p = pts[i] + t * (pts[i + 1] - pts[i])
    rot = math.atan2(pts[i + 1, 1] - pts[i, 1], pts[i + 1, 0] - pts[i, 0])
    x = p[0] - math.sin(rot) * lateral                        # + = right of the lane direction (rotation + = right)
    y = p[1] + math.cos(rot) * lateral
    f = TruckSim(TruckGeometry(), x=float(x), y=float(y), rotation=rot, speed=speed).frame(25.0)
    f.input_throttle, f.wear_chassis = throttle, wear
    return f


def _drive_lane(lr, lm, lane, t, speed, lateral=0.0, wear=0.0, s0=0.0, s1=None):
    s = s0
    end = lm.lane_length(lane) if s1 is None else s1
    while s < end:
        lr.observe_driver(t, _frame_at(lm, lane, s, speed, lateral, wear=wear))
        t += 0.1
        s += max(speed, 0.5) * 0.1
    return t


def test_a_lesson_keeps_your_speed_and_line_through_a_junction():
    lm = _course()
    lr = Learner(None, lm)
    f = _frame_at(lm, 0, 280.0, 15.0)
    lr.start_lesson(0.0, f, "sharp turn ahead")
    t = _drive_lane(lr, lm, 0, 0.0, 15.0, s0=280.0)
    t = _drive_lane(lr, lm, 1, t, 8.0, lateral=-1.0)
    t = _drive_lane(lr, lm, 2, t, 12.0, s1=100.0)
    assert lr.end_lesson(_frame_at(lm, 2, 100.0, 12.0)) == 1
    assert lr.bins(0) is None and lr.bins(2) is None          # straight roads aren't places worth a lesson
    bins = lr.bins(1)
    speeds = [r[0] for r in bins if r[0] is not None]
    lats = [r[1] for r in bins if r[1] is not None]
    assert speeds and float(np.median(speeds)) == pytest.approx(8.0, abs=0.01)
    assert lats and float(np.median(lats)) == pytest.approx(-1.0, abs=0.2)    # left of the lane
    assert lr.lessons[lane_key(lm, 1)]["why"] == "sharp turn ahead"


def test_a_lane_change_isnt_taken_for_your_line():
    """Seen on real drives: -3.4 m. Copying that would steer towards the next lane or a rail."""
    lm = _course(radius=60.0, curve_m=60.0)
    lr = Learner(None, lm)
    lr.start_lesson(0.0, _frame_at(lm, 1, 0.0, 10.0), "you took over")
    t, s = 0.0, 0.0
    while s < 58.0:                                           # across the path, from +1.5 to -2.5
        lr.observe_driver(t, _frame_at(lm, 1, s, 10.0, lateral=1.5 - 4.0 * s / 58.0))
        t, s = t + 0.1, s + 1.0
    lr.end_lesson(_frame_at(lm, 2, 0.0, 10.0))
    bins = lr.bins(1)
    assert bins and all(r[1] is None for r in bins)           # no line
    assert any(r[0] is not None for r in bins)                # the speed still counts
    lr = Learner(None, lm)                                    # a curved road: speed, but never a line
    curved = build([(arc_polyline([(100, 1 / 60)]), KIND_ROAD, 0xC1, 1)])
    lr = Learner(None, curved)
    lr.start_lesson(0.0, _frame_at(curved, 0, 0.0, 10.0), "you took over")
    t = _drive_lane(lr, curved, 0, 0.0, 10.0, lateral=0.5)
    lr.end_lesson(_frame_at(curved, 0, 99.0, 10.0))
    bins = lr.bins(0)
    assert bins and all(r[1] is None for r in bins) and any(r[0] is not None for r in bins)


def test_damage_during_a_lesson_means_nothing_is_learned():
    lm = _course()
    for crash, wear in ((True, 0.0), (False, 0.08)):
        lr = Learner(None, lm)
        lr.start_lesson(0.0, _frame_at(lm, 1, 0.0, 8.0), "you took over")
        t = _drive_lane(lr, lm, 1, 0.0, 8.0)
        if crash:
            lr.on_fine("crash", 8.0, 1)
        assert lr.end_lesson(_frame_at(lm, 2, 0.0, 8.0, wear=wear)) == 0 and not lr.lessons


def test_speeds_around_a_stop_arent_about_the_place():
    """Waiting at a red light in a junction doesn't mean it should crawl there for ever."""
    lm = _course(radius=60.0, curve_m=90.0)
    lr = Learner(None, lm)
    lr.start_lesson(0.0, _frame_at(lm, 1, 10.0, 6.0), "you took over")
    t = 0.0
    s = 10.0
    while s < 45.0:                                           # slowing to a stop at 45 m, then off again
        v = max(0.2, math.sqrt(2 * 0.5 * (45.0 - s)))
        lr.observe_driver(t, _frame_at(lm, 1, s, v))
        t, s = t + 0.1, s + v * 0.1
    for _ in range(50):
        lr.observe_driver(t, _frame_at(lm, 1, 45.0, 0.0))
        t += 0.1
    lr.end_lesson(_frame_at(lm, 1, 45.0, 0.0))
    bins = lr.bins(1) or []
    assert all(r[0] is None for r in bins)                    # no speed from that (the line it did show is kept)


def test_lessons_blend_survive_a_restart_and_relax_with_clean_passes(tmp_path):
    lm = _course()
    lr = Learner(tmp_path / "learned.json", lm)
    for speed in (8.0, 6.0):
        lr.start_lesson(0.0, _frame_at(lm, 1, 0.0, speed), "you took over")
        _drive_lane(lr, lm, 1, 0.0, speed)
        lr.end_lesson(_frame_at(lm, 1, 50.0, speed))
    assert lr.lessons[lane_key(lm, 1)]["n"] == 2
    first = [r[0] for r in lr.bins(1) if r[0] is not None][0]
    assert first == pytest.approx(7.0, abs=0.01)
    lr.save()
    again = Learner(tmp_path / "learned.json", lm)
    assert [r[0] for r in again.bins(1) if r[0] is not None][0] == pytest.approx(7.0, abs=0.01)
    again.observe(0.0, _frame_at(lm, 1, 1.0, 7.0), {"lane": 1}, True, False)
    again.observe(0.1, _frame_at(lm, 2, 1.0, 7.0), {"lane": 2}, True, False)   # drove it alone, nothing wrong
    assert [r[0] for r in again.bins(1) if r[0] is not None][0] == pytest.approx(7.0 * 1.03, abs=0.01)


class _Taught:
    def __init__(self, lane, n, speed, lateral):
        self.lane, self.rows = lane, [[speed, lateral] for _ in range(n)]

    def bins(self, lane):
        return self.rows if lane == self.lane else None


def _through_the_curve(lm, teach, speed=15.0):
    ap, sim = _ap(lm, x=200.0, speed=speed)
    ap.teach = teach
    worst_v, lats, reasons = 0.0, [], []
    d_curve = lm.lane_points(1)
    for _ in range(int(25 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        assert not st.fault, st.fault
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
        reasons.append(st.target_reason)
        if st.lane == 1:
            worst_v = max(worst_v, sim.s.speed)
            h = lm.nearest(sim.s.x, sim.s.y, sim.s.rotation, max_dist=8.0)[0]
            done = float(lm.seg_len[lm.lane_first[1]:h.segment].sum())
            if 20.0 < done < 45.0:
                lats.append(h.lateral)
    assert len(d_curve) >= 2
    return worst_v, float(np.mean(lats)) if lats else 0.0, reasons


def test_the_autopilot_follows_your_line_at_your_speed():
    lm = _course()
    n = 1 + int(lm.lane_length(1) / 5.0)
    v_alone, lat_alone, _ = _through_the_curve(lm, None)
    v, lat, reasons = _through_the_curve(lm, _Taught(1, n, 7.0, 1.0))
    assert abs(lat_alone) < 0.7
    assert lat == pytest.approx(1.0, abs=0.45), lat           # your line (+ = right), not the lane's centre
    assert v < 7.0 * 1.05 + 0.5 and v_alone > v + 1.0         # your speed
    assert any("as you took it" in r for r in reasons)


def test_a_sharp_turn_you_have_shown_it_isnt_a_reason_to_hand_back():
    lm = _course(radius=4.0, curve_m=10.0)                    # tighter than it would take on a route
    ap, sim = _ap(lm, x=240.0, speed=8.0)
    fault = None
    for _ in range(int(10 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        fault = fault or st.fault
        if fault:
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert fault and fault.startswith("sharp turn ahead")
    n = 1 + int(lm.lane_length(1) / 5.0)
    ap, sim = _ap(lm, x=240.0, speed=8.0)
    ap.teach = _Taught(1, n, 6.0, 0.0)
    for _ in range(int(20 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        assert not (st.fault or "").startswith("sharp turn"), st.fault
        if st.fault:
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert sim.s.speed < 8.0


# ------------------------------------------------------------------ toll booths and style
def _plaza():
    a = arc_polyline([(60, 0.0)])
    b = arc_polyline([(60, 0.0)], x=0, y=8.0)
    c = arc_polyline([(60, 0.0)], x=0, y=16.0)
    lm = build([(a, KIND_PREFAB, 0xB1, 0), (b, KIND_PREFAB, 0xB1, 1), (c, KIND_PREFAB, 0xB1, 2)])
    rules = SimpleNamespace(sem_id=np.array([0, 1, 2]))
    return lm, rules


def test_paying_at_a_booth_yourself_teaches_the_booth_and_where_to_stop():
    lm, rules = _plaza()
    lr = Learner(None, lm)
    lr.attach(None, None, rules=rules, gate_s=lambda lane, item: 50.0, front_bumper=3.0)
    lr.on_toll_paid(_frame_at(lm, 1, 43.0, 0.0))              # front bumper at 46 m, the gate at 50 m
    key = lane_key(lm, 1)
    assert lr.toll_lanes[key]["n"] == 1 and lr.toll_lanes[key]["back_m"] == pytest.approx(4.0, abs=0.2)
    assert lr.toll_back(1) == pytest.approx(4.0, abs=0.2) and lr.toll_back(0) is None
    assert lr.paid_booths() == {(0xB1, 1)}
    assert lr.lane_penalties() == {0: 150.0, 2: 150.0}        # the other booths cost more


def test_the_gate_stop_goes_where_you_stopped():
    from autohaul.control.giveway import GiveWay, GiveWayResult
    gw = GiveWay.__new__(GiveWay)
    gw.p = SimpleNamespace(toll_back_m=2.0, toll_approach_kmh=12.0, toll_settle_s=0.8, toll_press_every_s=3.0, toll_presses=3,
                           toll_open_s=2.5, toll_pass_kmh=10.0)
    gw._toll, gw._paid = {}, False
    gw._gate_s = lambda lane, item: 50.0
    for back, expect in ((None, 48.0), (4.0, 46.0)):
        gw.toll_back = (lambda lane, b=back: b)
        gw._toll = {}
        blocked = []
        gw._toll_gate(1, 0xB1, 0.0, 0.0, 5.0, 1 / 60, GiveWayResult(), blocked)
        assert blocked[0][0] == pytest.approx(expect)


def test_your_curve_speed_and_acceleration_pull_its_values_your_way():
    from autohaul.control.speed import SpeedParams
    lm = _course(radius=40.0, curve_m=60.0)
    sp = SpeedParams()
    lr = Learner(None, lm)
    lr.attach(sp, None)
    before = sp.lat_acc_max
    t = _drive_lane(lr, lm, 1, 0.0, 10.0)                     # 2.5 m/s^2 through the curve
    _drive_lane(lr, lm, 2, t, 10.0, s1=30.0)
    assert sp.lat_acc_max > before
    accel_before = sp.accel_max
    t, v = 100.0, 5.0
    for _ in range(100):
        f = _frame_at(lm, 2, 50.0 + (t - 100.0) * 5.0, v, throttle=0.8)
        lr.observe_driver(t, f)
        t, v = t + 0.1, v + 0.1                               # 1 m/s^2
    assert sp.accel_max > accel_before



# ------------------------------------------------------------------ the ride of 2026-10-30
def _join(back=0.0, side=0.0, gap=0.0):
    """A straight road (0), then a curve of radius 40 m (1) starting `back` behind / `gap` ahead of its end, `side` aside."""
    a = arc_polyline([(300, 0.0)])
    curve = arc_polyline([(60, 1 / 40)], x=300 - back + gap, y=side)
    end, rot = curve[-1], math.atan2(curve[-1, 1] - curve[-2, 1], curve[-1, 0] - curve[-2, 0])
    b = arc_polyline([(300, 0.0)], x=end[0], y=end[1], rot=rot)
    return build([(a, KIND_PREFAB, 1, 0), (curve, KIND_PREFAB, 1, 1), (b, KIND_ROAD, 3, 1)])


@pytest.mark.parametrize("back,side", [(1.1, 0.3), (2.0, 0.0), (0.5, 0.5), (1.5, -0.8)])
def test_a_join_that_steps_back_isnt_a_hairpin(back, side):
    """2026-10-30: a path starting 1.1 m behind the last one's end and 0.3 m aside. The stitched path went forward, jumped
    back, then turned: curvature -0.21, +0.11, -0.08 on a curve of radius 12 m, read as "sharp turn ahead"."""
    from autohaul.control.path import PathRef
    lm = _join(back=back, side=side)
    ap, sim = _ap(lm, x=200.0, speed=12.0)
    cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
    assert not st.fault, st.fault
    pts = ap.tracker.path_ahead(180.0)
    d = np.diff(pts, axis=0)
    d = d[np.hypot(d[:, 0], d[:, 1]) > 0.01]                  # (the truck's own point starts it twice)
    u = d / np.hypot(d[:, 0], d[:, 1])[:, None]
    assert (np.einsum("ij,ij->i", u[:-1], u[1:]) > 0.5).all()    # never turns back on itself
    path = PathRef(pts)
    k = np.abs(np.asarray(path.curvature(np.arange(0.0, path.length, 2.0), 6.0)))
    assert k.max() < 1 / 12.0, 1 / k.max()                    # the radius 40 m curve doesn't read as a tight one


def test_a_real_gap_ahead_is_still_bridged():
    lm = _join(gap=2.0)
    ap, sim = _ap(lm, x=200.0, speed=12.0)
    ap.step(sim.frame(25.0), [], sim.p.dt)
    pts = ap.tracker.path_ahead(180.0)
    assert any(np.hypot(*(p - np.array([302.0, 0.0]))) < 0.01 for p in pts)
    assert ap.tracker.path_lanes[:2] == [0, 1]


def test_a_truck_drives_a_stepped_back_join_without_fuss():
    lm = _join(back=1.1, side=0.3)
    ap, sim = _ap(lm, x=200.0, speed=12.0)
    worst = 0.0
    for _ in range(int(22 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        assert not st.fault, st.fault
        worst = max(worst, abs(st.lateral))
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert worst < 1.0 and st.lane == 2


@pytest.mark.parametrize("radius", [6.0, 8.0, 12.0])
def test_it_drives_through_tight_turns_on_a_route(radius):
    lm = _course(radius=radius, curve_m=radius * 1.5)         # about 85 degrees
    ap, sim = _ap(lm, x=240.0, speed=10.0)
    worst = 0.0
    passed = False
    for _ in range(int(35 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        assert not st.fault, st.fault
        worst = max(worst, abs(st.lateral))
        passed |= st.lane == 2
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert passed and worst < 1.6


def test_a_jump_in_the_map_isnt_leaving_the_lane():
    """2026-10-30: -0.39 to -1.5 m within 0.2 s, at walking pace, where two junction paths don't line up."""
    lm = _course()
    ap, _ = _ap(lm)
    assert not ap._off_lane(1.7, 2.0, 0.1)                    # slow: it may be the map
    for _ in range(8):
        assert not ap._off_lane(1.7, 2.0, 0.1)
    assert ap._off_lane(1.7, 2.0, 0.2)                        # still off after a second: it's the truck
    ap._off_for = 0.0
    assert not ap._off_lane(1.7, 20.0, 0.1)
    assert ap._off_lane(1.7, 20.0, 0.1)                       # at speed: at once, nearly
    ap._off_for = 0.0
    assert ap._off_lane(3.6, 2.0, 0.01)                       # far off: at once
    ap._off_for = 0.0
    ap._off_lane(1.7, 2.0, 0.5)
    assert not ap._off_lane(0.2, 2.0, 0.5) and ap._off_for == 0.0


def test_the_straight_way_over_the_bridge_beats_the_ramp_down_and_up():
    """From an exit-only lane: changing lanes (80 m) to go straight over, or down the ramp, through the junction at the
    bottom and back up, hardly longer. It took the ramp, at about 20 km/h, for over a minute, twice."""
    left, right = arc_polyline([(300, 0.0)], x=0, y=0), arc_polyline([(300, 0.0)], x=0, y=4.5)
    over = arc_polyline([(120, 0.0)], x=300, y=0)
    ramp = np.column_stack([np.linspace(300, 420, 25), 4.5 + 6.0 * np.sin(np.linspace(0, math.pi, 25))])
    on_left = arc_polyline([(300, 0.0)], x=420, y=0)
    on_right = arc_polyline([(300, 0.0)], x=420, y=4.5)
    lm = build([(left, KIND_ROAD, 1, 1), (right, KIND_ROAD, 1, 2), (over, KIND_PREFAB, 7, 0), (ramp, KIND_PREFAB, 7, 1),
                (on_left, KIND_ROAD, 9, 1), (on_right, KIND_ROAD, 9, 2)])
    n = lm.lane_last[3] - lm.lane_first[3] + 1
    i = lm.lane_first[3]
    z = 9.0 * np.abs(np.linspace(-1, 1, n + 1))               # down 9 m, and up again
    lm.seg[i:i + n, 4], lm.seg[i:i + n, 5] = z[:-1], z[1:]
    g = LaneGraph.build(lm, log=lambda *_: None)
    target = tuple(lm.lane_points(5)[-1])
    flat = Router(lm, g, climb_m_per_m=0.0).route(1, 10.0, target, goal=(5, 290.0))
    assert 3 in flat.lanes                                    # what it did
    r = Router(lm, g)
    way = r.route(1, 10.0, target, goal=(5, 290.0))
    assert 3 not in way.lanes and 2 in way.lanes, way.lanes   # over the bridge
    assert sum(way.changes) >= 1
    alone = Router(lm, g).route(1, 10.0, target, goal=(5, 290.0), no_change_m=1e9)
    assert alone is not None and 3 in alone.lanes             # with no room to change, the ramp is still a way on


def _gw_for_spans(lm, sem_lane=None):
    from autohaul.control.giveway import GiveWay
    from autohaul.world.junctions import Junctions
    gw = GiveWay.__new__(GiveWay)
    gw.p = SimpleNamespace(conflict_before_m=30.0, conflict_after_m=10.0)
    gw.j = Junctions(lm)
    gw.g = LaneGraph.build(lm, log=lambda *_: None)
    sem = np.full(len(lm.lane_kind), -1)
    if sem_lane is not None:
        sem[sem_lane] = 0
    gw.rules = SimpleNamespace(sem_id=sem)
    gw._spans = {}
    return gw


def test_only_real_conflicts_slow_a_junction_path():
    """Straight over a bridge in a junction that has a crossroads somewhere: it crawled at 20 km/h for a third of a drive."""
    crossing_a = arc_polyline([(200, 0.0)], x=0, y=0)
    crossing_b = arc_polyline([(80, 0.0)], x=100, y=-40, rot=math.pi / 2)
    elsewhere = arc_polyline([(500, 0.0)], x=0, y=300)
    lm = build([(crossing_a, KIND_PREFAB, 7, 0), (crossing_b, KIND_PREFAB, 7, 1), (elsewhere, KIND_PREFAB, 7, 2)])
    gw = _gw_for_spans(lm)
    assert gw.j.is_intersection(7) is True
    assert gw._slow_spans(2) == []                            # a junction with a crossing, but not on this path
    spans = gw._slow_spans(0)
    assert len(spans) == 1 and spans[0] == pytest.approx((70.0, 110.0), abs=2.0)   # around the crossing, not all 200 m
    gw = _gw_for_spans(lm, sem_lane=2)
    assert gw._slow_spans(2) == [(0.0, pytest.approx(500.0, abs=2.0))]   # under a signal: all of it
