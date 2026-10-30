"""Auto-learning (autohaul/learn), auto wipers and the overlay feed."""

from __future__ import annotations

import json
import socket
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from autohaul.control.speed import SpeedParams
from autohaul.control.steering import LateralController
from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.drive.wipers import WeatherWatch, WiperControl, WiperParams, rain_now
from autohaul.infra.saves import Weather, parse_weather
from autohaul.infra.overlay_feed import OverlayFeed, explain, why_speed
from autohaul.learn.learner import BOUNDS, Learner, LearnParams, lane_key
from autohaul.mission.manager import MissionStatus
from autohaul.sim.course import arc_polyline
from autohaul.sim.truck import TruckSim
from autohaul.world.graph import LaneGraph
from autohaul.world.lanemap import KIND_ROAD
from autohaul.world.route import Router
from tests.test_route import build

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


def _frame(speed=20.0, nav=100000.0, scale=19.0):
    return SimpleNamespace(speed=speed, navigation_distance=nav, local_scale=scale)


def _lm():
    a = arc_polyline([(300, 0.0)])
    b = arc_polyline([(300, 0.0)], x=300, y=0)
    return build([(a, KIND_ROAD, 0xA1, 1), (b, KIND_ROAD, 0xA2, 1)])


def _curve(learner, t, peak, kappa=0.02, v=10.0, n=30, target_kmh=36.0, lat_entry=0.0):
    """One curve: `n` notes in it with the error rising to `peak`, then straight road."""
    for i in range(n):
        note = {"kappa_ff": kappa, "ctrl_lateral": peak * (i + 1) / n if i >= 10 else lat_entry, "target_kmh": target_kmh,
                "lateral": 0.1, "lane": 0, "throttle": 0.0, "brake": 0.0, "curve_ratio": 1.0}
        learner.observe(t, _frame(v), note, True, False)
        t += 0.1
    for _ in range(15):
        learner.observe(t, _frame(v), {"kappa_ff": 0.0, "ctrl_lateral": 0.0, "target_kmh": 60.0, "lane": 0}, True, False)
        t += 0.1
    return t


def _attached(tmp_path=None, **kw):
    sp = SpeedParams()
    lat = LateralController(TruckGeometry())
    lr = Learner(tmp_path / "learned.json" if tmp_path else None, _lm(), LearnParams(**kw))
    lr.attach(sp, lat)
    return lr, sp, lat


def test_running_wide_in_curves_slows_them_and_tracking_well_speeds_them_up(tmp_path):
    lr, sp, _ = _attached(tmp_path)
    t = _curve(lr, 0.0, peak=1.5)
    assert sp.lat_acc_max == pytest.approx(2.0 - lr.p.lat_step_down)
    for _ in range(200):
        t = _curve(lr, t, peak=1.5)
    assert sp.lat_acc_max == BOUNDS["lat_acc_max"][0]         # never below the bound
    log = (tmp_path / "learning.jsonl").read_text(encoding="utf-8")
    assert "ran 1.5 m wide" in log                            # every change says why
    lr2, sp2, _ = _attached()
    before = sp2.lat_acc_max
    _curve(lr2, 0.0, peak=0.2)                                # at 2.0 m/s^2: the limit set the speed, and it tracked
    assert sp2.lat_acc_max > before
    lr3, sp3, _ = _attached()
    _curve(lr3, 0.0, peak=0.2, v=5.5)                         # a slow curve says nothing about the limit
    assert sp3.lat_acc_max == before


def test_taking_over_in_a_curve_slows_curves():
    lr, sp, _ = _attached()
    t = _curve(lr, 0.0, peak=0.5)
    before = sp.lat_acc_max
    lr.on_handback("driving key LEFT", 10.0, 0)
    assert sp.lat_acc_max == pytest.approx(before - lr.p.takeover_step)
    lr._prev_t = t + 60.0
    before = sp.lat_acc_max
    lr.on_handback("driving key LEFT", 10.0, 0)               # a minute after the curve: nothing to do with it
    assert sp.lat_acc_max == before


def _signed_curve(lr, t, error, kappa=0.02):
    """A curve with a constant error (+ = right of the line)."""
    for _ in range(30):
        lr.observe(t, _frame(10.0), {"kappa_ff": kappa, "ctrl_lateral": error, "target_kmh": 36.0, "lane": 0}, True, False)
        t += 0.1
    for _ in range(15):
        lr.observe(t, _frame(10.0), {"kappa_ff": 0.0, "ctrl_lateral": 0.0, "target_kmh": 60.0, "lane": 0}, True, False)
        t += 0.1
    return t


def test_turn_in_moves_against_the_error_in_curves():
    lr, _, lat = _attached()
    start = lat.p.predict_s
    _signed_curve(lr, 0.0, -0.4, kappa=0.02)                  # right curve, left of the line: outside
    assert lat.p.predict_s > start
    lr, _, lat = _attached()
    _signed_curve(lr, 0.0, -0.4, kappa=-0.02)                 # left curve, left of the line: inside
    assert lat.p.predict_s < start
    lr, _, lat = _attached()
    _signed_curve(lr, 0.0, 0.05, kappa=0.02)                  # fine: leave it
    assert lat.p.predict_s == start


def test_loose_curves_firm_up_the_correction_and_weaving_softens_it():
    lr, _, lat = _attached()
    start = lat.p.omega_max
    t = _signed_curve(lr, 0.0, 0.5)
    assert lat.p.omega_max > start
    firm = lat.p.omega_max
    for i in range(300):                                      # weaving on a straight: +-0.3 m every 2 s
        t += 0.1
        lr.observe(t, _frame(20.0), {"kappa_ff": 0.0, "lateral": 0.3 if i % 20 < 10 else -0.3, "lane": 0}, True, False)
    assert lat.p.omega_max < firm
    soft = lat.p.omega_max
    _signed_curve(lr, t, 0.5)                                 # just after weaving: no firming up
    assert lat.p.omega_max == soft
    for i in range(300):                                      # a steady offset on a straight isn't weaving
        t += 0.1
        lr.observe(t, _frame(20.0), {"kappa_ff": 0.0, "lateral": 0.3, "lane": 0}, True, False)
    assert lat.p.omega_max == soft


def test_learning_really_improves_tracking_in_the_simulator():
    """From poor values, on the real controller and truck model: the same course, tracked better afterwards."""
    from autohaul.sim.truck import SimParams
    segs = []
    for _ in range(10):
        segs += [(150, 0.0), (120, 1 / 90), (150, 0.0), (120, -1 / 90)]
    pts = arc_polyline(segs)
    lm = build([(pts, KIND_ROAD, 1, 1)])
    g = LaneGraph.build(lm, log=lambda *_: None)
    geom = TruckGeometry()
    # (much bigger steps than on the road: the same rules, in a few simulated minutes)
    lr = Learner(None, lm, LearnParams(predict_step=0.02, omega_step_up=0.02))

    def drive(learn: bool, seconds: float) -> float:
        rot = float(np.arctan2(pts[1, 1] - pts[0, 1], pts[1, 0] - pts[0, 0]))
        sim = TruckSim(geom, SimParams(input_delay_frames=3), x=pts[3, 0], y=pts[3, 1], rotation=rot, speed=10.0)
        ap = Autopilot(lm, geom, "relative", router=Router(lm, g))
        ap.lat.p.predict_s, ap.lat.p.omega_max = 0.42, 0.5     # the poor start
        lr.attach(ap.sp, ap.lat)
        if not learn:
            lr._write_sinks()
        ap.set_destination(*pts[-1])
        assert ap.engage(sim.frame(80 / 3.6)) is None
        sq, n, t_note = 0.0, 0, 0.0
        for _ in range(int(seconds / sim.p.dt)):
            f = sim.frame(80 / 3.6)
            cmd, st = ap.step(f, [], sim.p.dt)
            assert not st.fault, st.fault
            sim.step(cmd.steer, cmd.throttle, cmd.brake)
            if sim.t - t_note >= 0.1:
                t_note = sim.t
                if learn:
                    lr.observe(sim.t, f, {k: getattr(st, k) for k in ("kappa_ff", "ctrl_lateral", "lateral", "target_kmh", "lane", "throttle",
                                                                     "brake", "curve_ratio", "lane_change", "junction", "on_route", "route_km")},
                               True, False)
                if abs(st.kappa_ff) > 0.006:
                    sq += st.ctrl_lateral ** 2
                    n += 1
        return float(np.sqrt(sq / max(n, 1)))

    before = drive(False, 60.0)
    for _ in range(3):
        drive(True, 150.0)
    after = drive(False, 60.0)
    assert after < 0.75 * before, (before, after, lr.values)


def test_arriving_too_fast_brakes_earlier():
    lr, sp, _ = _attached()
    start = sp.plan_decel
    _curve(lr, 0.0, peak=0.3, target_kmh=20.0)                # 36 km/h into a 20 km/h curve
    assert sp.plan_decel < start


def test_pedal_values_are_learned_per_truck_state():
    lr, sp, _ = _attached()
    t, v = 0.0, 5.0
    for _ in range(3000):
        lr.observe(t, _frame(v), {"throttle": 0.5, "brake": 0.0, "lane": 0}, True, True)   # trailer on
        t += 0.1
        v += 0.01                                             # 0.1 m/s^2 at half throttle
        if v > 20:
            v = 5.0
    assert sp.thr_accel < 1.0                                 # (0.1 + 0.25) / 0.5 = 0.7
    assert lr.values["trailer"]["thr_accel"] == pytest.approx(0.7, abs=0.05)
    assert "thr_accel" not in lr.values["tractor"]
    lr.set_context(False)
    assert sp.thr_accel == 1.0                                # the bare tractor has its own


def test_trouble_spots_slow_down_then_wear_off():
    lr, _, _ = _attached()
    assert lr.caution(0) is None
    lr.on_handback("driving key A", 12.0, 0)
    assert lr.caution(0) is None                              # once could be anything
    lr.on_handback("driving key W", 10.0, 0)
    assert lr.caution(0) == pytest.approx(0.85 * 10.0)
    lr.on_handback("driving key W", 30.0, 1)                  # a takeover at motorway speed isn't about a place
    lr.on_handback("driving key W", 30.0, 1)
    assert lr.caution(1) is None
    for lane in (1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0):         # clean passes
        lr.observe(0.0, _frame(), {"lane": lane}, True, False)
    assert lr.caution(0) is None


def test_autopilot_slows_for_a_trouble_spot_ahead():
    lm = _lm()
    g = LaneGraph.build(lm, log=lambda *_: None)
    geom = TruckGeometry()
    sim = TruckSim(geom, x=20.0, y=0.0, rotation=0.0, speed=20.0)
    ap = Autopilot(lm, geom, "relative", router=Router(lm, g))
    ap.set_destination(*lm.lane_points(1)[-1])
    ap.caution = lambda lane: 8.0 if lane == 1 else None
    assert ap.engage(sim.frame(25.0)) is None
    reasons, entry = [], None
    for _ in range(int(25 / sim.p.dt)):
        cmd, st = ap.step(sim.frame(25.0), [], sim.p.dt)
        reasons.append(st.target_reason)
        if st.lane == 1 and entry is None:
            entry = sim.s.speed
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert any("take over near here" in r for r in reasons)
    assert entry is not None and entry < 11.0


def test_the_gps_teaches_the_route_only_when_autohauls_own_distance_didnt_jump():
    lr, _, _ = _attached()
    note = {"on_route": True, "lane": 0, "route_km": 10.0}
    lr.observe(0.0, _frame(nav=100000.0), note, True, False)
    lr.observe(0.1, _frame(nav=100000.0), note, True, False)
    lr.observe(0.2, _frame(nav=110000.0), note, True, False)  # the GPS wants another road; ours says the same
    key = lane_key(lr.lm, 0)
    assert lr.route_cost[key] == pytest.approx(0.5 * 10000 / 19.0)
    assert lr.s.gps_jumps == 1
    lr2, _, _ = _attached()
    lr2.observe(0.0, _frame(nav=100000.0), note, True, False)
    lr2.observe(0.1, _frame(nav=100000.0), note, True, False)
    lr2.observe(0.2, _frame(nav=110000.0), dict(note, route_km=10.6), True, False)   # we missed a turn
    assert not lr2.route_cost and lr2.s.misses == 1
    lr3, _, _ = _attached()
    lr3.observe(0.0, _frame(nav=100000.0), note, True, False)
    lr3.observe(0.1, _frame(nav=100000.0), note, True, False)
    lr3.observe(0.2, _frame(nav=600000.0), note, True, False)  # a new job
    assert not lr3.route_cost


def test_learned_route_costs_reach_the_router(tmp_path):
    lm = _lm()
    g = LaneGraph.build(lm, log=lambda *_: None)
    lr = Learner(tmp_path / "learned.json", lm)
    lr.route_cost[lane_key(lm, 1)] = 500.0
    r = Router(lm, g)
    before = r._cost[1]
    lr.attach(None, None, r)
    assert r._cost[1] == pytest.approx(before + 500.0)


def test_what_is_learned_survives_a_restart(tmp_path):
    lr, sp, _ = _attached(tmp_path)
    t = 0.0
    for _ in range(5):
        t = _curve(lr, t, peak=1.5)
    lr.on_handback("driving key A", 9.0, 0)
    lr.save()
    lr2, sp2, _ = _attached(tmp_path)
    assert sp2.lat_acc_max == pytest.approx(sp.lat_acc_max)
    assert lr2.spots == lr.spots


def test_a_scorecard_per_drive_and_rollback_when_drives_get_worse():
    lr, sp, _ = _attached(min_session_km=1.0)
    good = {"game_km": 100.0, "takeovers_100km": 1.0, "faults_100km": 0.0, "fines": {}}
    bad = {"game_km": 100.0, "takeovers_100km": 9.0, "faults_100km": 3.0, "fines": {}}
    lr.scorecards = [dict(good) for _ in range(3)]
    lr.snapshots = [{"t": 0, "values": {"tractor": {"lat_acc_max": 2.0}, "trailer": {}}} for _ in range(3)]
    for _ in range(3):
        lr.values["tractor"]["lat_acc_max"] = 2.4             # drifted up ...
        sp.lat_acc_max = 2.4
        lr.s.engaged_game_km, lr.s.takeovers, lr.s.faults = 100.0, 9, 3   # ... and the drives got worse
        card = lr.end_session()
        assert card["takeovers_100km"] == 9.0
    assert sp.lat_acc_max == 2.0


# ------------------------------------------------------------------ wipers
def _w(current, wetness, saved_at):
    return Weather(current, wetness, saved_at)


def test_rain_is_read_from_how_wet_the_saves_say_it_is():
    """The saves of 2026-10-30, as a shower began."""
    assert rain_now(None, _w(0, 0.0, 1)) is False
    assert rain_now(_w(0, 0.0, 1), _w(0, 0.0, 2)) is False
    assert rain_now(_w(0, 0.0, 2), _w(0, 0.051, 3)) is True   # it started before the weather field says so
    assert rain_now(_w(0, 0.051, 3), _w(1, 0.522, 4)) is True
    assert rain_now(_w(1, 1.0, 5), _w(1, 1.0, 6)) is True     # soaked, and still raining
    assert rain_now(_w(1, 1.0, 6), _w(0, 0.7, 7)) is False    # drying
    assert rain_now(None, _w(1, 0.0, 1)) is False             # (after a job: says rain, nothing's wet)
    assert rain_now(None, _w(1, 0.4, 1)) is True


def test_a_save_field_parses():
    text = "\n stored_weather_change_timer: &40eaac4a\n stored_current_weather: 1\n stored_rain_wetness: &3f05acef\n"
    w = parse_weather(text, 5.0)
    assert w.current == 1 and w.wetness == pytest.approx(0.522, abs=0.001) and w.saved_at == 5.0
    assert parse_weather("nothing here") is None


def test_weather_watch_uses_each_new_save_once_and_forgets_old_news():
    now = [1000.0]
    saves_seen = [_w(0, 0.0, 900.0)]
    watch = WeatherWatch(lambda: saves_seen[-1], clock=lambda: now[0])
    assert watch.raining is None
    watch.poll()
    assert watch.raining is False and "dry" in watch.describe()
    saves_seen.append(_w(0, 0.05, 1200.0))
    now[0] = 1210.0
    watch.poll()
    assert watch.raining is True and "raining" in watch.describe()
    watch.poll()                                              # the same save again: nothing new
    assert watch.raining is True
    now[0] = 1900.0
    assert watch.raining is None                              # too old to say


def test_weather_watch_survives_a_save_being_written():
    def broken():
        raise ValueError("half a file")

    watch = WeatherWatch(broken)
    watch.poll()
    assert watch.raining is None


def test_wipers_follow_the_rain_and_respect_the_driver():
    pressed = []
    w = WiperControl(pressed.append, WiperParams(driver_hold_s=30.0))
    w.step(True, None, 0.0, True)
    assert pressed == []                                      # not known: leave them
    w.step(True, False, 10.0, False)
    assert pressed == []                                      # not driving: they're yours
    w.step(True, False, 20.0, True)                           # left on, and it's dry: the complaint of 2026-10-30
    assert pressed == ["wipersback"]
    w.step(True, False, 20.5, True)
    assert pressed == ["wipersback"]                          # one at a time
    w.step(True, False, 22.0, True)                           # from the top stage it takes a few
    assert pressed == ["wipersback", "wipersback"]
    w.step(False, False, 23.0, True)
    assert len(pressed) == 2                                  # off
    w.step(False, True, 30.0, True)                           # it rains
    assert pressed[-1] == "wipers"
    w.step(True, True, 31.0, True)
    assert len(pressed) == 3
    w.step(False, True, 40.0, True)                           # you switched them off, in the rain
    w.step(False, True, 41.0, True)
    assert len(pressed) == 3                                  # left alone ...
    w.step(False, True, 71.0, True)
    assert pressed[-1] == "wipers" and len(pressed) == 4      # ... for a while


def test_wipers_give_up_if_the_button_does_nothing():
    pressed = []
    w = WiperControl(pressed.append)
    for i in range(30):
        w.step(True, False, float(i * 2), True)
    assert len(pressed) == 5


# ------------------------------------------------------------------ overlay
def test_overlay_explains_reads_decisions_and_reasons():
    from autohaul.drive.autopilot import Status
    f = SimpleNamespace(speed=20.0, navigation_speed_limit=25.0, displayed_gear=8, engine_enabled=1, parking_brake=0, wipers=0,
                        fuel=180.0, fuel_range=360.0, navigation_distance=120000.0)
    st = Status(lane_desc="road 1 sub 1", confidence=0.97, lateral=0.2, target_kmh=45.0, target_reason="curve in 60 m",
                route_km=6.3, on_route=True, lane_change="left: signalling (needed within 400 m)", indicator="left",
                steer_target=-0.2)
    ms = MissionStatus(state="to destination", job="bars: a -> b", next_stop="fuel at Lyon", decision="fuel runs out before b")
    d = explain(f, st, ms, "ENGAGED", True, 900.0, {"rain": "dry", "learned": ["Curves: 2.0"]})
    reads = dict(map(tuple, d["reads"]))
    assert reads["Speed"].startswith("72 km/h") and "limit 90" in reads["Speed"]
    assert "20 %" in reads["Fuel"] and "GPS 120 km" in reads["Distance"] and reads["Rain"] == "dry"
    doing = dict(map(tuple, d["doing"]))
    assert doing["Target"].startswith("45 km/h") and doing["Steering"].startswith("right") and doing["Next stop"] == "fuel at Lyon"
    assert any("curve 60 m ahead" in w for w in d["why"])
    assert any("needs this lane within 400 m" in w for w in d["why"])
    assert any("fuel runs out" in w for w in d["why"])
    assert d["learned"] == ["Curves: 2.0"]
    json.dumps(d)
    # handed back: what it reads, no decisions from a stale status
    d = explain(f, None, None, "handed back: driving key A", False)
    assert d["reads"] and not d["doing"] and not d["why"]


def test_why_speed_wording():
    assert "cruising" in why_speed("max speed", 80.0)
    assert "keeping distance" in why_speed("following at 34 m", 50.0).lower()
    assert why_speed("", 50.0) == ""


def test_overlay_feed_reaches_a_listener_and_never_blocks():
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    listener.settimeout(2.0)
    feed = OverlayFeed(listener.getsockname()[1])
    feed.event("engaged")
    feed.send({"mode": "ENGAGED"})
    data = json.loads(listener.recv(65535).decode("utf-8"))
    assert data["mode"] == "ENGAGED" and data["events"][0].endswith("engaged")
    listener.close()
    OverlayFeed(1).send({"mode": "nobody listening"})          # no overlay running: nothing happens
    assert feed.due(1000.0) and not feed.due(1000.05)


def test_overlay_height_grows_at_once_and_shrinks_only_after_a_while():
    import overlay
    o = SimpleNamespace(_h=0, _shrink_since=None, shrink_after_s=3.0)
    settle = lambda need, now: overlay.Overlay.settled_height(o, need, now)   # noqa: E731
    assert settle(400, 0.0) == 400
    assert settle(500, 0.2) == 500                            # a new line: room at once
    for t in (0.4, 0.6, 1.0, 2.0, 3.0):
        assert settle(400, t) == 500                          # the line's gone: keep the size for now
    assert settle(500, 3.1) == 500                            # it came back: nothing moved
    assert settle(400, 3.2) == 500
    assert settle(400, 6.3) == 400                            # gone for 3 s: shrink


def test_launcher_passes_the_learning_switch():
    import launcher
    assert "--no-learning" in launcher.drive_args("mission", "", 80, True, False)
    assert "--no-learning" not in launcher.drive_args("mission", "", 80, True, True)
