"""Controllers against the truck simulator: steering servo, lane keeping, speed, curves, ACC, faults."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autohaul.control.steering import RelativeServo
from autohaul.control.vehicle import TruckGeometry, bicycle_angle, wheel_angles
from autohaul.drive.autopilot import Autopilot
from autohaul.sim.course import HIGHWAY, Course, run
from autohaul.sim.truck import SimParams, SimVehicle, TruckSim

GEOM = TruckGeometry()
KMH = 1 / 3.6


@pytest.fixture(scope="module")
def highway() -> Course:
    return Course.from_pieces(HIGHWAY)


def sim_at(course: Course, speed_kmh: float, y0: float = 0.0, s0: float = 20.0, **params) -> TruckSim:
    x, y = course.path.point(s0)
    r = course.path.rotation_at(s0)
    return TruckSim(GEOM, SimParams(**params), x=x - y0 * math.sin(r), y=y + y0 * math.cos(r), rotation=r,
                    speed=speed_kmh * KMH)


def test_ackermann_roundtrip():
    for a in (-0.4, -0.05, 0.0, 0.02, 0.3):
        assert bicycle_angle(*wheel_angles(a, GEOM)) == pytest.approx(a, abs=1e-9)
    left, right = wheel_angles(0.3, GEOM)
    assert left > right > 0                       # turning left: the left wheel is the inner, steeper one


def test_relative_servo_tracks_and_learns_the_rate():
    servo = RelativeServo(gain=0.42)
    eff, true_gain, dt, delayed = 0.0, 0.30, 1 / 60, 0.0
    for i in range(240):
        target = 0.2 if i < 120 else -0.1
        eff = max(-1, min(1, eff - true_gain * delayed * dt))
        delayed = servo.step(target, eff, dt)
    assert eff == pytest.approx(-0.1, abs=0.01)
    assert servo.gain == pytest.approx(true_gain, rel=0.15)
    assert not servo.fighting


def test_pulse_modulator_never_steers_two_frames_running_and_keeps_the_average():
    from autohaul.control.steering import PulseModulator
    pm = PulseModulator()
    cmds = [0.3] * 50 + [-0.45] * 50 + [0.8] * 50 + [0.02] * 50
    out = [pm.step(c) for c in cmds]
    assert all(not (a and b) for a, b in zip(out, out[1:]))
    # nothing is lost: the delivered total matches the (0.5-capped) commands to within one frame
    wanted = sum(max(-0.5, min(0.5, c)) for c in cmds)
    assert sum(out) == pytest.approx(wanted, abs=0.5)
    assert sum(out[10:50]) == pytest.approx(0.3 * 40, abs=0.61)
    assert max(abs(o) for o in out) <= 1.0


@pytest.mark.parametrize("pulse", [False, True])
def test_full_mode_with_one_input_event_per_frame(highway, pulse):
    """The game takes one input change per frame (measured 2026-10-20): steering and pedals share it."""
    sim = sim_at(highway, 50, y0=0.3, events_per_frame=1)
    log = run(Autopilot(highway.lm, GEOM, "relative", pulse_steering=pulse), sim, highway, 60, speed_limit=80 * KMH)
    assert log.fault is None
    assert log.arr("speed").max() * 3.6 > 75                      # accelerated to the limit
    assert np.abs(log.arr("lateral"))[log.arr("t") > 8].max() < 0.4


def test_hands_back_when_the_game_ignores_the_pedals(highway):
    sim = sim_at(highway, 50, y0=0.3, ignore_pedals=True)
    log = run(Autopilot(highway.lm, GEOM, "relative"), sim, highway, 30, speed_limit=80 * KMH)
    assert log.fault and log.fault.startswith("the game isn't taking AutoHaul's throttle")
    assert log.t[-1] < 3.0


def test_servo_notices_someone_else_turning_the_wheel(highway):
    sim = sim_at(highway, 70)
    ap = Autopilot(highway.lm, GEOM, "relative")
    ap.engage(sim.frame(80 * KMH))
    faults = []
    for i in range(300):
        if i == 120:
            sim.extra_steer_rate = 0.4               # e.g. a joystick axis AutoHaul doesn't know about
        cmd, st = ap.step(sim.frame(80 * KMH), [], sim.p.dt)
        if st.fault:
            faults.append((i, st.fault))
            break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
    assert faults and 120 < faults[0][0] < 160 and "someone else" in faults[0][1]


@pytest.mark.parametrize("params", [
    dict(),
    dict(rate_gain=0.25),
    dict(rate_gain=0.8),
    dict(input_delay_frames=3),
    dict(understeer_v=26.0),
    dict(mass=40000.0, grade=0.03),
], ids=["nominal", "slow-wheel", "fast-wheel", "lag", "understeer", "loaded-uphill"])
def test_lane_keeping_on_highway_course(highway, params):
    sim = sim_at(highway, 78, y0=0.8, **params)
    log = run(Autopilot(highway.lm, GEOM, "relative"), sim, highway, 150, speed_limit=80 * KMH)
    assert log.fault is None
    t, lat = log.arr("t"), np.abs(log.arr("lateral"))
    assert lat[t > 8].max() < 0.4
    assert np.percentile(lat[t > 8], 95) < 0.3
    assert log.s_end > 2500


def test_game_side_deadzone_is_compensated_when_known(highway):
    sim = sim_at(highway, 78, y0=0.5, deadzone=0.16)
    log = run(Autopilot(highway.lm, GEOM, "relative", relative_deadzone=0.16), sim, highway, 120, speed_limit=80 * KMH)
    t, lat = log.arr("t"), np.abs(log.arr("lateral"))
    assert log.fault is None and lat[t > 8].max() < 0.4


def test_absolute_mode(highway):
    sim = sim_at(highway, 78, y0=0.5, steering_mode="absolute")
    log = run(Autopilot(highway.lm, GEOM, "absolute"), sim, highway, 120, speed_limit=80 * KMH)
    t, lat = log.arr("t"), np.abs(log.arr("lateral"))
    assert log.fault is None and lat[t > 8].max() < 0.4


@pytest.mark.parametrize("mass", [9000.0, 40000.0])
def test_speed_follows_the_limit_without_overshoot(mass):
    course = Course.from_pieces([(3000, 0.0)])
    sim = sim_at(course, 40, mass=mass)
    limit = [60 * KMH]

    def every(t, sim, st):
        limit[0] = 60 * KMH if t < 60 else 40 * KMH

    ap = Autopilot(course.lm, GEOM, "relative")
    speeds, t_all = [], []
    frame = sim.frame(limit[0])
    assert ap.engage(frame) is None
    for _ in range(int(110 / sim.p.dt)):
        frame = sim.frame(limit[0])
        cmd, st = ap.step(frame, [], sim.p.dt)
        every(sim.t, sim, st)
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
        speeds.append(sim.s.speed / KMH)
        t_all.append(sim.t)
    v, t = np.array(speeds), np.array(t_all)
    assert v[t < 60].max() <= 59.5                                   # never above the posted 60
    assert abs(v[(t > 45) & (t < 60)].mean() - 58) < 1.0             # settles at limit - 2 km/h
    assert v[t > 95].max() <= 39.5 and abs(v[t > 95].mean() - 38) < 1.5


def test_slows_down_before_a_tight_curve():
    course = Course.from_pieces([(400, 0.0), (180, 1 / 120), (300, 0.0)])
    sim = sim_at(course, 75)
    curve_speeds = []

    def every(t, s, st):
        pr = course.path.project(s.s.x, s.s.y)
        if 400 < pr.s < 580:
            curve_speeds.append(s.s.speed / KMH)

    log = run(Autopilot(course.lm, GEOM, "relative"), sim, course, 90, speed_limit=80 * KMH, every=every)
    assert log.fault is None
    v_ok = math.sqrt(1.4 * 120) / KMH            # lat_acc_max 1.4 m/s^2 -> 46.7 km/h
    assert max(curve_speeds) < v_ok + 2.0
    assert np.abs(log.arr("lateral")).max() < 0.5


def test_acc_follows_then_stops_behind_a_braking_lead():
    course = Course.from_pieces([(4000, 0.0)], two_lanes=False)
    sim = sim_at(course, 70)
    lead = SimVehicle(1, course.path, s=20 + 80, speed=60 * KMH, length=16.0, width=2.5,
                      profile=[(0.0, 60 * KMH), (60.0, 0.0)])
    log = run(Autopilot(course.lm, GEOM, "relative"), sim, course, 90, speed_limit=80 * KMH, vehicles=[lead])
    assert log.fault is None
    t, gap, v = log.arr("t"), log.arr("gap"), log.arr("speed")
    following = (t > 40) & (t < 60)
    desired = 10 + 2.2 * 60 * KMH
    assert abs(gap[following].mean() - desired) < 5.0
    assert gap.min() > 3.0                       # never closer than 3 m, including the hard stop
    assert v[-1] < 0.3                           # stopped behind it
    assert log.arr("brake").max() > 0.3


def test_hands_back_well_before_a_junction_sharp_turn():
    course = Course.from_pieces([(500, 0.0), (40, 1 / 20), (200, 0.0)], two_lanes=False)
    sim = sim_at(course, 70)
    log = run(Autopilot(course.lm, GEOM, "relative"), sim, course, 60, speed_limit=80 * KMH)
    assert log.fault and log.fault.startswith("sharp turn ahead")
    warning_s = (500 - log.s_end) / max(log.arr("speed")[-1], 1.0)
    assert warning_s > 4.0                         # seconds of warning at the speed it handed back


def test_hands_back_before_a_toll_plaza_and_refuses_to_engage_there():
    course = Course.from_pieces([(1200, 0.0)], two_lanes=False)
    toll = np.array([[650.0, 25.0]])                    # plaza icon beside the road, 25 m off the lane
    sim = sim_at(course, 70)
    log = run(Autopilot(course.lm, GEOM, "relative", hazards=toll), sim, course, 60, speed_limit=80 * KMH)
    assert log.fault and log.fault.startswith("toll plaza ahead")
    assert 650 - log.s_end > 120                       # handed back well before it
    near = sim_at(course, 30, s0=600)
    assert "toll plaza" in Autopilot(course.lm, GEOM, "relative", hazards=toll).engage(near.frame())
    for mode in (dict(lateral=False), dict(longitudinal=False)):   # speed-only and steer-only too
        sim = sim_at(course, 70)
        log = run(Autopilot(course.lm, GEOM, "relative", hazards=toll, **mode), sim, course, 60, speed_limit=80 * KMH)
        assert log.fault and log.fault.startswith("toll plaza ahead")


def test_speed_only_mode_ignores_the_drivers_wheel_angle():
    """Regression (2026-10-19 drive): in speed-only mode the driver's normal steering corrections made
    the steering-rate check demand ~11 km/h on an 800 m curve."""
    course = Course.from_pieces([(200, 0.0), (400, 1 / 800)], two_lanes=False)
    sim = sim_at(course, 50)
    ap = Autopilot(course.lm, GEOM, "relative", lateral=False)
    assert ap.engage(sim.frame(80 * KMH)) is None
    sim.s.eff = 0.09                                    # driver holding ~3.4 deg more wheel than the curve needs
    _, st = ap.step(sim.frame(80 * KMH), [], sim.p.dt)
    assert st.target_kmh > 70


def test_steering_rate_limits_curve_speed():
    """A quick left-right S-bend: how fast the wheel can swing, not lateral grip, sets the speed."""
    from autohaul.control.path import PathRef
    from autohaul.control.speed import SpeedParams, SpeedPlanner
    from autohaul.sim.course import arc_polyline
    path = PathRef(arc_polyline([(100, 0.0), (40, -1 / 30), (40, 1 / 30), (100, 0.0)]))
    planner = SpeedPlanner(SpeedParams(lat_acc_max=3.0))
    in_first_arc = math.atan(-GEOM.wheelbase / 30)          # wheel angle now (map convention: + = right)
    v_grip, *_ = planner.ahead(path, 135.0, 9.0, steer_now=in_first_arc)
    v_rate, *_ = planner.ahead(path, 135.0, 9.0, steer_rate=0.42 * GEOM.rad_per_unit, wheelbase=GEOM.wheelbase,
                               steer_now=in_first_arc)
    assert v_rate < 0.9 * v_grip


def test_stops_before_the_end_of_the_mapped_lane():
    course = Course.from_pieces([(400, 0.0)], two_lanes=False)
    sim = sim_at(course, 60)
    log = run(Autopilot(course.lm, GEOM, "relative"), sim, course, 60, speed_limit=80 * KMH)
    assert log.arr("speed")[-1] < 0.3
    assert course.path.length - 25 < log.s_end + GEOM.front_bumper < course.path.length - 5


def test_engage_refusals(highway):
    ap = Autopilot(highway.lm, GEOM, "relative")
    frame = sim_at(highway, 50, y0=1.5).frame()
    assert "off the lane centre" in ap.engage(frame)
    frame = sim_at(highway, 50).frame()
    frame.parking_brake = 1
    assert ap.engage(frame) == "parking brake is on"
    frame = TruckSim(GEOM, x=0, y=500).frame()
    assert "not on a mapped lane" in ap.engage(frame)
    assert Autopilot(highway.lm, GEOM, "relative", lateral=False).engage(sim_at(highway, 50, y0=1.5).frame()) is None


def test_speed_only_mode_faults_when_the_truck_leaves_the_lane():
    course = Course.from_pieces([(200, 0.0), (400, 1 / 300)], two_lanes=False)
    sim = sim_at(course, 60)
    log = run(Autopilot(course.lm, GEOM, "relative", lateral=False), sim, course, 60, speed_limit=60 * KMH)
    assert log.fault and "lost the lane" in log.fault        # nobody steered: it went straight off the curve
