"""Phase 5: coupling the job trailer (drive in line with it, reverse under it, attach)."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from autohaul.control.steering import RelativeServo
from autohaul.control.vehicle import TruckGeometry
from autohaul.drive.autopilot import Autopilot
from autohaul.drive.coupling import Coupling, CouplingParams, dubins_words, sample_dubins
from autohaul.drive.keeper import Keeper
from autohaul.infra.saves import TrailerPose, kingpin, parse_trailer
from autohaul.mission.actions import ActionParams
from autohaul.mission.decisions import DecisionLog
from autohaul.mission.job import JobHistory
from autohaul.mission.manager import Mission, MissionDriver, MissionParams
from autohaul.sim.game import SimGame
from autohaul.sim.truck import TruckSim
from autohaul.world.places import Place
from tests.test_mission import make_station_world


def test_dubins_paths_end_at_the_goal():
    rng = __import__("random").Random(5)
    for _ in range(200):
        q0 = (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-math.pi, math.pi))
        q1 = (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-math.pi, math.pi))
        ends = []
        for word, segs in dubins_words(q0, q1, 8.0):
            pts = sample_dubins(q0, word, segs, 8.0)
            ends.append(math.hypot(pts[-1, 0] - q1[0], pts[-1, 1] - q1[1]))
        assert ends and min(ends) < 0.3


def test_trailer_from_save_text():
    text = ("\nplayer : _nameless.1 {\n trailer_placement: (-32047.23, 34.5, 4874.42) (&3f0d6d3a; 0, &3f54d7a2, 0)\n"
            " assigned_trailer_connected: false\n}\n")
    t = parse_trailer(text, 1.0)
    assert t is not None and not t.connected and (t.x, t.y) == (-32047.23, 4874.42)
    kx, ky = kingpin(t, 5.0)
    assert math.hypot(kx - t.x, ky - t.y) == pytest.approx(5.0)


def _run(trailer: TrailerPose, x, y, rot_deg, game_kw=None, params=None, seconds=240):
    geom = TruckGeometry()
    sim = TruckSim(geom, x=x, y=y, rotation=math.radians(rot_deg))
    game = SimGame(sim, job=None, fuel_l=500, trailer=False, trailer_pose=trailer, **(game_kw or {}))
    c = Coupling(geom, trailer, 1.57, params)
    servo = RelativeServo()
    pressed = []
    for _ in range(int(seconds / sim.p.dt)):
        f = game.frame()
        st = c.step(f, sim.p.dt)
        pressed += st.press
        if st.failed or st.done:
            return st, game, pressed
        game.buttons(press=st.press)
        game.step(servo.step(st.steer, f.effective_steering, sim.p.dt), st.throttle, st.brake)
    return st, game, pressed


@pytest.mark.parametrize("start", [
    (-32062.0, 4877.0, 139.0),          # the Paris yard, 2026-10-29: the trailer behind, off to the side
    (40.0, 10.0, 180.0),                # facing the trailer's front: a turn round
    (30.0, 20.0, 0.0),                  # beside it, facing the same way
    (-25.0, -18.0, 90.0),               # behind it, across
])
def test_couples_from_around_the_trailer(start):
    trailer = TrailerPose(-32047.23, 4874.42, math.radians(-125.0), False, 0.0) if start[0] < -1000 \
        else TrailerPose(0.0, 0.0, 0.0, False, 0.0)
    st, game, pressed = _run(trailer, *start)
    assert st.done and not st.failed, st.failed
    d, dh = game.hook_error()
    assert game.trailer and d < 1.2 and abs(dh) < 3
    assert game.sim.s.gear == 1                         # back in drive
    assert pressed.count("gearreverse") == pressed.count("geardrive") >= 1 and "attach" in pressed


def test_never_drives_through_the_trailer():
    trailer = TrailerPose(0.0, 0.0, 0.0, False, 0.0)
    c = Coupling(TruckGeometry(), trailer, 1.57)
    for x, y, r in ((40.0, 10.0, 180.0), (-25.0, -18.0, 90.0), (10.0, -12.0, 180.0)):
        frame = TruckSim(TruckGeometry(), x=x, y=y, rotation=math.radians(r)).frame()
        c.phase, c.path = "plan", None
        c.step(frame, 1 / 60)
        assert c.path is not None and not c._hits_trailer(c._body(c.path[: len(c.path) - 24]))


def test_hands_back_when_attach_does_nothing():
    st, game, pressed = _run(TrailerPose(0.0, 0.0, 0.0, False, 0.0), 30.0, 0.5, 0.0, game_kw={"couple_m": 0.0})
    assert st.failed and "past the kingpin" in st.failed and not game.trailer
    assert pressed.count("attach") >= 2


def test_hands_back_when_the_gearbox_stays_in_drive():
    class NoReverse(SimGame):
        def buttons(self, press=(), hold=None):
            super().buttons([b for b in press if b != "gearreverse"], hold)

    geom = TruckGeometry()
    sim = TruckSim(geom, x=30.0, y=0.0, rotation=0.0)
    trailer = TrailerPose(0.0, 0.0, 0.0, False, 0.0)
    game = NoReverse(sim, job=None, fuel_l=500, trailer=False, trailer_pose=trailer)
    c, servo = Coupling(geom, trailer, 1.57), RelativeServo()
    for _ in range(int(120 / sim.p.dt)):
        f = game.frame()
        st = c.step(f, sim.p.dt)
        if st.failed or st.done:
            break
        game.buttons(press=st.press)
        game.step(servo.step(st.steer, f.effective_steering, sim.p.dt), st.throttle, st.brake)
    assert st.failed and "gearreverse" in st.failed and sim.s.speed == pytest.approx(0.0, abs=0.3)


def test_no_way_when_the_trailer_is_too_far():
    st, _, _ = _run(TrailerPose(0.0, 0.0, 0.0, False, 0.0), 400.0, 0.0, 0.0, params=CouplingParams(max_path_m=100.0),
                    seconds=1)
    assert st.failed and "no way to line up" in st.failed


def test_keeper_leaves_the_reverse_gear_alone_while_coupling():
    f = SimpleNamespace(engine_enabled=1, electric_enabled=1, parking_brake=0, displayed_gear=-1, speed=-0.5)
    assert Keeper().step(f, True, 1 / 60, 10.0, reversing=True).press is None
    f.speed = 0.0
    assert Keeper().step(f, True, 1 / 60, 10.0).press == "geardrive"


def test_mission_couples_the_job_trailer_at_its_depot(tmp_path):
    """Standing in the source depot with the trailer (as the save shows it) 40 m away: AutoHaul couples it
    instead of trying quick park, then sets off for the destination."""
    world, services, pump, bed = make_station_world()
    geom = TruckGeometry()
    sim = TruckSim(geom, x=3790.0, y=0.0, rotation=0.0, speed=0.0)
    job = {"cargo": "bars", "loaded": True, "deadline": 600 + 600, "income": 9000, "source_city": "b",
           "source_company": "d", "destination_city": "c", "destination_company": "e"}
    world.places = SimpleNamespace(company=lambda city, company: Place(3800.0 if city == "b" else 1e6, 0.0,
                                                                         f"{company} in {city}"))
    trailer = TrailerPose(3815.0, 30.0, math.radians(-90.0), False, 0.0)
    game = SimGame(sim, job=job, fuel_l=400.0, capacity_l=500.0, game_time=600, fatigue=False, trailer=False,
                   trailer_pose=trailer)
    mission = Mission(world, services, DecisionLog(tmp_path / "d.jsonl"), JobHistory(tmp_path / "j.jsonl"),
                      MissionParams(fatigue="off"), action_params=ActionParams(settle_s=0.3, park_wait_s=2.0))
    ap = Autopilot(world.lm, geom, "relative", router=world.router, junctions=world.junctions)
    md = MissionDriver(ap, mission, trailer_source=lambda: trailer, hook_back_m=1.57)
    mission.update(game.frame(80 / 3.6), game.config(), [])
    assert md.engage(game.frame(80 / 3.6)) is None and mission.state == "at trailer"
    pressed, reversed_ = [], False
    for _ in range(int(240 / sim.p.dt)):
        out = md.step(game.frame(80 / 3.6), game.config(), game.events(), [], sim.p.dt)
        pressed += out.press
        reversed_ |= out.reverse
        if out.fault or out.status.fault:
            break
        game.buttons(out.press, out.hold)
        game.step(out.cmd.steer, out.cmd.throttle, out.cmd.brake)
        if game.trailer and mission.state == "to destination":
            break
    assert game.trailer and mission.state == "to destination", (out.fault, out.status.fault)
    assert "attach" in pressed and "quickpark" not in pressed and reversed_
    assert any(r["decision"] == "couple" for r in mission.log.records)
