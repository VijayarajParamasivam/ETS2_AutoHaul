"""End-to-end tests: real autohaul_bridge.dll inside fake_game.exe, real Python reader/writer."""

from __future__ import annotations

import math
import threading
import time

import pytest

from autohaul.io import _layout as L
from autohaul.io.controls import Controls
from autohaul.io.telemetry import TelemetryReader, channel_seen, text
from tests.fakegame import FakeGame, native_available

pytestmark = pytest.mark.skipif(not native_available(), reason="run `python tools/build_native.py bridge` first")


def connect(game: FakeGame) -> TelemetryReader:
    game.wait_ready()
    reader = TelemetryReader(game.suffix)
    deadline = time.time() + 5
    while not reader.connect():
        assert time.time() < deadline, "telemetry mapping never appeared"
        time.sleep(0.01)
    return reader


class Driver:
    """Commits a fixed command set at ~100 Hz on a thread, like the real control loop."""

    def __init__(self, ctl: Controls):
        self.ctl = ctl
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while self.running:
            self.ctl.commit()
            time.sleep(0.01)

    def stop(self) -> None:
        self.running = False
        self.thread.join()


def test_telemetry_frames_values_and_missing_channel():
    with FakeGame(frames=240, hz=120) as game:
        reader = connect(game)
        frames = []
        while len(frames) < 50:
            f = reader.wait_frame(500)
            assert f is not None
            frames.append(f)
        hdr = reader.header()
        assert hdr.telemetry_game_version == (1 << 16) | 18
        assert hdr.bridge_flags & L.BRIDGE_FLAG_TELEMETRY_OK and hdr.bridge_flags & L.BRIDGE_FLAG_INPUT_OK
        assert text(hdr.game_id) == "eut2"
        for f in frames:
            n = f.frame_index
            assert f.speed == pytest.approx(10.0 + 0.01 * n, abs=1e-4)
            assert f.truck_placement.x == pytest.approx(100.0 + n)
            assert f.truck_placement.z == pytest.approx(-200.0 - n)
            assert f.truck_placement.heading == pytest.approx(0.25)
            assert f.game_time == 1000 + n // 10
            assert f.next_rest_stop == 600 - n // 10
            assert f.trailers[0].connected == 1 and f.trailers[1].connected == 0
            assert f.wheel_steering[3] == pytest.approx(0.3)
            # mandatory.break doesn't exist on 1.53: never seen, sentinel kept
            assert not channel_seen(f, "mandatory.break")
            assert f.next_mandatory_break == -(2**31)
            assert channel_seen(f, "truck.speed")
        indices = [f.frame_index for f in frames]
        assert indices == sorted(indices) and len(set(indices)) == len(indices)
        assert reader.torn_reads == 0 or reader.torn_reads < len(frames)
        reader.close()
        assert game.wait_done() == 0


def test_config_block_and_job_end():
    with FakeGame(frames=200, hz=200) as game:
        reader = connect(game)
        game.wait_frame(10)
        cfg = reader.read_config()
        assert cfg is not None
        assert cfg.truck.present and text(cfg.truck.brand) == "Scania"
        assert cfg.truck.wheel_count == 6 and cfg.truck.forward_gear_count == 12
        assert cfg.truck.wheel_position[1].x == pytest.approx(1.0)
        assert cfg.truck.wheel_steerable[0] == 1 and cfg.truck.wheel_steerable[3] == 0
        assert cfg.truck.forward_ratio[0] == pytest.approx(10.0)
        assert cfg.trailers[0].present and text(cfg.trailers[0].id) == "scs_box"
        assert cfg.trailers[0].hook_position.z == pytest.approx(-5.0)
        assert cfg.job.present and text(cfg.job.cargo) == "Apples" and cfg.job.income == 12345
        assert cfg.job.delivery_time == 5000 and text(cfg.job.destination_city) == "Berlin"
        assert cfg.job.cargo_loaded == 1 and text(cfg.job.job_market) == "cargo_market"
        assert text(cfg.controls.shifter_type) == "automatic"
        job_seq = cfg.job.seq
        game.wait_frame(110)  # job config cleared at frame 100
        cfg2 = reader.read_config()
        assert cfg2 is not None and cfg2.job.present == 0 and cfg2.job.seq == job_seq + 1
        reader.close()
        game.wait_done()


def test_gameplay_events():
    with FakeGame(frames=120, hz=200) as game:
        reader = connect(game)
        game.wait_frame(100)
        events = reader.poll_events()
        ids = [e.id for e in events]
        assert ids == ["player.fined", "player.fined", "job.delivered"]
        assert events[0].attrs == {"fine.offence": "speeding", "fine.amount": "250"}
        delivered = events[2]
        assert delivered.get_int("revenue") == 12345 and delivered.attrs["auto.park.used"] == "1"
        assert delivered.get_float("distance.km") == pytest.approx(512.5)
        assert reader.poll_events() == []  # nothing new
        reader.close()
        game.wait_done()


def test_controls_axes_holds_and_pulses():
    with FakeGame(frames=360, hz=120) as game:
        game.wait_ready()
        with Controls(game.suffix) as ctl:
            ctl.set_enabled(True)
            ctl.steer(0.5)
            ctl.throttle(0.3)
            ctl.hold("horn")
            ctl.press("lblinker", 2)
            drv = Driver(ctl)
            game.wait_frame(game.frame + 30)
            drv.stop()
        game.wait_done()
    assert {round(v, 6) for _, v in game.values("steering")} == {0.0, 0.5}
    assert any(abs(v - 0.3) < 1e-6 for _, v in game.values("aforward"))
    assert any(v == 1 for _, v in game.values("horn"))
    presses = [f for f, v in game.values("lblinker") if v == 1]
    releases = [f for f, v in game.values("lblinker") if v == 0 and f > 1]
    assert len(presses) == 2, game.values("lblinker")
    # each press lasts exactly one frame and presses are separated by a released frame
    assert all(r - p == 1 for p, r in zip(presses, releases))
    # close() released everything: final values are neutral
    assert game.values("steering")[-1][1] == 0 and game.values("horn")[-1][1] == 0


def test_watchdog_releases_after_heartbeat_loss():
    with FakeGame(frames=480, hz=120) as game:
        reader = connect(game)
        ctl = Controls(game.suffix, watchdog_ms=250)
        ctl.set_enabled(True)
        ctl.steer(-0.4)
        drv = Driver(ctl)
        game.wait_frame(game.frame + 30)
        assert game.value_at("steering", game.frame) == pytest.approx(-0.4)
        drv.stop()                      # Python "hangs": no more heartbeats
        stop_frame = game.frame
        game.wait_frame(stop_frame + 90)
        released = [f for f, v in game.values("steering") if f > stop_frame and v == 0]
        assert released, "watchdog never released steering"
        frames_to_release = released[0] - stop_frame
        # 250 ms at 120 Hz = 30 frames; allow scheduling slack
        assert 20 <= frames_to_release <= 45, frames_to_release
        hdr = reader.header()
        assert hdr.watchdog_trips == 1
        assert not hdr.bridge_flags & L.BRIDGE_FLAG_CONTROL_ALIVE
        ctl.close()
        reader.close()
        game.wait_done()


def test_kill_switch_latches_until_rearm():
    with FakeGame("--kill-at", "60", "--kill-frames", "10", frames=360, hz=120) as game:
        reader = connect(game)
        ctl = Controls(game.suffix)
        ctl.set_enabled(True)
        ctl.steer(0.25)
        drv = Driver(ctl)
        game.wait_frame(50)
        assert game.value_at("steering", 50) == pytest.approx(0.25)
        game.wait_frame(120)            # kill held 60..69, released afterwards
        assert game.value_at("steering", 61) == 0
        assert game.value_at("steering", 119) == 0, "kill must stay latched after the key is released"
        hdr = reader.header()
        assert hdr.kill_count == 1 and hdr.bridge_flags & L.BRIDGE_FLAG_KILL_LATCHED
        ctl.rearm()
        f = game.frame
        game.wait_frame(f + 20)
        assert game.value_at("steering", game.frame) == pytest.approx(0.25)
        assert not reader.header().bridge_flags & L.BRIDGE_FLAG_KILL_LATCHED
        drv.stop()
        ctl.close()
        reader.close()
        game.wait_done()


def test_pause_releases_and_resume_restores():
    with FakeGame("--pause-at", "80", "--resume-at", "140", frames=260, hz=120) as game:
        game.wait_ready()
        ctl = Controls(game.suffix)
        ctl.set_enabled(True)
        ctl.throttle(0.6)
        drv = Driver(ctl)
        game.wait_frame(200)
        drv.stop()
        ctl.close()
        game.wait_done()
    assert game.value_at("aforward", 70) == pytest.approx(0.6)
    assert game.value_at("aforward", 100) == 0
    assert game.value_at("aforward", 180) == pytest.approx(0.6)


def test_disabled_client_sends_nothing():
    with FakeGame(frames=120, hz=120) as game:
        game.wait_ready()
        ctl = Controls(game.suffix)
        ctl.set_enabled(False)
        ctl.steer(1.0)
        drv = Driver(ctl)
        game.wait_frame(80)
        drv.stop()
        ctl.close()
        game.wait_done()
    assert all(v == 0 for _, v in game.values("steering"))


def test_frame_rate_and_latency():
    with FakeGame(frames=400, hz=120) as game:
        reader = connect(game)
        ages, stamps = [], []
        while len(stamps) < 200:
            f = reader.wait_frame(500)
            assert f is not None
            ages.append(reader.frame_age_ms(f))
            stamps.append(f.publish_qpc)
        intervals = [(b - a) * 1000 / reader.qpc_freq for a, b in zip(stamps, stamps[1:])]
        mean = sum(intervals) / len(intervals)
        assert mean == pytest.approx(1000 / 120, rel=0.25)
        ages.sort()
        assert ages[len(ages) // 2] < 5.0, f"median frame age {ages[len(ages) // 2]:.2f} ms"
        assert not math.isnan(mean)
        reader.close()
        game.wait_done()
