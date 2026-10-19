"""Unit tests that need neither the game nor the native build."""

from __future__ import annotations

import ctypes
import os
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

from autohaul.infra.config import PROJECT_ROOT, load_config
from autohaul.io import _layout as L
from autohaul.io.controls import Controls
from autohaul.io.gamedata import GameDataReader, load_spec
from autohaul.io.recorder import KIND_CONTROLS, KIND_EVENT, KIND_FRAME, KIND_NOTE, Recorder, RecordingReader
from autohaul.io.telemetry import GameplayEvent
from autohaul.io.win32 import create_mapping, current_pid
from autohaul.safety.keyboard import KeyState
from autohaul.safety.keys import vk, vks
from autohaul.safety.mp_guard import multiplayer_reasons, process_modules
from autohaul.safety.supervisor import Command, SafetyMonitor


def test_generated_layout_is_up_to_date():
    r = subprocess.run([sys.executable, str(PROJECT_ROOT / "tools/gen_layout.py"), "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_layout_sizes_and_channel_table():
    assert ctypes.sizeof(L.ControlsBlock) <= L.CONTROLS_SIZE
    assert L.EVENTS_OFFSET + L.EVENT_SLOTS * ctypes.sizeof(L.EventSlot) <= L.TELEMETRY_SIZE
    assert L.TelemetryFrame.seq_begin.offset == 0
    assert L.TelemetryFrame.seq_end.offset == ctypes.sizeof(L.TelemetryFrame) - 8
    names = {c[1] for c in L.CHANNELS}
    assert {"truck.speed", "rest.stop", "trailer.0.connected", "truck.navigation.speed.limit"} <= names
    assert len(L.CHANNELS) <= 64 * L.CHANNEL_SEEN_WORDS
    assert L.AXES[0][0] == "steering" and L.AXIS_INDEX["abackward"] == 2
    assert len(set(L.BUTTON_INDEX)) == L.NUM_BUTTONS


def test_inputs_exist_in_users_controls_sii():
    """Every semantical input must be referenced by the profile's controls.sii (else the game ignores it)."""
    cfg = load_config()
    profiles = Path(cfg.get("game.home_dir")) / "profiles"
    files = list(profiles.glob("*/controls.sii")) if profiles.exists() else []
    if not files:
        pytest.skip("no ETS2 profile on this machine")
    textdata = files[0].read_text(encoding="utf-8", errors="replace")
    missing = [m for m in [a[0] for a in L.AXES] + [b[0] for b in L.BUTTONS] if f"semantical.{m}?" not in textdata]
    assert not missing, f"not semantical-capable in this game version: {missing}"


def test_config_expands_env_vars():
    cfg = load_config()
    assert "%" not in cfg.get("paths.cache_dir")
    assert cfg.get("paths.cache_dir").startswith(os.environ["USERPROFILE"])
    assert cfg.get("game.expected_version") == "1.53.2.1"
    assert "AppData" not in cfg.get("paths.cache_dir")


def test_keys():
    assert vk("backslash") == 0xDC and vk("PAUSE") == 0x13 and vk("W") == 0x57 and vk("F12") == 0x7B
    assert vks(["UP", "SPACE"]) == (0x26, 0x20)
    with pytest.raises(KeyError):
        vk("NOPE")


def test_keystate_counts_presses_not_repeats():
    k = KeyState()
    k.on_event(0x41, True)
    k.on_event(0x41, True)   # auto-repeat
    assert k.press_count(0x41) == 1 and k.is_down(0x41)
    k.on_event(0x41, False)
    k.on_event(0x41, True)
    assert k.press_count(0x41) == 2
    assert k.any_down((0x42, 0x41)) == 0x41
    assert k.last_press_qpc(0x41) > 0


def test_recorder_roundtrip(tmp_path):
    path = tmp_path / "run.ahrec"
    with Recorder(path, {"tool": "test"}, flush_every=7) as rec:
        for i in range(1, 30):
            f = L.TelemetryFrame()
            f.frame_index = i
            f.speed = i * 0.5
            rec.frame(f)
        rec.event(GameplayEvent(1, "player.fined", 1234, {"fine.offence": "speeding"}))
        rec.controls(bytes(L.ControlsBlock()))
        rec.note(decision="refuel", reason="range low")
    reader = RecordingReader(path)
    assert reader.meta["tool"] == "test"
    kinds = [r.kind for r in reader]
    assert kinds.count(KIND_FRAME) == 29 and KIND_EVENT in kinds and KIND_CONTROLS in kinds and KIND_NOTE in kinds
    frames = [f for _, f in reader.frames()]
    assert [f.frame_index for f in frames] == list(range(1, 30))
    assert frames[-1].speed == pytest.approx(14.5)


def test_recording_with_truncated_tail_still_reads(tmp_path):
    path = tmp_path / "crash.ahrec"
    rec = Recorder(path, flush_every=5)
    for i in range(12):
        rec.frame(L.TelemetryFrame(frame_index=i))
    rec.f.flush()
    rec.f.write(b"\x10\x00\x00\x00garbage")   # simulates a crash mid-chunk
    rec.f.close()
    assert len(list(RecordingReader(path).frames())) == 10


def test_controls_names_and_pulse_wrap():
    suffix = f"_unit{os.getpid()}"
    with Controls(suffix) as ctl:
        with pytest.raises(KeyError):
            ctl.press("does_not_exist")
        with pytest.raises(KeyError):
            ctl.axis("nope", 1.0)
        j = L.BUTTON_INDEX["horn"]
        ctl._pulses[j] = 0xFFFFFFFF
        ctl.press("horn")
        assert ctl._pulses[j] == 0
        ctl.steer(5.0)
        ctl.throttle(-1)
        ctl.set_enabled(True)
        seq = ctl.commit()
        assert ctl.block.axes[L.AXIS_INDEX["steering"]] == 1.0
        assert ctl.block.axes[L.AXIS_INDEX["aforward"]] == 0.0
        assert ctl.block.header.enable == 1 and seq == ctl.block.header.write_seq
        assert ctl.block.header.magic == L.CONTROLS_MAGIC and ctl.block.header.layout_hash == L.LAYOUT_HASH


def test_gamedata_parsing_from_synthetic_blocks():
    spec = load_spec()
    suffix = f"_unit{os.getpid()}"
    for b in spec.values():
        b.mapping += suffix
    maps = {name: create_mapping(b.mapping, b.size) for name, b in spec.items()}
    try:
        cam = spec["camera"].record.pack(72.0, 10.0, 2.0, 20.0, 3, -2, 1.0, 0.0, 0.0, 0.0)
        maps["camera"][: len(cam)] = cam
        veh = spec["traffic"].record.pack(1000.0, 5.0, 2000.0, 1, 0, 0, 0, 2.5, 3.8, 16.5, 22.0, 0.1, 1, 77)
        trl = spec["traffic"].trailer.pack(1000.0, 5.0, 2010.0, 1, 0, 0, 0, 2.5, 4.0, 13.6)
        maps["traffic"][: len(veh) + len(trl)] = veh + trl
        sem = spec["semaphore"].record.pack(10.0, 1.0, 20.0, 2.0, -1.0, 1, 0, 0, 0, 1, 4.5, 2, 9)
        maps["semaphore"][: len(sem)] = sem
        route = spec["route"].record.pack(0x1234, 5000.0, 300.0)
        maps["route"][: len(route)] = route

        gd = GameDataReader(spec)
        assert all(gd.available().values())
        c = gd.camera()
        assert c["wx"] == pytest.approx(10 + 512 * 3) and c["wz"] == pytest.approx(20 - 512 * 2)
        t = gd.traffic()
        assert len(t) == 1 and t[0]["id"] == 77 and t[0]["trailers"][0]["length"] == pytest.approx(13.6)
        assert t[0]["wx"] == 1000.0     # traffic positions are already world coordinates
        s = gd.semaphores()
        assert len(s) == 1 and s[0]["state_name"] == "red" and s[0]["wx"] == pytest.approx(10 + 1024)
        r = gd.route()
        assert len(r) == 1 and r[0]["node_uid"] == 0x1234
        gd.close()
    finally:
        for m in maps.values():
            m.close()


def test_steering_mode_from_controls_sii(tmp_path, monkeypatch):
    from autohaul.infra import gameenv
    prof = tmp_path / "profiles" / "ABC"
    prof.mkdir(parents=True)
    sii = prof / "controls.sii"
    sii.write_text('SiiNunit\n{\n config_lines[40]: "constant c_relatsteer 1.000000"\n}\n', encoding="utf-8")
    cfg = load_config()
    data = cfg.as_dict()
    data["game"] = dict(data["game"], home_dir=str(tmp_path))
    from autohaul.infra.config import Config
    c2 = Config(data)
    assert gameenv.steering_mode(c2) == "relative"
    sii.write_text('"constant c_relatsteer 0.000000"', encoding="utf-8")
    assert gameenv.steering_mode(c2) == "absolute"
    sii.write_text('"constant c_other 1.0"', encoding="utf-8")
    assert gameenv.steering_mode(c2) is None


def test_elevation_check_on_own_process():
    from autohaul.infra.gameenv import is_elevated
    assert is_elevated(current_pid()) in (True, False)
    assert is_elevated(0) is None


def test_mp_guard_module_scan_on_own_process():
    mods = process_modules(current_pid())
    assert mods and any(m.startswith("python3") for m in mods)
    own = next(m for m in mods if m.startswith("python3") and m.endswith(".dll"))
    assert multiplayer_reasons(None, current_pid(), [own])
    assert multiplayer_reasons(None, current_pid(), ["core_ets2mp.dll"]) == []


# ---------------------------------------------------------------- SafetyMonitor with fakes

@dataclass
class FakeHeader:
    bridge_flags: int = L.BRIDGE_FLAG_CONTROL_ACTIVE
    game_pid: int = 0


class FakeReader:
    def __init__(self):
        self.hdr = FakeHeader()
        self.age = 5.0

    def header(self):
        return self.hdr

    def frame_age_ms(self, f):
        return self.age


class FakeKeyboard:
    def __init__(self):
        self.state = KeyState()


def frame(steer_in=0.0, thr=0.0, brk=0.0, paused=0) -> L.TelemetryFrame:
    f = L.TelemetryFrame()
    f.input_steering, f.input_throttle, f.input_brake, f.paused = steer_in, thr, brk, paused
    return f


def monitor():
    reader, kb = FakeReader(), FakeKeyboard()
    return SafetyMonitor(load_config(local=None), reader, kb, use_xinput=False), reader, kb


def test_safety_ok_when_inputs_match_command():
    m, _, _ = monitor()
    for _ in range(10):
        assert m.check(frame(steer_in=-0.3, thr=0.2), Command(steer=0.3, throttle=0.2)).ok


def test_safety_kill_key_latches_until_reset():
    m, _, kb = monitor()
    kb.state.on_event(vk("BACKSLASH"), True)
    kb.state.on_event(vk("BACKSLASH"), False)      # quick tap between checks still counts
    v = m.check(frame(), Command())
    assert not v.ok and v.kind == "kill" and v.latched
    assert m.check(frame(), Command()).kind == "kill"
    m.reset()
    assert m.check(frame(), Command()).ok


def test_safety_override_key():
    m, _, kb = monitor()
    kb.state.on_event(vk("A"), True)
    v = m.check(frame(), Command())
    assert v.kind == "override" and "A" in v.reason


def test_safety_crosscheck_needs_consecutive_frames():
    m, _, _ = monitor()
    cmd = Command(steer=0.0)
    assert m.check(frame(steer_in=0.5), cmd).ok          # 1 frame: could be lag
    assert m.check(frame(steer_in=0.5), cmd).ok          # 2
    v = m.check(frame(steer_in=0.5), cmd)                # 3 -> someone else is steering
    assert v.kind == "override" and "steering" in v.reason


def test_safety_crosscheck_tolerates_command_change_lag():
    m, _, _ = monitor()
    m.check(frame(steer_in=-0.2), Command(steer=0.2))
    for _ in range(5):  # telemetry still shows the previous command for a frame each time
        assert m.check(frame(steer_in=-0.2), Command(steer=0.25)).ok
        assert m.check(frame(steer_in=-0.25), Command(steer=0.2)).ok


def test_safety_user_pressing_throttle_is_override():
    m, _, _ = monitor()
    for _ in range(2):
        m.check(frame(thr=0.6), Command(throttle=0.1))
    assert m.check(frame(thr=0.6), Command(throttle=0.1)).kind == "override"


def test_controller_kill_chord_and_override_with_real_button_values():
    """Values as logged from the user's Xbox 360 pad on 2026-10-19: BACK 0x20, START 0x10, both 0x30."""
    from autohaul.safety.xinput import PadState
    m, _, _ = monitor()

    def pad(buttons=0, lx=0.0, lt=0.0, rt=0.0):
        return PadState(0, buttons, lx, 0.0, 0.0, 0.0, lt, rt)

    assert m.pad_kill(pad(0x0030)) == "controller 0 kill chord"
    assert m.pad_kill(pad(0x0020)) is None and m.pad_kill(pad(0x0010)) is None
    assert m.pad_kill(pad(0x0004)) is None                       # D-pad left, as logged
    assert "steering stick" in m.pad_override(pad(lx=-0.40))
    assert "steering stick" in m.pad_override(pad(lx=+0.35))
    assert "pedal trigger" in m.pad_override(pad(lt=1.0))
    assert "pedal trigger" in m.pad_override(pad(rt=0.17))
    assert m.pad_override(pad(lx=0.2, lt=0.1, rt=0.1)) is None   # inside dead zones


def test_safety_bridge_kill_flag_and_transients():
    m, reader, _ = monitor()
    reader.age = 500
    assert m.check(frame(), Command()).kind == "stale"
    reader.age = 5
    assert m.check(frame(paused=1), Command()).kind == "paused"
    assert m.check(frame(), Command()).ok                # transient reasons don't latch
    reader.hdr.bridge_flags |= L.BRIDGE_FLAG_KILL_LATCHED
    assert m.check(frame(), Command()).kind == "kill"


def test_struct_format_sizes_match_plugin_code():
    spec = load_spec()
    assert spec["traffic"].stride == 132 and spec["traffic"].size == 2640
    assert spec["semaphore"].record.size == 52 and spec["semaphore"].size == 1040
    assert spec["route"].record.size == 16 and spec["route"].size == 80000
    assert spec["camera"].record.size == 36
    assert struct.calcsize("<ffffffffffffhh") == 52
