"""The patched ETS2LA plugin (1.53 pin + AutoHaul guard) inside fake_game.exe: it must load, tick
every frame, keep the crashing traffic_objects reader off, and survive a reader access violation."""

from __future__ import annotations

import os
import subprocess

import pytest

from tests.fakegame import FAKE, OUT

PLUGIN = OUT / "ets2_la_plugin.dll"
pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and FAKE.exists()),
                                reason="run `python tools/build_native.py` first")


def run(env_extra: dict[str, str] | None = None, frames: int = 60) -> list[str]:
    env = dict(os.environ, **(env_extra or {}))
    r = subprocess.run([str(FAKE), str(PLUGIN), "--telemetry-only", "--frames", str(frames), "--hz", "120"],
                       capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    return r.stdout.splitlines()


def test_guarded_plugin_ticks_with_traffic_objects_disabled():
    lines = run()
    assert any("steering hook disabled" in ln for ln in lines)
    assert any("traffic_objects reader disabled" in ln for ln in lines)
    assert not any("faulted" in ln for ln in lines)
    assert lines[-1] == "DONE"


def test_guard_survives_an_access_violation_in_a_reader():
    lines = run({"AUTOHAUL_TEST_FAULT_READER": "navigation"})
    faulted = [ln for ln in lines if "navigation reader faulted (access violation)" in ln]
    assert len(faulted) == 1, "the faulting reader must be reported once and then stay disabled"
    assert lines[-1] == "DONE", "the game process must keep running after the fault"
