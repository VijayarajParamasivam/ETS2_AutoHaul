"""Test helper: runs native fake_game.exe (which loads autohaul_bridge.dll) with isolated shared-memory names."""

from __future__ import annotations

import itertools
import os
import subprocess
import threading
import time
from pathlib import Path

from autohaul.infra.config import load_config
from autohaul.infra.paths import build_dir
from autohaul.io import _layout as L

OUT = build_dir(load_config()) / "out"
DLL = OUT / "autohaul_bridge.dll"
FAKE = OUT / "fake_game.exe"
_counter = itertools.count()


def native_available() -> bool:
    return DLL.exists() and FAKE.exists()


class FakeGame:
    def __init__(self, *args: str, frames: int = 600, hz: int = 120):
        self.hz = hz
        self.suffix = f"_test{os.getpid()}_{next(_counter)}"
        env = dict(os.environ, AUTOHAUL_SHM_SUFFIX=self.suffix)
        self.proc = subprocess.Popen(
            [str(FAKE), str(DLL), "--frames", str(frames), "--hz", str(hz), *args],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
        )
        self.lines: list[str] = []
        self.inputs: list[tuple[int, int, float]] = []   # (frame, input index, value)
        self.frame = 0
        self.ready = threading.Event()
        self.done = threading.Event()
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self) -> None:
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.lines.append(line)
            if line == "READY":
                self.ready.set()
            elif line.startswith("INPUT "):
                _, f, i, v = line.split()
                self.inputs.append((int(f), int(i), float(v)))
            elif line.startswith("FRAME "):
                self.frame = int(line.split()[1])
        self.done.set()

    def wait_ready(self, timeout: float = 10.0) -> None:
        if not self.ready.wait(timeout):
            raise TimeoutError("fake_game did not become ready:\n" + "\n".join(self.lines[-20:]))

    def wait_done(self, timeout: float = 30.0) -> int:
        if not self.done.wait(timeout):
            self.proc.kill()
            raise TimeoutError("fake_game did not finish")
        return self.proc.wait(timeout=5)

    def wait_frame(self, frame: int, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        while self.frame < frame:
            if time.time() > deadline or self.done.is_set():
                raise TimeoutError(f"fake_game never reached frame {frame} (at {self.frame})")
            time.sleep(0.002)

    def values(self, name: str) -> list[tuple[int, float]]:
        idx = L.AXIS_INDEX[name] if name in L.AXIS_INDEX else L.NUM_AXES + L.BUTTON_INDEX[name]
        return [(f, v) for f, i, v in self.inputs if i == idx]

    def value_at(self, name: str, frame: int) -> float:
        """Last value sent for `name` at or before `frame` (inputs are change-only)."""
        last = 0.0
        for f, v in self.values(name):
            if f > frame:
                break
            last = v
        return last

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.wait(timeout=5)

    def __enter__(self) -> "FakeGame":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
