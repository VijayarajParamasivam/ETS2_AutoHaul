"""Shared setup for command-line tools: config, logging, telemetry connection, controls, safety, recording."""

from __future__ import annotations

import os
import time
from pathlib import Path

from autohaul.infra.config import Config, load_config
from autohaul.infra.gameenv import steering_mode
from autohaul.infra.log import get_logger, setup_logging
from autohaul.infra.paths import logs_dir, recordings_dir
from autohaul.io import _layout as L
from autohaul.io.controls import Controls
from autohaul.io.recorder import Recorder
from autohaul.io.telemetry import TelemetryReader, text
from autohaul.safety.keyboard import KeyboardMonitor
from autohaul.safety.keys import vks
from autohaul.safety.supervisor import SafetyMonitor

log = get_logger("session")


def enable_ansi() -> bool:
    """Turn on VT escape processing in the Windows console (used by live displays)."""
    import ctypes

    k32 = ctypes.windll.kernel32
    handle = k32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
    mode = ctypes.c_uint32()
    if handle and k32.GetConsoleMode(handle, ctypes.byref(mode)):
        # ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING
        return bool(k32.SetConsoleMode(handle, mode.value | 0x0001 | 0x0004))
    os.system("")  # not a console (e.g. redirected): best effort
    return False


class Session:
    def __init__(self, name: str, *, controls: bool = False, record: bool = False, keyboard: bool = True,
                 cfg: Config | None = None, suffix: str | None = None):
        self.cfg = cfg or load_config()
        self.run_id = setup_logging(logs_dir(self.cfg), f"{time.strftime('%Y%m%d-%H%M%S')}-{name}")
        # AUTOHAUL_SHM_SUFFIX points the tools at a fake_game.exe instance instead of the real game.
        self.suffix = os.environ.get("AUTOHAUL_SHM_SUFFIX", "") if suffix is None else suffix
        self.reader = TelemetryReader(self.suffix)
        self.keyboard: KeyboardMonitor | None = None
        if keyboard:
            self.keyboard = KeyboardMonitor()
            if not self.keyboard.start():
                log.warning(f"keyboard hook unavailable: {self.keyboard.error}")
                self.keyboard = None
        self.safety = SafetyMonitor(self.cfg, self.reader, self.keyboard, steering_mode=steering_mode(self.cfg))
        self.controls: Controls | None = None
        self._want_controls = controls
        self.recorder: Recorder | None = None
        if record and self.cfg.get("recorder.enabled", True):
            path = recordings_dir(self.cfg) / f"{self.run_id}.ahrec"
            self.recorder = Recorder(path, {"tool": name, "run_id": self.run_id},
                                     flush_every=self.cfg.get("recorder.flush_every_frames", 120))

    def connect(self, timeout_s: float = 0.0) -> bool:
        """Wait for the bridge's telemetry mapping. timeout 0 = wait forever (Ctrl+C to stop)."""
        deadline = time.time() + timeout_s if timeout_s else None
        announced = False
        while not self.reader.connect():
            if not announced:
                print("Waiting for the game: start ETS2 with autohaul_bridge.dll installed "
                      "(python tools/install_plugin.py) and load your profile ...")
                announced = True
            if deadline and time.time() > deadline:
                return False
            time.sleep(0.5)
        hdr = self.reader.header()
        log.info("connected", extra={"ah_fields": {
            "game": text(hdr.game_id), "telemetry_game_version": f"{hdr.telemetry_game_version >> 16}.{hdr.telemetry_game_version & 0xFFFF}",
            "bridge": text(hdr.bridge_version), "input_registered": hdr.input_registered}})
        if self._want_controls:
            wd = 250
            extra = vks(self.cfg.get("safety.kill_keys"))
            self.controls = Controls(self.suffix, watchdog_ms=wd, extra_kill_vks=extra)
        if self.recorder:
            cfg_block = self.reader.read_config()
            if cfg_block is not None:
                self.recorder.config(cfg_block)
        return True

    def record_frame(self, frame: L.TelemetryFrame) -> None:
        if self.recorder:
            self.recorder.frame(frame)

    def close(self) -> None:
        if self.controls:
            self.controls.close()
            self.controls = None
        if self.recorder:
            self.recorder.close()
            log.info("recording saved", extra={"ah_fields": {"path": str(self.recorder.path)}})
        if self.keyboard:
            self.keyboard.stop()
        self.reader.close()

    @property
    def recording_path(self) -> Path | None:
        return self.recorder.path if self.recorder else None

    def __enter__(self) -> "Session":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
