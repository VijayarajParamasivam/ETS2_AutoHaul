"""SafetyMonitor: the one gate every control loop asks before sending commands.

Checks, in priority order:
  kill       kill key (keyboard hook), Xbox BACK+START chord, or the DLL's own latched kill  -> latched
  override   driving keys, stick/trigger movement, or telemetry inputs that differ from ours  -> latched
  multiplayer Convoy / TruckersMP detected                                                    -> latched
  stale      no fresh telemetry frame                                                         -> transient
  paused     game paused                                                                      -> transient
Latched reasons stay until reset() (an explicit re-engage by the user).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from autohaul.infra.config import Config
from autohaul.io import _layout as L
from autohaul.io.telemetry import TelemetryReader
from autohaul.safety import xinput
from autohaul.safety.keyboard import KeyboardMonitor
from autohaul.safety.keys import VK_NAME, vks
from autohaul.safety.mp_guard import multiplayer_reasons


@dataclass
class Command:
    steer: float = 0.0     # +1 right
    throttle: float = 0.0
    brake: float = 0.0


@dataclass
class Verdict:
    ok: bool
    reason: str | None = None
    kind: str | None = None      # kill | override | multiplayer | stale | paused
    latched: bool = False


class SafetyMonitor:
    def __init__(self, cfg: Config, reader: TelemetryReader, keyboard: KeyboardMonitor | None, use_xinput: bool = True):
        s = cfg.section("safety")
        self.reader = reader
        self.keyboard = keyboard
        self.use_xinput = use_xinput
        self.kill_vks = vks(s["kill_keys"])
        self.kill_chord = xinput.chord_mask(s["kill_xinput_chord"])
        self.override_vks = vks(s["override_keys"])
        self.stick_dz = s["override_stick_deadzone"]
        self.trigger_th = s["override_trigger_threshold"]
        self.steer_tol = s["override_steering_tolerance"]
        self.pedal_tol = s["override_pedal_tolerance"]
        self.mismatch_needed = s["override_crosscheck_frames"]
        self.steer_gain = s.get("steering_input_gain", -1.0)
        self.stale_ms = s["telemetry_stale_ms"]
        self.mp_modules = s["mp_guard_modules"]
        self._mp_checked = 0.0
        self._mp_reasons: list[str] = []
        self._prev_cmd = Command()
        self.reset()

    def reset(self) -> None:
        """Re-engage: clear latched reasons and re-baseline key press counters."""
        self.latched: Verdict | None = None
        self.mismatch_frames = 0
        if self.keyboard:
            self._kill_base = self.keyboard.state.press_counts(self.kill_vks)
            self._override_base = self.keyboard.state.press_counts(self.override_vks)

    def _latch(self, kind: str, reason: str) -> Verdict:
        self.latched = Verdict(False, reason, kind, latched=True)
        return self.latched

    def _keys_since_reset(self, vk_list: tuple[int, ...], base: tuple[int, ...]) -> int | None:
        assert self.keyboard is not None
        counts = self.keyboard.state.press_counts(vk_list)
        for v, c, b in zip(vk_list, counts, base):
            if c > b:
                return v
        return self.keyboard.state.any_down(vk_list)

    def check(self, frame: L.TelemetryFrame | None, cmd: Command | None) -> Verdict:
        if self.latched:
            return self.latched
        # --- kill
        if self.keyboard:
            k = self._keys_since_reset(self.kill_vks, self._kill_base)
            if k is not None:
                return self._latch("kill", f"kill key {VK_NAME.get(k, hex(k))}")
        pads = xinput.poll() if self.use_xinput else []
        for p in pads:
            reason = self.pad_kill(p)
            if reason:
                return self._latch("kill", reason)
        hdr = self.reader.header()
        if hdr.bridge_flags & L.BRIDGE_FLAG_KILL_LATCHED:
            return self._latch("kill", "kill key seen by the bridge DLL")
        # --- manual override
        if self.keyboard:
            k = self._keys_since_reset(self.override_vks, self._override_base)
            if k is not None:
                return self._latch("override", f"driving key {VK_NAME.get(k, hex(k))}")
        for p in pads:
            reason = self.pad_override(p)
            if reason:
                return self._latch("override", reason)
        if frame is not None and cmd is not None and hdr.bridge_flags & L.BRIDGE_FLAG_CONTROL_ACTIVE:
            reason = self._crosscheck(frame, cmd)
            if reason:
                return self._latch("override", reason)
        self._prev_cmd = cmd or Command()
        # --- multiplayer (module scan is slow-ish: once per second)
        now = time.monotonic()
        if now - self._mp_checked > 1.0:
            self._mp_checked = now
            self._mp_reasons = multiplayer_reasons(frame, hdr.game_pid, self.mp_modules)
        if self._mp_reasons:
            return self._latch("multiplayer", "; ".join(self._mp_reasons))
        # --- transient
        if frame is None or self.reader.frame_age_ms(frame) > self.stale_ms:
            return Verdict(False, "telemetry stale", "stale")
        if frame.paused:
            return Verdict(False, "game paused", "paused")
        return Verdict(True)

    def pad_kill(self, p: xinput.PadState) -> str | None:
        """Reason if this controller state is the kill chord (all chord buttons held)."""
        if self.kill_chord and (p.buttons & self.kill_chord) == self.kill_chord:
            return f"controller {p.index} kill chord"
        return None

    def pad_override(self, p: xinput.PadState) -> str | None:
        """Reason if this controller state means the driver is taking over (stick or pedal trigger)."""
        if abs(p.left_x) > self.stick_dz:
            return f"controller {p.index} steering stick ({p.left_x:+.2f})"
        if p.left_trigger > self.trigger_th or p.right_trigger > self.trigger_th:
            return f"controller {p.index} pedal trigger"
        return None

    def _crosscheck(self, frame: L.TelemetryFrame, cmd: Command) -> str | None:
        """Telemetry shows the game's combined input; if it differs from both our current and our
        previous command for several frames, someone else is driving."""
        def dev(c: Command) -> tuple[float, float, float]:
            return (abs(frame.input_steering - self.steer_gain * c.steer),
                    frame.input_throttle - c.throttle,
                    frame.input_brake - c.brake)
        a, b = dev(cmd), dev(self._prev_cmd)
        ds, dt, db = min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2])
        if ds > self.steer_tol or dt > self.pedal_tol or db > self.pedal_tol:
            self.mismatch_frames += 1
        else:
            self.mismatch_frames = 0
        if self.mismatch_frames >= self.mismatch_needed:
            which = "steering" if ds > self.steer_tol else ("throttle" if dt > self.pedal_tol else "brake")
            return f"telemetry {which} differs from AutoHaul's command (off by: steer {ds:.2f}, throttle {dt:.2f}, brake {db:.2f})"
        return None
