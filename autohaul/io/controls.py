"""Writer for the bridge's controls mapping (Local\\AutoHaul_Controls).

    ctl = Controls()
    ctl.set_enabled(True)
    ctl.steer(0.1); ctl.throttle(0.3); ctl.press("lblinker")
    ctl.commit()            # call every control tick: it is also the watchdog heartbeat
    ...
    ctl.close()             # releases everything

Conventions: steer(+1) = full right, -1 = full left. Pedals 0..1.
If commit() stops being called for longer than the watchdog (default 250 ms), the DLL releases
every input on its own.
"""

from __future__ import annotations

import ctypes

from autohaul.io import _layout as L
from autohaul.io.win32 import create_mapping, current_pid, qpc, qpc_frequency


class ClientConflict(RuntimeError):
    """Another live AutoHaul process already owns the controls mapping."""


class Controls:
    def __init__(self, suffix: str = "", watchdog_ms: int = 250, extra_kill_vks: tuple[int, ...] = (),
                 takeover: bool = False):
        self.mm = create_mapping(L.CONTROLS_MAPPING + suffix, L.CONTROLS_SIZE)
        self.block = L.ControlsBlock.from_buffer(self.mm)
        h = self.block.header
        pid = current_pid()
        if not takeover and h.magic == L.CONTROLS_MAGIC and h.client_pid not in (0, pid) and h.enable:
            age_s = (qpc() - h.heartbeat_qpc) / qpc_frequency()
            if 0 <= age_s < 1.0:
                raise ClientConflict(f"controls are owned by live process {h.client_pid}")
        self._pulses = [int(v) for v in self.block.pulses]
        self._axes = [0.0] * L.NUM_AXES
        self._holds = [0] * L.NUM_BUTTONS
        h.magic = 0
        ctypes.memset(ctypes.addressof(self.block.axes), 0, ctypes.sizeof(self.block.axes))
        ctypes.memset(ctypes.addressof(self.block.holds), 0, ctypes.sizeof(self.block.holds))
        h.layout_version = L.LAYOUT_VERSION
        h.layout_hash = L.LAYOUT_HASH
        h.client_pid = pid
        h.enable = 0
        h.watchdog_ms = watchdog_ms
        for i in range(L.MAX_EXTRA_KILL_VK):
            h.extra_kill_vk[i] = extra_kill_vks[i] if i < len(extra_kill_vks) else 0
        h.heartbeat_qpc = 0
        h.magic = L.CONTROLS_MAGIC
        self.enabled = False

    # ------------------------------------------------------------- commands
    def axis(self, name: str, value: float) -> None:
        try:
            i = L.AXIS_INDEX[name]
        except KeyError:
            raise KeyError(f"unknown axis {name!r}; known: {', '.join(L.AXIS_INDEX)}") from None
        self._axes[i] = max(-1.0, min(1.0, float(value)))

    def steer(self, value: float) -> None:
        self.axis("steering", value)

    def throttle(self, value: float) -> None:
        self.axis("aforward", max(0.0, value))

    def brake(self, value: float) -> None:
        self.axis("abackward", max(0.0, value))

    def clutch(self, value: float) -> None:
        self.axis("clutch", max(0.0, value))

    def _button(self, name: str) -> int:
        try:
            return L.BUTTON_INDEX[name]
        except KeyError:
            raise KeyError(f"unknown button {name!r}; see native/spec/inputs.toml") from None

    def hold(self, name: str, on: bool = True) -> None:
        self._holds[self._button(name)] = 1 if on else 0

    def press(self, name: str, count: int = 1) -> None:
        """Queue `count` presses; the DLL turns each into one pressed frame + one released frame."""
        j = self._button(name)
        self._pulses[j] = (self._pulses[j] + count) & 0xFFFFFFFF

    def set_enabled(self, on: bool) -> None:
        self.enabled = bool(on)

    def rearm(self) -> None:
        """Clear a latched kill switch (the DLL ignores it while a kill key is still held)."""
        h = self.block.header
        h.rearm_counter = (h.rearm_counter + 1) & 0xFFFFFFFF

    def neutral(self) -> None:
        """Zero every axis and hold (pending presses are kept)."""
        self._axes = [0.0] * L.NUM_AXES
        self._holds = [0] * L.NUM_BUTTONS

    # ------------------------------------------------------------- publishing
    def commit(self) -> int:
        """Publish the current command set and refresh the heartbeat. Returns write_seq."""
        b = self.block
        for i, v in enumerate(self._axes):
            b.axes[i] = v
        for j, v in enumerate(self._holds):
            b.holds[j] = v
        for j, v in enumerate(self._pulses):
            b.pulses[j] = v
        h = b.header
        h.enable = 1 if self.enabled else 0
        h.write_seq += 1
        h.heartbeat_qpc = qpc()  # last: the DLL treats a fresh heartbeat as "this block is current"
        return h.write_seq

    def release(self) -> None:
        self.neutral()
        self.enabled = False
        self.commit()

    def snapshot(self) -> bytes:
        """Raw bytes of the block as committed (for recordings)."""
        return bytes(self.mm[: ctypes.sizeof(L.ControlsBlock)])

    def close(self) -> None:
        if self.block is None:
            return
        try:
            self.release()
            self.block.header.client_pid = 0
        finally:
            del self.block
            self.block = None
            self.mm.close()

    def __enter__(self) -> "Controls":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
