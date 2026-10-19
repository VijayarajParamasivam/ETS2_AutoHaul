"""Reader for the bridge's telemetry mapping (Local\\AutoHaul_Telemetry).

    reader = TelemetryReader()
    while not reader.connect(): time.sleep(0.5)
    while True:
        frame = reader.wait_frame(timeout_ms=100)   # TelemetryFrame or None
        for ev in reader.poll_events(): ...

Frames are read from a seqlock ring, so a reader never sees a half-written frame.
"""

from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass, field

from autohaul.io import _layout as L
from autohaul.io.win32 import NamedEvent, ReadOnlyMapping, qpc, qpc_frequency

_SEQ_END_OFFSET = ctypes.sizeof(L.TelemetryFrame) - 8
_FRAME_COUNTER_OFFSET = L.TelemetryHeader.frame_counter.offset
_EVENT_WRITE_OFFSET = L.TelemetryHeader.event_write_index.offset


class LayoutMismatch(RuntimeError):
    """The DLL was built from a different layout spec than this Python package."""


@dataclass
class GameplayEvent:
    seq: int
    id: str
    game_time: int
    attrs: dict[str, str] = field(default_factory=dict)

    def get_float(self, name: str, default: float = 0.0) -> float:
        try:
            return float(self.attrs[name])
        except (KeyError, ValueError):
            return default

    def get_int(self, name: str, default: int = 0) -> int:
        try:
            return int(self.attrs[name])
        except (KeyError, ValueError):
            return default


def text(value: bytes) -> str:
    """Decode a fixed char[] field."""
    return value.decode("utf-8", "replace")


def heading_rad(h: float) -> float:
    """SDK heading (0..1, CCW, 0 = north/-Z) -> radians."""
    return h * 2.0 * math.pi


def channel_seen(frame: L.TelemetryFrame, sdk: str, index: int | None = None) -> bool:
    cid = L.CHANNEL_IDS.get((sdk, index))
    if cid is None:
        return False
    return bool(frame.channel_seen[cid // 64] & (1 << (cid % 64)))


class TelemetryReader:
    def __init__(self, suffix: str = ""):
        self.suffix = suffix
        self.map = ReadOnlyMapping(L.TELEMETRY_MAPPING + suffix, L.TELEMETRY_SIZE)
        self.event = NamedEvent(L.FRAME_EVENT + suffix)
        self.last_seq = 0
        self.last_event_seq = 0
        self.qpc_freq = qpc_frequency()
        self.resyncs = 0
        self.torn_reads = 0

    # ------------------------------------------------------------- connection
    def connect(self) -> bool:
        """True once the bridge's mapping exists and is initialised. Raises LayoutMismatch."""
        if not self.map.open():
            return False
        hdr = self.header()
        if hdr.magic != L.TELEMETRY_MAGIC:
            return False
        if hdr.layout_hash != L.LAYOUT_HASH or hdr.layout_version != L.LAYOUT_VERSION:
            raise LayoutMismatch(
                f"bridge layout v{hdr.layout_version} 0x{hdr.layout_hash:016x} != python v{L.LAYOUT_VERSION} "
                f"0x{L.LAYOUT_HASH:016x}: rebuild the DLL (tools/build_native.py) or regenerate (tools/gen_layout.py)")
        return True

    def close(self) -> None:
        self.event.close()
        self.map.close()

    def header(self) -> L.TelemetryHeader:
        return self.map.read_into(L.TelemetryHeader, 0)

    # ------------------------------------------------------------- frames
    def _read_slot(self, n: int) -> L.TelemetryFrame | None:
        off = L.FRAMES_OFFSET + ((n - 1) % L.RING_SLOTS) * ctypes.sizeof(L.TelemetryFrame)
        for _ in range(4):
            end = self.map.u64(off + _SEQ_END_OFFSET)
            frame = self.map.read_into(L.TelemetryFrame, off)
            begin = self.map.u64(off)
            if begin == end == n and frame.seq_begin == n and frame.seq_end == n:
                return frame
            self.torn_reads += 1
            if begin > n:  # the writer lapped this slot; caller should take a newer frame
                return None
        return None

    def latest_seq(self) -> int:
        return self.map.u64(_FRAME_COUNTER_OFFSET)

    def read_latest(self) -> L.TelemetryFrame | None:
        n = self.latest_seq()
        if n == 0:
            return None
        if n < self.last_seq:  # DLL re-initialised (sdk reload / game restart)
            self.resyncs += 1
            self.last_seq = 0
            self.last_event_seq = 0
        frame = self._read_slot(n)
        if frame is not None:
            self.last_seq = n
        return frame

    def wait_frame(self, timeout_ms: int = 100) -> L.TelemetryFrame | None:
        """Block until a frame newer than the last one returned, or timeout. Returns None on timeout."""
        deadline = qpc() + self.qpc_freq * timeout_ms // 1000
        while True:
            n = self.latest_seq()
            if n != self.last_seq and n != 0:
                frame = self.read_latest()
                if frame is not None:
                    return frame
            remaining_ms = (deadline - qpc()) * 1000 // self.qpc_freq
            if remaining_ms <= 0:
                return None
            # Short waits: another reader may consume the auto-reset event.
            self.event.wait(int(min(2, remaining_ms)))

    def frame_age_ms(self, frame: L.TelemetryFrame) -> float:
        return (qpc() - frame.publish_qpc) * 1000.0 / self.qpc_freq

    # ------------------------------------------------------------- config / events
    def read_config(self) -> L.ConfigBlock | None:
        for _ in range(8):
            s1 = self.map.u64(L.CONFIG_OFFSET)
            if s1 & 1:
                continue
            cfg = self.map.read_into(L.ConfigBlock, L.CONFIG_OFFSET)
            if self.map.u64(L.CONFIG_OFFSET) == s1 and cfg.config_seq == s1:
                return cfg
        return None

    def poll_events(self) -> list[GameplayEvent]:
        hdr_w = self.map.u64(_EVENT_WRITE_OFFSET)
        if hdr_w < self.last_event_seq:
            self.last_event_seq = 0
        out: list[GameplayEvent] = []
        first = max(self.last_event_seq + 1, hdr_w - L.EVENT_SLOTS + 1, 1)
        size = ctypes.sizeof(L.EventSlot)
        for n in range(first, hdr_w + 1):
            off = L.EVENTS_OFFSET + ((n - 1) % L.EVENT_SLOTS) * size
            slot = self.map.read_into(L.EventSlot, off)
            if slot.seq != n or self.map.u64(off) != n:
                continue  # overwritten or being written; skip
            attrs = {}
            for part in text(slot.attrs).split(";"):
                k, sep, v = part.partition("=")
                if sep:
                    attrs[k] = v
            out.append(GameplayEvent(n, text(slot.id), slot.game_time, attrs))
        self.last_event_seq = hdr_w
        return out
