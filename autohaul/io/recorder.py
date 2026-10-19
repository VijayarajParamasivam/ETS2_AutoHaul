"""Replayable recordings (.ahrec): every telemetry frame, config change, gameplay event,
controls command and free-form note, with wall-clock timestamps.

File: b"AHREC1\\n", one JSON metadata line, then zlib-compressed chunks: [u32 length][data].
Inside a chunk, records are [u8 kind][f64 wall_time][u32 length][payload].
Frames / config / controls are raw struct bytes (layout identified by metadata.layout_hash);
events and notes are JSON.
"""

from __future__ import annotations

import ctypes
import json
import struct
import time
import zlib
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from autohaul.io import _layout as L

MAGIC = b"AHREC1\n"
KIND_FRAME = 1
KIND_CONFIG = 2
KIND_EVENT = 3
KIND_CONTROLS = 4
KIND_NOTE = 5
_REC = struct.Struct("<BdI")
_CHUNK = struct.Struct("<I")


def struct_fingerprint(cls) -> str:
    """Field names/types/offsets of a ctypes structure: recordings stay readable across layout
    changes that don't touch the recorded structures (e.g. new header counters)."""
    import hashlib
    desc = repr([(n, getattr(cls, n).offset, getattr(cls, n).size) for n, *_ in cls._fields_]) + str(ctypes.sizeof(cls))
    return hashlib.sha1(desc.encode()).hexdigest()[:16]


_RECORDED = {"frame": L.TelemetryFrame, "config": L.ConfigBlock, "controls": L.ControlsBlock}


class Recorder:
    def __init__(self, path: Path, meta: dict[str, Any] | None = None, flush_every: int = 120):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "wb")
        header = {"layout_version": L.LAYOUT_VERSION, "layout_hash": f"0x{L.LAYOUT_HASH:016x}",
                  "structs": {k: struct_fingerprint(c) for k, c in _RECORDED.items()},
                  "created": time.time(), **(meta or {})}
        self.f.write(MAGIC)
        self.f.write(json.dumps(header).encode() + b"\n")
        self.f.flush()  # a run that aborts early still leaves a readable file
        self.buf = bytearray()
        self.pending = 0
        self.flush_every = flush_every
        self.records = 0

    def _add(self, kind: int, payload: bytes) -> None:
        self.buf += _REC.pack(kind, time.time(), len(payload))
        self.buf += payload
        self.records += 1
        self.pending += 1
        if self.pending >= self.flush_every:
            self.flush()

    def frame(self, frame: L.TelemetryFrame) -> None:
        self._add(KIND_FRAME, bytes(frame))

    def config(self, cfg: L.ConfigBlock) -> None:
        self._add(KIND_CONFIG, bytes(cfg))

    def event(self, ev: Any) -> None:
        self._add(KIND_EVENT, json.dumps({"seq": ev.seq, "id": ev.id, "game_time": ev.game_time,
                                          "attrs": ev.attrs}).encode())

    def controls(self, raw: bytes) -> None:
        self._add(KIND_CONTROLS, raw)

    def note(self, **fields: Any) -> None:
        self._add(KIND_NOTE, json.dumps(fields, default=str).encode())

    def flush(self) -> None:
        if self.buf:
            data = zlib.compress(bytes(self.buf), 1)
            self.f.write(_CHUNK.pack(len(data)))
            self.f.write(data)
            self.f.flush()
            self.buf.clear()
            self.pending = 0

    def close(self) -> None:
        if not self.f.closed:
            self.flush()
            self.f.close()

    def __enter__(self) -> "Recorder":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


@dataclass
class Record:
    kind: int
    t: float
    payload: bytes

    def _decode(self, cls):
        if len(self.payload) != ctypes.sizeof(cls):
            raise ValueError(f"recorded {cls.__name__} is {len(self.payload)} bytes, this package's is "
                             f"{ctypes.sizeof(cls)}: recorded with an incompatible layout")
        return cls.from_buffer_copy(self.payload)

    def frame(self) -> L.TelemetryFrame:
        return self._decode(L.TelemetryFrame)

    def config(self) -> L.ConfigBlock:
        return self._decode(L.ConfigBlock)

    def controls(self) -> L.ControlsBlock:
        return self._decode(L.ControlsBlock)

    def json(self) -> dict[str, Any]:
        return json.loads(self.payload)


class RecordingReader:
    def __init__(self, path: Path):
        self.path = Path(path)
        with open(self.path, "rb") as f:
            if f.read(len(MAGIC)) != MAGIC:
                raise ValueError(f"{path}: not an AutoHaul recording")
            self.meta = json.loads(f.readline())
            self._data_start = f.tell()
        # A different overall layout is fine as long as the recorded structures are unchanged: exact
        # fingerprints when the file has them, otherwise (older files) exact sizes, checked per record.
        self.layout_mismatch = self.meta.get("layout_hash") != f"0x{L.LAYOUT_HASH:016x}"
        recorded = self.meta.get("structs")
        if self.layout_mismatch and recorded:
            bad = [k for k, c in _RECORDED.items() if recorded.get(k) != struct_fingerprint(c)]
            if bad:
                raise ValueError(f"{path}: recorded with layout {self.meta.get('layout_hash')}; its "
                                 f"{', '.join(bad)} structures differ from this package's")

    def __iter__(self) -> Iterator[Record]:
        with open(self.path, "rb") as f:
            f.seek(self._data_start)
            while True:
                head = f.read(_CHUNK.size)
                if len(head) < _CHUNK.size:
                    return
                (n,) = _CHUNK.unpack(head)
                data = f.read(n)
                if len(data) < n:
                    return  # truncated final chunk (crash mid-write)
                chunk = zlib.decompress(data)
                pos = 0
                while pos < len(chunk):
                    kind, t, size = _REC.unpack_from(chunk, pos)
                    pos += _REC.size
                    yield Record(kind, t, chunk[pos:pos + size])
                    pos += size

    def frames(self) -> Iterator[tuple[float, L.TelemetryFrame]]:
        for r in self:
            if r.kind == KIND_FRAME:
                yield r.t, r.frame()

