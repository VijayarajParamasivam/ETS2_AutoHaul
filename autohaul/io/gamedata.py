"""Reader for the shared-memory blocks of the pinned ETS2LA game-memory plugin (1.53 pin c328dc38).

Layouts come from native/spec/ets2la_c328dc38.toml. These blocks have no sequence numbers or
timestamps, so freshness is judged by change detection in the caller (see tools/selftest_gamedata.py).
"""

from __future__ import annotations

import struct
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from autohaul.io.win32 import ReadOnlyMapping

SPEC = Path(__file__).resolve().parents[2] / "native/spec/ets2la_c328dc38.toml"

TRAFFIC_LIGHT = 1
GATE = 2
LIGHT_STATES = {0: "off", 1: "orange_to_red", 2: "red", 4: "orange_to_green", 8: "green", 32: "sleep"}
GATE_STATES = {0: "closing", 1: "closed", 2: "opening", 3: "open"}


@dataclass
class Block:
    name: str
    mapping: str
    record: struct.Struct
    count: int
    fields: list[str]
    world_from_sector: bool
    trailer: struct.Struct | None = None
    trailer_fields: list[str] = field(default_factory=list)
    trailers_per_vehicle: int = 0

    @property
    def stride(self) -> int:
        return self.record.size + (self.trailer.size * self.trailers_per_vehicle if self.trailer else 0)

    @property
    def size(self) -> int:
        return self.stride * self.count


def load_spec(path: Path = SPEC) -> dict[str, Block]:
    raw = tomllib.loads(path.read_text(encoding="utf-8"))
    blocks = {}
    for name, b in raw.items():
        blocks[name] = Block(
            name=name, mapping=b["mapping"], record=struct.Struct(b["record"]), count=b["count"],
            fields=b["fields"], world_from_sector=b["world_from_sector"],
            trailer=struct.Struct(b["trailer"]) if "trailer" in b else None,
            trailer_fields=b.get("trailer_fields", []), trailers_per_vehicle=b.get("trailers_per_vehicle", 0),
        )
    return blocks


class GameDataReader:
    """Reads all four ETS2LA blocks. Each read_* returns None when the block doesn't exist."""

    def __init__(self, spec: dict[str, Block] | None = None):
        self.blocks = spec or load_spec()
        self.maps = {name: ReadOnlyMapping(b.mapping, b.size) for name, b in self.blocks.items()}

    def available(self) -> dict[str, bool]:
        return {name: m.open() for name, m in self.maps.items()}

    def raw(self, name: str) -> bytes | None:
        m = self.maps[name]
        if not m.open():
            return None
        return m.read(0, self.blocks[name].size)

    def _records(self, name: str) -> list[dict] | None:
        data = self.raw(name)
        if data is None:
            return None
        b = self.blocks[name]
        out = []
        for i in range(b.count):
            base = i * b.stride
            rec = dict(zip(b.fields, b.record.unpack_from(data, base)))
            if b.world_from_sector:
                rec["wx"] = rec["x"] + 512.0 * rec["cx"]
                rec["wz"] = rec["z"] + 512.0 * rec["cz"]
            elif "x" in rec:
                rec["wx"], rec["wz"] = rec["x"], rec["z"]
            if b.trailer:
                rec["trailers"] = [
                    dict(zip(b.trailer_fields, b.trailer.unpack_from(data, base + b.record.size + t * b.trailer.size)))
                    for t in range(b.trailers_per_vehicle)
                ]
            out.append(rec)
        return out

    def camera(self) -> dict | None:
        recs = self._records("camera")
        return recs[0] if recs else None

    def traffic(self) -> list[dict] | None:
        recs = self._records("traffic")
        if recs is None:
            return None
        # Empty slots are all zeros.
        return [r for r in recs if r["width"] > 0 or r["length"] > 0]

    def semaphores(self) -> list[dict] | None:
        recs = self._records("semaphore")
        if recs is None:
            return None
        live = [r for r in recs if r["type"] in (TRAFFIC_LIGHT, GATE)]
        for r in live:
            states = LIGHT_STATES if r["type"] == TRAFFIC_LIGHT else GATE_STATES
            r["state_name"] = states.get(r["state"], f"unknown({r['state']})")
        return live

    def route(self) -> list[dict] | None:
        recs = self._records("route")
        if recs is None:
            return None
        return [r for r in recs if r["node_uid"] != 0]

    def close(self) -> None:
        for m in self.maps.values():
            m.close()
