"""Lane connection graph over the LaneMap, for routing.

For every lane:
  successors   lanes that continue it (their start within 2.5 m of its end, turning at most 40 degrees),
               sorted by turn angle - the same rule as LaneMap.successors, computed for all lanes at once
  neighbours   parallel lanes of the same road in the same direction (one lane-change step left/right)
Stored as CSR arrays and cached next to the lane map (`lanes_graph.npz`), keyed by the lane map's build time.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from autohaul.world.lanemap import KIND_ROAD, LaneMap, wrap

GRAPH_VERSION = 2
MAX_GAP_M = 2.5          # lane-map seams up to ~1.6 m exist (Amsterdam); a lane width is 4.5 m
MAX_TURN = math.radians(40)


@dataclass
class LaneGraph:
    succ_ptr: np.ndarray      # (L+1,) int64
    succ_idx: np.ndarray      # (E,) int32, successors of lane i are succ_idx[succ_ptr[i]:succ_ptr[i+1]]
    succ_turn: np.ndarray     # (E,) float32, turn angle at the join (rad, + = right)
    nbr_ptr: np.ndarray       # (L+1,) int64
    nbr_idx: np.ndarray       # (N,) int32, parallel same-direction lanes of the same road
    length: np.ndarray        # (L,) float32 lane length (m)
    meta: dict
    _pred: tuple | None = field(default=None, init=False, repr=False, compare=False)

    def successors(self, lane: int) -> np.ndarray:
        return self.succ_idx[self.succ_ptr[lane]:self.succ_ptr[lane + 1]]

    def predecessors(self, lane: int) -> np.ndarray:
        """Lanes that lead into `lane` (the reverse links, built on first use)."""
        if self._pred is None:
            src = np.repeat(np.arange(len(self.length), dtype=np.int32), np.diff(self.succ_ptr))
            order = np.argsort(self.succ_idx, kind="stable")
            ptr = np.searchsorted(self.succ_idx[order], np.arange(len(self.length) + 1)).astype(np.int64)
            self._pred = (ptr, src[order])
        ptr, idx = self._pred
        return idx[ptr[lane]:ptr[lane + 1]]

    def neighbours(self, lane: int) -> np.ndarray:
        return self.nbr_idx[self.nbr_ptr[lane]:self.nbr_ptr[lane + 1]]

    # ------------------------------------------------------------- build
    @classmethod
    def build(cls, lm: LaneMap, log=print) -> "LaneGraph":
        t0 = time.time()
        n = len(lm.lane_kind)
        first, last = lm.lane_first, lm.lane_last
        sx, sy, srot = lm.seg[first, 0], lm.seg[first, 1], lm.seg_rot[first]
        ex, ey, erot = lm.seg[last, 2], lm.seg[last, 3], lm.seg_rot[last]
        cell = 2.0
        off = 1 << 22

        def key(cx, cy):
            return (cx + off) * (1 << 23) + (cy + off)

        skey = key(np.floor(sx / cell).astype(np.int64), np.floor(sy / cell).astype(np.int64))
        order = np.argsort(skey, kind="stable")
        sk = skey[order]
        ecx, ecy = np.floor(ex / cell).astype(np.int64), np.floor(ey / cell).astype(np.int64)
        lanes = np.arange(n)
        pa, pb = [], []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = key(ecx + dx, ecy + dy)
                lo = np.searchsorted(sk, k, "left")
                cnt = np.searchsorted(sk, k, "right") - lo
                has = cnt > 0
                c = cnt[has]
                if not len(c):
                    continue
                a = np.repeat(lanes[has], c)
                within = np.arange(c.sum()) - np.repeat(np.cumsum(c) - c, c)
                pa.append(a)
                pb.append(order[np.repeat(lo[has], c) + within])
        a = np.concatenate(pa) if pa else np.zeros(0, np.int64)
        b = np.concatenate(pb) if pb else np.zeros(0, np.int64)
        gap = np.hypot(sx[b] - ex[a], sy[b] - ey[a])
        turn = wrap(srot[b] - erot[a])
        ok = (a != b) & (gap <= MAX_GAP_M) & (np.abs(turn) <= MAX_TURN)
        a, b, turn = a[ok], b[ok], turn[ok]
        o = np.lexsort((np.abs(turn), a))
        a, b, turn = a[o], b[o], turn[o]
        succ_ptr = np.searchsorted(a, np.arange(n + 1)).astype(np.int64)

        # parallel lanes: same road item, same direction (sign of sub), adjacent |sub|
        road = np.flatnonzero(lm.lane_kind == KIND_ROAD)
        slot = {}
        for ln in road.tolist():
            sub = int(lm.lane_sub[ln])
            slot[(int(lm.lane_item[ln]), sub)] = ln
        nbr = [[] for _ in range(n)]
        for (item, sub), ln in slot.items():
            step = 1 if sub > 0 else -1
            for other in (sub + step, sub - step):
                if other != 0 and (other > 0) == (sub > 0):
                    m = slot.get((item, other))
                    if m is not None:
                        nbr[ln].append(m)
        counts = np.array([len(x) for x in nbr], dtype=np.int64)
        nbr_ptr = np.concatenate([[0], np.cumsum(counts)]).astype(np.int64)
        nbr_idx = np.array([m for x in nbr for m in x], dtype=np.int32)

        length = np.zeros(n, dtype=np.float64)
        np.add.at(length, lm.seg_lane, lm.seg_len)
        meta = {"graph_version": GRAPH_VERSION, "lanes_built": lm.meta.get("built"), "edges": int(len(b)),
                "lane_changes": int(len(nbr_idx)), "build_s": round(time.time() - t0, 1)}
        log(f"lane graph: {len(b)} links, {len(nbr_idx)} lane-change pairs in {meta['build_s']} s")
        return cls(succ_ptr, b.astype(np.int32), turn.astype(np.float32), nbr_ptr, nbr_idx,
                   length.astype(np.float32), meta)

    # ------------------------------------------------------------- persistence
    def save(self, path: Path) -> None:
        np.savez(path, succ_ptr=self.succ_ptr, succ_idx=self.succ_idx, succ_turn=self.succ_turn,
                 nbr_ptr=self.nbr_ptr, nbr_idx=self.nbr_idx, length=self.length,
                 meta=np.frombuffer(json.dumps(self.meta).encode(), dtype=np.uint8))

    @classmethod
    def load(cls, path: Path) -> "LaneGraph":
        z = np.load(path)
        meta = json.loads(bytes(z["meta"]).decode())
        return cls(z["succ_ptr"], z["succ_idx"], z["succ_turn"], z["nbr_ptr"], z["nbr_idx"], z["length"], meta)

    @classmethod
    def load_or_build(cls, lm: LaneMap, map_dir: Path, log=print) -> "LaneGraph":
        path = Path(map_dir) / "lanes_graph.npz"
        if path.exists():
            g = cls.load(path)
            if g.meta.get("graph_version") == GRAPH_VERSION and g.meta.get("lanes_built") == lm.meta.get("built"):
                return g
            log("lane graph is out of date; rebuilding")
        g = cls.build(lm, log=log)
        g.save(path)
        return g
