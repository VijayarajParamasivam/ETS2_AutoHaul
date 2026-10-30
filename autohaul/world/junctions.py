"""Junction geometry: where the paths through a junction (prefab) cross or merge with each other.

A junction path ("curve") conflicts with another path of the same junction where the two cross at the same
height, or where both lead into the same lane (a merge: a turn joining a through lane, a roundabout entry
joining the circle). Junctions whose paths cross (town crossroads, roundabouts) are intersections; paths that
only split or merge (motorway exits, interchanges, lane splits) or pass over each other (flyovers) aren't.
Who gives way at each conflict comes from the map's junction rules (autohaul.world.rules).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from autohaul.world.lanemap import KIND_PREFAB, LaneMap


def _first_crossing(a: np.ndarray, b: np.ndarray) -> tuple[int, float, int, float] | None:
    """The first proper intersection along polyline a (n,2) with polyline b (m,2): segment index and
    fraction on each (i, t, j, u), or None."""
    p, r = a[:-1, None, :], (a[1:] - a[:-1])[:, None, :]
    q, s = b[None, :-1, :], (b[1:] - b[:-1])[None, :, :]
    rxs = r[..., 0] * s[..., 1] - r[..., 1] * s[..., 0]
    qp = q - p
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (qp[..., 0] * s[..., 1] - qp[..., 1] * s[..., 0]) / rxs
        u = (qp[..., 0] * r[..., 1] - qp[..., 1] * r[..., 0]) / rxs
    # half-open intervals: a crossing exactly at a shared vertex counts once; parallel overlaps don't count
    hit = (np.abs(rxs) > 1e-9) & (t >= 0) & (t < 1) & (u >= 0) & (u < 1)
    if not hit.any():
        return None
    ii, jj = np.nonzero(hit)
    k = int(np.lexsort((t[ii, jj], ii))[0])
    i, j = int(ii[k]), int(jj[k])
    return i, float(t[i, j]), j, float(u[i, j])


def _zone_entry(pts: np.ndarray, d: np.ndarray, s_conf: float, other: np.ndarray, clearance: float) -> float:
    """Walking back along polyline `pts` (cumulative length `d`) from `s_conf`: where it is first `clearance`
    metres from polyline `other`, i.e. where the shared conflict zone starts. 0 if never (the lane start)."""
    a, b = other[:-1], other[1:]
    ab = b - a
    l2 = np.maximum((ab * ab).sum(axis=1), 1e-9)
    for i in np.flatnonzero(d <= s_conf)[::-1]:
        t = np.clip(((pts[i] - a) * ab).sum(axis=1) / l2, 0.0, 1.0)
        if np.hypot(*(a + t[:, None] * ab - pts[i]).T).min() >= clearance:
            return float(d[i])
    return 0.0


@dataclass(frozen=True)
class Conflict:
    other: int          # the other path (lane) of the same junction
    kind: str           # "cross" or "merge"
    s_mine: float       # metres along this path from its start to the conflict point
    s_other: float      # ... and along the other path
    x: float
    y: float
    zone_mine: float = 0.0    # metres along this path where the conflict zone starts (paths `clearance` apart)
    zone_other: float = 0.0   # ... and along the other path


class Junctions:
    def __init__(self, lm: LaneMap, end_margin_m: float = 3.0, level_m: float = 3.0, clearance_m: float = 3.0):
        self.lm = lm
        self.end_margin = end_margin_m
        self.level = level_m                # paths crossing this much higher or lower pass over each other
        self.clearance = clearance_m        # centre lines this far apart: a truck and a car pass side by side
        prefab = np.flatnonzero(lm.lane_kind == KIND_PREFAB)
        items = lm.lane_item[prefab]
        order = np.argsort(items, kind="stable")
        self._lanes = prefab[order]
        self._items = items[order]
        self._cache: dict[int, list[Conflict]] = {}
        self._items_cache: dict[int, bool] = {}
        self._geom: dict[int, dict] = {}             # per junction: its paths' trimmed lines and bounding boxes
        self._succ: dict[int, set[int]] = {}

    def lanes_of(self, item: int) -> np.ndarray:
        lo = np.searchsorted(self._items, np.uint64(item), "left")
        hi = np.searchsorted(self._items, np.uint64(item), "right")
        return self._lanes[lo:hi]

    def _trimmed(self, lane: int) -> tuple[np.ndarray, int, np.ndarray]:
        """The lane's polyline without its first/last `end_margin` metres (where curves merge or split), the
        index of its first kept point, and the full polyline's cumulative length."""
        pts = self.lm.lane_points(lane)
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        if d[-1] <= 2 * self.end_margin:
            return pts[:0], 0, d
        keep = (d >= self.end_margin) & (d <= d[-1] - self.end_margin)
        return pts[keep], int(np.argmax(keep)), d

    def _item_geom(self, item: int) -> dict:
        """The junction's paths, trimmed, with bounding boxes, so pairs that can't touch are skipped: on a big
        interchange (100-140 paths) working through every pair blocked the driving loop for 2-4 s the first time it
        came into range (2026-10-30: blind for 66 m at 79 km/h)."""
        g = self._geom.get(item)
        if g is None:
            lanes = self.lanes_of(item).tolist()
            trimmed = {ln: self._trimmed(ln) for ln in lanes}
            usable = [ln for ln in lanes if len(trimmed[ln][0]) >= 2]
            box = np.array([[*trimmed[ln][0].min(axis=0), *trimmed[ln][0].max(axis=0)] for ln in usable]).reshape(-1, 4)
            g = self._geom[item] = {"trimmed": trimmed, "usable": usable, "box": box,
                                    "index": {ln: i for i, ln in enumerate(usable)}}
        return g

    def _near_paths(self, g: dict, lane: int) -> list[int]:
        """The junction's other paths whose bounding box overlaps this one's."""
        i = g["index"].get(lane)
        if i is None:
            return []
        box = g["box"]
        hit = (box[:, 0] <= box[i, 2]) & (box[:, 2] >= box[i, 0]) & (box[:, 1] <= box[i, 3]) & (box[:, 3] >= box[i, 1])
        hit[i] = False
        usable = g["usable"]
        return [usable[k] for k in np.flatnonzero(hit).tolist()]

    def _successors(self, lane: int) -> set[int]:
        s = self._succ.get(lane)
        if s is None:
            s = self._succ[lane] = {c for c, _ in self.lm.successors(lane)}
        return s

    def _crossing(self, g: dict, lane: int, other: int):
        """Where `lane` first crosses `other` at the same height: (i, t, j, u, k0, k1), or None."""
        mine, k0, _ = g["trimmed"][lane]
        theirs, k1, _ = g["trimmed"][other]
        hit = _first_crossing(mine, theirs)
        if hit is None:
            return None
        i, t, j, u = hit
        if abs(self._height(lane, k0 + i, t) - self._height(other, k1 + j, u)) > self.level:
            return None                                     # a flyover, not a crossing
        return i, t, j, u, k0, k1

    def _height(self, lane: int, seg: int, t: float) -> float:
        z0, z1 = self.lm.seg[self.lm.lane_first[lane] + seg, 4:6]
        return float(z0 + t * (z1 - z0))

    def conflicts(self, lane: int) -> list[Conflict]:
        """Other paths of the same junction that cross this one at the same height or merge with it (cached).
        [] for road lanes."""
        if lane in self._cache:
            return self._cache[lane]
        out: list[Conflict] = []
        lm = self.lm
        if lm.lane_kind[lane] == KIND_PREFAB:
            item = int(lm.lane_item[lane])
            g = self._item_geom(item)
            mine, k0, d0 = g["trimmed"][lane]
            others = [o for o in self.lanes_of(item).tolist() if o != lane]
            if len(mine) >= 2:
                for other in self._near_paths(g, lane):
                    found = self._crossing(g, lane, other)
                    if found is None:
                        continue
                    i, t, j, u, k0, k1 = found
                    d1 = g["trimmed"][other][2]
                    x, y = mine[i] + t * (mine[i + 1] - mine[i])
                    s_mine = d0[k0 + i] + t * (d0[k0 + i + 1] - d0[k0 + i])
                    s_other = d1[k1 + j] + u * (d1[k1 + j + 1] - d1[k1 + j])
                    out.append(self._with_zone(Conflict(other, "cross", float(s_mine), float(s_other),
                                                        float(x), float(y)), lane))
            # merges: both lead into the same lane, from different places
            nxt = self._successors(lane)
            if nxt:
                start, end = lm.seg[lm.lane_first[lane], 0:2], lm.seg[lm.lane_last[lane], 2:4]
                for other in others:
                    if not nxt & self._successors(other):
                        continue
                    if np.hypot(*(lm.seg[lm.lane_first[other], 0:2] - start)) < 2.0:
                        continue                                    # same start: an alternative, not a merge
                    out.append(self._with_zone(Conflict(other, "merge", lm.lane_length(lane), lm.lane_length(other),
                                                        float(end[0]), float(end[1])), lane))
        self._cache[lane] = out
        return out

    def _with_zone(self, c: Conflict, lane: int) -> Conflict:
        a, b = self.lm.lane_points(lane), self.lm.lane_points(c.other)
        da = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(a, axis=0).T))])
        db = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(b, axis=0).T))])
        return Conflict(c.other, c.kind, c.s_mine, c.s_other, c.x, c.y,
                        _zone_entry(a, da, c.s_mine, b, self.clearance),
                        _zone_entry(b, db, c.s_other, a, self.clearance))

    def crossings(self, lane: int) -> list[int]:
        """Other paths of the same junction that cross this one at the same height (cached). [] for road lanes."""
        return [c.other for c in self.conflicts(lane) if c.kind == "cross"]

    def is_intersection(self, item: int) -> bool:
        """A junction with crossing paths (crossroads, roundabout). Traffic lights there can stop even the
        curves that don't cross, so the whole junction counts."""
        item = int(item)
        if item not in self._items_cache:
            g = self._item_geom(item)
            self._items_cache[item] = any(self._crossing(g, ln, other) is not None
                                          for ln in g["usable"] for other in self._near_paths(g, ln))
        return self._items_cache[item]
