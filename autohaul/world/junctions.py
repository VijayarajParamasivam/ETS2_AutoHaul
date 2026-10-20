"""Junction geometry: does a path through a junction (prefab) cross another path of the same junction?

Until the map extraction exports traffic rules (give way / stop / which traffic light controls a curve),
AutoHaul treats every junction curve that crosses another curve of the same prefab as a conflict point it
must not drive into by itself (town crossroads, roundabouts, anything with lights or give-way). Curves that
only split from or merge with others (motorway exits, interchanges, lane splits) have no crossing.
"""

from __future__ import annotations

import numpy as np

from autohaul.world.lanemap import KIND_PREFAB, LaneMap


def _segments_cross(a: np.ndarray, b: np.ndarray) -> bool:
    """Do any segments of polyline a (n,2) properly intersect any of polyline b (m,2)?"""
    p, r = a[:-1, None, :], (a[1:] - a[:-1])[:, None, :]
    q, s = b[None, :-1, :], (b[1:] - b[:-1])[None, :, :]
    rxs = r[..., 0] * s[..., 1] - r[..., 1] * s[..., 0]
    qp = q - p
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (qp[..., 0] * s[..., 1] - qp[..., 1] * s[..., 0]) / rxs
        u = (qp[..., 0] * r[..., 1] - qp[..., 1] * r[..., 0]) / rxs
    # half-open intervals: a crossing exactly at a shared vertex counts once; parallel overlaps don't count
    hit = (np.abs(rxs) > 1e-9) & (t >= 0) & (t < 1) & (u >= 0) & (u < 1)
    return bool(hit.any())


class Junctions:
    def __init__(self, lm: LaneMap, end_margin_m: float = 3.0):
        self.lm = lm
        self.end_margin = end_margin_m
        prefab = np.flatnonzero(lm.lane_kind == KIND_PREFAB)
        items = lm.lane_item[prefab]
        order = np.argsort(items, kind="stable")
        self._lanes = prefab[order]
        self._items = items[order]
        self._cache: dict[int, list[int]] = {}
        self._items_cache: dict[int, bool] = {}

    def lanes_of(self, item: int) -> np.ndarray:
        lo = np.searchsorted(self._items, np.uint64(item), "left")
        hi = np.searchsorted(self._items, np.uint64(item), "right")
        return self._lanes[lo:hi]

    def _trimmed(self, lane: int) -> np.ndarray:
        """The lane's polyline without its first/last `end_margin` metres (where curves merge or split)."""
        pts = self.lm.lane_points(lane)
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        if d[-1] <= 2 * self.end_margin:
            return pts[:0]
        keep = (d >= self.end_margin) & (d <= d[-1] - self.end_margin)
        return pts[keep]

    def crossings(self, lane: int) -> list[int]:
        """Other curves of the same junction that cross this curve (cached). [] for road lanes."""
        if lane in self._cache:
            return self._cache[lane]
        out: list[int] = []
        if self.lm.lane_kind[lane] == KIND_PREFAB:
            mine = self._trimmed(lane)
            if len(mine) >= 2:
                for other in self.lanes_of(int(self.lm.lane_item[lane])).tolist():
                    if other == lane:
                        continue
                    theirs = self._trimmed(other)
                    if len(theirs) >= 2 and _segments_cross(mine, theirs):
                        out.append(other)
        self._cache[lane] = out
        return out

    def is_intersection(self, item: int) -> bool:
        """A junction with crossing paths (crossroads, roundabout). Traffic lights there can stop even the
        curves that don't cross, so AutoHaul treats the whole junction as one it must not enter by itself."""
        item = int(item)
        if item not in self._items_cache:
            self._items_cache[item] = any(self.crossings(ln) for ln in self.lanes_of(item).tolist())
        return self._items_cache[item]
