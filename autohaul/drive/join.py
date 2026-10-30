"""Getting onto the lane map from off it: out of a company yard's parking spot, a fuel station's apron, a lay-by.

From the truck's pose, a smooth curve (cubic Hermite, the same shape the map's roads use) to a point on a lane
within `max_dist_m` that runs roughly the way the truck can go: the point ahead of the truck, the lane's direction
at most `max_turn_deg` from the truck's, the curve no tighter than `min_radius_m`. Among those, the shortest
curve to a lane with a route to the destination wins. The autopilot drives the curve slowly, then the lane.

Only the map's lanes are known: buildings, fences and parked trailers in the yard are not. Supervise it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from autohaul.world.lanemap import LaneMap, hermite


@dataclass
class Join:
    pts: np.ndarray            # (n, 2): the curve from the truck, then the lane onwards
    lane: int                  # the lane joined
    s: float                   # metres along that lane where the curve meets it
    length: float              # metres of the curve itself
    route: object = None       # Route from the join point, when there is a destination


def _curve(x, y, rot, px, py, prot):
    d = math.hypot(px - x, py - y)
    pts = hermite((x, y), (px, py), rot, prot, max(8, int(d / 2)))
    seg = np.hypot(*np.diff(pts, axis=0).T)
    h = np.unwrap(np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0])))
    k = np.abs(np.diff(h)) / np.maximum((seg[1:] + seg[:-1]) / 2, 1e-6) if len(h) > 1 else np.zeros(1)
    return pts, float(seg.sum()), float(k.max()) if len(k) else 0.0


def plan_join(lm: LaneMap, router, x: float, y: float, rot: float, destination=None, goal=None,
              no_go=None, max_dist_m: float = 60.0, max_turn_deg: float = 100.0, min_radius_m: float = 9.0,
              ahead_m: float = 150.0, prefer=None) -> Join | None:
    """`prefer(lane)`: lanes tried first, whatever the curve length: a company entrance's lanes (2026-10-29: in a
    Paris yard the shortest curve led to the road behind the fence, and the truck headed for the fence)."""
    seen = set()
    cands = []
    for dx in (-40.0, 0.0, 40.0):
        for dy in (-40.0, 0.0, 40.0):
            for h in lm.nearest(x + dx, y + dy, None, max_dist=64.0, k=40):
                if h.lane in seen or (no_go is not None and no_go[h.lane]):
                    continue
                seen.add(h.lane)
                cands.append(h.lane)
    found = []                                              # (curve length, lane, s, curve points), best per lane
    for lane in cands:
        p = lm.lane_points(lane)
        if len(p) < 2:
            continue
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
        best = None
        for i in range(len(p) - 1):
            px, py = p[i + 1]
            dist = math.hypot(px - x, py - y)
            if dist > max_dist_m or dist < 6.0:
                continue
            if (px - x) * math.cos(rot) + (py - y) * math.sin(rot) < 3.0:
                continue                                    # behind or beside the truck
            prot = math.atan2(p[i + 1, 1] - p[i, 1], p[i + 1, 0] - p[i, 0])
            if abs(math.degrees(math.atan2(math.sin(prot - rot), math.cos(prot - rot)))) > max_turn_deg:
                continue
            pts, length, kmax = _curve(x, y, rot, px, py, prot)
            if kmax <= 1.0 / min_radius_m and (best is None or length < best[0]):
                best = (length, lane, float(d[i + 1]), pts)
        if best is not None:
            found.append(best)
    if prefer is not None and any(prefer(ln) for ln in cands) and not any(prefer(f[1]) for f in found):
        # a company entrance is right here but not ahead: the way out is round the yard, which isn't mapped.
        # Any other lane would be across the yard's fence (2026-10-29): don't guess
        return None
    found.sort(key=lambda f: (0 if prefer is not None and prefer(f[1]) else 1, f[0]))
    for length, lane, s, pts in found[:8]:
        rt = None
        if destination is not None and router is not None:
            rt = router.route(lane, s, destination, goal=goal)
            if rt is None:
                continue                                    # that lane leads nowhere useful: try the next
        tail = _ahead(lm, lane, s, rt, ahead_m)
        return Join(np.vstack([pts, tail[1:]]) if len(tail) > 1 else pts, lane, s, length, rt)
    return None


def _ahead(lm: LaneMap, lane: int, s: float, route, metres: float) -> np.ndarray:
    """The lane from `s` onwards (and the route's next lanes), `metres` long."""
    out, acc, cur, start = [], 0.0, lane, s
    while acc < metres and cur is not None:
        p = lm.lane_points(cur)
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
        keep = p[d >= start]
        if out and len(keep) and np.hypot(*(keep[0] - out[-1][-1])) < 0.05:
            keep = keep[1:]
        if len(keep):
            out.append(keep)
        acc += float(d[-1] - start)
        start = 0.0
        nxt = route.next_after(cur) if route is not None else None
        if nxt is None:
            succ = lm.successors(cur)
            nxt = succ[0][0] if succ else None
        cur = nxt
    return np.vstack(out) if out else np.zeros((0, 2))
