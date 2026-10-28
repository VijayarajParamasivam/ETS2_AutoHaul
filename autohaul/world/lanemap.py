"""Lane-level map built from the TruckSim Maps parser output (game 1.53.2.1).

Coordinates: the map plane is (x, y) = game (X, Z); z = game Y (height). Directions are angles in that
plane: direction = (cos r, sin r), so north (game -Z) is r = -pi/2.

Every drivable lane becomes a polyline of short segments:
  * road lanes: cubic Hermite curve between the road's start/end nodes (tangent = node rotation x chord
    length), offset sideways per lane. Each road end's lanes sit where the junction (prefab) lanes at that
    node start and end (road_end_offsets), carried along chains of roads without a junction; elsewhere, where
    that road look's lanes sit, calibrated from all its junction connections (calibrate_looks), or by rule
    (rule_layout). Road lanes are only linked to junction lanes by position, so a lane a few metres off
    breaks the link: the router then has to go round (2026-10-24: 41 world km instead of ~23 to Paris);
  * prefab lanes (junctions, roundabouts, yards): the prefab description's AI nav curves, placed in the
    world by the prefab's origin node.
Segments are stored in flat numpy arrays and indexed by a uniform grid for fast nearest-lane queries.
Build once with `LaneMap.build(raw_dir)`; it caches to `<map dir>/lanes.npz`.
"""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

LANE_WIDTH = 4.5          # m, standard SCS lane
SAMPLE_STEP = 6.0         # m between samples on gentle curves
GRID_CELL = 64.0          # m
CACHE_VERSION = 9          # 9: seams compare the offsets actually used; 8: road-to-road seams blended; 7: small end corrections blend in over 40 m; 6: road-end evidence wins over
                           # the look's layout; 5: road ends placed on the
                           # junction lanes they meet;
                           # 4: "5m offset" looks put their lanes 5 m out; 3: hidden (non-drivable) roads and prefabs left out; 2: calibrated look layouts

KIND_ROAD = 0
KIND_PREFAB = 1


def heading_to_rotation(h: float) -> float:
    """SDK heading (0..1, CCW from north/-Z) -> map rotation (radians, direction = (cos, sin) in X/Z)."""
    return -math.pi / 2 - 2 * math.pi * h


def quat_forward_rotation(qw: float, qx: float, qy: float, qz: float) -> float:
    """Map rotation of a game object's forward axis (-Z in object space) for a unit quaternion."""
    # v' = q v q*, v = (0, 0, -1)
    fx = -(2 * (qx * qz + qw * qy))
    fz = -(1 - 2 * (qx * qx + qy * qy))
    return math.atan2(fz, fx)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def hermite(p0, p1, r0, r1, steps):
    """Sample a cubic Hermite curve (SCS road/nav-curve convention). Returns (steps+1, 2) points."""
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    m0 = (math.cos(r0) * dist, math.sin(r0) * dist)
    m1 = (math.cos(r1) * dist, math.sin(r1) * dist)
    t = np.linspace(0.0, 1.0, steps + 1)
    t2, t3 = t * t, t * t * t
    h00, h10, h01, h11 = 2 * t3 - 3 * t2 + 1, t3 - 2 * t2 + t, -2 * t3 + 3 * t2, t3 - t2
    x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
    y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
    return np.stack([x, y], axis=1)


def sample_steps(length: float, r0: float, r1: float) -> int:
    turn = abs(math.atan2(math.sin(r1 - r0), math.cos(r1 - r0)))
    return int(min(200, max(2, math.ceil(length / SAMPLE_STEP), math.ceil(turn / 0.05))))


def offset_polyline(pts: np.ndarray, d: float) -> np.ndarray:
    """Shift a polyline sideways by d metres (positive = to the right of travel direction)."""
    tang = np.gradient(pts, axis=0)
    norm = np.linalg.norm(tang, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    tang /= norm
    right = np.stack([-tang[:, 1], tang[:, 0]], axis=1)  # right of travel in the X/Z plane (Z points south)
    return pts + right * d


def rule_layout(look: dict) -> tuple[list[float], list[float]]:
    """Lane-centre offsets (m, + = right of the road's forward direction) when no prefab shows them.

    Measured on 1.53 prefab connections: two-way roads put the lanes `offset` m out from the centre line
    (for small offsets); one-way roads split their lanes around the line with ceil(n/2) on the left."""
    nl, nr = len(look.get("lanesLeft", [])), len(look.get("lanesRight", []))
    w = LANE_WIDTH
    if nl == 0 or nr == 0:
        n = nl + nr
        m = -w * math.ceil(n / 2)
        offs = [m + (i + 0.5) * w for i in range(n)]
        return (offs, []) if nr else ([], [-o for o in offs][::-1])
    off = look.get("offset") or 0.0
    m = off if off <= 3.0 else max(off - 4.5, 0.0)
    named = re.search(r"(\d+(?:\.\d+)?)m offset", look.get("name", ""))
    if named and float(named.group(1)) >= 5.0:
        # every "5m/10m offset" look with prefab evidence has its inner lane edge exactly there, whatever its
        # `offset` says; "Highway 2 lanes 5m offset" had its inner lane missing (you drove ~5 m off the map)
        m = float(named.group(1))
    return [m + (i + 0.5) * w for i in range(nr)], [-(m + (j + 0.5) * w) for j in range(nl)]


def fit_lanes(obs: np.ndarray, n: int) -> tuple[list[float], int] | None:
    """Fit n lane centres m + (i + 0.5) w to observed connection offsets (increasing = further right).
    Returns (offsets, support = observations within 0.2 m of a fitted lane)."""
    if n == 0 or len(obs) == 0:
        return None
    if len(obs) > 600:
        obs = np.random.default_rng(0).choice(obs, 600, replace=False)
    vals, counts = np.unique(np.round(obs, 1), return_counts=True)
    anchors = vals[np.argsort(-counts)[:12]]
    idx = np.arange(n) + 0.5
    best = None
    for w in (4.5, 4.0, 3.5, 3.0, 2.5):
        for v in anchors:
            for i in range(n):
                model = v - (i + 0.5) * w + idx * w
                d = np.abs(obs[:, None] - model[None, :]).min(axis=1)
                support = int((d < 0.2).sum())
                # a non-standard width must explain clearly more connections to win
                if best is None or support > best[1] + (0 if w == 4.5 else 0.02 * len(obs)):
                    best = (model, support)
    model, support = best
    # refine the shift by least squares over the supporting observations
    near = model[np.abs(obs[:, None] - model[None, :]).argmin(axis=1)]
    ok = np.abs(obs - near) < 0.2
    if ok.any():
        model = model + float(np.mean(obs[ok] - near[ok]))
    return [round(float(x), 3) for x in model], support


class PrefabEnds:
    """Where prefab (junction) lanes start and end, for lookups at road nodes."""

    def __init__(self, prefab_lanes: list):
        self.cell = cell = 16.0
        ends = np.array([(p[-1, 0], p[-1, 1], math.atan2(p[-1, 1] - p[-2, 1], p[-1, 0] - p[-2, 0]))
                         for p, *_ in prefab_lanes]).reshape(-1, 3)
        starts = np.array([(p[0, 0], p[0, 1], math.atan2(p[1, 1] - p[0, 1], p[1, 0] - p[0, 0]))
                           for p, *_ in prefab_lanes]).reshape(-1, 3)

        def grid(pts):
            cx = np.floor(pts[:, 0] / cell).astype(np.int64)
            cy = np.floor(pts[:, 1] / cell).astype(np.int64)
            keys = (cx + (1 << 20)) * (1 << 21) + (cy + (1 << 20))
            order = np.argsort(keys, kind="stable")
            uk, first = np.unique(keys[order], return_index=True)
            return pts[order], uk, first, np.append(first[1:], len(order))

        self.g_end, self.g_start = grid(ends), grid(starts)

    def lateral(self, g, x: float, y: float, rot: float, opposite: bool) -> np.ndarray:
        """Lateral offsets (the road layouts' sign) of the prefab lane ends in grid `g` that lie on the line across
        direction `rot` through (x, y), heading along it (or against it: `opposite`)."""
        cell = self.cell
        pts, uk, first, last = g
        cx, cy = int(math.floor(x / cell)), int(math.floor(y / cell))
        chunks = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = (cx + dx + (1 << 20)) * (1 << 21) + (cy + dy + (1 << 20))
                i = np.searchsorted(uk, k)
                if i < len(uk) and uk[i] == k:
                    chunks.append(pts[first[i]:last[i]])
        if not chunks:
            return np.empty(0)
        c = np.concatenate(chunks)
        ct, st = math.cos(rot), math.sin(rot)
        along = (c[:, 0] - x) * ct + (c[:, 1] - y) * st
        lat = (c[:, 0] - x) * -st + (c[:, 1] - y) * ct
        d = wrap(c[:, 2] - rot + (math.pi if opposite else 0.0))
        ok = (np.abs(along) < 1.0) & (np.abs(lat) < 30.0) & (np.abs(d) < math.radians(5))
        return lat[ok]

    def at_node(self, node) -> tuple[np.ndarray, np.ndarray]:
        """Prefab lane ends at a road node: offsets of the lanes running along the node's direction and of those
        running against it (where a road's forward and backward lanes must meet them)."""
        x, y, _, rot = node
        along = np.concatenate([self.lateral(self.g_end, x, y, rot, False),
                                self.lateral(self.g_start, x, y, rot, False)])
        against = np.concatenate([self.lateral(self.g_start, x, y, rot, True),
                                  self.lateral(self.g_end, x, y, rot, True)])
        return along, against


def snap_offsets(obs: np.ndarray, layout: list[float], max_shift: float = 3.0) -> list[float] | None:
    """The lane offsets a road end really has: the junction lane ends observed there (clustered), matched to the
    layout's lanes, in the layout's order.
      * same count: taken as they are, however far from the layout. A look's layout is a majority vote over
        all its roads and can be wrong for a given road: "Highway 3 lanes 2m offset" put the forward lanes on
        the left (264 road ends say so, 30 the opposite), which cut a Rotterdam motorway off the lane graph.
      * more observed (turn lanes, a parallel road): each layout lane (or each lane of the mirrored layout) takes
        the nearest unused one within `max_shift`.
    None when it doesn't add up."""
    n = len(layout)
    if n == 0 or len(obs) == 0:
        return None
    vals = np.sort(np.asarray(obs, float))
    groups = [[vals[0]]]
    for v in vals[1:]:
        if v - groups[-1][-1] < 0.6:
            groups[-1].append(v)
        else:
            groups.append([v])
    u = [float(np.mean(gr)) for gr in groups]
    if len(u) == n:
        if n == 1:
            return u
        side = layout[0] > layout[-1]                       # backward lanes: innermost (largest) first
        return sorted(u, reverse=side)
    if len(u) < n:
        return None
    best = None
    for lay in (layout, [-o for o in layout]):
        used, out, err = set(), [], 0.0
        for off in lay:
            k = min((i for i in range(len(u)) if i not in used), key=lambda i: abs(u[i] - off), default=None)
            if k is None or abs(u[k] - off) > max_shift:
                break
            used.add(k)
            out.append(u[k])
            err += abs(u[k] - off)
        else:
            if best is None or err < best[0]:
                best = (err, out)
    return best[1] if best else None


FAR_OFF_M = 6.0          # junction evidence further than this from a look's layout: the layout is wrong for that road
END_BLEND_M = 40.0       # otherwise the lanes move from the layout to the junction's lanes over this distance


def road_end_offsets(roads: list[dict], nodes: dict, layouts: dict, ends: PrefabEnds) -> dict:
    """Per road uid: {"a": (forward offsets, backward offsets, direct) or None, "b": ...}: the junction lanes at
    the node (direct), or carried along a chain of roads (two roads meeting at a node, no junction) from the
    nearest end that has them."""
    node_obs: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    at: dict[str, list[tuple[int, str]]] = {}               # node -> [(road index, "a"/"b")]
    for i, rd in enumerate(roads):
        at.setdefault(rd["startNodeUid"], []).append((i, "a"))
        at.setdefault(rd["endNodeUid"], []).append((i, "b"))
    key = {"a": "startNodeUid", "b": "endNodeUid"}
    val: dict[tuple[int, str], tuple[list | None, list | None]] = {}
    for i, rd in enumerate(roads):
        lay = layouts.get(rd["roadLookToken"])
        if lay is None:
            continue
        for end in ("a", "b"):
            uid = rd[key[end]]
            node = nodes.get(uid)
            if node is None:
                continue
            if uid not in node_obs:
                node_obs[uid] = ends.at_node(node)
            along, against = node_obs[uid]
            f, b = snap_offsets(along, lay[0]), snap_offsets(against, lay[1])
            if f is not None or b is not None:
                val[(i, end)] = (f, b)
    # carry along chains: an end without junction evidence takes its road's other end's offsets, and a node of
    # exactly two roads (no junction) passes them on to the next road
    frontier = list(val)
    carried: set[tuple[int, str]] = set()
    while frontier:
        i, end = frontier.pop()
        f, b = val[(i, end)]
        other = "b" if end == "a" else "a"
        node_uid = roads[i][key[other]]
        targets = [(i, other)]
        if len(at.get(node_uid, [])) == 2:
            targets += [(j, e) for j, e in at[node_uid] if j != i]
        for j, e in targets:
            if (j, e) in val:
                continue
            lay = layouts.get(roads[j]["roadLookToken"])
            if lay is None or (f is not None and len(f) != len(lay[0])) or (b is not None and len(b) != len(lay[1])):
                continue
            val[(j, e)] = (f, b)
            carried.add((j, e))
            frontier.append((j, e))
    # seams: two roads of different looks meeting without a junction ("Highway 3 lanes 2m offset" into "... 5m
    # offset": lanes 3 m apart, not linked, a dead end the router can't pass). Both ends meet in the middle,
    # blended in like a junction correction.
    for node_uid, here in at.items():
        if len(here) != 2:
            continue
        (i, ei), (j, ej) = here
        li, lj = layouts.get(roads[i]["roadLookToken"]), layouts.get(roads[j]["roadLookToken"])
        if li is None or lj is None:
            continue

        def used(k, e, lay):
            """The offsets lane_offsets will put at this end: a direct or far value, else the layout."""
            v = val.get((k, e))
            out = []
            for side in (0, 1):
                vals = v[side] if v and v[side] is not None else None
                if vals is not None and (((k, e) not in carried)
                                         or any(abs(a - b) > FAR_OFF_M for a, b in zip(vals, lay[side]))):
                    out.append(list(vals))
                else:
                    out.append(list(lay[side]))
            return out

        fi, bi = used(i, ei, li)
        fj, bj = used(j, ej, lj)
        if len(fi) != len(fj) or len(bi) != len(bj) or not (fi or bi):
            continue
        diff = max(abs(a - b) for a, b in zip(list(fi) + list(bi), list(fj) + list(bj)))
        if diff < 0.3 or diff > FAR_OFF_M:
            continue
        mid = ([(a + b) / 2 for a, b in zip(fi, fj)], [(a + b) / 2 for a, b in zip(bi, bj)])
        for k, e in here:
            val[(k, e)] = mid
            carried.discard((k, e))
    out = {}
    for i, rd in enumerate(roads):
        out[rd["uid"]] = {e: (val[(i, e)] + ((i, e) not in carried,)) if (i, e) in val else None for e in ("a", "b")}
    return out


def lane_offsets(base: float, va: float | None, vb: float | None, a_direct: bool, b_direct: bool, frac: np.ndarray,
                 length: float) -> np.ndarray:
    """One lane's offset along a road (frac 0..1 from A to B). `base`: the look's layout; `va`/`vb`: what the
    ends say (None: nothing).
      * an end far from the layout (> FAR_OFF_M, e.g. a look whose layout has the lanes on the wrong side): the
        layout is wrong for this road, the ends' values run its whole length (carried values too);
      * otherwise the layout runs along the road and each directly observed end blends into the junction's lane
        over END_BLEND_M (lanes do move in near junctions: "Highway 2 lanes 5m offset" by ~3 m)."""
    far = [v for v in (va, vb) if v is not None and abs(v - base) > FAR_OFF_M]
    if far:
        a = va if va is not None else vb
        b = vb if vb is not None else va
        return a + (b - a) * frac
    off = np.full_like(frac, base)
    s = frac * length
    blend = max(min(END_BLEND_M, length / 2), 1e-6)
    if va is not None and a_direct:
        off = off + (va - base) * np.clip(1 - s / blend, 0, 1)
    if vb is not None and b_direct:
        off = off + (vb - base) * np.clip(1 - (length - s) / blend, 0, 1)
    return off


def calibrate_looks(roads: list[dict], nodes: dict, looks: dict, prefab_lanes) -> dict:
    """Per road look: (forward lane offsets, backward lane offsets, source).

    Prefab nav curves must join a road's lanes seamlessly (AI traffic drives across), so where they end
    and start at a road's nodes shows exactly where that look's lanes are, including narrow lanes and
    medians the look's own numbers don't describe."""
    pe = prefab_lanes if isinstance(prefab_lanes, PrefabEnds) else PrefabEnds(prefab_lanes)
    g_end, g_start, lateral = pe.g_end, pe.g_start, pe.lateral

    fwd_obs: dict[str, list] = {}
    bwd_obs: dict[str, list] = {}
    for rd in roads:
        tok = rd["roadLookToken"]
        a, b = nodes.get(rd["startNodeUid"]), nodes.get(rd["endNodeUid"])
        if a is None or b is None or tok not in looks:
            continue
        # forward lanes: prefab curves end at A (into the road) or start at B (out of it), same direction
        f = np.concatenate([lateral(g_end, a[0], a[1], a[3], False), lateral(g_start, b[0], b[1], b[3], False)])
        # backward lanes: prefab curves start at A or end at B, running the opposite way
        r = np.concatenate([lateral(g_start, a[0], a[1], a[3], True), lateral(g_end, b[0], b[1], b[3], True)])
        if len(f):
            fwd_obs.setdefault(tok, []).extend(f.tolist())
        if len(r):
            bwd_obs.setdefault(tok, []).extend(r.tolist())

    out = {}
    for tok, look in looks.items():
        nl, nr = len(look.get("lanesLeft", [])), len(look.get("lanesRight", []))
        fwd, bwd = rule_layout(look)
        source = "rule"
        fo = np.array(fwd_obs.get(tok, []))
        ff = fit_lanes(fo, nr)
        if ff and ff[1] >= max(3, 0.5 * len(fo[: 600])):
            fwd, source = ff[0], "prefab evidence"
        bo = -np.array(bwd_obs.get(tok, []))                    # mirrored, so innermost lane comes first
        fb = fit_lanes(bo, nl)
        if fb and fb[1] >= max(3, 0.5 * len(bo[: 600])):
            bwd, source = [-o for o in fb[0]], "prefab evidence"
        out[tok] = (fwd, bwd, source)
    return out


@dataclass
class LaneHit:
    lane: int            # lane id
    segment: int         # segment index
    lateral: float       # signed distance to the lane centre, + = right of the lane direction (m)
    heading_err: float   # vehicle rotation - lane rotation (rad, wrapped)
    dist: float          # unsigned distance (m)
    t: float             # position along the segment, 0..1


class LaneMap:
    def __init__(self, seg: np.ndarray, seg_lane: np.ndarray, lane_kind: np.ndarray, lane_item: np.ndarray,
                 lane_sub: np.ndarray, meta: dict):
        self.seg = seg                  # (N, 6) float64: x0, y0, x1, y1, z0, z1
        self.seg_lane = seg_lane        # (N,) int32 lane id
        self.lane_kind = lane_kind      # (L,) uint8 KIND_*
        self.lane_item = lane_item      # (L,) uint64 road/prefab uid
        self.lane_sub = lane_sub        # (L,) int16: road lane slot (+ forward, - backward) or prefab nav-curve index
        self.meta = meta
        d = seg[:, 2:4] - seg[:, 0:2]
        self.seg_len = np.hypot(d[:, 0], d[:, 1])
        self.seg_rot = np.arctan2(d[:, 1], d[:, 0])
        self._build_grid()
        self._build_lane_ends()

    # ------------------------------------------------------------- grid
    def _cell(self, x, y):
        return np.floor(x / GRID_CELL).astype(np.int64), np.floor(y / GRID_CELL).astype(np.int64)

    @staticmethod
    def _key(cx, cy):
        return (cx + (1 << 20)) * (1 << 21) + (cy + (1 << 20))

    def _build_grid(self) -> None:
        mx = (self.seg[:, 0] + self.seg[:, 2]) / 2
        my = (self.seg[:, 1] + self.seg[:, 3]) / 2
        cx, cy = self._cell(mx, my)
        keys = self._key(cx, cy)
        order = np.argsort(keys, kind="stable")
        self._grid_order = order.astype(np.int32)
        sorted_keys = keys[order]
        self._grid_keys, self._grid_start = np.unique(sorted_keys, return_index=True)
        self._grid_end = np.append(self._grid_start[1:], len(sorted_keys))

    def _build_lane_ends(self) -> None:
        """First/last segment of every lane (segments of a lane are contiguous) and a grid of lane starts."""
        n_lanes = len(self.lane_kind)
        change = np.flatnonzero(np.diff(self.seg_lane)) + 1
        firsts = np.concatenate([[0], change])
        lasts = np.concatenate([change - 1, [len(self.seg_lane) - 1]])
        self.lane_first = np.full(n_lanes, -1, dtype=np.int64)
        self.lane_last = np.full(n_lanes, -1, dtype=np.int64)
        self.lane_first[self.seg_lane[firsts]] = firsts
        self.lane_last[self.seg_lane[lasts]] = lasts
        sx, sy = self.seg[self.lane_first, 0], self.seg[self.lane_first, 1]
        cx, cy = self._cell(sx, sy)
        keys = self._key(cx, cy)
        order = np.argsort(keys, kind="stable")
        self._start_order = order.astype(np.int32)
        sk = keys[order]
        self._start_keys, self._start_begin = np.unique(sk, return_index=True)
        self._start_end = np.append(self._start_begin[1:], len(sk))

    def lane_length(self, lane: int) -> float:
        return float(self.seg_len[self.lane_first[lane]:self.lane_last[lane] + 1].sum())

    def successors(self, lane: int, max_gap: float = 2.5, max_turn: float = math.radians(40)) -> list[tuple[int, float]]:
        """Lanes that continue `lane`: start within max_gap of its end, heading roughly the same way.
        Returns (lane, turn angle) pairs sorted by |turn|."""
        last = self.lane_last[lane]
        ex, ey, erot = self.seg[last, 2], self.seg[last, 3], self.seg_rot[last]
        cx, cy = int(math.floor(ex / GRID_CELL)), int(math.floor(ey / GRID_CELL))
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = self._key(cx + dx, cy + dy)
                i = np.searchsorted(self._start_keys, k)
                if i < len(self._start_keys) and self._start_keys[i] == k:
                    for cand in self._start_order[self._start_begin[i]:self._start_end[i]]:
                        if cand == lane:
                            continue
                        f = self.lane_first[cand]
                        gap = math.hypot(self.seg[f, 0] - ex, self.seg[f, 1] - ey)
                        turn = math.atan2(math.sin(self.seg_rot[f] - erot), math.cos(self.seg_rot[f] - erot))
                        if gap <= max_gap and abs(turn) <= max_turn:
                            out.append((int(cand), turn))
        out.sort(key=lambda p: abs(p[1]))
        return out

    def candidates(self, x: float, y: float) -> np.ndarray:
        """Segment indices whose midpoint lies in the 3x3 grid cells around (x, y)."""
        cx, cy = int(math.floor(x / GRID_CELL)), int(math.floor(y / GRID_CELL))
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = self._key(cx + dx, cy + dy)
                i = np.searchsorted(self._grid_keys, k)
                if i < len(self._grid_keys) and self._grid_keys[i] == k:
                    out.append(self._grid_order[self._grid_start[i]:self._grid_end[i]])
        return np.concatenate(out) if out else np.empty(0, dtype=np.int32)

    # ------------------------------------------------------------- queries
    def nearest(self, x: float, y: float, rotation: float | None = None, max_dist: float = 12.0,
                max_heading_err: float = math.radians(35), k: int = 4) -> list[LaneHit]:
        """Closest lanes to a point (one hit per lane), optionally only lanes running in `rotation`'s direction."""
        idx = self.candidates(x, y)
        if idx.size == 0:
            return []
        s = self.seg[idx]
        ax, ay = s[:, 0], s[:, 1]
        dx, dy = s[:, 2] - ax, s[:, 3] - ay
        l2 = dx * dx + dy * dy
        l2[l2 == 0] = 1e-9
        t = np.clip(((x - ax) * dx + (y - ay) * dy) / l2, 0.0, 1.0)
        px, py = ax + t * dx, ay + t * dy
        dist = np.hypot(x - px, y - py)
        rot = self.seg_rot[idx]
        # signed lateral: + when the point is to the right of the segment direction
        cross = dx * (y - ay) - dy * (x - ax)
        lateral = np.sign(cross) * dist
        ok = dist <= max_dist
        herr = np.zeros_like(rot)
        if rotation is not None:
            herr = wrap(rotation - rot)
            ok &= np.abs(herr) <= max_heading_err
        if not ok.any():
            return []
        sel = np.nonzero(ok)[0]
        sel = sel[np.argsort(dist[sel])]
        hits, seen = [], set()
        for j in sel:
            lane = int(self.seg_lane[idx[j]])
            if lane in seen:
                continue
            seen.add(lane)
            hits.append(LaneHit(lane, int(idx[j]), float(lateral[j]), float(herr[j]), float(dist[j]), float(t[j])))
            if len(hits) >= k:
                break
        return hits

    def lane_points(self, lane: int) -> np.ndarray:
        """Ordered polyline of a lane, (n, 2)."""
        segs = self.seg[self.lane_first[lane]:self.lane_last[lane] + 1]      # a lane's segments are contiguous
        return np.vstack([segs[:, 0:2], segs[-1:, 2:4]])

    def describe(self, lane: int) -> str:
        kind = "road" if self.lane_kind[lane] == KIND_ROAD else "prefab"
        return f"{kind} {int(self.lane_item[lane]):x} sub {int(self.lane_sub[lane])}"

    # ------------------------------------------------------------- persistence
    def save(self, path: Path) -> None:
        np.savez_compressed(path, seg=self.seg, seg_lane=self.seg_lane, lane_kind=self.lane_kind,
                            lane_item=self.lane_item, lane_sub=self.lane_sub,
                            meta=np.frombuffer(json.dumps(self.meta).encode(), dtype=np.uint8))

    @classmethod
    def load(cls, path: Path) -> "LaneMap":
        z = np.load(path)
        meta = json.loads(bytes(z["meta"]).decode())
        if meta.get("cache_version") != CACHE_VERSION:
            raise ValueError(f"{path}: cache version {meta.get('cache_version')} != {CACHE_VERSION}, rebuild it")
        return cls(z["seg"], z["seg_lane"], z["lane_kind"], z["lane_item"], z["lane_sub"], meta)

    @classmethod
    def load_or_build(cls, map_dir: Path, log=print) -> "LaneMap":
        cache = map_dir / "lanes.npz"
        if cache.exists():
            try:
                return cls.load(cache)
            except (ValueError, KeyError) as exc:
                log(f"lane cache unusable ({exc}); rebuilding")
        lm = cls.build(map_dir / "raw", log=log)
        lm.save(cache)
        return lm

    # ------------------------------------------------------------- build
    @classmethod
    def build(cls, raw_dir: Path, log=print) -> "LaneMap":
        t0 = time.time()

        def load(name):
            with open(raw_dir / f"europe-{name}.json", encoding="utf-8") as f:
                return json.load(f)

        log("loading nodes ...")
        nodes = {}
        for n in load("nodes"):
            nodes[n["uid"]] = (n["x"], n["y"], n["z"], n["rotation"])
        looks = {lk["token"]: lk for lk in load("roadLooks")}
        xs: list[np.ndarray] = []
        lane_kind, lane_item, lane_sub = [], [], []

        def add_lane(pts: np.ndarray, z0: float, z1: float, kind: int, item: int, sub: int):
            if len(pts) < 2:
                return
            lane_id = len(lane_kind)
            n = len(pts) - 1
            z = np.linspace(z0, z1, n + 1)
            arr = np.empty((n, 7))
            arr[:, 0:2] = pts[:-1]
            arr[:, 2:4] = pts[1:]
            arr[:, 4] = z[:-1]
            arr[:, 5] = z[1:]
            arr[:, 6] = lane_id
            xs.append(arr)
            lane_kind.append(kind)
            lane_item.append(item)
            lane_sub.append(sub)

        log("building prefab lanes ...")
        descs = {d["token"]: d for d in load("prefabDescriptions")}
        prefab_lanes = []                   # (pts, z0, z1, uid, curve index), added after the roads
        for pf in load("prefabs"):
            desc = descs.get(pf["token"])
            if pf.get("hidden") or not desc or not desc.get("navCurves") or not pf.get("nodeUids"):
                continue      # hidden items are not drivable in the game (closed / superseded pieces)
            origin = nodes.get(pf["nodeUids"][0])
            if origin is None:
                continue
            pnode = desc["nodes"][pf["originNodeIndex"]]
            rot = origin[3] - pnode["rotation"]
            c, s = math.cos(rot), math.sin(rot)
            ox, oy = origin[0], origin[1]

            def to_world(px, py):
                # translate so the prefab's origin node sits on the map node, then rotate about it
                tx, ty = px - pnode["x"], py - pnode["y"]
                return ox + tx * c - ty * s, oy + tx * s + ty * c

            uid = int(pf["uid"], 16)
            for ci, cv in enumerate(desc["navCurves"]):
                p0 = to_world(cv["start"]["x"], cv["start"]["y"])
                p1 = to_world(cv["end"]["x"], cv["end"]["y"])
                r0, r1 = cv["start"]["rotation"] + rot, cv["end"]["rotation"] + rot
                steps = sample_steps(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), r0, r1)
                prefab_lanes.append((hermite(p0, p1, r0, r1, steps), origin[2] + cv["start"].get("z", 0.0),
                                     origin[2] + cv["end"].get("z", 0.0), uid, ci))
        log(f"  {len(prefab_lanes)} prefab lanes")

        log("calibrating road looks from prefab connections ...")
        roads = [rd for rd in load("roads") if not rd.get("hidden")]   # hidden = not drivable (closed road)
        ends = PrefabEnds(prefab_lanes)
        layouts = calibrate_looks(roads, nodes, looks, ends)
        n_cal = sum(1 for v in layouts.values() if v[2] == "prefab evidence")
        log(f"  {n_cal} of {len(layouts)} road looks calibrated from prefab connections, the rest by rule")
        log("placing road ends on the junction lanes they meet ...")
        road_offs = road_end_offsets(roads, nodes, layouts, ends)
        del ends

        log("building road lanes ...")
        skipped = 0
        for rd in roads:
            a, b = nodes.get(rd["startNodeUid"]), nodes.get(rd["endNodeUid"])
            look = looks.get(rd["roadLookToken"])
            if a is None or b is None or look is None:
                skipped += 1
                continue
            steps = sample_steps(rd.get("length") or math.hypot(b[0] - a[0], b[1] - a[1]), a[3], b[3])
            centre = hermite((a[0], a[1]), (b[0], b[1]), a[3], b[3], steps)
            fwd, bwd, _ = layouts[rd["roadLookToken"]]
            ends_at = road_offs[rd["uid"]]
            ea, eb = ends_at["a"] or (None, None, False), ends_at["b"] or (None, None, False)
            uid = int(rd["uid"], 16)
            seg = np.hypot(*np.diff(centre, axis=0).T)
            total = float(seg.sum())
            frac = (np.concatenate([[0.0], np.cumsum(seg)]) / max(total, 1e-6))[:, None]
            for i, base in enumerate(fwd):                  # forward lanes, leftmost first
                off = lane_offsets(base, ea[0][i] if ea[0] else None, eb[0][i] if eb[0] else None, ea[2], eb[2],
                                   frac, total)
                add_lane(offset_polyline(centre, off), a[2], b[2], KIND_ROAD, uid, i + 1)
            for j, base in enumerate(bwd):                  # backward lanes (offsets < 0: left), innermost first
                off = lane_offsets(base, ea[1][j] if ea[1] else None, eb[1][j] if eb[1] else None, ea[2], eb[2],
                                   frac, total)
                add_lane(offset_polyline(centre, off)[::-1], b[2], a[2], KIND_ROAD, uid, -(j + 1))
        del roads
        log(f"  {len(lane_kind)} road lanes ({skipped} roads skipped)")

        for pts, z0, z1, uid, ci in prefab_lanes:
            add_lane(pts, z0, z1, KIND_PREFAB, uid, ci)
        del prefab_lanes

        allseg = np.concatenate(xs)
        meta = {"cache_version": CACHE_VERSION, "built": time.time(), "lane_width": LANE_WIDTH,
                "source": str(raw_dir), "build_s": round(time.time() - t0, 1)}
        lm = cls(allseg[:, :6].copy(), allseg[:, 6].astype(np.int32), np.array(lane_kind, dtype=np.uint8),
                 np.array(lane_item, dtype=np.uint64), np.array(lane_sub, dtype=np.int16), meta)
        log(f"lane map: {len(lane_kind)} lanes, {len(allseg)} segments in {meta['build_s']} s")
        return lm
