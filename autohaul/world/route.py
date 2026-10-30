"""Lane-level routing (A*) over the LaneGraph to a destination point.

1.53.2.1's GPS route can't be read (the memory reader's route block stays empty), so AutoHaul plans its own
route and checks it against the game: telemetry `navigation.distance` is the GPS distance to the current
target, and AutoHaul's route length should agree with it.

Cost model (metres): lane length, plus a turn penalty at sharp joins, plus `lane_change_m` per lane change.
A node is a whole lane, reached "at its end"; a lane change moves to the parallel lane of the same road,
whose end is at the same place, so it only adds the penalty. `no_go` lanes are never used (paths the map
reserves for cars, e.g. the car pumps of a fuel station).

`avoid` lanes cost `avoid_m` extra instead: junction paths the map flags for small vehicles only. The flag steers
the AI; the game's GPS sends the player's truck through them (2026-10-29: treated as impassable, 3 lanes of 3-53 m
made AutoHaul's way Amsterdam -> Paris 65 game km longer than the GPS's). They're never a destination.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from autohaul.world.graph import LaneGraph
from autohaul.world.lanemap import LaneMap


@dataclass
class Route:
    lanes: list[int]                    # lanes in driving order (a lane change shows as two parallel lanes)
    changes: list[bool]                 # changes[i]: lanes[i] was reached by a lane change from lanes[i-1]
    length_m: float                     # driving distance from the start position to the destination point
    target: tuple[float, float]
    expanded: int = 0
    rest_after: list = field(default_factory=list)   # rest_after[i]: driving metres from the end of lanes[i]
    lane_index: dict = field(default_factory=dict)   # lane -> position in `lanes`
    scaled_m: float = 0.0               # length in open-road-equivalent metres (a city metre counts 3/19)

    def __post_init__(self):
        self.lane_index = {ln: i for i, ln in enumerate(self.lanes)}

    def remaining_m(self, lane: int, metres_to_lane_end: float) -> float | None:
        """Driving distance to the destination from a point `metres_to_lane_end` before the end of `lane`."""
        i = self.lane_index.get(lane)
        if i is None or not self.rest_after:
            return None
        return max(metres_to_lane_end + self.rest_after[i], 0.0)

    def change_after(self, lane: int) -> bool:
        """The route's next step after `lane` is a lane change."""
        i = self.lane_index.get(lane)
        return i is not None and i + 1 < len(self.lanes) and self.changes[i + 1]

    def next_after(self, lane: int) -> int | None:
        """The lane the route takes after `lane` (None if `lane` is not on the route or is the last)."""
        i = self.lane_index.get(lane)
        if i is None or i + 1 >= len(self.lanes):
            return None
        return self.lanes[i + 1]


class Router:
    def __init__(self, lm: LaneMap, graph: LaneGraph, lane_change_m: float = 80.0, turn_penalty_m_per_rad: float = 20.0,
                 no_go: np.ndarray | None = None, snap_m: float = 2000.0, weight: np.ndarray | None = None,
                 avoid: np.ndarray | None = None, avoid_m: float = 200.0):
        """`weight`: per lane, what a metre of it costs (city_weights: 3/19 inside a city area, 1 outside). The
        game's GPS counts distance in game km, and a city metre is 3 game metres against 19 on the open road;
        unweighted, AutoHaul took routes the GPS doesn't (a U-turn out of Rotterdam instead of through it)."""
        self.lm = lm
        self.g = graph
        self.snap_m = snap_m          # no lane near the destination point (a city centre): the nearest within this
        self._no_go = (np.zeros(len(graph.length), dtype=bool) if no_go is None else np.asarray(no_go, bool)).tolist()
        soft = np.zeros(len(graph.length), dtype=bool) if avoid is None else np.asarray(avoid, bool)
        self._no_goal = soft.tolist()
        self.lane_change_m = lane_change_m
        self.turn_penalty = turn_penalty_m_per_rad
        # plain Python lists: much faster than numpy scalars inside the search loop
        last = lm.lane_last
        self._ex = lm.seg[last, 2].tolist()
        self._ey = lm.seg[last, 3].tolist()
        self._len = graph.length.tolist()
        w = np.ones(len(graph.length)) if weight is None else np.asarray(weight, float)
        self._scaled = (graph.length * w).tolist()
        self.penalty = np.where(soft, avoid_m, 0.0)     # extra cost per lane (the learned ones are added to it)
        self._cost = (graph.length * w + self.penalty).tolist()
        self._h = float(w.min()) if len(w) else 1.0          # admissible: no metre costs less than this
        self._sp = graph.succ_ptr.tolist()
        self._si = graph.succ_idx.tolist()
        self._st = graph.succ_turn.tolist()
        self._np = graph.nbr_ptr.tolist()
        self._ni = graph.nbr_idx.tolist()

    def goal_lanes(self, x: float, y: float, radius: float) -> dict[int, float]:
        """Lanes passing within `radius` of (x, y) -> distance from that closest point to the lane's end."""
        idx = self.lm.candidates(x, y)
        if not len(idx):
            return {}
        s = self.lm.seg[idx]
        d = s[:, 2:4] - s[:, 0:2]
        l2 = np.maximum((d * d).sum(axis=1), 1e-9)
        t = np.clip(((x - s[:, 0]) * d[:, 0] + (y - s[:, 1]) * d[:, 1]) / l2, 0, 1)
        dist = np.hypot(x - (s[:, 0] + t * d[:, 0]), y - (s[:, 1] + t * d[:, 1]))
        out: dict[int, float] = {}
        for j in np.argsort(dist):
            if dist[j] > radius:
                break
            seg = int(idx[j])
            lane = int(self.lm.seg_lane[seg])
            if lane in out or self._no_go[lane] or self._no_goal[lane]:
                continue
            # metres from the closest point to the lane's end
            rest = float(self.lm.seg_len[seg] * (1 - t[j]) + self.lm.seg_len[seg + 1:self.lm.lane_last[lane] + 1].sum())
            out[lane] = rest
        return out

    def nearest_road_point(self, x: float, y: float, max_dist: float) -> tuple[float, float] | None:
        """The closest point on a drivable lane within `max_dist` of (x, y), or None. Scans every segment
        (~30 ms), so only for destinations without a lane nearby."""
        s = self.lm.seg
        mx, my = (s[:, 0] + s[:, 2]) / 2, (s[:, 1] + s[:, 3]) / 2
        d = np.hypot(mx - x, my - y)
        d[(np.asarray(self._no_go) | np.asarray(self._no_goal))[self.lm.seg_lane]] = np.inf
        i = int(np.argmin(d))
        return (float(mx[i]), float(my[i])) if d[i] <= max_dist else None

    def route(self, start_lane: int, start_progress_m: float, target: tuple[float, float], goal_radius: float = 60.0,
              max_expand: int = 3_000_000, goal: tuple[int, float] | None = None,
              no_change_m: float = 0.0) -> Route | None:
        """Route from `start_progress_m` metres along `start_lane` to the destination point: any lane passing
        within `goal_radius` of it, or exactly `goal` = (lane, metres along it) when given (a fuel pump).
        `no_change_m`: no lane changes in the first metres (there's no room left for one: stay in lane)."""
        tx, ty = target
        if goal is not None:
            goals = {goal[0]: max(self._len[goal[0]] - goal[1], 0.0)}
        else:
            goals = self.goal_lanes(tx, ty, goal_radius)
            if not goals and self.snap_m > goal_radius:
                near = self.nearest_road_point(tx, ty, self.snap_m)
                goals = self.goal_lanes(near[0], near[1], goal_radius) if near else {}
        if not goals:
            return None
        ex, ey, ln_len, cost, no_go, hw = self._ex, self._ey, self._len, self._cost, self._no_go, self._h
        sp, si, st, np_, ni = self._sp, self._si, self._st, self._np, self._ni
        g0 = max(ln_len[start_lane] - start_progress_m, 0.0)
        c0 = g0 * (cost[start_lane] / ln_len[start_lane] if ln_len[start_lane] > 0 else 1.0)
        best = {start_lane: c0}
        parent: dict[int, tuple[int, bool]] = {}
        heap = [(c0 + hw * math.hypot(ex[start_lane] - tx, ey[start_lane] - ty), c0, start_lane)]
        expanded = 0
        while heap:
            f, g, lane = heapq.heappop(heap)
            if g > best.get(lane, math.inf) + 1e-6:
                continue
            if lane in goals:
                return self._make(lane, parent, g0, goals[lane], target, expanded)
            expanded += 1
            if expanded > max_expand:
                return None
            for k in range(sp[lane], sp[lane + 1]):
                nxt = si[k]
                if no_go[nxt]:
                    continue
                ng = g + cost[nxt] + self.turn_penalty * abs(st[k])
                if ng < best.get(nxt, math.inf):
                    best[nxt] = ng
                    parent[nxt] = (lane, False)
                    heapq.heappush(heap, (ng + hw * math.hypot(ex[nxt] - tx, ey[nxt] - ty), ng, nxt))
            for k in range(np_[lane], np_[lane + 1]):
                nxt = ni[k]
                if no_go[nxt] or g < no_change_m:
                    continue
                ng = g + self.lane_change_m
                if ng < best.get(nxt, math.inf):
                    best[nxt] = ng
                    parent[nxt] = (lane, True)
                    heapq.heappush(heap, (ng + hw * math.hypot(ex[nxt] - tx, ey[nxt] - ty), ng, nxt))
        return None

    def _make(self, goal: int, parent: dict, g0: float, goal_rest: float, target, expanded: int) -> Route:
        lanes, changes = [goal], []
        cur = goal
        while cur in parent:
            prev, changed = parent[cur]
            changes.append(changed)
            lanes.append(prev)
            cur = prev
        lanes.reverse()
        changes.reverse()
        changes = [False] + changes
        # driving distance: rest of the start lane, every lane entered at its start, nothing for a lane
        # change (it runs alongside), minus what lies beyond the destination point on the last lane
        length = g0 + sum(self._len[ln] for ln, ch in zip(lanes[1:], changes[1:]) if not ch) - goal_rest
        rest = [0.0] * len(lanes)
        rest[-1] = -goal_rest                     # the destination lies goal_rest before the last lane's end
        for i in range(len(lanes) - 2, -1, -1):
            rest[i] = rest[i + 1] + (0.0 if changes[i + 1] else self._len[lanes[i + 1]])
        ratio = lambda ln: self._scaled[ln] / self._len[ln] if self._len[ln] > 0 else 1.0     # noqa: E731
        scaled = g0 * ratio(lanes[0]) + sum(self._scaled[ln] for ln, ch in zip(lanes[1:], changes[1:]) if not ch) \
            - goal_rest * ratio(lanes[-1])
        return Route(lanes, changes, max(length, 0.0), target, expanded, rest, scaled_m=max(scaled, 0.0))

    def add_penalties(self, penalties: dict[int, float]) -> None:
        """Extra cost (metres) on lanes: what `autohaul.learn` has learned from the game's GPS."""
        for lane, extra in penalties.items():
            self.penalty[lane] += extra
            self._cost[lane] += extra
