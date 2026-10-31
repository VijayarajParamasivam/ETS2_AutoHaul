"""Lane tracking on the LaneMap: which lane the truck is in, how far off its centre, and the path ahead.

The truck's pose comes straight from telemetry, so this is map matching, not SLAM. Continuity matters
more than raw distance: the tracker keeps the current lane (or a successor) unless another lane is
clearly closer, so it doesn't flicker between adjacent lanes or across junction curves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from autohaul.world.lanemap import LaneHit, LaneMap


def _still(x: float) -> float:
    """How much of a step is left to ease across at share `x` of the way: 1 to 0, smoothly."""
    x = min(max(x, 0.0), 1.0)
    return 1.0 - x * x * (3.0 - 2.0 * x)


@dataclass
class LaneState:
    lane: int
    segment: int
    t: float
    lateral: float        # + = truck is right of the lane centre (m)
    heading_err: float    # truck rotation - lane rotation (rad)
    confidence: float     # 0..1
    switched: bool        # lane changed this update (to a non-successor lane)


class LaneTracker:
    def __init__(self, lm: LaneMap, max_lateral: float = 4.0, hysteresis_m: float = 1.2):
        self.lm = lm
        self.max_lateral = max_lateral
        self.hysteresis = hysteresis_m
        self.state: LaneState | None = None
        self.prev_lane: int | None = None    # the lane the tracked one continues (None after a jump)
        self.path_lanes: list[int] = []      # lanes covered by the last path_ahead() call, in order
        self.path_back = 0.0                 # metres of that path behind the truck's projection
        self.bias = 0.0                      # where the truck is meant to be relative to the lane centre (+ right)
        self._step = np.zeros(2)             # the sideways step at the join into the tracked lane, still being eased across
        self._ease_m = 0.0                   # ... over this much of the lane (0: none)

    def reset(self) -> None:
        self.state = None
        self.prev_lane = None
        self._step, self._ease_m = np.zeros(2), 0.0

    def update(self, x: float, y: float, rotation: float, prefer: list[int] | None = None,
               force: int | None = None, force_max_dist: float = 7.5) -> LaneState | None:
        """`prefer`: lanes of the path we are following (from path_ahead); at a split they win over the
        other branch while both are about equally close, so the tracker stays on the planned branch."""
        if force is not None:
            # a lane change: track the target lane even while the truck is still in the old one
            for h in self.lm.nearest(x, y, rotation, max_dist=force_max_dist, k=8):
                if h.lane == force:
                    switched = self.state is not None and self.state.lane != h.lane
                    if switched:
                        cont = h.lane in {lane for lane, _ in self.lm.successors(self.state.lane)}
                        self.prev_lane = self.state.lane if cont else None
                        self._step, self._ease_m = np.zeros(2), 0.0
                    # the lane is known (a deliberate change), so being between two lanes is not ambiguity;
                    # the autopilot checks the truck against the eased line instead
                    self.state = LaneState(h.lane, h.segment, h.t, h.lateral, h.heading_err, 0.9, switched)
                    return self.state
        hits = self.lm.nearest(x, y, rotation, max_dist=self.max_lateral, k=6)
        if not hits:
            self.state = None
            return None
        best = hits[0]
        chosen, switched, known = best, True, False
        if self.state is not None:
            keep = {self.state.lane} | {lane for lane, _ in self.lm.successors(self.state.lane)}
            planned = set(prefer or ()) & keep
            ranked = [h for h in hits if h.lane in planned] + [h for h in hits if h.lane in keep]
            for h in ranked:
                if h.dist <= best.dist + self.hysteresis:
                    chosen, switched, known = h, False, h.lane in planned
                    break
        conf = self._confidence(chosen, hits, known, self.bias)
        if self.state is None or switched:
            self.prev_lane = None
            self._step, self._ease_m = np.zeros(2), 0.0
        elif chosen.lane != self.state.lane:
            self.prev_lane = self.state.lane
            self._cross_join(self.state.lane, chosen.lane)
        lateral = chosen.lateral - self._eased_off(chosen.lane, chosen.segment, chosen.t)
        if lateral != chosen.lateral:          # against the eased line the truck follows: the step is the map's, not its own
            chosen = replace(chosen, lateral=lateral)
            conf = self._confidence(chosen, hits, known, self.bias)
        self.state = LaneState(chosen.lane, chosen.segment, chosen.t, chosen.lateral, chosen.heading_err, conf,
                               switched and self.state is not None)
        return self.state

    def _join(self, end: np.ndarray, heading: float, s: int) -> tuple[float, np.ndarray, float]:
        """The join from a path ending at `end` with `heading` into the path starting at segment `s`:
        how far ahead that start lies, the step to its side, and how much road to ease across the step over."""
        first = self.lm.seg[s, 0:2]
        u = np.array([math.cos(heading), math.sin(heading)])
        ahead = float(np.dot(first - end, u))
        step = (first - end) - ahead * u
        size = float(np.hypot(*step))
        return ahead, step, (max(self.join_ease_m, self.join_ease_per_m * size) if size > 0.1 else 0.0)

    def _along(self, s: int, lane: int) -> float:
        """Metres along `lane` to the start of its segment `s`."""
        first = self.lm.lane_first[lane]
        return float(np.sum(self.lm.seg_len[first:s]))

    def _cross_join(self, old: int, new: int) -> None:
        """Keep the eased line the truck was following as it crosses from `old` into `new`."""
        lm = self.lm
        last = lm.lane_last[old]
        end = lm.seg[last, 2:4].copy()
        if self._ease_m > 0.0:                 # (a short lane: the last step may not be eased out yet)
            end = end - self._step * _still((self._along(last, old) + float(lm.seg_len[last])) / self._ease_m)
        _, self._step, self._ease_m = self._join(end, float(lm.seg_rot[last]), lm.lane_first[new])

    def _eased_off(self, lane: int, s: int, t: float) -> float:
        """How far right of the lane's own centre the eased line runs at segment `s` (negative: left)."""
        if self._ease_m <= 0.0:
            return 0.0
        lm = self.lm
        along = self._along(s, lane) + t * float(lm.seg_len[s])
        rot = float(lm.seg_rot[s])
        return float(self._step[0] * math.sin(rot) - self._step[1] * math.cos(rot)) * _still(along / self._ease_m)

    @staticmethod
    def _confidence(h: LaneHit, hits: list[LaneHit], known: bool = False, bias: float = 0.0) -> float:
        """`known`: the lane continues the tracked one along the planned path, so other lanes close by (the
        overlapping curves of a junction) don't make it ambiguous."""
        c = max(0.0, 1.0 - abs(h.lateral - bias) / 3.0) * max(0.0, 1.0 - abs(h.heading_err) / math.radians(35))
        others = [o.dist for o in hits if o.lane != h.lane]
        if others and others[0] - h.dist < 0.5 and not known:      # two lanes almost equally close: ambiguous
            c *= 0.6
        return c

    join_skip_m = 1.0                  # where paths join, skip next-path points less than this ahead of the last one
    join_ease_m = 8.0                  # ease across a sideways step over at least this much road ...
    join_ease_per_m = 16.0             # ... or this many metres per metre of step

    def path_ahead(self, distance: float, choose=None, behind: float = 0.0) -> np.ndarray:
        """Polyline (n, 2) from the truck's projection on its lane, `distance` metres forward.

        `choose(lane, successors)` picks the next lane at a split; default: the straightest.
        `behind`: also this many metres before the projection (the lane's, then the previous lane's), so the
        rear axle is measured against the real lane, not a straight extension of the path's first segment;
        `path_back` says how much was added (arc lengths on the path include it).
        """
        self.path_lanes = []
        self.path_back = 0.0
        if self.state is None:
            return np.empty((0, 2))
        lm, st = self.lm, self.state
        seg = lm.seg[st.segment]
        lane, s = st.lane, st.segment
        step, ease_m = self._step, self._ease_m
        eased = self._along(s, lane) if ease_m > 0.0 else 0.0
        start = seg[0:2] + st.t * (seg[2:4] - seg[0:2])
        if ease_m > 0.0:                       # the line stays where it was when the truck crosses the join
            start = start - step * _still((eased + st.t * float(lm.seg_len[s])) / ease_m)
        back = self._behind(start, behind) if behind > 0 else []
        pts = back + [start]
        acc = 0.0
        last = lm.lane_last[lane]
        prev = start
        visited = {lane}
        self.path_lanes = [lane]
        while acc < distance:
            p = lm.seg[s, 2:4]
            if eased < ease_m:                 # still easing across a sideways step at the last join
                eased += float(lm.seg_len[s])
                p = p - step * _still(eased / ease_m)
            acc += float(np.hypot(*(p - prev)))
            pts.append(p)
            prev = p
            heading = float(lm.seg_rot[s])
            if s < last:
                s += 1
                continue
            succ = [c for c in lm.successors(lane) if c[0] not in visited]
            if not succ:
                break
            lane = choose(lane, succ) if choose else succ[0][0]
            visited.add(lane)
            self.path_lanes.append(lane)
            s, last = lm.lane_first[lane], lm.lane_last[lane]
            # Where a junction's paths join, the next one often starts behind this one's end, or to its side (6 % and 7 %
            # of the map's joins, up to 2.5 m). Bridge only real gaps ahead, and skip what lies behind: a point back
            # there draws a hairpin, which the steering and the curve speeds then take for a tight turn the other way
            # (2026-10-30: curvature flipping -0.21, +0.11, -0.08 on a curve of radius 12 m, "sharp turn ahead").
            # A step to the side (7 % of joins, up to 2.5 m) is eased across over the next metres: as a jump it reads as a
            # sharp turn the wrong way (2026-10-30: 0.96 m, curvature +0.24 in a left turn, the wheel steered right).
            u = np.array([math.cos(heading), math.sin(heading)])
            first = lm.seg[s, 0:2]
            ahead, step, ease_m = self._join(prev, heading, s)
            eased = 0.0
            if ahead > 1.0:
                prev = first - step
                pts.append(prev)
            else:
                while s < last and float(np.dot(lm.seg[s, 2:4] - prev, u)) < self.join_skip_m:
                    eased += float(lm.seg_len[s])
                    s += 1
            heading = float(lm.seg_rot[s])
        return np.array(pts)

    def _behind(self, start: np.ndarray, behind: float) -> list[np.ndarray]:
        """Points from `behind` metres back along the tracked lane (and the previous one) up to `start`."""
        lm, st = self.lm, self.state
        out, prev, acc = [], start, 0.0
        lane, s = st.lane, st.segment
        while acc < behind:
            p = lm.seg[s, 0:2]
            if lane == st.lane and self._ease_m > 0.0:     # the line eased across the step at this lane's join
                p = p - self._step * _still(self._along(s, lane) / self._ease_m)
            d = float(np.hypot(*(p - prev)))
            if d > 1e-6:
                out.append(p)
                acc += d
                prev = p
            if s > lm.lane_first[lane]:
                s -= 1
                continue
            pl = self.prev_lane
            if lane == st.lane and pl is not None and np.hypot(*(lm.seg[lm.lane_last[pl], 2:4] - p)) < 1.5:
                lane, s = pl, lm.lane_last[pl]
                continue
            break
        self.path_back = acc
        out.reverse()
        return out
