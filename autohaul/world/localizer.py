"""Lane tracking on the LaneMap: which lane the truck is in, how far off its centre, and the path ahead.

The truck's pose comes straight from telemetry, so this is map matching, not SLAM. Continuity matters
more than raw distance: the tracker keeps the current lane (or a successor) unless another lane is
clearly closer, so it doesn't flicker between adjacent lanes or across junction curves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from autohaul.world.lanemap import LaneHit, LaneMap


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

    def reset(self) -> None:
        self.state = None
        self.prev_lane = None

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
        elif chosen.lane != self.state.lane:
            self.prev_lane = self.state.lane
        self.state = LaneState(chosen.lane, chosen.segment, chosen.t, chosen.lateral, chosen.heading_err, conf,
                               switched and self.state is not None)
        return self.state

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
        start = seg[0:2] + st.t * (seg[2:4] - seg[0:2])
        back = self._behind(start, behind) if behind > 0 else []
        pts = back + [start]
        acc = 0.0
        lane, s = st.lane, st.segment
        last = lm.lane_last[lane]
        prev = start
        visited = {lane}
        self.path_lanes = [lane]
        while acc < distance:
            p = lm.seg[s, 2:4]
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
            u = np.array([math.cos(heading), math.sin(heading)])
            first = lm.seg[s, 0:2]
            if float(np.dot(first - prev, u)) > 1.0:
                pts.append(first)
                prev = first
            else:
                while s < last and float(np.dot(lm.seg[s, 2:4] - prev, u)) < self.join_skip_m:
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
