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
        self.path_lanes: list[int] = []      # lanes covered by the last path_ahead() call, in order

    def reset(self) -> None:
        self.state = None

    def update(self, x: float, y: float, rotation: float, prefer: list[int] | None = None) -> LaneState | None:
        """`prefer`: lanes of the path we are following (from path_ahead); at a split they win over the
        other branch while both are about equally close, so the tracker stays on the planned branch."""
        hits = self.lm.nearest(x, y, rotation, max_dist=self.max_lateral, k=6)
        if not hits:
            self.state = None
            return None
        best = hits[0]
        chosen, switched = best, True
        if self.state is not None:
            keep = {self.state.lane} | {lane for lane, _ in self.lm.successors(self.state.lane)}
            planned = set(prefer or ()) & keep
            ranked = [h for h in hits if h.lane in planned] + [h for h in hits if h.lane in keep]
            for h in ranked:
                if h.dist <= best.dist + self.hysteresis:
                    chosen, switched = h, False
                    break
        conf = self._confidence(chosen, hits)
        self.state = LaneState(chosen.lane, chosen.segment, chosen.t, chosen.lateral, chosen.heading_err, conf,
                               switched and self.state is not None)
        return self.state

    @staticmethod
    def _confidence(h: LaneHit, hits: list[LaneHit]) -> float:
        c = max(0.0, 1.0 - abs(h.lateral) / 3.0) * max(0.0, 1.0 - abs(h.heading_err) / math.radians(35))
        others = [o.dist for o in hits if o.lane != h.lane]
        if others and others[0] - h.dist < 0.5:      # two lanes almost equally close: ambiguous
            c *= 0.6
        return c

    def path_ahead(self, distance: float, choose=None) -> np.ndarray:
        """Polyline (n, 2) from the truck's projection on its lane, `distance` metres forward.

        `choose(lane, successors)` picks the next lane at a split; default: the straightest.
        """
        self.path_lanes = []
        if self.state is None:
            return np.empty((0, 2))
        lm, st = self.lm, self.state
        seg = lm.seg[st.segment]
        start = seg[0:2] + st.t * (seg[2:4] - seg[0:2])
        pts = [start]
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
            first = lm.seg[s, 0:2]
            # bridge real gaps with a point, but not sub-metre ones: a tiny connector segment can point
            # anywhere and would look like a hairpin to the curvature estimate
            if np.hypot(*(first - prev)) > 1.0:
                pts.append(first)
                prev = first
        return np.array(pts)
