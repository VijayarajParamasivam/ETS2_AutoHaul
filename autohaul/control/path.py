"""A reference path (polyline) with arc length, projection and a smoothed curvature profile."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Projection:
    s: float          # arc length of the closest point (m; < 0 or > length when extrapolated past an end)
    lateral: float    # signed distance, + = right of the path direction
    rotation: float   # path direction at s
    index: int        # segment index


class PathRef:
    def __init__(self, pts: np.ndarray, curvature_window: float = 12.0):
        pts = np.asarray(pts, dtype=float)
        keep = np.concatenate([[True], np.hypot(*np.diff(pts, axis=0).T) > 1e-3])
        self.pts = pts[keep]
        if len(self.pts) < 2:
            raise ValueError("path needs at least two distinct points")
        d = np.diff(self.pts, axis=0)
        self.seg_len = np.hypot(d[:, 0], d[:, 1])
        self.seg_rot = np.arctan2(d[:, 1], d[:, 0])
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.s[-1])
        self._window = curvature_window
        # unwrapped heading at segment midpoints, for curvature by windowed differences; very short
        # segments (joins between lanes) have unreliable directions and are left out
        use = self.seg_len >= 0.5
        if not use.any():
            use[:] = True
        self._mid_s = ((self.s[:-1] + self.s[1:]) / 2)[use]
        self._mid_rot = np.unwrap(self.seg_rot[use])

    def project(self, x: float, y: float) -> Projection:
        a = self.pts[:-1]
        d = self.pts[1:] - a
        l2 = self.seg_len ** 2
        t = ((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / l2
        tc = np.clip(t, 0.0, 1.0)
        px, py = a[:, 0] + tc * d[:, 0], a[:, 1] + tc * d[:, 1]
        dist = np.hypot(x - px, y - py)
        i = int(np.argmin(dist))
        ti = float(t[i])
        if not ((i == 0 and ti < 0) or (i == len(t) - 1 and ti > 1)):
            ti = min(max(ti, 0.0), 1.0)          # extrapolate only beyond the path's ends
        cross = d[i, 0] * (y - a[i, 1]) - d[i, 1] * (x - a[i, 0])
        lat = cross / self.seg_len[i] if (ti < 0 or ti > 1) else math.copysign(float(dist[i]), cross)
        return Projection(float(self.s[i] + ti * self.seg_len[i]), float(lat), float(self.seg_rot[i]), i)

    def point(self, s: float) -> tuple[float, float]:
        s = min(max(s, 0.0), self.length)
        i = int(min(np.searchsorted(self.s, s, side="right") - 1, len(self.seg_len) - 1))
        f = (s - self.s[i]) / self.seg_len[i]
        p = self.pts[i] + f * (self.pts[i + 1] - self.pts[i])
        return float(p[0]), float(p[1])

    def rotation_at(self, s: float) -> float:
        return float(np.interp(s, self._mid_s, self._mid_rot))

    def curvature(self, s) -> np.ndarray | float:
        """Curvature (1/m, + = right turn) averaged over the window centred at s."""
        h = self._window / 2
        s = np.asarray(s, dtype=float)
        lo, hi = np.clip(s - h, 0, self.length), np.clip(s + h, 0, self.length)
        span = np.maximum(hi - lo, 1e-6)
        k = (np.interp(hi, self._mid_s, self._mid_rot) - np.interp(lo, self._mid_s, self._mid_rot)) / span
        if len(self._mid_s) == 1:
            k = np.zeros_like(s)
        return float(k) if k.ndim == 0 else k

    def mean_curvature(self, s0: float, s1: float) -> float:
        """Average curvature between s0 and s1 (heading change / distance)."""
        s0, s1 = max(0.0, s0), min(self.length, s1)
        if s1 - s0 < 1e-3:
            return float(self.curvature(s0))
        return float((np.interp(s1, self._mid_s, self._mid_rot) - np.interp(s0, self._mid_s, self._mid_rot)) / (s1 - s0))
