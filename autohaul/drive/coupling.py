"""Coupling the job trailer (Phase 5): drive to a point in line with it, reverse under it, press attach.

The trailer's position and heading come from the save (autohaul.infra.saves); its kingpin is `kingpin_ahead_m`
in front of its origin. With the game's simple coupling (g_trailer_advanced_coupling 0) the trailer couples
when the fifth wheel is near the kingpin and `attach` is pressed.

  back out  (if planned) reverse up to `back_out_m` straight or on full lock first: a truck parked nose-in
            (2026-10-29, Paris: windscreen against a wall, the trailer behind) can't loop round forwards. When the
            line-up point is behind the truck, backing out is required and the forward way may not go beyond
            where the front bumper stood (what's in front is unknown)
  approach  forward along the shortest path with turns no tighter than `min_radius_m` (Dubins: arcs and a
            straight), ending with `straight_m` straight on to the line-up pose: in line with the trailer,
            facing the same way, the fifth wheel `stage_m` in front of the kingpin; then stop
  shift     `gearreverse` until the gearbox shows reverse
  reverse   straight back along the trailer's axis at `reverse_kmh` (pure pursuit on the rear axle, reversed);
            within `attach_window_m` of the kingpin press `attach` every `press_every_s` until telemetry shows
            the trailer coupled; more than `overshoot_m` past it: stop and hand back
  drive     brake, `geardrive` until the gearbox shows drive: done
It knows nothing about other trailers, walls or fences in the yard: supervise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from autohaul.control.vehicle import TruckGeometry, pose_from_frame


def _mod(a: float) -> float:
    return a % (2 * math.pi)


def dubins_words(q0, q1, r: float) -> list[tuple[str, tuple[float, float, float]]]:
    """Candidate Dubins paths from pose q0 (x, y, heading) to q1 with turn radius r: (word, segment lengths in
    units of r). "L" turns towards +heading. Callers check where a candidate ends (`sample_dubins`)."""
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / r
    th = _mod(math.atan2(dy, dx))
    a, b = _mod(q0[2] - th), _mod(q1[2] - th)
    sa, sb, ca, cb, cab = math.sin(a), math.sin(b), math.cos(a), math.cos(b), math.cos(a - b)
    out = []
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        t1 = math.atan2(cb - ca, d + sa - sb)
        out.append(("LSL", (_mod(t1 - a), math.sqrt(p2), _mod(b - t1))))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        t1 = math.atan2(ca - cb, d - sa + sb)
        out.append(("RSR", (_mod(a - t1), math.sqrt(p2), _mod(t1 - b))))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        pp = math.sqrt(p2)
        t0 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, pp)
        out.append(("LSR", (_mod(t0 - a), pp, _mod(t0 - b))))
    p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        pp = math.sqrt(p2)
        t0 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, pp)
        out.append(("RSL", (_mod(a - t0), pp, _mod(b - t0))))
    c = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
    if abs(c) <= 1:
        pp = _mod(2 * math.pi - math.acos(c))
        t0 = _mod(a - math.atan2(ca - cb, d - sa + sb) + pp / 2)
        out.append(("RLR", (t0, pp, _mod(a - b - t0 + pp))))
    c = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8
    if abs(c) <= 1:
        pp = _mod(2 * math.pi - math.acos(c))
        t0 = _mod(-a - math.atan2(ca - cb, d + sa - sb) + pp / 2)
        out.append(("LRL", (t0, pp, _mod(b - a - t0 + pp))))
    return out


def sample_dubins(q0, word: str, segs, r: float, step: float = 0.25) -> np.ndarray:
    """Points along a Dubins word from q0, every ~`step` m."""
    x, y, h = q0
    pts = [(x, y)]
    for kind, length in zip(word, segs):
        n = max(1, int(math.ceil(length * r / step)))
        ds = length * r / n
        turn = {"L": 1.0, "R": -1.0, "S": 0.0}[kind] * ds / r
        for _ in range(n):
            hm = h + turn / 2
            x, y, h = x + ds * math.cos(hm), y + ds * math.sin(hm), h + turn
            pts.append((x, y))
    return np.array(pts)


@dataclass
class CouplingParams:
    kingpin_ahead_m: float = 5.0        # trailer origin -> kingpin (measured on a coupled 1.53 box trailer)
    stage_m: tuple = (16.0, 22.0, 30.0)  # line up with the fifth wheel this far in front of the kingpin (tried)
    straight_m: float = 8.0             # the last metres to the line-up point are straight
    min_radius_m: float = 8.0           # rear axle; the tractor alone manages ~5 m
    max_path_m: float = 160.0           # a longer way round: hand back
    trailer_length_m: float = 12.3      # kingpin -> rear of a 13.6 m trailer
    kingpin_front_m: float = 1.3        # kingpin -> front of the trailer
    trailer_margin_m: float = 1.5       # the approach keeps the truck's body this far off the trailer
    back_out_m: tuple = (0.0, 6.0, 12.0, 18.0)   # reversing first: these lengths (straight or full lock) tried
    behind_deg: float = 100.0           # the line-up point this far off the heading: back out first
    reverse_cost: float = 1.5           # a metre reversed counts as this many driven forwards
    forward_kmh: float = 5.0
    reverse_kmh: float = 2.5
    final_kmh: float = 1.2              # the last metres under the trailer
    lookahead_m: float = 4.0
    reverse_lookahead_m: float = 5.0
    attach_window_m: float = 2.0
    press_every_s: float = 0.8
    overshoot_m: float = 1.5
    line_up_m: float = 0.8              # at the line-up point: this far off the axis at most ...
    line_up_deg: float = 10.0           # ... and this far off its heading
    gear_every_s: float = 1.5
    gear_presses: int = 4
    max_steer_units: float = 0.98


@dataclass
class CouplingStep:
    steer: float = 0.0                  # target effective steering (+ = left), for the caller's servo
    throttle: float = 0.0
    brake: float = 0.0
    press: list = field(default_factory=list)
    reverse: bool = False               # the truck is meant to be in reverse (don't shift it back to drive)
    done: bool = False
    failed: str | None = None
    note: str = ""


class Coupling:
    name = "couple"

    def __init__(self, geom: TruckGeometry, trailer, hook_back_m: float, params: CouplingParams | None = None):
        """`trailer`: TrailerPose (x, y, rotation); `hook_back_m`: fifth wheel behind the truck's origin."""
        self.g, self.t, self.hook_back = geom, trailer, hook_back_m
        self.p = params or CouplingParams()
        self.fwd = np.array([math.cos(trailer.rotation), math.sin(trailer.rotation)])
        self.side = np.array([-self.fwd[1], self.fwd[0]])
        self.kp = np.array([trailer.x, trailer.y]) + self.p.kingpin_ahead_m * self.fwd
        self.phase, self.t_phase, self.presses = "plan", 0.0, 0
        self.path: np.ndarray | None = None              # the rear axle's way to the line-up point
        self.back: np.ndarray | None = None              # ... and its way backing out first (or None)
        self._back_len = 0.0
        self._v = 0.0
        self._len = 0.0
        self._s = 0.0

    # ------------------------------------------------------------- geometry
    def _hook(self, pose) -> np.ndarray:
        return np.array([pose.x, pose.y]) - self.hook_back * np.array([math.cos(pose.rotation), math.sin(pose.rotation)])

    def _rear(self, pose) -> np.ndarray:
        return np.array([pose.x, pose.y]) - self.g.rear_axle * np.array([math.cos(pose.rotation), math.sin(pose.rotation)])

    def _body(self, rear_pts: np.ndarray, backwards: bool = False) -> np.ndarray:
        """Points on the truck (rear overhang to front bumper) along a rear-axle way."""
        d = np.diff(rear_pts, axis=0)
        h = np.arctan2(d[:, 1], d[:, 0]) + (math.pi if backwards else 0.0)
        u = np.stack([np.cos(h), np.sin(h)], axis=1)
        front = self.g.rear_axle + self.g.front_bumper               # rear axle -> bumper
        return np.vstack([rear_pts[1:] + k * front * u for k in (-0.3, 0.0, 0.5, 1.0)])

    def _hits_trailer(self, body: np.ndarray) -> bool:
        """Do these points on the truck (plus a margin) touch the trailer?"""
        p = self.p
        pts = body - self.kp
        along, across = pts @ self.fwd, pts @ self.side
        m = p.trailer_margin_m
        return bool(np.any((along < p.kingpin_front_m + m) & (along > -p.trailer_length_m - m)
                           & (np.abs(across) < 1.3 + m)))

    def _plan(self, pose) -> str | None:
        """The rear axle's way: backing out (or not), then forwards to the line-up point (plus 15 m straight on)."""
        p = self.p
        rear = self._rear(pose)
        h0 = pose.rotation
        u0 = np.array([math.cos(h0), math.sin(h0)])
        bumper0 = float(np.dot(rear, u0)) + self.g.rear_axle + self.g.front_bumper
        r = p.min_radius_m
        best = None
        for stage in p.stage_m:
            end = self.kp + (stage + self.hook_back - self.g.rear_axle) * self.fwd    # the rear axle there
            pre = end - p.straight_m * self.fwd
            q1 = (float(pre[0]), float(pre[1]), self.t.rotation)
            to = pre - rear
            off = abs(math.degrees(math.atan2(math.sin(math.atan2(to[1], to[0]) - h0),
                                              math.cos(math.atan2(to[1], to[0]) - h0))))
            behind = off > p.behind_deg
            for b in p.back_out_m:
                if behind and b == 0.0:
                    continue
                for turn in ("S", "L", "R") if b else ("S",):
                    # backing out = driving the reversed heading forwards
                    back = sample_dubins((float(rear[0]), float(rear[1]), h0 + math.pi), turn, (b / r,), r)
                    if b and self._hits_trailer(self._body(back, backwards=True)):
                        continue
                    hb = h0 + {"S": 0.0, "L": 1.0, "R": -1.0}[turn] * b / r
                    q0 = (float(back[-1, 0]), float(back[-1, 1]), hb)
                    for word, segs in dubins_words(q0, q1, r):
                        pts = sample_dubins(q0, word, segs, r)
                        if math.hypot(pts[-1, 0] - q1[0], pts[-1, 1] - q1[1]) > 0.3:
                            continue                                   # not a solution for this pair of poses
                        body = self._body(pts)
                        if self._hits_trailer(body):
                            continue
                        if behind and float(np.max(body @ u0)) > bumper0 + 0.5:
                            continue                                   # into what the truck was parked against
                        length = sum(segs) * r + p.straight_m
                        cost = length + p.reverse_cost * (b + stage)
                        if best is None or cost < best[0]:
                            best = (cost, length, pts, end, back if b else None, b)
        if best is None or best[1] > p.max_path_m:
            return "coupling: no way to line up with the trailer from here: position the truck in front of it"
        _, length, pts, end, back, b = best
        straight = end + np.outer(np.linspace(-p.straight_m, 15.0, int(p.straight_m + 15.0) + 1), self.fwd)
        self.path = np.vstack([pts, straight[1:]])
        self._len, self._s = length, 0.0
        if back is not None:
            u = back[-1] - back[-2]
            self.back = np.vstack([back, back[-1] + np.outer(np.arange(1.0, 6.0), u / np.hypot(*u))])
        else:
            self.back = None
        self._back_len = b
        return None

    @staticmethod
    def _project(path: np.ndarray, xy: np.ndarray, near: float | None = None, window: float = 12.0) -> float:
        """Arc length of the closest point of `path` to `xy` (within `window` of `near`: the path may loop)."""
        d = np.diff(path, axis=0)
        l2 = np.maximum((d * d).sum(axis=1), 1e-9)
        cum = np.concatenate([[0.0], np.cumsum(np.sqrt(l2))])
        t = np.clip(((xy - path[:-1]) * d).sum(axis=1) / l2, 0, 1)
        dist = np.hypot(*(path[:-1] + d * t[:, None] - xy).T)
        if near is not None:
            dist = np.where((cum[1:] >= near - 2.0) & (cum[:-1] <= near + window), dist, np.inf)
        i = int(np.argmin(dist))
        return float(cum[i] + t[i] * math.sqrt(l2[i]))

    @staticmethod
    def _point(path: np.ndarray, s: float) -> np.ndarray:
        d = np.diff(path, axis=0)
        seg = np.hypot(d[:, 0], d[:, 1])
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        s = min(max(s, 0.0), cum[-1])
        i = min(int(np.searchsorted(cum, s, "right")) - 1, len(seg) - 1)
        return path[i] + d[i] * ((s - cum[i]) / max(seg[i], 1e-9))

    def _steer(self, rear: np.ndarray, rot: float, target: np.ndarray, backwards: bool) -> float:
        """Pure pursuit from the rear axle to `target`: effective steering (+ = left)."""
        heading = rot + (math.pi if backwards else 0.0)
        v = target - rear
        alpha = math.atan2(v[1], v[0]) - heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        ld = max(float(np.hypot(*v)), 1.0)
        delta = math.atan(2.0 * self.g.wheelbase * math.sin(alpha) / ld)   # + = towards +rotation = right (map)
        units = -delta / self.g.rad_per_unit                               # effective steering: + = left
        if backwards:
            units = -units                                                  # reversing: the wheel acts mirrored
        return max(-self.p.max_steer_units, min(self.p.max_steer_units, units))

    @staticmethod
    def _pedals(speed: float, target: float) -> tuple[float, float]:
        """Throttle and brake for a crawl at `target` m/s (speed signed the same way)."""
        err = target - speed
        if err > 0:
            return min(0.45, 0.18 + 0.25 * err), 0.0
        return 0.0, (min(0.6, 0.15 - 0.4 * err) if err < -0.3 else 0.0)

    def line_up_error(self, pose) -> tuple[float, float]:
        """The fifth wheel's distance off the trailer's axis (m) and the heading difference (deg)."""
        off = float(np.dot(self._hook(pose) - self.kp, self.side))
        dh = math.degrees(math.atan2(math.sin(pose.rotation - self.t.rotation), math.cos(pose.rotation - self.t.rotation)))
        return off, dh

    def _shift(self, out: CouplingStep, button: str, note: str) -> CouplingStep:
        """Press `button` every `gear_every_s` (standing on the brake) until the gearbox shows it, or give up."""
        p = self.p
        if self.t_phase >= p.gear_every_s and abs(self._v) < 0.15:     # standing: the gearbox ignores it rolling
            if self.presses >= p.gear_presses:
                out.failed = f"coupling: the gearbox didn't take `{button}`: couple the trailer yourself"
                return out
            out.press.append(button)
            self.presses += 1
            self.t_phase = 0.0
        out.note = f"coupling: {note}"
        return out

    # ------------------------------------------------------------- step
    def step(self, frame, dt: float) -> CouplingStep:
        p = self.p
        pose = pose_from_frame(frame)
        self.t_phase += dt
        out = CouplingStep()
        trailers = getattr(frame, "trailers", None)
        coupled = bool(trailers and trailers[0].connected)
        gear = int(getattr(frame, "displayed_gear", 1))
        v = self._v = float(frame.speed)
        if self.phase == "plan":
            why = self._plan(pose)
            if why:
                out.failed = why
                return out
            self.phase, self.t_phase, self.presses = ("to reverse", 1e9, 0) if self.back is not None \
                else ("approach", 0.0, 0)
        if self.phase == "to reverse":
            out.brake, out.reverse = 0.4, True
            if gear < 0:
                self.phase, self.t_phase, self._s = "back out", 0.0, 0.0
            else:
                return self._shift(out, "gearreverse", "backing out: shifting to reverse")
        if self.phase == "back out":
            out.reverse = True
            rear = self._rear(pose)
            s = self._s = max(self._s, self._project(self.back, rear, self._s))
            left = self._back_len - s
            if left <= 0.3 and abs(v) < 0.2:
                self.phase, self.t_phase, self.presses = "to drive", 1e9, 0
            else:
                target = self._point(self.back, s + p.reverse_lookahead_m)
                out.steer = self._steer(rear, pose.rotation, target, backwards=True)
                want = min(p.reverse_kmh / 3.6, math.sqrt(2 * 0.5 * max(left - 0.3, 0.0)))
                out.throttle, out.brake = self._pedals(-v, want)
                if left <= 0.3:
                    out.throttle, out.brake = 0.0, 0.6
                out.note = f"coupling: backing out ({left:.0f} m)"
                return out
        if self.phase == "to drive":
            out.brake = 0.4
            if gear > 0:
                self.phase, self.t_phase, self._s = "approach", 0.0, 0.0
            else:
                out.reverse = True                      # still in reverse until the shift shows
                return self._shift(out, "geardrive", "backed out: shifting to drive")
        if self.phase == "approach":
            rear = self._rear(pose)
            s = self._s = max(self._s, self._project(self.path, rear, self._s))
            left = self._len - s
            if left <= 0.3 and abs(v) < 0.2:
                self.phase, self.t_phase = "check", 0.0
            else:
                target = self._point(self.path, s + p.lookahead_m)
                out.steer = self._steer(rear, pose.rotation, target, backwards=False)
                want = min(p.forward_kmh / 3.6, math.sqrt(2 * 0.6 * max(left - 0.3, 0.0)))
                out.throttle, out.brake = self._pedals(v, want)
                if left <= 0.3:
                    out.throttle, out.brake = 0.0, 0.6
                out.note = f"coupling: lining up with the trailer ({left:.0f} m)"
                return out
        if self.phase == "check":
            off, dh = self.line_up_error(pose)
            if abs(off) > p.line_up_m or abs(dh) > p.line_up_deg:
                out.brake = 0.6
                out.failed = (f"coupling: lined up {abs(off):.1f} m / {abs(dh):.0f} deg off the trailer's axis: "
                              "couple it yourself, then press / to go on")
                return out
            self.phase, self.t_phase, self.presses = "shift", 1e9, 0
        if self.phase == "shift":
            out.brake, out.reverse = 0.4, True
            if gear < 0:
                self.phase, self.t_phase = "reverse", 0.0
            else:
                return self._shift(out, "gearreverse", "shifting to reverse")
        if self.phase == "reverse":
            out.reverse = True
            if coupled:
                self.phase, self.t_phase, self.presses = "drive", 1e9, 0
            else:
                along = float(np.dot(self._hook(pose) - self.kp, self.fwd))     # > 0: still in front of the kingpin
                if along < -p.overshoot_m:
                    out.brake = 0.6
                    out.failed = "coupling: reversed past the kingpin without coupling: couple it yourself"
                    return out
                rear = self._rear(pose)
                # aim at a point on the trailer's axis `reverse_lookahead_m` behind the rear axle
                r_along = float(np.dot(rear - self.kp, self.fwd))
                target = self.kp + (r_along - p.reverse_lookahead_m) * self.fwd
                out.steer = self._steer(rear, pose.rotation, target, backwards=True)
                crawl = p.final_kmh if along < p.attach_window_m + 2.0 else p.reverse_kmh
                out.throttle, out.brake = self._pedals(-v, crawl / 3.6)       # reversing: speed is negative
                if along < p.attach_window_m and self.t_phase >= p.press_every_s:
                    out.press.append("attach")
                    self.t_phase = 0.0
                out.note = f"coupling: reversing under the trailer ({max(along, 0.0):.1f} m)"
                return out
        if self.phase == "drive":
            out.brake = 0.5
            if gear > 0:
                out.done, out.note = True, "trailer coupled"
                return out
            if self.t_phase >= p.gear_every_s and abs(v) < 0.15:
                if self.presses >= p.gear_presses:
                    out.done, out.note = True, "trailer coupled (still in reverse)"
                    return out
                out.press.append("geardrive")
                self.presses += 1
                self.t_phase = 0.0
            out.note = "coupling: coupled, shifting to drive"
        return out
