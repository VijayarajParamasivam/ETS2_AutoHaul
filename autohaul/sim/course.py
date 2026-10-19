"""Synthetic lane maps and a closed-loop runner (Autopilot <-> TruckSim) for tests and tuning."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from autohaul.control.path import PathRef
from autohaul.sim.truck import SimVehicle, TruckSim
from autohaul.world.lanemap import CACHE_VERSION, KIND_ROAD, LaneMap


def arc_polyline(pieces: list[tuple[float, float]], x: float = 0.0, y: float = 0.0, rot: float = 0.0,
                 step: float = 4.0) -> np.ndarray:
    """Polyline from (length m, curvature 1/m; + = right turn) pieces, like a road built from clothoid-free arcs."""
    pts = [(x, y)]
    for length, kappa in pieces:
        n = max(1, int(math.ceil(length / step)))
        ds = length / n
        for _ in range(n):
            r_mid = rot + kappa * ds / 2
            x += ds * math.cos(r_mid)
            y += ds * math.sin(r_mid)
            rot += kappa * ds
            pts.append((x, y))
    return np.array(pts)


def split_lanes(pts: np.ndarray, lane_len: float = 200.0) -> list[np.ndarray]:
    """Cut a polyline into consecutive lanes (sharing end points), like roads joined at map nodes."""
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    cuts = [0]
    for i in range(1, len(pts)):
        if s[i] - s[cuts[-1]] >= lane_len:
            cuts.append(i)
    if cuts[-1] != len(pts) - 1:
        cuts.append(len(pts) - 1)
    return [pts[a:b + 1] for a, b in zip(cuts[:-1], cuts[1:]) if b > a]


def lanes_to_map(lanes: list[np.ndarray]) -> LaneMap:
    rows, ids = [], []
    for i, pts in enumerate(lanes):
        n = len(pts) - 1
        arr = np.zeros((n, 6))
        arr[:, 0:2], arr[:, 2:4] = pts[:-1], pts[1:]
        rows.append(arr)
        ids.append(np.full(n, i, dtype=np.int32))
    k = len(lanes)
    return LaneMap(np.concatenate(rows), np.concatenate(ids), np.full(k, KIND_ROAD, np.uint8),
                   np.arange(k, dtype=np.uint64), np.ones(k, np.int16), {"cache_version": CACHE_VERSION})


def offset(pts: np.ndarray, d: float) -> np.ndarray:
    """Parallel polyline d metres to the right."""
    t = np.gradient(pts, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    return pts + np.stack([-t[:, 1], t[:, 0]], axis=1) * d


# A highway-ish test course: straights, gentle and medium curves both ways, an S-bend (radii in m).
HIGHWAY = [(300, 0.0), (250, 1 / 600), (200, 0.0), (300, -1 / 450), (150, 0.0), (200, 1 / 350), (200, -1 / 350),
           (400, 0.0), (300, 1 / 900), (300, 0.0)]


@dataclass
class Course:
    lanes: list[np.ndarray]
    lm: LaneMap
    centre: np.ndarray            # the polyline the truck should follow
    path: PathRef = field(init=False)

    def __post_init__(self):
        self.path = PathRef(self.centre)

    @classmethod
    def from_pieces(cls, pieces, two_lanes: bool = True, lane_len: float = 200.0) -> "Course":
        centre = arc_polyline(pieces)
        lanes = split_lanes(centre, lane_len)
        if two_lanes:                                  # a parallel lane to the right, to tempt the tracker
            lanes += split_lanes(offset(centre, 4.5), lane_len)
        return cls(lanes, lanes_to_map(lanes), centre)


@dataclass
class RunLog:
    t: list = field(default_factory=list)
    lateral: list = field(default_factory=list)       # true offset of the truck origin from the course centre
    speed: list = field(default_factory=list)
    target: list = field(default_factory=list)
    gap: list = field(default_factory=list)           # true gap to vehicle 0 (bumper to rear), if any
    steer_cmd: list = field(default_factory=list)
    brake: list = field(default_factory=list)
    fault: str | None = None
    s_end: float = 0.0

    def arr(self, name: str) -> np.ndarray:
        return np.array(getattr(self, name), dtype=float)


def run(ap, sim: TruckSim, course: Course, seconds: float, speed_limit: float = 80 / 3.6,
        vehicles: list[SimVehicle] | None = None, stop_on_fault: bool = True, every=None) -> RunLog:
    """Closed loop for `seconds` of simulated time. `every(t, sim, status)` is called each step (tests)."""
    vehicles = vehicles or []
    log = RunLog()
    frame = sim.frame(speed_limit)
    reason = ap.engage(frame)
    if reason:
        log.fault = f"engage refused: {reason}"
        return log
    dt = sim.p.dt
    for _ in range(int(seconds / dt)):
        frame = sim.frame(speed_limit)
        cmd, st = ap.step(frame, [v.record() for v in vehicles], dt)
        if every:
            every(sim.t, sim, st)
        pr = course.path.project(sim.s.x, sim.s.y)
        log.t.append(sim.t)
        log.lateral.append(pr.lateral)
        log.speed.append(sim.s.speed)
        log.target.append(st.target_kmh / 3.6)
        log.steer_cmd.append(cmd.steer)
        log.brake.append(cmd.brake)
        if vehicles:
            v0 = vehicles[0]
            log.gap.append(v0.s - v0.length / 2 - (pr.s + sim.geom.front_bumper))
        log.s_end = pr.s
        if st.fault:
            log.fault = st.fault
            if stop_on_fault:
                break
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
        for v in vehicles:
            v.step(sim.t, dt)
        if pr.s > course.path.length - 5:
            break
    return log
