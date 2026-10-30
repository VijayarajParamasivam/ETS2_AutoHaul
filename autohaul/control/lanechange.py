"""Lane changes: is the target lane clear, and the smooth sideways move.

The move follows the target lane with a sideways offset that eases from where the truck is (about one lane
width away) to 0 along a quintic (zero lateral speed and acceleration at both ends) over
L = max(min_len_m, len_s x speed) - about 1 m/s^2 of lateral acceleration at motorway speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from autohaul.control.speed import _boxes
from autohaul.control.vehicle import Pose
from autohaul.world.lanemap import quat_forward_rotation, wrap


@dataclass
class LaneChangeParams:
    signal_s: float = 3.0              # indicator on at least this long before moving
    min_len_m: float = 90.0
    len_s: float = 5.0
    lane_width: float = 4.5
    band_margin: float = 0.6           # widen the target-lane band by this much each side
    front_gap_min: float = 10.0        # m to the vehicle ahead in the target lane ...
    front_gap_s: float = 0.8           # ... or this many seconds at our speed
    front_brake_decel: float = 1.5     # m/s^2: closing on it, room to brake to its speed as well
    stopped_ahead_m: float = 150.0     # a standing vehicle this close ahead in the target lane while we close on it
                                       # (2026-10-29: a wreck across two lanes): wait until past it
    rear_gap_min: float = 10.0         # m from our rear to the vehicle behind in the target lane ...
    rear_gap_s: float = 0.5
    rear_closing_s: float = 3.0        # ... plus this long at its closing speed (a move takes ~5 s)
    tractor_rear_m: float = 3.5        # our rear end behind the origin: tractor only ...
    trailer_rear_m: float = 17.0       # ... and with a semi-trailer
    deadline_margin_m: float = 20.0    # the move must end this far before the lane does
    reroute_max_extra_m: float = 300.0   # no room for the change: a way on from this lane may be this much ...
    reroute_max_extra_share: float = 0.03  # ... or this share of the remaining route longer (2026-10-29: a
                                           # 15 km detour was taken instead of merging off a 98 m slip lane)
    short_len_m: float = 35.0          # otherwise squeeze the change into what's left, down to this length
    short_signal_s: float = 1.0        # ... with a shorter signal
    stand_wait_s: float = 30.0         # standing this long waiting for a gap: hand back
    signal_wait_m: float = 300.0       # no gap yet: the indicator stays off until the change is this close ...
    signal_wait_s: float = 15.0        # ... or this many seconds away (2026-10-30: on for up to 76 s, waiting)
    signal_keep_s: float = 5.0         # once on, it stays on at least this long (a gap that comes and goes)
    parallel_m: float = 7.0            # the neighbouring lane still runs along the route if it ends this close


def quintic(u: float) -> tuple[float, float, float]:
    """S(u), S'(u), S''(u) of the 0 -> 1 quintic with zero slope and curvature at both ends."""
    u = min(max(u, 0.0), 1.0)
    return (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5, 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4,
            60 * u - 180 * u ** 2 + 120 * u ** 3)


def offset_profile(o0: float, length: float):
    """y(d), dy/dd, d2y/dd2 for d metres into the move: from o0 (start offset, + = right) to 0."""
    def f(d: float) -> tuple[float, float, float]:
        s, s1, s2 = quintic(d / length)
        return o0 * (1 - s), -o0 * s1 / length, -o0 * s2 / (length * length)
    return f


def gap_check(pose: Pose, side: int, traffic: list[dict] | None, front_bumper: float, rear_extent: float,
              p: LaneChangeParams) -> str | None:
    """Why the lane on `side` (+1 right, -1 left) is not clear right now, or None if it is."""
    if not traffic:
        return None
    ct, st = math.cos(pose.rotation), math.sin(pose.rotation)
    v = max(pose.speed, 0.0)
    lo, hi = p.lane_width / 2 - p.band_margin, 1.5 * p.lane_width + p.band_margin
    for veh in traffic:
        for x, z, qw, qx, qy, qz, length, width in _boxes(veh):
            dx = (x - pose.x) * ct + (z - pose.y) * st            # ahead of our origin
            dy = -(x - pose.x) * st + (z - pose.y) * ct           # right of our heading
            if not (lo - width / 2 <= side * dy <= hi + width / 2):
                continue
            vel = veh["speed"] * math.cos(float(wrap(quat_forward_rotation(qw, qx, qy, qz) - pose.rotation)))
            front, rear = dx + length / 2, dx - length / 2
            if rear > front_bumper:
                gap = rear - front_bumper
                closing = max(v - vel, 0.0)
                if vel < 1.5 and closing > 3.0 and gap < p.stopped_ahead_m:
                    return f"stopped vehicle ahead in the target lane at {gap:.0f} m"
                need = (max(p.front_gap_min, p.front_gap_s * v) + closing * 1.0
                        + closing * closing / (2 * p.front_brake_decel))
                if gap < need:
                    return f"vehicle ahead in the target lane at {gap:.0f} m"
            elif front < -rear_extent:
                gap = -rear_extent - front
                need = max(p.rear_gap_min, p.rear_gap_s * v + p.rear_closing_s * max(vel - v, 0.0))
                if gap < need:
                    return f"vehicle behind in the target lane at {gap:.0f} m"
            else:
                return "vehicle alongside in the target lane"
    return None


@dataclass
class LaneChange:
    target: int                # lane to move into
    side: int                  # +1 right, -1 left
    started: float             # autopilot clock when the indicator went on
    phase: str = "signal"      # signal -> move
    d0: float = 0.0            # odometer at the start of the move
    length: float = 0.0
    profile: object = None
    waiting: str = ""          # why it hasn't moved yet
    stood_since: float | None = None   # autopilot clock since the truck stands waiting for a gap
    signalling: bool = False           # the indicator is on for this change
    signal_on: float = 0.0             # ... since then

    @property
    def side_name(self) -> str:
        return "right" if self.side > 0 else "left"
