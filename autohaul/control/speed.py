"""Longitudinal control: target speed (limit, curves, set speed), following distance (ACC), pedals.

Structure:
  SpeedPlanner   -> the speed we may drive now: min(set speed, posted limit - margin, curve speed with
                    braking distance to every curve on the path ahead)
  find_lead      -> the nearest AI vehicle (or its trailer) in our lane on the path ahead
  acc_accel      -> acceleration that keeps a constant time gap to that lead vehicle
  PedalController-> turns the acceleration request into throttle/brake: a jerk-limited reference
                    speed tracked by a PI loop with feedforward (robust to gear shifts and unknown mass)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from autohaul.control.path import PathRef
from autohaul.world.lanemap import quat_forward_rotation, wrap

KMH = 1 / 3.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class SpeedParams:
    max_speed_kmh: float = 85.0
    limit_margin_kmh: float = 2.0
    unknown_limit_kmh: float = 60.0     # when telemetry reports no limit
    lat_acc_max: float = 2.0            # m/s^2 in curves (cargo + rollover margin; drivers take ~2.6)
    steer_rate_margin: float = 0.6      # plan curves with this fraction of the wheel's top turn rate
    min_curve_kmh: float = 8.0          # the steering-rate limit never plans slower than this
    steer_reach_deadband: float = 0.06  # rad (3.4 deg) of wheel-angle difference ignored by the reach check
    steer_rate_window_m: float = 6.0    # curvature window for the wheel-rate checks
    plan_decel: float = 0.8             # m/s^2 planned braking toward curves / lower limits
    reaction_s: float = 1.0             # s of travel before planned braking starts
    stop_arrive_m: float = 1.0          # this close to a stop point the truck has arrived
    accel_max: float = 0.7              # m/s^2
    decel_comfort: float = 1.5          # m/s^2, normal braking authority
    decel_max: float = 5.0              # m/s^2, emergency
    jerk_max: float = 0.8               # m/s^3 (normal)
    release_fast_below: float = 5.0     # m/s: below this, braking is let off at 4 x jerk_max
    k_speed: float = 0.35               # 1/s, speed error -> acceleration request
    acc_time_gap_s: float = 2.2
    acc_standstill_m: float = 10.0
    acc_k_gap: float = 0.12             # 1/s^2
    acc_k_rel: float = 0.55             # 1/s
    acc_range_m: float = 180.0
    lane_half_width: float = 2.25
    crossing_moving_ms: float = 2.0     # a vehicle crossing our lane faster than this clears it by itself
    oncoming_lateral_m: float = 1.0     # an oncoming vehicle is a lead only this close to our lane centre
    beside_gap_m: float = -1.0          # a same-way vehicle whose rear is this far behind our bumper is beside us
    # pedal loop
    kp: float = 0.25                    # pedal per m/s
    ki: float = 0.08                    # pedal per m (integral of speed error)
    i_limit: float = 0.4
    thr_accel: float = 1.0              # m/s^2 at full throttle (feedforward guess; the integrator trims)
    brake_decel: float = 4.0            # m/s^2 at full brake
    coast_decel: float = 0.25           # m/s^2 the truck slows by itself with pedals released
    throttle_rate: float = 0.8          # pedal units/s
    brake_rate: float = 1.5


@dataclass
class Lead:
    gap: float          # m, our front bumper to its rear (along the path)
    speed: float        # m/s along our path direction
    accel: float
    lateral: float
    vid: int


class SpeedPlanner:
    def __init__(self, params: SpeedParams):
        self.p = params

    def limit_speed(self, nav_limit: float) -> float:
        p = self.p
        lim = nav_limit if nav_limit > 0.5 else p.unknown_limit_kmh * KMH
        return min(p.max_speed_kmh * KMH, lim - p.limit_margin_kmh * KMH)

    def ahead(self, path: PathRef, s0: float, speed: float, stop_at: float | None = None,
              steer_rate: float | None = None, wheelbase: float = 3.82, steer_now: float = 0.0,
              points: list[tuple[float, float, str]] | None = None) -> tuple[float, float, float, str]:
        """Constraints on the path ahead: every point's curve speed, and a stop at `stop_at` (arc length).

        Returns (allowed speed now, deceleration needed now (<= 0), distance to the limiting point, kind),
        where "allowed now" still lets us slow at plan_decel after the reaction distance, and the
        needed deceleration is what reaches the tightest constraint exactly (used as feedforward, so
        the truck brakes on a plan instead of trailing a falling target).

        `steer_rate` (rad/s the wheel can turn; relative steering): also slow down so the wheel keeps up
        with how fast the curvature changes (v <= rate / |d steer / ds|) and can get from its current
        angle (`steer_now`, map convention: + = right) to each point's angle in time.
        `points`: more (arc length, speed, kind) constraints, e.g. a slow approach to a give-way line."""
        p = self.p
        s = np.arange(max(s0, 0.0), path.length, 4.0)
        k = np.asarray(path.curvature(s)) if len(s) else np.empty(0)
        v_c = np.sqrt(p.lat_acc_max / np.maximum(np.abs(k), 1e-5))
        if steer_rate and len(s) > 2:
            r = steer_rate * p.steer_rate_margin
            # the wheel must follow short curves too (junction pieces of a few metres): finer window here
            delta = np.arctan(np.asarray(path.curvature(s, p.steer_rate_window_m)) * wheelbase)
            v_rate = r / np.maximum(np.abs(np.gradient(delta, s)), 1e-6)
            # differences of a few degrees are ordinary corrections, not a wheel that must swing; and
            # a point is never closer than half a second of travel
            swing = np.abs(delta - steer_now) - p.steer_reach_deadband
            room = np.maximum(s - s0, 0.0) + 0.5 * max(speed, 2.0)
            reach = np.where(swing > 0, room * r / np.maximum(swing, 1e-6), np.inf)
            v_c = np.maximum(np.minimum(v_c, np.minimum(v_rate, reach)), p.min_curve_kmh / 3.6)
        kinds = ["curve"] * len(s)
        if stop_at is not None:
            s = np.append(s, stop_at)
            v_c = np.append(v_c, 0.0)
            kinds.append("stop")
        for at, v_at, kind in points or ():
            s = np.append(s, at)
            v_c = np.append(v_c, v_at)
            kinds.append(kind)
        if len(s) == 0:
            return math.inf, 0.0, 0.0, ""
        d = np.maximum(s - s0, 0.0)
        d_brake = np.maximum(d - speed * p.reaction_s, 0.0)
        # within the last metre of a stop point the truck has arrived: no creeping up to it (and past it)
        d_brake = np.where(v_c == 0.0, np.maximum(d - max(speed * p.reaction_s, p.stop_arrive_m), 0.0), d_brake)
        allowed = np.sqrt(v_c ** 2 + 2 * p.plan_decel * d_brake)
        i = int(np.argmin(allowed))
        slower = v_c < speed
        a_need = 0.0
        if slower.any():
            # a stop point is braked for over all the room left (down to the last decimetres), so the truck
            # stops on it instead of rolling past at walking pace
            room = np.where(v_c == 0.0, np.maximum(d, 0.2), np.maximum(d_brake, 1.0))
            need = (v_c[slower] ** 2 - speed ** 2) / (2 * room[slower])
            a_need = float(need.min())
        return float(allowed[i]), a_need, float(d[i]), kinds[i]


def _boxes(v: dict):
    yield v["wx"], v["wz"], v["qw"], v["qx"], v["qy"], v["qz"], v["length"], v["width"]
    for t in v.get("trailers", [])[: max(0, int(v.get("trailer_count", 0)))]:
        if t["length"] > 0:
            yield t["x"], t["z"], t["qw"], t["qx"], t["qy"], t["qz"], t["length"], t["width"]


def find_lead(path: PathRef, s_front: float, traffic: list[dict] | None, params: SpeedParams,
              ignore_ids: tuple[int, ...] = ()) -> Lead | None:
    """Nearest vehicle body or trailer in our lane ahead of the front bumper that we must follow or stop for.

    By its heading against our path:
      * same way (within 45 deg): overlapping our lane ahead. One beside us (its rear behind our bumper) is
        passing or being passed: not a lead.
      * crossing (45-135 deg): only one standing or crawling in our lane. A moving one clears the lane by itself
        (crossing traffic at junctions is the give-way logic's job). These had AutoHaul braking for cars that
        crossed in front of it at 40-50 km/h.
      * oncoming (over 135 deg): only one in the middle of our lane (head on). On bends and narrow roads the
        next lane's oncoming cars project close to our lane: one of them triggered a full emergency brake."""
    if not traffic:
        return None
    best: Lead | None = None
    for v in traffic:
        if v["id"] in ignore_ids:
            continue
        for x, z, qw, qx, qy, qz, length, width in _boxes(v):
            pr = path.project(x, z)
            if pr.s > s_front + params.acc_range_m or pr.s > path.length + 5:
                continue
            rel_rot = float(wrap(quat_forward_rotation(qw, qx, qy, qz) - pr.rotation))
            c, sn = math.cos(rel_rot), abs(math.sin(rel_rot))
            along = 0.5 * (length * abs(c) + width * sn)       # half extents along and across our lane
            across = 0.5 * (length * sn + width * abs(c))
            gap = pr.s - along - s_front
            sp = v["speed"] * c
            if c > 0.707:
                if gap < params.beside_gap_m or abs(pr.lateral) > params.lane_half_width + across - 0.6:
                    continue
            elif c < -0.707:
                if gap < 0 or abs(pr.lateral) > params.oncoming_lateral_m:
                    continue
            else:
                if gap < 0 or abs(v["speed"]) > params.crossing_moving_ms                         or abs(pr.lateral) > params.lane_half_width + across - 0.6:
                    continue
            if best is None or gap < best.gap:
                best = Lead(gap, sp, v.get("acceleration", 0.0) * c, pr.lateral, int(v["id"]))
    return best


def acc_accel(lead: Lead, speed: float, p: SpeedParams) -> tuple[float, bool]:
    """Constant-time-gap following. Returns (acceleration request, emergency?)."""
    desired = p.acc_standstill_m + p.acc_time_gap_s * max(speed, 0.0)
    a = p.acc_k_gap * (lead.gap - desired) + p.acc_k_rel * (lead.speed - speed) + 0.3 * min(lead.accel, 0.0)
    closing = speed - lead.speed
    emergency = False
    if closing > 0.5:
        room = max(lead.gap - 4.0, 0.5)
        need = closing * closing / (2 * room)          # decel that stops the closing before the gap is gone
        if need > p.decel_comfort * 0.8 or lead.gap / closing < 3.0:
            a = min(a, -1.2 * need)
            emergency = need > p.decel_comfort
    if lead.gap < 0.5 * desired and lead.gap < 15:
        a = min(a, -p.decel_comfort)
    return a, emergency


class PedalController:
    def __init__(self, params: SpeedParams):
        self.p = params
        self.reset(0.0)

    def reset(self, speed: float) -> None:
        self.v_ref = speed
        self.a_ref = 0.0
        self.i = 0.0
        self.throttle = 0.0
        self.brake = 0.0

    def step(self, a_request: float, speed: float, dt: float, emergency: bool = False,
             v_cap: float = math.inf) -> tuple[float, float]:
        """`v_cap`: the target speed; the reference never runs past it (no overshoot after long climbs)."""
        p = self.p
        a_min = -p.decel_max if emergency else -p.decel_comfort
        a_goal = _clamp(a_request, a_min, p.accel_max)
        # faster: emergencies, sudden hard braking, and letting off the brake at low speed (in a slow junction
        # turn the comfort limit made the truck brake on past its target, down to walking pace)
        fast = emergency or a_goal < self.a_ref - 1.0 or (a_goal > self.a_ref and speed < p.release_fast_below)
        jerk = 4.0 * p.jerk_max if fast else p.jerk_max
        self.a_ref += _clamp(a_goal - self.a_ref, -jerk * dt, jerk * dt)
        self.v_ref = max(0.0, min(self.v_ref + self.a_ref * dt, max(v_cap, speed - 1.5)))
        # don't let the reference run away from the truck (e.g. a loaded truck that can't keep up)
        self.v_ref = _clamp(self.v_ref, speed - 1.5, speed + 1.5)
        e = self.v_ref - speed
        if v_cap < 0.5:
            self.i = min(self.i, 0.0)          # stopping: what the integrator learned must not push throttle now
        a_ff = self.a_ref + p.coast_decel
        ff = a_ff / p.thr_accel if a_ff > 0 else a_ff / p.brake_decel
        u = ff + p.kp * e + self.i
        # integrate only near the reference and not into saturation (no windup on long climbs)
        if abs(e) < 1.0 and (-1.0 < u < 1.0 or (u >= 1.0 and e < 0) or (u <= -1.0 and e > 0)):
            self.i = _clamp(self.i + p.ki * e * dt, -p.i_limit, p.i_limit)
        thr_goal, brk_goal = max(u, 0.0), max(-u - 0.02, 0.0)     # small band: coast, don't drag the brakes
        if speed < 0.3 and a_goal <= 0:
            thr_goal, brk_goal = 0.0, max(brk_goal, 0.3)      # hold the truck at a standstill
        brate = 6.0 if emergency else p.brake_rate
        self.throttle += _clamp(min(thr_goal, 1.0) - self.throttle, -2.0 * dt, p.throttle_rate * dt)
        self.brake += _clamp(min(brk_goal, 1.0) - self.brake, -2.0 * dt, brate * dt)
        if self.brake > 0.05:
            self.throttle = 0.0
        return self.throttle, self.brake
