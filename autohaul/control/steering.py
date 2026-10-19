"""Steering: an inner servo that puts the game's wheel where we want it, and the lane-keeping law.

Relative steering mode (the user's setting, c_relatsteer 1): the game *integrates* the semantical
steering axis, so the axis sets how fast the wheel turns, not where it is:
    d(effective steering)/dt = -c_rsteersens * command        (measured 0.42 /s per unit on 1.53.2.1)
and the wheel stays where it is at command 0. RelativeServo closes the loop on telemetry's effective
steering. Absolute mode (c_relatsteer 0): effective steering = -command, so AbsoluteServo is a pass-through.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from autohaul.control.path import PathRef, _wrap
from autohaul.control.vehicle import Pose, TruckGeometry, curvature_to_left


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class RelativeServo:
    def __init__(self, gain: float = 0.42, k: float = 6.0, max_cmd: float = 1.0, deadzone: float = 0.0,
                 adapt: bool = True, fight_window_s: float = 0.3, fight_threshold: float = 0.05):
        self.prior = gain
        self.k = k
        self.max_cmd = max_cmd
        self.deadzone = deadzone            # game-side axis deadzone (fraction), compensated if > 0
        self.adapt = adapt
        self.fight_window_s = fight_window_s
        self.fight_threshold = fight_threshold
        self.reset()

    def reset(self) -> None:
        self.gain = self.prior
        self._prev_eff: float | None = None
        self._prev_cmd = 0.0
        self._prev_prev_cmd = 0.0
        self._resid: deque[tuple[float, float, float]] = deque()   # (dt, residual, |predicted change|)
        self.fighting = False

    def _to_axis(self, cmd: float) -> float:
        """Pre-compensate a game-side deadzone: the game maps |axis| in [dz, 1] onto [0, 1]."""
        if self.deadzone <= 0 or cmd == 0.0:
            return cmd
        return math.copysign(self.deadzone + abs(cmd) * (1 - self.deadzone), cmd)

    def step(self, target: float, eff: float, dt: float) -> float:
        """Axis command (+ = steer right) that moves effective steering (+ = left) toward target."""
        target = _clamp(target, -1.0, 1.0)
        if self._prev_eff is not None and 0.004 < dt < 0.1:
            change = eff - self._prev_eff
            predicted = -self.gain * self._prev_cmd * dt
            # learn the rate gain from steady commands (the game applies a command a frame later)
            if self.adapt and abs(self._prev_cmd) > 0.15 and abs(self._prev_cmd - self._prev_prev_cmd) < 0.05:
                inst = -change / (self._prev_cmd * dt)
                if 0 < inst < 5 * self.prior:
                    self.gain = _clamp(self.gain + 0.03 * (inst - self.gain), 0.4 * self.prior, 2.5 * self.prior)
            self._resid.append((dt, change - predicted, abs(predicted)))
            span = sum(r[0] for r in self._resid)
            while self._resid and span - self._resid[0][0] >= self.fight_window_s:
                span -= self._resid.popleft()[0]
            resid = sum(r[1] for r in self._resid)
            moved = sum(r[2] for r in self._resid)
            # the wheel moved by more than our own command explains: someone else is steering
            self.fighting = span >= self.fight_window_s * 0.9 and abs(resid) > self.fight_threshold + 0.4 * moved
        self._prev_eff = eff
        rate = self.k * (target - eff)
        cmd = _clamp(-rate / self.gain, -self.max_cmd, self.max_cmd)
        if abs(target - eff) < 0.0002:
            cmd = 0.0
        self._prev_prev_cmd, self._prev_cmd = self._prev_cmd, cmd
        return self._to_axis(cmd)


class AbsoluteServo:
    """Absolute steering mode: the axis *is* the wheel position (effective = -command)."""

    def __init__(self, gain: float = -1.0):
        self.gain = gain
        self.fighting = False

    def reset(self) -> None:
        pass

    def step(self, target: float, eff: float, dt: float) -> float:
        return _clamp(target / self.gain, -1.0, 1.0)


@dataclass
class LateralParams:
    omega_max: float = 0.6          # rad/s, natural frequency of the lateral-error response at speed
    zeta: float = 0.9
    d_min: float = 12.0             # m: at low speed omega = v / d_min (converge within ~a truck length)
    predict_s: float = 0.25         # s: control the predicted pose (covers servo + game latency)
    ff_preview_s: float = 0.15      # s: curvature feedforward looks this far ahead of the rear axle
    ki: float = 0.04                # 1/s: integral on lateral error (removes offsets in long curves)
    i_limit_m: float = 0.5
    max_corr_lat_acc: float = 1.0   # m/s^2: cap on the correction's lateral acceleration
    max_corr_kappa: float = 0.06    # 1/m: and on its curvature
    # The truck turns less than the kinematic model says at speed (tyre slip). Learn the ratio
    # measured curvature (yaw rate / speed) : kinematic curvature (wheel angle) and steer by it.
    adapt_tau_s: float = 2.0
    ratio_min: float = 0.5
    ratio_max: float = 1.3
    # Relative steering turns the wheel at a limited rate (0.42 units/s at full axis): look ahead and
    # start turning early enough that the wheel can follow the curvature of the next seconds.
    preview_horizon_s: float = 3.0
    rate_margin: float = 0.7        # plan with this fraction of the wheel's top rate


@dataclass
class LateralOut:
    steer_left: float      # rad, single-track steer angle target (+ = left)
    kappa_cmd: float
    kappa_ff: float
    lateral: float         # m, rear-axle lateral error at the predicted pose (+ = right of path)
    heading_err: float     # rad (+ = pointing right of the path)
    s_rear: float
    ratio: float           # learned curvature response (1 = kinematic)
    rate_limited: bool = False   # the wheel can't turn fast enough for the curvature ahead at this speed


class LateralController:
    def __init__(self, geom: TruckGeometry, params: LateralParams | None = None):
        self.geom = geom
        self.p = params or LateralParams()
        self.y_int = 0.0
        self.ratio = 1.0

    def reset(self) -> None:
        self.y_int = 0.0      # keep the learned ratio: it belongs to the truck, not to the engagement

    def learn(self, pose: Pose, steer_left: float, dt: float) -> None:
        """Update the curvature-response ratio from the measured yaw rate and wheel angle."""
        v = pose.speed
        k_kin = -math.tan(steer_left) / self.geom.wheelbase
        if v > 8.0 and abs(k_kin) > 2e-4:
            inst = (pose.yaw_rate / v) / k_kin
            if 0.2 < inst < 2.0:
                a = min(1.0, dt / self.p.adapt_tau_s) * min(1.0, abs(k_kin) / 1e-3)
                self.ratio = _clamp(self.ratio + a * (inst - self.ratio), self.p.ratio_min, self.p.ratio_max)

    def step(self, pose: Pose, path: PathRef, dt: float, steer_measured: float | None = None,
             steer_rate: float | None = None) -> LateralOut:
        """`steer_rate`: fastest the wheel can turn, rad/s of single-track angle (None = no limit)."""
        p, g = self.p, self.geom
        if steer_measured is not None:
            self.learn(pose, steer_measured, dt)
        v = max(abs(pose.speed), 0.5)
        # predicted pose after the loop latency (constant speed and yaw rate)
        tau = p.predict_s
        r_mid = pose.rotation + pose.yaw_rate * tau / 2
        x = pose.x + pose.speed * tau * math.cos(r_mid)
        y = pose.y + pose.speed * tau * math.sin(r_mid)
        rot = pose.rotation + pose.yaw_rate * tau
        rx, ry = x - g.rear_axle * math.cos(rot), y - g.rear_axle * math.sin(rot)
        pr = path.project(rx, ry)
        path_rot = path.rotation_at(pr.s) if 0 <= pr.s <= path.length else pr.rotation
        psi = float(_wrap(rot - path_rot))
        lat = pr.lateral
        kappa_ff = path.mean_curvature(pr.s, pr.s + max(v * p.ff_preview_s, 4.0))
        omega = min(p.omega_max, v / p.d_min)
        if v > 4.0 and abs(lat) < 1.0:
            self.y_int = _clamp(self.y_int + p.ki * lat * dt, -p.i_limit_m, p.i_limit_m)
        v_eff = max(v, 3.0)
        corr = -(2 * p.zeta * omega * psi + omega * omega * (lat + self.y_int) / v_eff) / v_eff
        lim = min(p.max_corr_kappa, p.max_corr_lat_acc / (v * v))
        corr = _clamp(corr, -lim, lim)
        kappa = kappa_ff + corr
        steer = curvature_to_left(kappa / self.ratio, g.wheelbase)
        limited = False
        if steer_rate:
            # the wheel must be within rate x tau of the feedforward angle needed tau seconds from now
            r = steer_rate * p.rate_margin
            taus = np.arange(0.2, p.preview_horizon_s + 1e-9, 0.2)
            k_f = np.asarray(path.curvature(pr.s + v * taus))
            d_f = -np.arctan(k_f / self.ratio * g.wheelbase)
            lo, hi = float(np.max(d_f - r * taus)), float(np.min(d_f + r * taus))
            if lo <= hi:
                steer = _clamp(steer, lo, hi)
            else:
                steer, limited = 0.5 * (lo + hi), True
        return LateralOut(steer, kappa, kappa_ff, lat, psi, pr.s, self.ratio, limited)
