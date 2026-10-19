"""A small truck simulator for testing the controllers offline.

It produces frames with the same attribute names as the telemetry frame (only the fields the controllers
read), driven by the same commands AutoHaul sends to the game:
  steering   relative mode: wheel turn rate = -rate_gain * axis (c_rsteersens), with an input delay,
             optional game-side deadzone; absolute mode: wheel = -axis
  lateral    kinematic single-track model at the rear axle, with an understeer factor and a yaw lag
  pedals     engine force limited by power, air brakes with a lag, rolling + air resistance, slope,
             engine braking when coasting, and gear-shift torque interruptions
Defaults are a Volvo FH tractor (wheels from telemetry on 1.53.2.1) without a trailer.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from types import SimpleNamespace

from autohaul.control.vehicle import TWO_PI, TruckGeometry, wheel_angles

G = 9.81


@dataclass
class SimParams:
    dt: float = 1 / 60
    steering_mode: str = "relative"
    rate_gain: float = 0.42           # effective steering units/s per unit axis (relative mode)
    deadzone: float = 0.0             # game-side axis deadzone
    input_delay_frames: int = 1
    understeer_v: float = 38.0        # m/s: curvature factor 1 / (1 + (v / understeer_v)^2)
    yaw_lag_s: float = 0.12
    mass: float = 9000.0              # kg (tractor only; ~40000 with a loaded trailer)
    power: float = 330e3              # W
    max_drive_force: float = 60e3     # N
    brake_force: float = 0.6 * G      # N per kg at full brake
    brake_lag_s: float = 0.3
    throttle_lag_s: float = 0.25
    engine_brake: float = 0.012 * G   # N per kg while coasting in gear
    c_rr: float = 0.006
    cda: float = 6.5                  # m^2
    shift_every_kmh: float = 9.0      # automatic: a shift (0.6 s of no drive) every this many km/h
    shift_time_s: float = 0.6
    grade: float = 0.0                # rise / run, + = uphill (constant)


@dataclass
class SimTruck:
    x: float = 0.0
    y: float = 0.0
    rotation: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    eff: float = 0.0                  # effective steering (+ = left)
    thr: float = 0.0                  # effective throttle (after lag)
    brk: float = 0.0
    shift_left: float = 0.0
    last_shift_speed: float = 0.0


class TruckSim:
    def __init__(self, geom: TruckGeometry | None = None, params: SimParams | None = None, **start):
        self.geom = geom or TruckGeometry()
        self.p = params or SimParams()
        self.s = SimTruck(**start)
        self.t = 0.0
        self.frame_index = 0
        self._pending: deque[tuple[float, float, float]] = deque([(0.0, 0.0, 0.0)] * (self.p.input_delay_frames + 1))
        self.input_steering = 0.0
        self.extra_steer_rate = 0.0       # tests: someone else turning the wheel (units/s)
        self.accel = 0.0

    def step(self, steer: float, throttle: float, brake: float) -> None:
        p, s, g, dt = self.p, self.s, self.geom, self.p.dt
        self._pending.append((steer, throttle, brake))
        steer, throttle, brake = self._pending.popleft()
        # --- steering
        if p.steering_mode == "relative":
            a = steer
            if abs(a) <= p.deadzone:
                a = 0.0
            elif p.deadzone > 0:
                a = math.copysign((abs(a) - p.deadzone) / (1 - p.deadzone), a)
            s.eff = max(-1.0, min(1.0, s.eff - p.rate_gain * a * dt + self.extra_steer_rate * dt))
            self.input_steering = -math.copysign(1.0, steer) if abs(steer) > 1e-6 else 0.0
        else:
            s.eff = max(-1.0, min(1.0, -steer))
            self.input_steering = s.eff
        delta = s.eff * g.rad_per_unit
        # --- lateral: kinematic yaw rate with understeer and a first-order lag
        under = 1.0 / (1.0 + (s.speed / p.understeer_v) ** 2)
        yaw_target = -s.speed * math.tan(delta) / g.wheelbase * under   # map convention: + = right
        s.yaw_rate += (yaw_target - s.yaw_rate) * min(1.0, dt / p.yaw_lag_s)
        # move the rear axle along its heading, then rotate
        rx = s.x - g.rear_axle * math.cos(s.rotation)
        ry = s.y - g.rear_axle * math.sin(s.rotation)
        r_mid = s.rotation + s.yaw_rate * dt / 2
        rx += s.speed * dt * math.cos(r_mid)
        ry += s.speed * dt * math.sin(r_mid)
        s.rotation = math.atan2(math.sin(s.rotation + s.yaw_rate * dt), math.cos(s.rotation + s.yaw_rate * dt))
        s.x = rx + g.rear_axle * math.cos(s.rotation)
        s.y = ry + g.rear_axle * math.sin(s.rotation)
        # --- longitudinal
        s.thr += (max(0.0, min(1.0, throttle)) - s.thr) * min(1.0, dt / p.throttle_lag_s)
        s.brk += (max(0.0, min(1.0, brake)) - s.brk) * min(1.0, dt / p.brake_lag_s)
        if s.shift_left > 0:
            s.shift_left -= dt
        elif abs(s.speed - s.last_shift_speed) * 3.6 > p.shift_every_kmh and s.thr > 0.1:
            s.shift_left = p.shift_time_s
            s.last_shift_speed = s.speed
        v = s.speed
        drive = 0.0 if s.shift_left > 0 else s.thr * min(p.max_drive_force, p.power / max(abs(v), 1.0))
        resist = p.mass * G * (p.c_rr + p.grade) + 0.5 * 1.2 * p.cda * v * abs(v)
        if s.thr < 0.05 and s.shift_left <= 0:
            resist += p.mass * p.engine_brake * min(1.0, abs(v) / 5.0)
        braking = s.brk * p.brake_force * p.mass
        f = drive - resist
        if abs(v) < 0.05 and abs(f) <= braking + p.mass * G * p.c_rr:
            self.accel = 0.0
            s.speed = 0.0
        else:
            f -= math.copysign(braking, v if abs(v) > 0.05 else f)
            self.accel = f / p.mass
            nv = v + self.accel * dt
            s.speed = 0.0 if (v > 0 > nv or v < 0 < nv) and s.brk > 0.05 else nv
        self.t += dt
        self.frame_index += 1

    # ------------------------------------------------------------- telemetry-shaped output
    def frame(self, speed_limit: float = 0.0) -> SimpleNamespace:
        s, g = self.s, self.geom
        heading = (-(s.rotation + math.pi / 2) / TWO_PI) % 1.0
        wl, wr = wheel_angles(s.eff * g.rad_per_unit, g)
        return SimpleNamespace(
            frame_index=self.frame_index, paused=0, simulation_time_us=int(self.t * 1e6),
            truck_placement=SimpleNamespace(x=s.x, y=0.0, z=s.y, heading=heading, pitch=math.atan(self.p.grade) / TWO_PI, roll=0.0),
            local_angular_velocity=SimpleNamespace(x=0.0, y=-s.yaw_rate / TWO_PI, z=0.0),
            local_linear_velocity=SimpleNamespace(x=0.0, y=0.0, z=-s.speed),
            local_linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=-self.accel),
            speed=s.speed, effective_steering=s.eff, input_steering=self.input_steering,
            effective_throttle=s.thr, effective_brake=s.brk, input_throttle=s.thr, input_brake=s.brk,
            wheel_steering=[wl / TWO_PI, wr / TWO_PI, 0.0, 0.0],
            navigation_speed_limit=speed_limit, parking_brake=0, engine_enabled=1, electric_enabled=1,
            displayed_gear=1 if s.speed >= 0 else -1, cruise_control=0.0, multiplayer_time_offset=0,
        )


@dataclass
class SimVehicle:
    """An AI vehicle driving along a polyline at a speed profile; looks like a gamedata.traffic() record."""
    vid: int
    path: object                  # PathRef
    s: float
    speed: float
    length: float = 4.5
    width: float = 1.9
    accel: float = 0.0
    profile: list[tuple[float, float]] = field(default_factory=list)   # (time, target speed)

    def step(self, t: float, dt: float) -> None:
        target = self.speed
        for when, sp in self.profile:
            if t >= when:
                target = sp
        a = max(-4.0, min(1.5, (target - self.speed) * 1.5))
        self.accel = a
        self.speed = max(0.0, self.speed + a * dt)
        self.s += self.speed * dt

    def record(self) -> dict:
        x, y = self.path.point(self.s)
        r = self.path.rotation_at(self.s)
        th = -(r + math.pi / 2)                      # rotation about +Y (see quat_forward_rotation)
        return {"wx": x, "wz": y, "x": x, "z": y, "y": 0.0, "qw": math.cos(th / 2), "qx": 0.0, "qy": math.sin(th / 2),
                "qz": 0.0, "length": self.length, "width": self.width, "height": 2.0, "speed": self.speed,
                "acceleration": self.accel, "trailer_count": 0, "id": self.vid, "trailers": []}
