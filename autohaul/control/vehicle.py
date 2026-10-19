"""Truck geometry and the conversions between telemetry and the planar (map) frame.

Conventions (see autohaul/world/lanemap.py): the map plane is (x, y) = game (X, Z), rotation r points along
(cos r, sin r). Because game Z points south, r *increases* when the truck turns right, so map curvature is
positive for right turns. The game's own steering values (effective steering, wheel angles) are
counterclockwise-positive, i.e. positive = left; `left_to_curvature` and friends handle the sign flip.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from autohaul.world.lanemap import heading_to_rotation

TWO_PI = 2 * math.pi


@dataclass
class TruckGeometry:
    wheelbase: float = 3.82          # m, steered axle to the (mean) driven axle
    rear_axle: float = 2.03          # m behind the telemetry placement origin
    front_axle: float = 1.79         # m ahead of the origin
    front_overhang: float = 1.45     # m from the front axle to the front bumper
    track: float = 2.08              # m between the steered wheels
    rad_per_unit: float = math.radians(38.0)   # bicycle steer angle per unit of effective steering

    @property
    def front_bumper(self) -> float:
        """m ahead of the origin."""
        return self.front_axle + self.front_overhang

    @classmethod
    def from_config(cls, truck_cfg, rad_per_unit: float, front_overhang: float) -> "TruckGeometry":
        """From the telemetry truck config block (wheel positions are vehicle space, -Z = forward)."""
        n = int(truck_cfg.wheel_count)
        steer = [truck_cfg.wheel_position[i] for i in range(n) if truck_cfg.wheel_steerable[i]]
        rear = [truck_cfg.wheel_position[i] for i in range(n) if not truck_cfg.wheel_steerable[i]]
        if not steer or not rear:
            return cls(rad_per_unit=rad_per_unit, front_overhang=front_overhang)
        fz = sum(p.z for p in steer) / len(steer)
        rz = sum(p.z for p in rear) / len(rear)
        track = max(p.x for p in steer) - min(p.x for p in steer)
        return cls(wheelbase=rz - fz, rear_axle=rz, front_axle=-fz, front_overhang=front_overhang,
                   track=track or 2.0, rad_per_unit=rad_per_unit)


def bicycle_angle(wheel_left: float, wheel_right: float) -> float:
    """Equivalent single-track steer angle (rad, + = left) from the two steered wheels' angles (rad).
    Ackermann: cot(bicycle) = mean of the wheels' cotangents."""
    if abs(wheel_left) < 1e-4 or abs(wheel_right) < 1e-4 or (wheel_left > 0) != (wheel_right > 0):
        return 0.5 * (wheel_left + wheel_right)
    cot = 0.5 * (1 / math.tan(wheel_left) + 1 / math.tan(wheel_right))
    return math.atan(1 / cot)


def wheel_angles(bicycle: float, geom: TruckGeometry) -> tuple[float, float]:
    """Inverse of bicycle_angle: (left wheel, right wheel) angles, rad, + = left."""
    if abs(bicycle) < 1e-6:
        return bicycle, bicycle
    r = geom.wheelbase / math.tan(abs(bicycle))       # turn radius at the rear axle centre
    inner = math.atan(geom.wheelbase / (r - geom.track / 2))
    outer = math.atan(geom.wheelbase / (r + geom.track / 2))
    s = 1.0 if bicycle > 0 else -1.0
    # turning left: the left wheel is the inner one
    return (s * inner, s * outer) if bicycle > 0 else (s * outer, s * inner)


def left_to_curvature(steer_left: float, wheelbase: float) -> float:
    """Kinematic path curvature (1/m, map convention: + = right) for a steer angle (rad, + = left)."""
    return -math.tan(steer_left) / wheelbase


def curvature_to_left(kappa: float, wheelbase: float) -> float:
    return -math.atan(kappa * wheelbase)


@dataclass
class Pose:
    x: float          # map plane, origin of the truck (telemetry placement)
    y: float
    rotation: float   # map rotation
    speed: float      # m/s along the truck's forward axis (negative = reversing)
    yaw_rate: float   # map rotation rate, rad/s (+ = turning right)

    def point(self, ahead: float) -> tuple[float, float]:
        """A point `ahead` metres along the truck's axis (negative = behind the origin)."""
        return self.x + ahead * math.cos(self.rotation), self.y + ahead * math.sin(self.rotation)


def pose_from_frame(frame) -> Pose:
    p = frame.truck_placement
    # local angular velocity is vehicle space, rotations/s, about +Y (up): + = counterclockwise = left
    yaw_rate = -TWO_PI * frame.local_angular_velocity.y
    return Pose(p.x, p.z, heading_to_rotation(p.heading), frame.speed, yaw_rate)


def measured_steer_left(frame, geom: TruckGeometry) -> float:
    """Current single-track steer angle (rad, + = left) from the telemetry wheel angles (SDK: rotations,
    counterclockwise); falls back to effective steering x calibration."""
    w0, w1 = frame.wheel_steering[0] * TWO_PI, frame.wheel_steering[1] * TWO_PI
    if w0 == 0.0 and w1 == 0.0:
        return frame.effective_steering * geom.rad_per_unit
    return bicycle_angle(w0, w1)
