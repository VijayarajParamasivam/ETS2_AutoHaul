"""Game camera (from the ETS2LA plugin's camera block): world point -> pixel on the game's picture.

Game world axes: x east, y up, z south; the map plane is (x, z) = the lane map's (x, y). The camera's quaternion
turns camera space into world space and the camera looks down its -Z axis (like every game object).
`fov` is the vertical field of view in degrees (checked against lane markings on screen).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def quat_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


@dataclass
class Camera:
    pos: np.ndarray            # world (x, y, z)
    rot: np.ndarray            # 3x3, camera -> world
    fov_deg: float
    width: int
    height: int
    vertical_fov: bool = True

    @classmethod
    def from_block(cls, cam: dict, width: int, height: int, vertical_fov: bool = True) -> "Camera":
        return cls(np.array([cam["wx"], cam["y"], cam["wz"]]),
                   quat_matrix(cam["qw"], cam["qx"], cam["qy"], cam["qz"]), float(cam["fov"]), width, height,
                   vertical_fov)

    @property
    def focal(self) -> float:
        half = math.radians(self.fov_deg) / 2
        return (self.height if self.vertical_fov else self.width) / 2 / math.tan(half)

    def project(self, world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(N, 3) world points -> ((N, 2) pixels (u right, v down), (N,) depth in m; depth <= 0: behind)."""
        p = (np.asarray(world, float).reshape(-1, 3) - self.pos) @ self.rot     # world -> camera (rot is orthonormal)
        depth = -p[:, 2]
        f = self.focal
        d = np.where(depth > 1e-3, depth, np.nan)
        u = self.width / 2 + f * p[:, 0] / d
        v = self.height / 2 - f * p[:, 1] / d
        return np.stack([u, v], axis=1), depth

    def metres_to_pixels(self, metres: float, depth: float) -> float:
        return self.focal * metres / max(depth, 1e-3)


def map_to_world(x: float, y: float, height: float) -> np.ndarray:
    """Lane-map point (x, y) at `height` -> game world (x, y up, z)."""
    return np.array([x, height, y])
