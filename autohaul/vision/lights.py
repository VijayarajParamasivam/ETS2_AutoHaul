"""Traffic-light colours from the screen.

The ETS2LA plugin's traffic-light reader crashes 1.53, so AutoHaul looks at the lights itself:
  * which lights: the map says which semaphore group controls each junction path, and where its heads stand
    (autohaul.world.rules). A head's map position is its pole base; the lamps hang somewhere above it or on an
    arm over the road, so a box around it is searched: `across_m` either side across the lane, `low_m`..`high_m`
    up, `along_m` along it.
  * where on the screen: the plugin's camera (position, rotation, field of view) projects that box
    (autohaul.vision.camera).
  * which colour: lit lamps are small, bright and fully saturated (measured on 1.53: green ~(40, 248, 35),
    red ~(250, 30, 27)). Pixels are counted per colour; a colour needs about a quarter of a lamp's area at that
    distance. Red wins over amber over green (red + amber is still a stop).
  * steady: a colour counts after `stable_reads` reads in a row agree (`read_every_s` apart per light).
A light it can't see (off screen, too far, too close, hidden) reads None: the junction logic then stops at the
line and hands back if it stays unseen.

`stop_back_m`: from the cab, lamps above the windscreen's top edge can't be seen. The truck stops far enough
before the line that at least one head of the light stays below `max_elev_deg` (map geometry only).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from autohaul.vision.camera import Camera
from autohaul.world.lanemap import LaneMap
from autohaul.world.rules import JunctionRules


@dataclass
class LightVisionParams:
    across_m: float = 9.0
    low_m: float = 2.0
    high_m: float = 8.0
    along_m: float = 1.5
    min_dist_m: float = 4.0
    max_dist_m: float = 160.0
    max_box_px: int = 6000              # (crops are clipped to the screen; 1600 skipped a light 15 m away)
    lamp_radius_m: float = 0.12         # lit part of a lamp
    min_fraction: float = 0.25          # of the lamp's expected pixel area
    min_pixels: int = 3
    max_lamp_ratio: float = 25.0        # a colour patch over this many lamps' areas is a sign or a roof, not lamps
    read_every_s: float = 0.1
    stable_reads: int = 2
    lamp_height_m: float = 5.0          # for the viewing distance: typical lamp height above the road
    eye_height_m: float = 2.6           # driver's eye above the road
    eye_behind_bumper_m: float = 1.5
    max_elev_deg: float = 12.0          # lamps higher than this above the eye are hidden by the cab roof
    max_stop_back_m: float = 20.0
    beside_m: float = 10.0              # a head this close and out of sight is beside the cab: can't be checked


def classify(rgb: np.ndarray) -> dict[str, int]:
    """Pixel counts of lit red, amber and green lamp colours in an (h, w, 3) uint8 picture. Lit lamps are near
    full brightness and saturation (1.53: green ~(40, 248, 35), red ~(250, 30, 27)); sunlit leaves, red roofs
    and orange signs are dimmer or less pure, and were read as lamps with looser limits."""
    a = rgb.reshape(-1, 3).astype(np.int16)
    r, g, b = a[:, 0], a[:, 1], a[:, 2]
    red = (r >= 230) & (g <= 90) & (b <= 90)
    amber = (r >= 235) & (g >= 120) & (g <= 210) & (b <= 80)
    green = (g >= 225) & (r <= 110) & (g - b >= 40)
    return {"red": int(red.sum()), "amber": int(amber.sum()), "green": int(green.sum())}


class LightVision:
    def __init__(self, rules: JunctionRules, lm: LaneMap, grab: Callable[[int, int, int, int], np.ndarray | None],
                 camera: Callable[[], Camera | None], clock: Callable[[], float],
                 params: LightVisionParams | None = None):
        self.rules, self.lm = rules, lm
        self.grab, self.camera, self.clock = grab, camera, clock
        self.p = params or LightVisionParams()
        self._cache: dict[tuple[int, int], tuple[float, list[str | None], str | None]] = {}
        self.last: dict[tuple[int, int], dict] = {}            # per light: the last read, for notes and debugging
        self.changes: list[dict] = []                           # reads where a light's steady colour changed

    def _lane_frame(self, lane: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Start point of the lane (the line), unit vector along it, unit vector to its right (map plane)."""
        x0, y0, x1, y1 = self.lm.seg[self.lm.lane_first[lane], 0:4]
        rot = math.atan2(y1 - y0, x1 - x0)
        along = np.array([math.cos(rot), math.sin(rot)])
        return np.array([x0, y0]), along, np.array([-along[1], along[0]])

    def stop_back_m(self, lane: int, item: int) -> float:
        p = self.p
        heads = self.rules.semaphores(lane, item)
        if not len(heads):
            return 0.0
        line, along, _ = self._lane_frame(lane)
        need = (p.lamp_height_m - p.eye_height_m) / math.tan(math.radians(p.max_elev_deg))
        backs = [need - float(np.dot(self.rules.sem_xyz[h, 0:2] - line, along)) - p.eye_behind_bumper_m
                 for h in heads]
        return float(min(max(min(backs), 0.0), p.max_stop_back_m))

    def read(self, lane: int, item: int) -> tuple[str | None, dict]:
        """One look at the light controlling `lane` of junction `item`: (colour or None, details)."""
        p = self.p
        cam = self.camera()
        heads = self.rules.semaphores(lane, item)
        info: dict = {"heads": len(heads), "seen": 0, "counts": {"red": 0, "amber": 0, "green": 0}}
        if cam is None or not len(heads):
            return None, info
        info["nearest_m"] = float(min(np.hypot(self.rules.sem_xyz[h, 0] - cam.pos[0], self.rules.sem_xyz[h, 1] - cam.pos[2])
                                      for h in heads))
        _, along, right = self._lane_frame(lane)
        up = np.array([0.0, 1.0, 0.0])
        a3 = np.array([along[0], 0.0, along[1]])
        r3 = np.array([right[0], 0.0, right[1]])
        need = 0.0
        for h in heads:
            x, y, z = self.rules.sem_xyz[h]
            base = np.array([x, z, y])
            corners = np.array([base + sa * p.across_m * r3 + sb * p.along_m * a3 + hh * up
                                for sa in (-1, 1) for sb in (-1, 1) for hh in (p.low_m, p.high_m)])
            px, depth = cam.project(corners)
            d = float(np.hypot(x - cam.pos[0], y - cam.pos[2]))
            ok = depth >= p.min_dist_m                 # close up, part of the box is beside or behind the camera:
            if ok.sum() < 2 or d > p.max_dist_m:      # search the part in front (2026-10-29: a light 15 m away
                continue                              # counted as unseen and the truck handed back)
            px, depth = px[ok], depth[ok]
            u0, v0 = np.floor(px.min(axis=0)).astype(int)
            u1, v1 = np.ceil(px.max(axis=0)).astype(int)
            u0, v0 = max(u0, 0), max(v0, 0)
            u1, v1 = min(u1, cam.width), min(v1, cam.height)
            if u1 - u0 < 2 or v1 - v0 < 2 or max(u1 - u0, v1 - v0) > p.max_box_px:
                continue
            img = self.grab(int(u0), int(v0), int(u1 - u0), int(v1 - v0))
            if img is None:
                continue
            info["seen"] += 1
            for k, n in classify(img).items():
                info["counts"][k] += n
            lamp_px = math.pi * cam.metres_to_pixels(p.lamp_radius_m, float(depth.mean())) ** 2
            need = max(need, max(p.min_pixels, p.min_fraction * lamp_px))
        if not info["seen"]:
            return None, info
        info["need"] = round(need, 1)
        too_big = p.max_lamp_ratio * max(need / p.min_fraction, p.min_pixels) * info["seen"] + 50
        for colour in ("red", "amber", "green"):
            if need <= info["counts"][colour] <= too_big:
                return colour, info
        return None, info

    def dark(self, lane: int, item: int) -> bool:
        """No light to go by: the last read saw where the lamps would be and none was lit (a signal only for the
        AI, or lights off), or the only heads stand beside the cab, out of sight. The junction logic then gives
        way to everyone, which also waits out a red light while cross traffic flows."""
        info = self.last.get((item, int(self.rules.sem_id[lane])))
        if not info:
            return False
        if info.get("seen"):
            return info.get("colour") is None
        return info.get("nearest_m", 1e9) < self.p.beside_m

    def state(self, lane: int, item: int) -> str | None:
        key = (item, int(self.rules.sem_id[lane]))
        now = self.clock()
        last_t, history, stable = self._cache.get(key, (-1e9, [], None))
        if now - last_t < self.p.read_every_s:
            return stable
        colour, info = self.read(lane, item)
        history = (history + [colour])[-self.p.stable_reads:]
        before = stable
        if len(history) == self.p.stable_reads and len(set(history)) == 1:
            stable = colour
        self._cache[key] = (now, history, stable)
        self.last[key] = dict(info, colour=colour, stable=stable)
        if stable != before:
            self.changes.append(dict(self.last[key], junction=f"{item:x}", group=key[1], was=before))
            del self.changes[:-50]
        return stable
