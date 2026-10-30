"""Automatic wipers, learned from you.

Telemetry says whether the wipers are on, but nothing says it's raining, so rain has to be seen on the screen. What
rain looks like differs by truck, camera and time of day, so AutoHaul learns it: it keeps a small logistic model over
a few picture features of the upper part of the view, labelled by you. Wipers you switched on: this is rain. Wipers
off (and not because AutoHaul switched them): this isn't. It doesn't touch the wipers until it has seen enough of both
(`min_rain_s`, `min_clear_s`). After you switch them yourself it leaves them alone for `driver_hold_s`.

On 1.53 the stalk works as `wipers` (one stage up) and `wipersback` (one stage down); the direct stages do nothing.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

N_FEATURES = 5


def picture_features(img: np.ndarray) -> np.ndarray:
    """Features of the upper-middle part of the view (BGR(A) image): brightness, colour, fine detail, contrast and
    vertical-streak detail. Rain greys the picture, softens the distance and adds fine detail on the glass."""
    h, w = img.shape[:2]
    crop = img[h // 12:h // 3, w // 5:4 * w // 5, :3][::3, ::3].astype(np.float32)
    lum = crop.mean(axis=2)
    sat = (crop.max(axis=2) - crop.min(axis=2)).mean() / 255.0
    mean = float(lum.mean())
    dx = np.abs(np.diff(lum, axis=1)).mean()
    dy = np.abs(np.diff(lum, axis=0)).mean()
    norm = mean + 10.0
    return np.array([mean / 255.0, sat, 4.0 * (dx + dy) / norm, float(lum.std()) / 128.0, 4.0 * dx / norm], dtype=np.float64)


class RainModel:
    """Online logistic regression with running feature scaling."""

    def __init__(self, lr: float = 0.02):
        self.lr = lr
        self.w = np.zeros(N_FEATURES + 1)
        self.mean = np.zeros(N_FEATURES)
        self.var = np.ones(N_FEATURES)
        self.n = 0
        self.seen = {"rain": 0.0, "clear": 0.0}

    def _x(self, f: np.ndarray) -> np.ndarray:
        return np.append((f - self.mean) / np.sqrt(self.var + 1e-6), 1.0)

    def learn(self, f: np.ndarray, rain: bool, weight_s: float = 1.0) -> None:
        self.n += 1
        a = max(1.0 / self.n, 0.002)
        d = f - self.mean
        self.mean = self.mean + a * d
        self.var = (1 - a) * self.var + a * d * d
        self.seen["rain" if rain else "clear"] += weight_s
        # the two classes are far from equally common: each counts by how rare it is
        total = self.seen["rain"] + self.seen["clear"]
        balance = 0.5 * total / max(self.seen["rain" if rain else "clear"], 1.0)
        x = self._x(f)
        err = float(rain) - self._p(x)
        self.w = self.w + self.lr * min(balance, 20.0) * err * x

    def _p(self, x: np.ndarray) -> float:
        return float(1.0 / (1.0 + np.exp(-np.clip(self.w @ x, -30, 30))))

    def p(self, f: np.ndarray) -> float:
        return self._p(self._x(f))

    def to_dict(self) -> dict:
        return {"w": self.w.tolist(), "mean": self.mean.tolist(), "var": self.var.tolist(), "n": self.n, "seen": self.seen}

    @classmethod
    def from_dict(cls, d: dict) -> "RainModel":
        m = cls()
        m.w, m.mean, m.var = np.array(d["w"]), np.array(d["mean"]), np.array(d["var"])
        m.n, m.seen = int(d["n"]), dict(d["seen"])
        return m


@dataclass
class WiperParams:
    min_rain_s: float = 120.0
    min_clear_s: float = 600.0
    on_p: float = 0.85
    off_p: float = 0.2
    on_after_s: float = 20.0           # raining this long: on
    off_after_s: float = 60.0          # clear this long: off
    driver_hold_s: float = 300.0       # after you switch them, AutoHaul leaves them alone this long
    press_every_s: float = 1.2
    max_presses: int = 5


class WiperControl:
    def __init__(self, press, path: Path | None = None, params: WiperParams | None = None):
        self.press = press
        self.path = Path(path) if path else None
        self.p = params or WiperParams()
        self.model = RainModel()
        if self.path is not None and self.path.exists():
            try:
                self.model = RainModel.from_dict(json.loads(self.path.read_text(encoding="utf-8")))
            except (OSError, ValueError, KeyError):
                pass
        self.prob = 0.0
        self._last_state: bool | None = None
        self._mine_until = -1e9        # a change of state until then is AutoHaul's own press
        self._by_autohaul = False      # they're on because AutoHaul switched them on
        self._driver_at = -1e9
        self._since = None             # (state wanted, since when)
        self._pressed = -1e9
        self._presses = 0

    @property
    def ready(self) -> bool:
        return self.model.seen["rain"] >= self.p.min_rain_s and self.model.seen["clear"] >= self.p.min_clear_s

    def describe(self) -> str:
        if not self.ready:
            s = self.model.seen
            return (f"learning ({s['rain'] / 60:.0f}/{self.p.min_rain_s / 60:.0f} min rain, {s['clear'] / 60:.0f}/"
                    f"{self.p.min_clear_s / 60:.0f} min clear): put the wipers on yourself in rain")
        return f"{self.prob * 100:.0f} % sure it's raining"

    def step(self, wipers_on: bool, features: np.ndarray | None, now: float, dt: float, engaged: bool) -> None:
        """Once a second or so. `features`: picture_features of the view, or None if there's no picture."""
        on = bool(wipers_on)
        if self._last_state is not None and on != self._last_state:
            if now > self._mine_until:
                self._driver_at, self._by_autohaul = now, False        # you switched them
            elif on:
                self._by_autohaul = True
            else:
                self._by_autohaul = False
        self._last_state = on
        if features is None:
            return
        if not self._by_autohaul and now > self._mine_until:
            self.model.learn(features, on, dt)
        self.prob = self.model.p(features)
        if not engaged or not self.ready or now - self._driver_at < self.p.driver_hold_s:
            self._since = None
            return
        want = True if self.prob >= self.p.on_p else False if self.prob <= self.p.off_p else None
        if want is None or want == on:
            self._since = None
            self._presses = 0
            return
        if self._since is None or self._since[0] != want:
            self._since = (want, now)
        wait = self.p.on_after_s if want else self.p.off_after_s
        if now - self._since[1] < wait or now - self._pressed < self.p.press_every_s or self._presses >= self.p.max_presses:
            return
        self.press("wipers" if want else "wipersback")
        self._pressed, self._mine_until = now, now + self.p.press_every_s + 1.0
        self._presses += 1

    def save(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.model.to_dict()), encoding="utf-8")
        os.replace(tmp, self.path)
