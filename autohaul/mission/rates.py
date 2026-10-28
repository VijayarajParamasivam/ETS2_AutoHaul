"""How fuel and in-game time relate to distance on the map, learned while driving.

The lane map measures world metres. The game scales distance and time to make the map feel bigger (telemetry
`local_scale`: 19 on open roads, 3 in towns): a world metre costs `scale` metres' worth of fuel and passes
`scale` times as much game time as real time. So AutoHaul learns, per world metre driven:
  fuel   litres (refuels and standstill don't count)
  time   in-game minutes (stops don't count: they are planned separately)
starting from the open-road scale and the truck's average consumption. It also notices whether the game's
fatigue is on: with it off, the rest timer doesn't run down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

OPEN_ROAD_SCALE = 19.0


@dataclass
class RateParams:
    window_m: float = 400.0          # learn from every this many world metres driven
    alpha: float = 0.2               # weight of each new window
    cruise_ms: float = 20.0          # assumed world speed before anything is learned (72 km/h)
    fatigue_window_min: float = 30.0 # game minutes the rest timer must run (or not) to decide fatigue on/off


class Rates:
    def __init__(self, params: RateParams | None = None):
        self.p = params or RateParams()
        self.fuel_l_per_m: float | None = None
        self.min_per_m = OPEN_ROAD_SCALE / (self.p.cruise_ms * 60.0)
        self.fatigue_on: bool | None = None           # None: not known yet
        self.metres = 0.0                             # total world metres seen moving
        self._measured = {"fuel": False, "time": False}   # the first measurement replaces the guess
        self._anchor = None                           # (metres, fuel, game_time)
        self._prev_xy = None
        self._rest_anchor = None                      # (game_time, next_rest_stop)

    def seed(self, avg_consumption_l_per_km: float) -> None:
        """Before anything is learned: the dashboard's average consumption (litres per game km)."""
        if self.fuel_l_per_m is None and avg_consumption_l_per_km > 0.01:
            self.fuel_l_per_m = avg_consumption_l_per_km * OPEN_ROAD_SCALE / 1000.0

    def fuel_per_m(self) -> float:
        return self.fuel_l_per_m if self.fuel_l_per_m is not None else 0.35 * OPEN_ROAD_SCALE / 1000.0

    def update(self, x: float, y: float, speed: float, fuel: float, game_time: float, next_rest: float | None,
               avg_consumption: float = 0.0) -> None:
        self.seed(avg_consumption)
        if self._prev_xy is not None and speed > 0.5:
            d = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            if d < 50.0:                              # a teleport or a ferry isn't driving
                self.metres += d
            else:
                self._anchor = None
        self._prev_xy = (x, y)
        if self._anchor is None or fuel > self._anchor[1] + 0.5 or game_time < self._anchor[2]:
            self._anchor = (self.metres, fuel, game_time)       # start over after a refuel or a time jump
        else:
            m0, f0, t0 = self._anchor
            dm = self.metres - m0
            if dm >= self.p.window_m:
                a = self.p.alpha
                used = (f0 - fuel) / dm
                if used > 0:
                    self.fuel_l_per_m = self._learn("fuel", self.fuel_l_per_m, used)
                minutes = (game_time - t0) / dm
                if 0 < minutes < 1.0:                 # a stop or a sleep inside the window: skip it
                    self.min_per_m = self._learn("time", self.min_per_m, minutes)
                self._anchor = (self.metres, fuel, game_time)
        self._fatigue(game_time, next_rest)

    def _learn(self, what: str, old: float | None, new: float) -> float:
        if old is None or not self._measured[what]:
            self._measured[what] = True
            return new
        return (1 - self.p.alpha) * old + self.p.alpha * new

    def _fatigue(self, game_time: float, next_rest: float | None) -> None:
        if next_rest is None:
            return
        if self._rest_anchor is None or game_time < self._rest_anchor[0] or next_rest > self._rest_anchor[1] + 1:
            self._rest_anchor = (game_time, next_rest)          # start, a time jump, or a sleep reset it
            return
        t0, r0 = self._rest_anchor
        if game_time - t0 >= self.p.fatigue_window_min:
            self.fatigue_on = (r0 - next_rest) > 0.5 * (game_time - t0)
            self._rest_anchor = (game_time, next_rest)
