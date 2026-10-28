"""Headlights while AutoHaul drives: low beam on at night (game clock), off again in the morning.

The game's `light` button cycles off -> parking lights -> low beam -> off, so AutoHaul presses it and checks
telemetry (light_parking / light_low_beam) until the lights are where it wants them, one press at a time.
It switches off only lights it switched on itself; lights you switched on stay on. Wipers stay yours: the
game's telemetry has no rain signal.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LightParams:
    night_from_h: float = 20.0          # game clock: low beam from this hour ...
    night_until_h: float = 7.0          # ... until this hour
    press_every_s: float = 0.8          # one press, then wait for telemetry to show it
    button: str = "light"


class Headlights:
    def __init__(self, params: LightParams | None = None):
        self.p = params or LightParams()
        self.ours = False                # AutoHaul switched the low beam on
        self._last = -1e9

    def night(self, game_time_min: float) -> bool:
        h = (game_time_min % 1440) / 60.0
        return h >= self.p.night_from_h or h < self.p.night_until_h

    def step(self, game_time_min: float, low_beam: bool, parking: bool, now: float) -> str | None:
        """The button to press now, or None."""
        if now - self._last < self.p.press_every_s:
            return None
        if self.night(game_time_min):
            if not low_beam:
                self._last, self.ours = now, True
                return self.p.button
        elif self.ours and (low_beam or parking):
            self._last = now
            return self.p.button
        elif self.ours:
            self.ours = False
        return None
