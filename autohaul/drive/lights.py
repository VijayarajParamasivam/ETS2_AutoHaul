"""Headlights while AutoHaul drives: low beam on at night (game clock), off again in the morning.

The game's `light` button cycles off -> parking lights -> low beam -> off, so AutoHaul presses it and checks
telemetry (light_parking / light_low_beam) until the lights are where it wants them, one press at a time. By day it
switches them off, and the high beam at any hour, whoever switched them on (2026-10-30: it only switched off lights it
had switched on itself, and they'd been on all along, so it never did anything). After you change them yourself it
leaves them alone for `driver_hold_s`.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LightParams:
    night_from_h: float = 20.0          # game clock: low beam from this hour ...
    night_until_h: float = 7.0          # ... until this hour
    press_every_s: float = 0.8          # one press, then wait for telemetry to show it
    driver_hold_s: float = 600.0        # after you change them, AutoHaul leaves them alone this long
    high_beam_button: str = "hblight"
    button: str = "light"


class Headlights:
    def __init__(self, params: LightParams | None = None):
        self.p = params or LightParams()
        self._last = -1e9
        self._state: tuple | None = None
        self._driver_at = -1e9

    def night(self, game_time_min: float) -> bool:
        h = (game_time_min % 1440) / 60.0
        return h >= self.p.night_from_h or h < self.p.night_until_h

    def step(self, game_time_min: float, low_beam: bool, parking: bool, now: float, high_beam: bool = False) -> str | None:
        """The button to press now, or None."""
        state = (bool(low_beam), bool(parking), bool(high_beam))
        if self._state is not None and state != self._state and now - self._last > 2.0 * self.p.press_every_s:
            self._driver_at = now            # not one of its presses: you changed them
        self._state = state
        if now - self._last < self.p.press_every_s or now - self._driver_at < self.p.driver_hold_s:
            return None
        if high_beam:
            self._last = now
            return self.p.high_beam_button
        if self.night(game_time_min):
            if not low_beam:
                self._last = now
                return self.p.button
        elif low_beam or parking:
            self._last = now
            return self.p.button
        return None
