"""The truck's indicators while AutoHaul drives.

The game's buttons are toggles, so a button is pressed only when telemetry differs from what's wanted, at most every
`press_every_s` (so a press shows in telemetry before the next). AutoHaul owns the indicators while it's engaged: one
it didn't switch on is taken for the driver's and left alone for `driver_grace_s`, then cancelled. Before this
(2026-10-30) only an indicator AutoHaul remembered switching on was ever switched off, so one that escaped the
bookkeeping (a hand-back just after the press, on at engage) stayed on for the rest of the drive.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass


@dataclass
class IndicatorParams:
    press_every_s: float = 0.6
    driver_grace_s: float = 8.0


class IndicatorControl:
    def __init__(self, press: Callable[[str], None], params: IndicatorParams | None = None):
        self.press = press                     # press("lblinker") / press("rblinker")
        self.p = params or IndicatorParams()
        self.mine: str | None = None           # the side AutoHaul switched on
        self._pressed = -1e9
        self._foreign: dict[str, float | None] = {"left": None, "right": None}

    def reset(self) -> None:
        """Engaged: anything on now was switched on by someone else; the grace time starts."""
        self.mine = None
        self._foreign = {"left": None, "right": None}

    def step(self, left_on: bool, right_on: bool, want: str | None, now: float) -> None:
        if now - self._pressed < self.p.press_every_s:
            return                             # the last press may not show in telemetry yet
        state = {"left": bool(left_on), "right": bool(right_on)}
        if self.mine and not state[self.mine] and want != self.mine:
            self.mine = None                   # it's off (the game cancels it after a turn, too)
        for side, on in state.items():
            if want == side and not on:
                self._press(side, now)
                self.mine = side
                return
            if want == side:                   # on already, and wanted: it's ours now
                self.mine, self._foreign[side] = side, None
                continue
            if not on:
                self._foreign[side] = None
                continue
            if self.mine == side:
                self._press(side, now)
                self.mine = None
                return
            if self._foreign[side] is None:
                self._foreign[side] = now
            if now - self._foreign[side] >= self.p.driver_grace_s:
                self._press(side, now)
                self._foreign[side] = None
                return

    def _press(self, side: str, now: float) -> None:
        self.press(f"{side[0]}blinker")
        self._pressed = now

    def settle_s(self, now: float) -> float:
        """How long until the last press shows in telemetry (handing back: wait, then look at `mine`'s state)."""
        return max(0.0, self.p.press_every_s - (now - self._pressed))
