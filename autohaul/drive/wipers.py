"""Automatic wipers, from the game's own weather.

Telemetry says whether the wipers are on, not whether it's raining, but the game's saves do: `stored_rain_wetness`
rises while it rains and falls as things dry (2026-10-30: 0, 0.05, 0.52 over three autosaves as a shower began). The game
writes an autosave every 5 minutes, so the wipers follow the rain with up to that delay. Reading a save takes a moment,
so a thread does it, never the driving loop.

An earlier version tried to learn rain from the picture, labelled by the wiper switch. The wipers had been left on, so all
it ever learned was that everything is rain.

On 1.53 the stalk works as `wipers` (one stage up) and `wipersback` (one stage down); the direct stages do nothing. After
you switch them yourself AutoHaul leaves them alone for `driver_hold_s`.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


def rain_now(prev, cur) -> bool:
    """Is it raining, from a save's weather (and the one before, if there is one)?"""
    if prev is not None:
        if cur.wetness > prev.wetness + 0.005:
            return True                        # getting wetter
        if cur.wetness < prev.wetness - 0.005:
            return False                       # drying
    return cur.current == 1 and cur.wetness > 0.0


class WeatherWatch:
    """Keeps an eye on the game's autosaves on its own thread. `raining` is None until it knows, or if the newest save is
    too old to say (`max_age_s`)."""

    def __init__(self, read, every_s: float = 20.0, max_age_s: float = 600.0, clock=time.time):
        self.read, self.every_s, self.max_age_s, self.clock = read, every_s, max_age_s, clock
        self.weather = None
        self._raining: bool | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="weather", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            self.poll()
            self._stop.wait(self.every_s)

    def poll(self) -> None:
        try:
            cur = self.read()
        except Exception:                      # noqa: BLE001 (a save being written: the next look will do)
            return
        if cur is None or (self.weather is not None and cur.saved_at <= self.weather.saved_at):
            return
        self._raining, self.weather = rain_now(self.weather, cur), cur

    @property
    def raining(self) -> bool | None:
        w = self.weather
        if w is None or self.clock() - w.saved_at > self.max_age_s:
            return None
        return self._raining

    def describe(self) -> str:
        raining, w = self.raining, self.weather
        if raining is None:
            return "not known (no recent save)"
        age = max(0.0, self.clock() - w.saved_at) / 60.0
        return f"{'raining' if raining else 'dry'}, road {w.wetness * 100:.0f} % wet ({age:.0f} min ago)"


@dataclass
class WiperParams:
    driver_hold_s: float = 300.0       # after you switch them, AutoHaul leaves them alone this long
    press_every_s: float = 1.2
    max_presses: int = 5               # (from the top stage, off takes several)


class WiperControl:
    def __init__(self, press, params: WiperParams | None = None):
        self.press = press
        self.p = params or WiperParams()
        self._last_state: bool | None = None
        self._mine_until = -1e9        # a change of state until then is AutoHaul's own press
        self._driver_at = -1e9
        self._pressed = -1e9
        self._presses = 0

    def step(self, wipers_on: bool, raining: bool | None, now: float, engaged: bool) -> None:
        on = bool(wipers_on)
        if self._last_state is not None and on != self._last_state and now > self._mine_until:
            self._driver_at = now              # you switched them
        self._last_state = on
        if raining is None or not engaged or now - self._driver_at < self.p.driver_hold_s:
            return
        if raining == on:
            self._presses = 0
            return
        if now - self._pressed < self.p.press_every_s or self._presses >= self.p.max_presses:
            return
        self.press("wipers" if raining else "wipersback")
        self._pressed, self._mine_until = now, now + self.p.press_every_s + 1.0
        self._presses += 1
