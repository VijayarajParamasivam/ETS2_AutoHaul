"""Traffic lights for the simulator (the junction logic's LightReader, without a screen).

  ScriptedLights  a colour per (lane, junction, time) from a function: for tests.
  CycledLights    every junction's semaphore groups take turns: green, amber, then red while the others go,
                  like a fixed-time junction. For drives on the real map.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from autohaul.world.rules import JunctionRules


class ScriptedLights:
    def __init__(self, colour_at: Callable[[int, int, float], str | None], clock: Callable[[], float],
                 stop_back: float = 0.0, dark: bool = False):
        self.colour_at, self.clock, self.stop_back = colour_at, clock, stop_back
        self._dark = dark                         # "in view, no lamp lit" when the colour is None

    def dark(self, lane: int, item: int) -> bool:
        return self._dark and self.state(lane, item) is None

    def state(self, lane: int, item: int) -> str | None:
        return self.colour_at(lane, item, self.clock())

    def stop_back_m(self, lane: int, item: int) -> float:
        return self.stop_back


class CycledLights:
    def __init__(self, rules: JunctionRules, clock: Callable[[], float], green_s: float = 20.0, amber_s: float = 3.0,
                 all_red_s: float = 2.0):
        self.rules, self.clock = rules, clock
        self.green_s, self.amber_s, self.all_red_s = green_s, amber_s, all_red_s
        self._groups: dict[int, list[int]] = {}

    def _phase(self, item: int, sid: int) -> tuple[int, int]:
        groups = self._groups.get(item)
        if groups is None:
            lo = np.searchsorted(self.rules.sem_item, np.uint64(item), "left")
            hi = np.searchsorted(self.rules.sem_item, np.uint64(item), "right")
            groups = sorted({int(s) for s in self.rules.sem_sid[lo:hi]} | {sid})
            self._groups[item] = groups
        return groups.index(sid) if sid in groups else 0, len(groups)

    def state(self, lane: int, item: int) -> str | None:
        sid = int(self.rules.sem_id[lane])
        k, n = self._phase(item, sid)
        slot = self.green_s + self.amber_s + self.all_red_s
        t = (self.clock() + (item % 997) * 7.3) % (slot * max(n, 1)) - k * slot     # offset per junction
        if 0 <= t < self.green_s:
            return "green"
        if self.green_s <= t < self.green_s + self.amber_s:
            return "amber"
        return "red"

    def stop_back_m(self, lane: int, item: int) -> float:
        return 0.0

    def dark(self, lane: int, item: int) -> bool:
        return False
