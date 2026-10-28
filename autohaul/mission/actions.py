"""Scripted interactions with the game at a stop: refuel at a pump, sleep at a parking place.

Each script runs one step per frame while the truck stands (brake held) and only presses AutoHaul's own virtual
buttons. Each checks its effect in telemetry and fails with a reason (the driver takes over) when the game
doesn't respond as expected:
  refuel  hold `activate` at the pump (1.53.2.1: "Press and hold Enter to fill the fuel tank", checked in-game
          2026-10-22); if no fuel flows, a single press instead (older versions). Done when the tank is full or
          the flow stops after it started.
  sleep   press `activate` in the parking place (opens the sleep screen), then confirm with `selectfcs`.
          Done when the game clock jumps and the rest timer resets.
  park    at a depot: press `quickpark` (the game's quick park: it parks the trailer, or hooks it up, by itself).
          Done when the caller's check says so (the job delivered / the trailer attached); pressed again after
          `park_wait_s`, then the driver takes over.
The button names and timings are settings (ActionParams) so they can be adjusted to the game without code.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ActionParams:
    settle_s: float = 1.0               # stand still this long before acting
    hold_brake: float = 0.4
    activate: str = "activate"
    confirm: str = "selectfcs"
    refuel_modes: tuple = ("hold", "press")   # refuel: hold activate, else press it once
    flow_wait_s: float = 3.0            # fuel must start rising this soon in each mode ...
    hold_s: float = 120.0               # ... and holding goes on at most this long
    flow_min_l: float = 0.5             # litres that count as "flowing"
    stall_s: float = 3.0                # flow stopped this long: done
    full_share: float = 0.995
    sleep_open_s: float = 1.5           # sleep: wait after activate for the screen
    sleep_wait_s: float = 20.0          # ... and after confirming, for the clock to jump
    sleep_jump_min: float = 60.0        # a game-clock jump this big means the truck slept
    retries: int = 1
    quickpark: str = "quickpark"
    park_wait_s: float = 8.0            # quick park: wait this long for the game after each press
    park_presses: int = 2


@dataclass
class ActionStep:
    press: list[str] = field(default_factory=list)       # buttons to pulse this frame
    hold: dict[str, bool] = field(default_factory=dict)  # buttons to hold (True) or release (False)
    brake: float = 0.4
    done: bool = False
    failed: str | None = None
    note: str = ""


class Refuel:
    name = "refuel"

    def __init__(self, capacity_l: float, params: ActionParams | None = None):
        self.p = params or ActionParams()
        self.capacity = capacity_l
        self.phase, self.t, self.t_phase = "settle", 0.0, 0.0
        self.modes = list(self.p.refuel_modes)
        self.start_l = self.peak_l = None
        self.last_rise_t = 0.0
        self.litres = 0.0

    def step(self, frame, dt: float) -> ActionStep:
        p = self.p
        self.t += dt
        self.t_phase += dt
        out = ActionStep(brake=p.hold_brake)
        fuel = float(frame.fuel)
        if self.start_l is None:
            self.start_l = self.peak_l = fuel
        if fuel > self.peak_l + 0.05:
            self.peak_l, self.last_rise_t = fuel, self.t
        self.litres = self.peak_l - self.start_l
        flowing = self.litres >= p.flow_min_l
        full = fuel >= p.full_share * self.capacity
        if self.phase == "settle":
            out.note = "stopping at the pump"
            if abs(frame.speed) > 0.2:
                self.t_phase = 0.0
            elif self.t_phase >= p.settle_s:
                if full:
                    out.done, out.note = True, "the tank is already full"
                    return out
                self._next(out)
            return out
        holding = self.phase == "hold"
        if holding:
            out.hold[p.activate] = True
        stalled = flowing and self.t - self.last_rise_t >= p.stall_s
        if full or stalled:
            out.done = True
            out.note = f"refuelled {self.litres:.0f} l" + ("" if full else " (flow stopped)")
        elif holding and self.t_phase >= p.hold_s:
            out.done = True
            out.note = f"refuelled {self.litres:.0f} l (stopped holding after {p.hold_s:.0f} s)"
        elif not flowing and self.t_phase >= p.flow_wait_s:
            self._next(out, f"no flow ({self.phase})")
        else:
            out.note = f"refuelling: {fuel:.0f} / {self.capacity:.0f} l" if flowing else f"waiting for fuel ({self.phase})"
        if holding and (out.done or out.failed or self.phase != "hold"):
            out.hold[p.activate] = False
        return out

    def _next(self, out: ActionStep, why: str = "") -> None:
        """Start the next way of refuelling, or fail when none is left."""
        if not self.modes:
            out.failed = "the pump didn't start (no fuel flowing): refuel yourself, then press / to go on"
            return
        mode = self.modes.pop(0)
        self.phase, self.t_phase = mode, 0.0
        if mode == "press":
            out.press.append(self.p.activate)
        else:
            out.hold[self.p.activate] = True
        out.note = (f"{why}: " if why else "") + f"{'holding' if mode == 'hold' else 'pressing'} {self.p.activate}"


class QuickPark:
    name = "park"

    def __init__(self, what: str, check, params: ActionParams | None = None):
        """`what`: "deliver" or "attach" (for the notes); `check(frame)`: True once it worked."""
        self.p = params or ActionParams()
        self.what, self.check = what, check
        self.phase, self.t_phase = "settle", 0.0
        self.presses = 0

    def step(self, frame, dt: float) -> ActionStep:
        p = self.p
        self.t_phase += dt
        out = ActionStep(brake=p.hold_brake)
        if self.presses and self.check(frame):
            out.done = True
            out.note = "delivered" if self.what == "deliver" else "trailer attached"
            return out
        if self.phase == "settle":
            out.note = "stopping at the depot"
            if abs(frame.speed) > 0.2:
                self.t_phase = 0.0
            elif self.t_phase >= p.settle_s:
                self._press(out)
            return out
        out.note = f"quick park: waiting for the game ({self.presses}/{p.park_presses})"
        if self.t_phase >= p.park_wait_s:
            if self.presses < p.park_presses:
                self._press(out)
            else:
                out.failed = ("quick park didn't " + ("deliver" if self.what == "deliver" else "hook up the trailer")
                              + ": " + ("park and deliver yourself" if self.what == "deliver"
                                        else "couple the trailer yourself, then press / to go on"))
        return out

    def _press(self, out: ActionStep) -> None:
        out.press.append(self.p.quickpark)
        self.presses += 1
        self.phase, self.t_phase = "wait", 0.0
        out.note = f"pressing {self.p.quickpark} (quick park)"


class Sleep:
    name = "sleep"

    def __init__(self, params: ActionParams | None = None):
        self.p = params or ActionParams()
        self.phase, self.t_phase = "settle", 0.0
        self.time0 = self.rest0 = None
        self.tries = 0
        self.slept_min = 0.0

    def step(self, frame, dt: float) -> ActionStep:
        p = self.p
        self.t_phase += dt
        out = ActionStep(brake=p.hold_brake)
        gt = float(frame.game_time)
        rest = float(frame.next_rest_stop)
        if self.time0 is None:
            self.time0, self.rest0 = gt, rest
        self.slept_min = gt - self.time0
        slept = self.slept_min >= p.sleep_jump_min or rest > self.rest0 + p.sleep_jump_min
        if slept and self.phase != "settle":
            out.done = True
            out.note = f"slept {self.slept_min / 60:.1f} game hours; rest timer {rest / 60:.1f} h"
            return out
        if self.phase == "settle":
            out.note = "stopping in the parking place"
            if abs(frame.speed) > 0.2:
                self.t_phase = 0.0
            elif self.t_phase >= p.settle_s:
                self._go("open")
                out.press.append(p.activate)
                out.note = f"pressing {p.activate} to sleep"
        elif self.phase == "open":
            out.note = "waiting for the sleep screen"
            if self.t_phase >= p.sleep_open_s:
                self._go("confirm")
                out.press.append(p.confirm)
                out.note = f"confirming with {p.confirm}"
        elif self.phase == "confirm":
            out.note = "sleeping"
            if self.t_phase >= p.sleep_wait_s:
                if self.tries < p.retries:
                    self.tries += 1
                    self._go("open")
                    out.press.append(p.activate)
                    out.note = "no sleep yet: trying again"
                else:
                    out.failed = "the game didn't start sleeping: sleep yourself, then press / to go on"
        return out

    def _go(self, phase: str) -> None:
        self.phase, self.t_phase = phase, 0.0
