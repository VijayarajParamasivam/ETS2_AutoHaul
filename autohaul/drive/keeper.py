"""Truck housekeeping while AutoHaul drives: engine running, parking brake off, gearbox in drive.

The game's buttons for these are toggles or one-shot presses (`parkingbrake` toggles, `geardrive` selects drive),
so the keeper presses one only while telemetry shows it is needed, and then waits `press_every_s` for telemetry to
catch up before pressing again. The engine is a starter: `engine` must be held (~1.5 s, measured on 1.53; a short
press does nothing), with the electrics on (`engineelect` toggles them). While the truck isn't ready the caller
holds it on the brake instead of sending throttle the game can't use.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class KeeperParams:
    start_engine: bool = True
    release_parking_brake: bool = True
    keep_in_drive: bool = True
    press_every_s: float = 2.0          # one press (or one start attempt), then wait for telemetry
    starter_s: float = 1.5              # hold the engine button this long
    neutral_s: float = 0.5              # in neutral with the throttle down this long: shift to drive
    max_presses: int = 4                # per problem; then it's reported as not working (the caller hands back)


@dataclass
class KeeperStep:
    press: str | None = None            # a button to press once now
    hold: str | None = None             # a button to hold down this frame (released when no longer named)
    problem: str | None = None          # what isn't ready yet: hold the truck on the brake
    failure: str | None = None          # the presses didn't work: hand back with this


class Keeper:
    def __init__(self, params: KeeperParams | None = None):
        self.p = params or KeeperParams()
        self.reset()

    def reset(self) -> None:
        """A fresh start (on every engage): earlier failed attempts don't count against this one."""
        self._last = -1e9
        self._neutral = 0.0
        self._tries: dict[str, int] = {}
        self._hold_until = -1e9
        self._holding: str | None = None

    def _attempt(self, problem: str, now: float, press: str | None = None, hold: str | None = None) -> KeeperStep:
        p = self.p
        if now - self._last < p.press_every_s:
            return KeeperStep(problem=problem)
        n = self._tries.get(problem, 0)
        if n >= p.max_presses:
            what = f"holding `{hold}`" if hold else f"pressing `{press}`"
            return KeeperStep(problem=problem, failure=f"{problem} didn't work after {n} tries ({what}): take over")
        self._tries[problem] = n + 1
        self._last = now
        if hold:
            self._holding, self._hold_until = hold, now + p.starter_s
            return KeeperStep(hold=hold, problem=problem)
        return KeeperStep(press=press, problem=problem)

    def step(self, f, want_move: bool, dt: float, now: float) -> KeeperStep:
        p = self.p
        if self._holding and now < self._hold_until:
            return KeeperStep(hold=self._holding, problem="starting the engine")
        if self._holding:
            self._holding = None
            self._last = now                         # give the engine time to show in telemetry
        if p.start_engine and not f.engine_enabled:
            if not getattr(f, "electric_enabled", True):
                return self._attempt("switching the electrics on", now, press="engineelect")
            return self._attempt("starting the engine", now, hold="engine")
        if p.release_parking_brake and want_move and f.parking_brake:
            return self._attempt("releasing the parking brake", now, press="parkingbrake")
        if p.keep_in_drive and want_move and f.displayed_gear < 0:
            if f.speed < -0.3:
                return KeeperStep(problem="stopping before shifting out of reverse")
            return self._attempt("shifting from reverse to drive", now, press="geardrive")
        stuck = p.keep_in_drive and want_move and f.displayed_gear == 0 and f.speed > -0.2
        self._neutral = self._neutral + dt if stuck else 0.0
        if self._neutral > p.neutral_s:
            return self._attempt("shifting from neutral to drive", now, press="geardrive")
        self._tries.clear()
        return KeeperStep()
