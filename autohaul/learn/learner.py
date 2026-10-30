"""Auto-learning: AutoHaul adjusts its own driving from what happens on every drive, and remembers it.

It watches the same notes the recordings hold (10 per second), so what it does can be replayed from old drives
(tools/learn_replay.py) and checked in the simulator. Everything it changes is a small step inside hard bounds, and
every change is written to learning.jsonl with its reason. What it learns, per truck state (with / without a trailer):

  curves       lat_acc_max: how fast it takes curves. Runs wide (> `curve_bad_m`) or you take over in a curve: slower.
               Tracks well while the limit is what sets the speed: very slowly faster.
  turn-in      predict_s: how far ahead the steering looks to make up for the delays. Runs outside in curves: further;
               cuts the inside: less. (Simulated on the real controller: this sets the balance; ff_preview_s does
               nothing to it.)
  correction   omega_max: how firmly it pulls back to the line. Curves tracked too loosely: firmer (0.6 -> 1.0 cuts
               the error in curves 3x in the simulator); weaving on straights: softer, at once and by more.
  understeer   c_us: how much less the truck turns than the geometry says, so a drive starts already tuned.
  braking      plan_decel: how early it brakes for curves. Arrives too fast: earlier.
  pedals       thr_accel, brake_decel: how much a pedal really does (a loaded truck differs from a bare tractor).
And, per lane of the map (kept by map item and lane, so it survives a rebuilt map):
  trouble spots  you took over, it faulted or crashed here: from the second time it slows down before the place.
                 Clean passes wear this off again.
  route          the game's GPS is the teacher: following its own route, AutoHaul saw the GPS distance jump up while
                 its own remaining distance didn't (the GPS wanted another road): that lane costs more from now on,
                 so the routes come to agree. If its own distance jumped too, it missed a turn: no lesson for the route.
From you, whenever you drive:
  lessons        after any hand-over (a fault, a sharp turn, a toll gate, or you just taking control) it watches how you
                 drive the next stretch: your speed and your line, every 5 m along each lane of the junctions and curves
                 you pass. Next time it follows that line at that speed, and a sharp turn you've shown it is no longer a
                 reason to hand back. A lesson that ended in damage is thrown away, and speeds on the way to or from a
                 stop say nothing (that was traffic or a light).
  toll booths    where you paid: that booth is preferred, is no longer avoided, and it stops where you stopped.
  your style     your curve speed and how hard you accelerate pull its values your way, slowly.
A scorecard per drive goes in the same file. If the last drives are clearly worse than the ones before, the driving
values go back to what they were then.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from autohaul.control.vehicle import pose_from_frame

VERSION = 1
BOUNDS = {
    "lat_acc_max": (1.4, 2.6),
    "plan_decel": (0.5, 1.2),
    "predict_s": (0.1, 0.45),
    "omega_max": (0.5, 1.0),
    "thr_accel": (0.25, 2.5),
    "brake_decel": (2.0, 8.0),
    "c_us": (0.0, 0.004),
    "accel_max": (0.4, 1.2),
}
DRIVER_KEY = "driving key"
SPOT_FAULTS = ("sharp turn", "off the lane centre", "the wheel is moving against", "lost the lane", "left the way",
               "drifting", "lane match")


@dataclass
class LearnParams:
    curve_kappa: float = 0.006         # 1/m: a curve (radius under ~170 m)
    curve_min_ms: float = 5.0
    curve_bad_m: float = 1.0           # peak error in a curve that counts as running wide
    curve_good_m: float = 0.4          # ... and as tracking well
    lat_step_down: float = 0.03
    lat_step_up: float = 0.004
    takeover_step: float = 0.05        # you took over in a curve
    near_limit: float = 0.8            # the curve was driven at this share of lat_acc_max at least
    bias_m: float = 0.15               # mean error to one side in a curve before the turn-in moves
    predict_step: float = 0.005
    loose_m: float = 0.35              # peak error in a curve above which the correction firms up
    omega_step_up: float = 0.004
    omega_step_down: float = 0.02
    weave_window_s: float = 20.0       # on straights: this long ...
    weave_swings: int = 4              # ... this many swings from one side of the mean to the other ...
    weave_band_m: float = 0.15         # ... each past this on both sides: it's weaving (a kink in the map is one jump,
                                       # and noise around the mean doesn't count)
    weave_quiet_s: float = 60.0        # no firming up for this long after weaving
    overshoot_ms: float = 1.4          # into a curve this much above its speed: braked too late
    decel_step_down: float = 0.01
    decel_step_up: float = 0.001
    pedal_alpha: float = 0.002
    us_alpha: float = 0.002
    spot_slow_after: int = 2           # slow down at a spot from this many incidents
    spot_speed_share: float = 0.85     # to this share of the speed of the incident ...
    spot_min_ms: float = 6.0           # ... never under this
    spot_ahead_m: float = 150.0
    spot_max_ms: float = 25.0          # above this a spot isn't remembered (a takeover at motorway speed isn't a place)
    spot_clean_wear: float = 0.34      # a clean pass takes this off the count
    gps_rise_m: float = 1500.0         # game metres the GPS distance must jump up
    gps_max_rise_m: float = 150000.0   # more than this is a new destination
    gps_window_s: float = 20.0
    gps_penalty_share: float = 0.5     # of the jump in world metres
    gps_penalty_max_m: float = 1000.0
    lesson_m: float = 600.0            # a lesson: this far after the hand-over ...
    lesson_s: float = 120.0            # ... or this long
    lesson_lane_m: float = 3.5         # within this of a lane counts as driving it
    lesson_bin_m: float = 5.0
    lesson_curve: float = 1 / 200.0    # 1/m: only junctions and lanes this curved are places worth a lesson
    lesson_min_ms: float = 2.5         # slower says nothing about the place
    lesson_stop_ms: float = 1.0        # a stop: ...
    lesson_before_stop_m: float = 40.0  # ... the speed this far before it and ...
    lesson_after_stop_m: float = 30.0  # ... this far after it was about the stop, not the place
    lesson_speed_slack: float = 1.05   # it may take the place this much faster than you
    lesson_min_cap_ms: float = 5.0     # never a limit under this
    lesson_creep: float = 1.03         # each pass it drives alone relaxes the limit by this, until you show otherwise
    lesson_max_offset_m: float = 1.5   # the most it shifts its line
    lesson_line_range_m: float = 1.5   # your line must stay within this across the lane to count as a line
    lesson_max_wear: float = 0.005     # more damage than this during a lesson: nothing to learn from it
    toll_prefer_m: float = 150.0       # the other booths of a plaza cost this much more than the one you paid at
    toll_back_min_m: float = 0.5
    toll_back_max_m: float = 8.0
    style_alpha: float = 0.01          # your style pulls its values by this share per curve / acceleration
    style_accel_window_s: float = 1.0
    worse_factor: float = 1.5          # scorecards this much worse: roll back
    rollback_sessions: int = 3
    min_session_km: float = 30.0       # game km engaged for a drive to get a scorecard


@dataclass
class _Session:
    started: float = 0.0
    engaged_game_km: float = 0.0
    takeovers: int = 0
    faults: int = 0
    fines: dict = field(default_factory=dict)
    lat_sq: float = 0.0
    n: int = 0
    curve_sq: float = 0.0
    n_curve: int = 0
    overshoots: int = 0
    curves: int = 0
    gps_jumps: int = 0
    misses: int = 0


def lane_key(lm, lane: int) -> str:
    return f"{int(lm.lane_item[lane]):x}:{int(lm.lane_sub[lane])}"


class Learner:
    def __init__(self, path: Path | None, lm=None, params: LearnParams | None = None, clock=time.time):
        self.path = Path(path) if path else None
        self.lm = lm
        self.p = params or LearnParams()
        self.clock = clock
        self.values: dict[str, dict[str, float]] = {"tractor": {}, "trailer": {}}
        self.spots: dict[str, dict] = {}
        self.route_cost: dict[str, float] = {}
        self.lessons: dict[str, dict] = {}     # lane -> {"n", "why", "bins": [[speed or None, lateral or None], ...]}
        self.toll_lanes: dict[str, dict] = {}  # booth lane -> {"n", "back_m"}
        self.lesson: dict | None = None        # the one going on
        self._lane_rec: dict[int, dict | None] = {}
        self._rules = None
        self._gate_s = None
        self._front_bumper = 3.2
        self._driver_curve_state: dict | None = None
        self._driver_accel: tuple | None = None
        self.scorecards: list[dict] = []
        self.snapshots: list[dict] = []
        self.recent: list[str] = []
        self.s = _Session(started=clock())
        self.ctx = "tractor"
        self._sinks: list = []                 # objects the learned values are written into
        self._curve: dict | None = None
        self._prev_t = None
        self._prev_v = None
        self._last_lane = None
        self._lane_seen: dict[str, float] = {}
        self._gps_low = None
        self._last_curve_t = -1e9
        self._last_note: dict = {}
        self._weave: list[tuple[float, float]] = []
        self._weave_at = -1e9
        self._dirty = False
        self._load()

    # ------------------------------------------------------------- persistence
    def _load(self) -> None:
        if self.path is None or not self.path.exists():
            return
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        if d.get("version") != VERSION:
            return
        self.values = {"tractor": d.get("values", {}).get("tractor", {}), "trailer": d.get("values", {}).get("trailer", {})}
        self.spots = d.get("spots", {})
        self.route_cost = d.get("route_cost", {})
        self.lessons = d.get("lessons", {})
        self.toll_lanes = d.get("toll_lanes", {})
        self.scorecards = d.get("scorecards", [])
        self.snapshots = d.get("snapshots", [])

    def save(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"version": VERSION, "values": self.values, "spots": self.spots,
                                   "route_cost": self.route_cost, "lessons": self.lessons, "toll_lanes": self.toll_lanes,
                                   "scorecards": self.scorecards[-60:],
                                   "snapshots": self.snapshots[-20:]}, indent=0), encoding="utf-8")
        os.replace(tmp, self.path)
        self._dirty = False

    def _log(self, what: str, **fields) -> None:
        self.recent.append(what)
        self.recent = self.recent[-3:]
        if self.path is None:
            return
        try:
            with open(self.path.with_name("learning.jsonl"), "a", encoding="utf-8") as fh:
                fh.write(json.dumps({"t": round(self.clock(), 1), "what": what, **fields}) + "\n")
        except OSError:
            pass

    # ------------------------------------------------------------- values
    def value(self, name: str, default: float) -> float:
        return self.values[self.ctx].get(name, default)

    def _nudge(self, name: str, default: float, delta: float, why: str | None = None) -> None:
        lo, hi = BOUNDS[name]
        old = self.values[self.ctx].get(name, default)
        new = min(hi, max(lo, old + delta))
        if new == old:
            return
        self.values[self.ctx][name] = new
        self._dirty = True
        self._write_sinks()
        if why:
            self._log(f"{name} {old:.3f} -> {new:.3f}: {why} ({'with' if self.ctx == 'trailer' else 'no'} trailer)",
                      name=name, old=old, new=new, ctx=self.ctx)

    def attach(self, speed_params=None, lateral=None, router=None, rules=None, gate_s=None, front_bumper: float = 3.2) -> None:
        """Keep these up to date with what's learned: the SpeedParams, the LateralController, the Router's lane costs.
        `rules` (JunctionRules): to prefer the booths you paid at; `gate_s(lane, item)`: metres along a booth lane to its
        gate, to learn where you stop."""
        self._sp, self._lat = speed_params, lateral
        self._rules, self._gate_s, self._front_bumper = rules, gate_s, front_bumper
        self._defaults = {}
        if speed_params is not None:
            self._defaults.update(lat_acc_max=speed_params.lat_acc_max, plan_decel=speed_params.plan_decel,
                                  thr_accel=speed_params.thr_accel, brake_decel=speed_params.brake_decel,
                                  accel_max=speed_params.accel_max)
        if lateral is not None:
            self._defaults.update(predict_s=lateral.p.predict_s, omega_max=lateral.p.omega_max)
        self._write_sinks()
        if router is not None and self.lm is not None:
            router.add_penalties(self.lane_penalties())

    def _write_sinks(self) -> None:
        sp, lat = getattr(self, "_sp", None), getattr(self, "_lat", None)
        d = getattr(self, "_defaults", {})
        if sp is not None:
            for name in ("lat_acc_max", "plan_decel", "thr_accel", "brake_decel", "accel_max"):
                setattr(sp, name, self.values[self.ctx].get(name, d[name]))
        if lat is not None:
            lat.p.predict_s = self.values[self.ctx].get("predict_s", d["predict_s"])
            lat.p.omega_max = self.values[self.ctx].get("omega_max", d["omega_max"])
            if "c_us" in self.values[self.ctx] and self.values[self.ctx]["c_us"] > lat.c_us:
                lat.c_us = self.values[self.ctx]["c_us"]

    def _default(self, name: str, fallback: float) -> float:
        return getattr(self, "_defaults", {}).get(name, fallback)

    def lane_penalties(self) -> dict[int, float]:
        """Lane index -> extra route cost (metres), resolved in the current map."""
        out: dict[int, float] = {}
        for key, cost in self.route_cost.items():
            for ln in self._lanes_of_key(key):
                out[ln] = cost
        if self._rules is not None:                  # a plaza's other booths cost more than the one you paid at
            for key in self.toll_lanes:
                for ln in self._lanes_of_key(key):
                    item = self.lm.lane_item[ln]
                    others = np.flatnonzero((self.lm.lane_item == item) & (np.asarray(self._rules.sem_id) >= 0))
                    for other in others.tolist():
                        if other != ln and lane_key(self.lm, other) not in self.toll_lanes:
                            out[other] = out.get(other, 0.0) + self.p.toll_prefer_m
        return out

    def _lanes_of_key(self, key: str) -> list[int]:
        item, sub = key.split(":")
        return np.flatnonzero((self.lm.lane_item == np.uint64(int(item, 16))) & (self.lm.lane_sub == int(sub))).tolist()

    def paid_booths(self) -> set[tuple[int, int]]:
        """(item, sub) of the booths you've paid at: however a drive went there, they do take payment."""
        out = set()
        for key in self.toll_lanes:
            item, sub = key.split(":")
            out.add((int(item, 16), int(sub)))
        return out

    # ------------------------------------------------------------- what's asked while driving
    def caution(self, lane: int) -> float | None:
        """A speed limit (m/s) for this lane: a place where you've had to take over before, or None."""
        if not self.spots or self.lm is None:
            return None
        rec = self.spots.get(lane_key(self.lm, lane))
        if rec is None or rec["n"] < self.p.spot_slow_after:
            return None
        return max(self.p.spot_min_ms, self.p.spot_speed_share * rec["speed"])

    def bins(self, lane: int) -> list | None:
        """How you drove this lane, every `lesson_bin_m`: [speed or None, lateral or None], or None if you haven't."""
        if not self.lessons:
            return None
        if lane not in self._lane_rec:
            self._lane_rec[lane] = self.lessons.get(lane_key(self.lm, lane))
        rec = self._lane_rec[lane]
        return rec["bins"] if rec else None

    def toll_back(self, lane: int) -> float | None:
        """Where you stopped at this booth: metres before its gate, or None."""
        rec = self.toll_lanes.get(lane_key(self.lm, lane))
        return rec.get("back_m") if rec else None

    def summary(self) -> list[str]:
        """For the overlay."""
        c = self.values[self.ctx]
        lines = []
        for name, label, fallback in (("lat_acc_max", "Curves", 2.0), ("predict_s", "Turn-in", 0.25), ("omega_max", "Correction", 0.6),
                                       ("plan_decel", "Braking", 0.8)):
            lines.append(f"{label} {c.get(name, self._default(name, fallback)):.2f}")
        hot = sum(1 for r in self.spots.values() if r["n"] >= self.p.spot_slow_after)
        out = ["  ".join(lines), f"From you: {len(self.lessons)} lanes, {len(self.toll_lanes)} toll booths. "
               f"Trouble spots: {hot}, route corrections: {len(self.route_cost)}"]
        if self.lesson is not None:
            out.insert(0, f"Watching how you drive this ({self.lesson['m']:.0f} of {self.p.lesson_m:.0f} m)")
        return out + self.recent[-1:]

    # ------------------------------------------------------------- observing
    def set_context(self, trailer: bool) -> None:
        ctx = "trailer" if trailer else "tractor"
        if ctx != self.ctx:
            self.ctx = ctx
            self._write_sinks()

    def on_handback(self, reason: str, speed: float, lane: int | None) -> None:
        p = self.p
        driver = reason.startswith(DRIVER_KEY)
        spot = driver or any(s in reason for s in SPOT_FAULTS)
        if driver:
            self.s.takeovers += 1
        elif spot or "take over" in reason:
            self.s.faults += 1
        if lane is not None and self.lm is not None and spot and speed <= p.spot_max_ms and speed > 1.0:
            self._mark_spot(lane, speed, reason)
        if driver and self._prev_t is not None and self._prev_t - self._last_curve_t < 3.0:
            self._nudge("lat_acc_max", self._default("lat_acc_max", 2.0), -p.takeover_step, "you took over in a curve")

    def on_fine(self, offence: str, speed: float, lane: int | None) -> None:
        self.s.fines[offence] = self.s.fines.get(offence, 0) + 1
        if offence == "crash" and self.lesson is not None:
            self.lesson["crashed"] = True
        if offence == "crash" and lane is not None and self.lm is not None and speed <= self.p.spot_max_ms:
            self._mark_spot(lane, max(speed, 8.0), "a crash")

    def _mark_spot(self, lane: int, speed: float, why: str) -> None:
        key = lane_key(self.lm, lane)
        rec = self.spots.setdefault(key, {"n": 0.0, "speed": speed, "why": why})
        rec["n"] += 1.0
        rec["speed"] = min(rec["speed"], speed)
        rec["why"] = why
        self._dirty = True
        if rec["n"] >= self.p.spot_slow_after:
            self._log(f"trouble spot {key}: slows to {max(self.p.spot_min_ms, self.p.spot_speed_share * rec['speed']) * 3.6:.0f} km/h "
                      f"there ({why}, {rec['n']:.0f} times)", lane=key)

    def observe(self, t: float, f, note: dict, engaged: bool, trailer: bool | None = None) -> None:
        """A note of the drive (10 per second): `f` the telemetry frame, `note` the autopilot's status fields."""
        if trailer is not None:
            self.set_context(trailer)
        v = float(f.speed)
        dt = 0.0 if self._prev_t is None else t - self._prev_t
        if engaged and 0 < dt < 1.0:
            self.s.engaged_game_km += abs(v) * dt * float(getattr(f, "local_scale", 19.0) or 19.0) / 1000.0
            self._gps(t, f, note)
        if engaged and note:
            self._watch_lane(note)
            self._score(note)
            self._curves(t, v, note, dt)
            self._weaving(t, v, note)
            self._pedals(v, dt, note)
            self._understeer(v, note)
        else:
            self._curve = None
        self._prev_t, self._prev_v = t, v
        self._last_note = note

    def _score(self, note: dict) -> None:
        lat = float(note.get("lateral") or 0.0)
        self.s.lat_sq += lat * lat
        self.s.n += 1
        if abs(float(note.get("kappa_ff") or 0.0)) > self.p.curve_kappa:
            self.s.curve_sq += lat * lat
            self.s.n_curve += 1

    def _watch_lane(self, note: dict) -> None:
        lane = note.get("lane")
        if lane is None or lane == self._last_lane or self.lm is None:
            return
        prev, self._last_lane = self._last_lane, lane
        if prev is None:
            return
        key = lane_key(self.lm, prev)                # left that lane without an incident: a clean pass
        taught = self.lessons.get(key)
        if taught is not None and not note.get("fault"):
            for row in taught["bins"]:
                if row[0] is not None:
                    row[0] *= self.p.lesson_creep
            self._dirty = True
        rec = self.spots.get(key)
        if rec is not None and not note.get("fault"):
            rec["n"] -= self.p.spot_clean_wear
            if rec["n"] <= 0:
                del self.spots[key]
                self._log(f"trouble spot {key} worn off: clean passes", lane=key)
            self._dirty = True

    def _curves(self, t: float, v: float, note: dict, dt: float) -> None:
        p = self.p
        kappa = float(note.get("kappa_ff") or 0.0)
        lat = float(note.get("ctrl_lateral") or 0.0)
        busy = bool(note.get("lane_change")) or bool(note.get("junction"))
        in_curve = abs(kappa) > p.curve_kappa and v > p.curve_min_ms and not busy
        if in_curve:
            self._last_curve_t = t
            c = self._curve
            if c is None:
                c = self._curve = {"peak": 0.0, "acc": 0.0, "sum": 0.0, "n": 0, "side": math.copysign(1.0, kappa),
                                   "over": False, "quiet": 0.0, "t": t}
                tgt = float(note.get("target_kmh") or 0.0) / 3.6
                c["over"] = tgt > 0 and v - tgt > p.overshoot_ms
            c["quiet"] = 0.0
            c["peak"] = max(c["peak"], abs(lat))
            c["acc"] = max(c["acc"], v * v * abs(kappa))
            c["sum"] += lat * math.copysign(1.0, kappa)      # + = towards the inside of the curve
            c["n"] += 1
            return
        c = self._curve
        if c is None:
            return
        c["quiet"] += dt
        if c["quiet"] < 1.0 and not busy:
            return
        self._curve = None
        self._finish_curve(c, t)

    def _weaving(self, t: float, v: float, note: dict) -> None:
        p = self.p
        straight = abs(float(note.get("kappa_ff") or 0.0)) < 0.002 and v > 10.0 and not note.get("lane_change")
        if not straight:
            self._weave = []
            return
        self._weave.append((t, float(note.get("lateral") or 0.0)))
        while self._weave and t - self._weave[0][0] > p.weave_window_s:
            self._weave.pop(0)
        if t - self._weave[0][0] < 0.9 * p.weave_window_s:
            return
        lat = [x for _, x in self._weave]
        mean = sum(lat) / len(lat)
        swings, side = 0, 0
        for x in lat:
            d = x - mean
            if abs(d) >= p.weave_band_m and (d > 0) != (side > 0) or (side == 0 and abs(d) >= p.weave_band_m):
                swings += side != 0
                side = 1 if d > 0 else -1
        if swings >= p.weave_swings:
            self._weave, self._weave_at = [], t
            self._nudge("omega_max", self._default("omega_max", 0.6), -p.omega_step_down,
                        f"weaving on a straight ({max(lat) - min(lat):.2f} m swing)")

    def _finish_curve(self, c: dict, t: float) -> None:
        p = self.p
        self.s.curves += 1
        limit = self.value("lat_acc_max", self._default("lat_acc_max", 2.0))
        d_lat = self._default("lat_acc_max", 2.0)
        if c["peak"] > p.curve_bad_m:
            self._nudge("lat_acc_max", d_lat, -p.lat_step_down, f"ran {c['peak']:.1f} m wide in a curve")
        elif c["peak"] < p.curve_good_m and c["acc"] >= p.near_limit * limit:
            self._nudge("lat_acc_max", d_lat, p.lat_step_up)
        if c["n"] >= 10:
            mean = c["sum"] / c["n"]
            d = self._default("predict_s", 0.25)
            if mean < -p.bias_m:
                self._nudge("predict_s", d, p.predict_step)            # outside: look further ahead
            elif mean > p.bias_m:
                self._nudge("predict_s", d, -p.predict_step)           # cutting in: less
            if c["peak"] > p.loose_m and t - self._weave_at > p.weave_quiet_s:
                self._nudge("omega_max", self._default("omega_max", 0.6), p.omega_step_up)
        d_dec = self._default("plan_decel", 0.8)
        if c["over"]:
            self.s.overshoots += 1
            self._nudge("plan_decel", d_dec, -p.decel_step_down, "arrived too fast at a curve")
        else:
            self._nudge("plan_decel", d_dec, p.decel_step_up)

    def _pedals(self, v: float, dt: float, note: dict) -> None:
        if self._prev_v is None or not 0.02 < dt < 0.5:
            return
        a = (v - self._prev_v) / dt
        thr, brk = float(note.get("throttle") or 0.0), float(note.get("brake") or 0.0)
        p = self.p
        if thr > 0.3 and brk < 0.01 and a > 0.05 and v > 2.0:
            est = (a + 0.25) / thr
            old = self.value("thr_accel", self._default("thr_accel", 1.0))
            self._nudge("thr_accel", self._default("thr_accel", 1.0), p.pedal_alpha * (est - old))
        elif brk > 0.15 and thr < 0.01 and a < -0.1 and v > 2.0:
            est = max(-a - 0.25, 0.0) / brk
            old = self.value("brake_decel", self._default("brake_decel", 4.0))
            self._nudge("brake_decel", self._default("brake_decel", 4.0), p.pedal_alpha * (est - old))

    def _understeer(self, v: float, note: dict) -> None:
        ratio = float(note.get("curve_ratio") or 0.0)
        if ratio <= 0.0 or v < 12.0:
            return
        est = (1.0 / ratio - 1.0) / (v * v)
        old = self.values[self.ctx].get("c_us", est)
        self.values[self.ctx]["c_us"] = min(BOUNDS["c_us"][1], max(0.0, old + self.p.us_alpha * (est - old)))

    def _gps(self, t: float, f, note: dict) -> None:
        p = self.p
        nav = float(getattr(f, "navigation_distance", 0.0))
        if nav <= 0 or not note.get("on_route") or self.lm is None or note.get("lane") is None:
            self._gps_low = None
            return
        low = self._gps_low
        route_m = float(note.get("route_km") or 0.0) * 19000.0
        if low is None or nav < low[0] or t - low[1] > p.gps_window_s:
            self._gps_low = (nav, t, route_m)
            return
        rise = nav - low[0]
        if rise >= p.gps_rise_m:
            own_rise = route_m - low[2]
            self._gps_low = (nav, t, route_m)
            if rise > p.gps_max_rise_m:
                return                               # a new destination
            if own_rise > 0.3 * rise:
                self.s.misses += 1
                self._log(f"missed a turn (+{rise / 1000:.0f} km by the GPS, +{own_rise / 1000:.0f} km by its own route)")
                return
            lane = int(note["lane"])
            key = lane_key(self.lm, lane)
            extra = min(p.gps_penalty_max_m, self.route_cost.get(key, 0.0) + p.gps_penalty_share * rise / 19.0)
            self.route_cost[key] = extra
            self.s.gps_jumps += 1
            self._dirty = True
            self._log(f"route: the GPS wanted another way at {key} (+{rise / 1000:.0f} km): that lane now costs {extra:.0f} m extra",
                      lane=key, rise_m=round(rise))

    # ------------------------------------------------------------- learning from the driver
    @staticmethod
    def _wear(f) -> float:
        return max((float(getattr(f, n, 0.0)) for n in ("wear_cabin", "wear_chassis", "wear_engine", "wear_transmission", "wear_wheels")),
                   default=0.0)

    def start_lesson(self, t: float, f, why: str) -> None:
        """You have the truck now: watch how you drive it (the reason it was handed over goes with the lesson)."""
        self.end_lesson(f)
        self.lesson = {"t0": t, "t": t, "why": why, "m": 0.0, "wear0": self._wear(f), "samples": [], "crashed": False}

    def end_lesson(self, f=None) -> int:
        """Keep what the lesson showed. Returns the number of lanes learned."""
        les, self.lesson = self.lesson, None
        if les is None or self.lm is None:
            return 0
        p = self.p
        if les["crashed"] or (f is not None and self._wear(f) - les["wear0"] > p.lesson_max_wear):
            self._log(f"not learning from that: damage while you drove ({les['why']})")
            return 0
        samples = les["samples"]
        stops = [m for m, _, _, v, _ in samples if v < p.lesson_stop_ms]
        per: dict[int, dict[int, list]] = {}
        for m, lane, b, v, lat in samples:
            near_stop = any(-p.lesson_after_stop_m <= stop - m <= p.lesson_before_stop_m for stop in stops)
            row = per.setdefault(lane, {}).setdefault(b, [0.0, 0, 0.0, 0])
            if not near_stop and v >= p.lesson_min_ms:
                row[0] += v
                row[1] += 1
            row[2] += lat
            row[3] += 1
        learned = 0
        speeds = []
        for lane, rows in per.items():
            if not self._worth_a_lesson(lane):
                continue
            n_bins = 1 + int(self.lm.lane_length(lane) / p.lesson_bin_m)
            new = [[None, None] for _ in range(n_bins)]
            for b, (sv, nv, sl, nl) in rows.items():
                if b < n_bins:
                    new[b] = [sv / nv if nv else None, sl / nl if nl else None]
            lats = [r[1] for r in new if r[1] is not None]
            # your line only from a junction's path driven steadily: on a road, or swinging across, that's a lane change
            # (seen on real drives: -3.4 m), and copying it would steer towards the next lane or a rail
            if (self.lm.lane_kind[lane] == 0 or not lats or max(lats) - min(lats) > p.lesson_line_range_m
                    or max(abs(x) for x in lats) > p.lesson_max_offset_m):
                for r in new:
                    r[1] = None
            if not any(r[0] is not None or r[1] is not None for r in new):
                continue
            key = lane_key(self.lm, lane)
            old = self.lessons.get(key)
            if old is not None and len(old["bins"]) == n_bins:     # half what you showed before, half what you show now
                for a, b in zip(old["bins"], new):
                    for k in (0, 1):
                        if a[k] is not None and b[k] is not None:
                            b[k] = 0.5 * (a[k] + b[k])
                        elif b[k] is None:
                            b[k] = a[k]
            self.lessons[key] = {"n": (old["n"] if old else 0) + 1, "why": les["why"][:60], "bins": new}
            self._lane_rec.clear()
            learned += 1
            speeds += [r[0] for r in new if r[0] is not None]
        if learned:
            self._dirty = True
            slow = f", down to {min(speeds) * 3.6:.0f} km/h" if speeds else ""
            self._log(f"learned from you: {learned} lanes{slow} ({les['why'][:50]})", lanes=learned, why=les["why"])
        return learned

    def _worth_a_lesson(self, lane: int) -> bool:
        lm = self.lm
        if lm.lane_kind[lane] != 0:                  # KIND_ROAD is 0: anything else is a junction's path
            return True
        rot = np.unwrap(lm.seg_rot[lm.lane_first[lane]:lm.lane_last[lane] + 1])
        length = float(lm.lane_length(lane))
        return length > 0 and abs(np.diff(rot)).sum() / max(length, 1.0) >= self.p.lesson_curve

    def observe_driver(self, t: float, f) -> None:
        """A frame while you drive (not paused; ten a second is plenty)."""
        p = self.p
        v = float(f.speed)
        dt = 0.0 if self._prev_t is None else t - self._prev_t
        self._prev_t, self._prev_v_driver = t, v
        if self.lm is None:
            return
        pose = pose_from_frame(f)
        hits = self.lm.nearest(pose.x, pose.y, pose.rotation, max_dist=p.lesson_lane_m, max_heading_err=math.radians(50), k=1)
        hit = hits[0] if hits else None
        if self.lesson is not None:
            les = self.lesson
            if 0 < dt < 1.0:
                les["m"] += abs(v) * dt
            if hit is not None and v >= 0:
                done = float(self.lm.seg_len[self.lm.lane_first[hit.lane]:hit.segment].sum() + hit.t * self.lm.seg_len[hit.segment])
                les["samples"].append((les["m"], hit.lane, int(done / p.lesson_bin_m), v, hit.lateral))
            if les["m"] >= p.lesson_m or t - les["t0"] >= p.lesson_s:
                self.end_lesson(f)
        self._style(t, v, f, hit)

    def _style(self, t: float, v: float, f, hit) -> None:
        """Your curve speed and acceleration pull its own values slowly your way."""
        p = self.p
        kappa = 0.0
        if hit is not None:
            lm = self.lm
            a, b = max(hit.segment - 2, lm.lane_first[hit.lane]), min(hit.segment + 2, lm.lane_last[hit.lane])
            if b > a:
                turn = np.unwrap(lm.seg_rot[[a, b]])
                kappa = abs(float(turn[1] - turn[0])) / max(float(lm.seg_len[a:b].sum()), 1.0)
        c = self._driver_curve_state
        if kappa > p.curve_kappa and v > p.curve_min_ms:
            if c is None:
                c = self._driver_curve_state = {"peak": 0.0, "wear": self._wear(f)}
            c["peak"] = max(c["peak"], v * v * kappa)
        elif c is not None:
            self._driver_curve_state = None
            if self._wear(f) - c["wear"] < 0.001 and c["peak"] > 0.5:
                name = "lat_acc_max"
                lo, hi = BOUNDS[name]
                old = self.value(name, self._default(name, 2.0))
                self._nudge(name, self._default(name, 2.0), p.style_alpha * (min(hi, max(lo, c["peak"])) - old))
        thr = float(getattr(f, "input_throttle", 0.0))
        d = self._driver_accel
        if thr > 0.3 and 3.0 < v < 25.0:
            if d is None:
                self._driver_accel = (t, v)
            elif t - d[0] >= p.style_accel_window_s:
                a = (v - d[1]) / (t - d[0])
                self._driver_accel = (t, v)
                if 0.05 < a < 3.0:
                    old = self.value("accel_max", self._default("accel_max", 0.7))
                    self._nudge("accel_max", self._default("accel_max", 0.7), p.style_alpha * (a - old))
        else:
            self._driver_accel = None

    def on_toll_paid(self, f) -> None:
        """You paid at a booth yourself: that one takes payment, and you stopped in the right place."""
        if self.lm is None:
            return
        pose = pose_from_frame(f)
        hits = self.lm.nearest(pose.x, pose.y, pose.rotation, max_dist=self.p.lesson_lane_m, max_heading_err=math.radians(50), k=4)
        hit = None
        if self._rules is not None:
            for h in hits:
                if int(self._rules.sem_id[h.lane]) >= 0:
                    hit = h
                    break
        hit = hit or (hits[0] if hits else None)
        if hit is None:
            return
        key = lane_key(self.lm, hit.lane)
        rec = self.toll_lanes.setdefault(key, {"n": 0, "back_m": None})
        rec["n"] += 1
        if self._gate_s is not None:
            done = float(self.lm.seg_len[self.lm.lane_first[hit.lane]:hit.segment].sum() + hit.t * self.lm.seg_len[hit.segment])
            back = float(self._gate_s(hit.lane, int(self.lm.lane_item[hit.lane]))) - (done + self._front_bumper)
            if self.p.toll_back_min_m <= back <= self.p.toll_back_max_m:
                rec["back_m"] = back if rec["back_m"] is None else 0.5 * (rec["back_m"] + back)
        self._dirty = True
        where = f", {rec['back_m']:.1f} m before the gate" if rec["back_m"] is not None else ""
        self._log(f"you paid at toll booth {key}{where}: it prefers that one now", lane=key)

    # ------------------------------------------------------------- end of a drive
    def end_session(self) -> dict | None:
        s, p = self.s, self.p
        self.end_lesson()
        card = None
        if s.engaged_game_km >= p.min_session_km:
            per = 100.0 / s.engaged_game_km
            card = {"t": round(self.clock()), "game_km": round(s.engaged_game_km, 1),
                    "takeovers_100km": round(s.takeovers * per, 2), "faults_100km": round(s.faults * per, 2),
                    "fines": s.fines, "lateral_rms_m": round(math.sqrt(s.lat_sq / max(s.n, 1)), 3),
                    "curve_rms_m": round(math.sqrt(s.curve_sq / max(s.n_curve, 1)), 3), "curves": s.curves,
                    "overshoots": s.overshoots, "gps_jumps": s.gps_jumps, "missed_turns": s.misses}
            self.scorecards.append(card)
            self.snapshots.append({"t": card["t"], "values": json.loads(json.dumps(self.values))})
            self._log("drive finished: " + ", ".join(f"{k} {v}" for k, v in card.items() if k != "t"), card=card)
            self._maybe_roll_back()
            self._dirty = True
        if self._dirty:
            self.save()
        self.s = _Session(started=self.clock())
        return card

    @staticmethod
    def _badness(card: dict) -> float:
        fines = sum(card.get("fines", {}).values()) * 100.0 / max(card["game_km"], 1.0)
        return card["takeovers_100km"] + 2.0 * card["faults_100km"] + 3.0 * fines

    def _maybe_roll_back(self) -> None:
        n, cards = self.p.rollback_sessions, self.scorecards
        if len(cards) < 2 * n or len(self.snapshots) < n + 1:
            return
        recent = sum(self._badness(c) for c in cards[-n:]) / n
        before = sum(self._badness(c) for c in cards[-2 * n:-n]) / n
        if recent > self.p.worse_factor * max(before, 1.0):
            self.values = json.loads(json.dumps(self.snapshots[-n - 1]["values"]))
            self._write_sinks()
            self._log(f"rolled back: the last {n} drives scored {recent:.1f}, the {n} before {before:.1f}: "
                      "driving values back to what they were then")
