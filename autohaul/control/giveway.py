"""Junction rules while driving: give way, stop signs, and when the way is clear.

Every junction path ("curve") on the path ahead has conflicts: the other paths of the same junction that cross
it or merge into the same lane (autohaul.world.junctions). The game's map gives each path a priority; where
two meet, the lower priority gives way. AutoHaul also gives way at equal priority.

  give way      AI vehicles on the other path, or on the lanes leading to it (up to `look_back_m`), are
                timed to the conflict zone (where the two paths come within a vehicle width or so). The way is
                clear when each one gets there only after the truck has cleared it plus `yield_margin_s`.
                Vehicles standing still farther out are waiting themselves. Otherwise the truck stops at the
                junction entry (the "line"), or, already inside the junction (a left turn across oncoming
                traffic), short of the conflict zone.
  right of way  the truck still stops for a vehicle on a conflicting path that is too fast to stop at its own
                line (or already past it) when they would meet: not everybody gives way.
  stop sign     ("stop" traffic rule) a full stop at the line first, then as give way.
  slow approach the truck comes to a give-way line at `approach_kmh` at most, so it can still stop, and
                drives through intersections at `inside_kmh` at most.
  traffic light with a light reader (`lights`, autohaul/vision/lights.py): stop at the line on red and amber
                (amber only when the truck can still stop comfortably), go on green. On green, paths of the
                junction on another light phase are treated as stopped: the truck has right of way over them.
                A light it can't see: approach slowly, stop at the line, hand back after `light_unseen_s`. If it
                can see where the lamps would be and none is lit all that time (a signal only for the AI, or
                lights off), it gives way to everyone instead, as at an unsigned junction.
                The stop point is pulled back so at least one head of the light stays in view from the cab.
  toll gate     (`tolls` on): slow down, stop the front bumper `toll_back_m` before the booth's semaphore, press
                `activate` (pay), and wait for the game's toll-paid event (`toll_paid`); then `toll_open_s` for the
                barrier, and through at `toll_pass_kmh`. Three presses without payment: hand back.
  barriers, border checks, weigh stations, and lights without a reader: `controls_ahead` tells the autopilot to
                hand back before them.
Once the truck can no longer stop comfortably before the line or the conflict point, it is committed; after
that only following distance (ACC) can stop it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from autohaul.control.speed import _boxes
from autohaul.world.graph import LaneGraph
from autohaul.world.junctions import Conflict, Junctions
from autohaul.world.lanemap import KIND_PREFAB, LaneMap, quat_forward_rotation
from autohaul.world.rules import JunctionRules


@dataclass
class GiveWayParams:
    look_back_m: float = 150.0          # follow the other path's lanes back this far for approaching vehicles
    yield_margin_s: float = 3.0         # a vehicle with right of way must arrive this long after we've cleared
    priority_margin_s: float = 1.0
    occupied_m: float = 3.0             # a vehicle this close to the conflict point is in the way (at least)
    clear_m: float = 3.0                # we've cleared the conflict point when our rear is this far past it
    other_accel: float = 1.5            # m/s^2 assumed for others speeding up or pulling away
    other_vmax: float = 14.0            # m/s (50 km/h) others may speed up to
    stopped_ignore_m: float = 15.0      # vehicles standing still farther out than this are waiting themselves
    other_stop_decel: float = 3.0       # m/s^2: a vehicle that would need harder braking to stop at its give-way
                                        # line isn't going to give way
    own_accel: float = 0.5              # m/s^2, the truck pulling away
    junction_speed: float = 20 / 3.6    # m/s, the truck's speed through the junction (for its clearing time)
    approach_kmh: float = 15.0          # speed at a give-way line, so the truck can still stop
    inside_kmh: float = 20.0            # top speed inside an intersection (tight, overlapping junction curves)
    inside_lat_acc: float = 1.4         # m/s^2 in intersection turns: the wheel (relative steering) turns slowly,
                                        # and a town turn must be wound in and out within a few metres
    max_decel: float = 3.5              # m/s^2, the hardest braking a give-way stop may use
    line_back_m: float = 1.0            # stop the front bumper this far before the junction entry
    overshoot_m: float = 1.0            # rolling up to this far past a wait point still stops (both have margin)
    inside_back_m: float = 0.5          # ... or this far before a conflict zone inside the junction
    stop_full_s: float = 1.0            # stop sign: stand still at the line this long first
    max_wait_s: float = 45.0            # waiting longer than this: hand back
    match_m: float = 2.5                # AI vehicles are matched to lanes within this distance
    traffic_radius_m: float = 250.0     # only vehicles this close to the truck are considered
    amber_decel: float = 3.0            # m/s^2: stop for amber only if this is enough, else go through
    light_unseen_s: float = 6.0         # standing at a light it can't see this long: hand back
    light_max_wait_s: float = 150.0     # a red light can take a while
    toll_approach_kmh: float = 12.0     # into a toll booth lane
    toll_back_m: float = 2.0            # stop the front bumper this far before the booth's semaphore
    toll_settle_s: float = 0.8          # stand still this long before paying
    toll_press_every_s: float = 3.0     # press activate, then wait this long for the payment
    toll_presses: int = 3
    toll_open_s: float = 2.5            # after paying: the barrier takes this long to open
    toll_pass_kmh: float = 10.0         # through the gate


class LightReader(Protocol):
    def state(self, lane: int, item: int) -> str | None:
        """'red', 'amber', 'green', or None (can't see it)."""

    def stop_back_m(self, lane: int, item: int) -> float:
        """Metres before the line to stop so a head of the light stays in view."""

    def dark(self, lane: int, item: int) -> bool:
        """It can see where the lamps would be and none is lit."""


@dataclass
class GiveWayResult:
    stop_at: float | None = None        # path arc length where the front bumper must stop
    points: list = field(default_factory=list)   # (arc length, speed, kind) constraints for the speed planner
    note: str = ""                      # what the junction logic is doing, for the status line
    handback: str | None = None
    press: list = field(default_factory=list)    # game buttons to press now (paying at a toll gate)


@dataclass
class _Part:
    vid: int
    lane: int
    progress: float     # m along the lane to the part's centre
    speed: float
    length: float


def time_to(d: float, v: float, a: float, v_max: float) -> float:
    """Seconds to cover d metres from speed v, speeding up at a (m/s^2) to v_max."""
    if d <= 0:
        return 0.0
    v = max(v, 0.0)
    v_max = max(v_max, v, 0.1)
    if a <= 0 or v >= v_max:
        return d / max(v, 0.1)
    d_acc = (v_max * v_max - v * v) / (2 * a)
    if d <= d_acc:
        return (-v + math.sqrt(v * v + 2 * a * d)) / a
    return (v_max - v) / a + (d - d_acc) / v_max


class GiveWay:
    def __init__(self, lm: LaneMap, graph: LaneGraph, junctions: Junctions, rules: JunctionRules,
                 params: GiveWayParams | None = None):
        self.lm, self.g, self.j, self.rules = lm, graph, junctions, rules
        self.p = params or GiveWayParams()
        self._approach_cache: dict[tuple[int, int], dict[int, float]] = {}
        self._stopped: dict[int, float] = {}      # junction -> seconds stood still at its stop line
        self._wait = 0.0
        self.lights: LightReader | None = None
        self.tolls = False                         # handle toll gates (pay with activate) instead of handing back
        self._toll: dict[int, dict] = {}           # junction -> toll gate progress
        self._dark: set[int] = set()               # junctions whose light shows no lamp: give way to everyone
        self._paid = False                         # the game reported a toll payment since the last gate

    def toll_paid(self) -> None:
        """The game's `player.tollgate.paid` event."""
        self._paid = True

    def reset(self) -> None:
        self._stopped.clear()
        self._wait = 0.0
        self._toll.clear()
        self._paid = False
        self._dark.clear()

    # ------------------------------------------------------------- map
    def approach(self, lane: int, s_conf: float) -> dict[int, float]:
        """Lanes leading to point `s_conf` of `lane`, each with an offset: a vehicle at `progress` metres along
        lane L is offset[L] + length(L) - progress metres from the point."""
        key = (lane, int(round(s_conf * 10)))
        out = self._approach_cache.get(key)
        if out is not None:
            return out
        length = self.g.length
        out = {lane: s_conf - float(length[lane])}
        frontier = [(lane, s_conf)]                 # (lane, distance from its start to the point)
        while frontier:
            ln, d = frontier.pop()
            if d > self.p.look_back_m:
                continue
            for pre in self.g.predecessors(ln).tolist():
                if pre not in out:
                    out[pre] = d
                    frontier.append((pre, d + float(length[pre])))
        self._approach_cache[key] = out
        return out

    def _section(self, path_lanes: list[int], s_start: list[float], horizon_end: float):
        """Junction lanes on the path up to `horizon_end`, as (index on the path, lane, junction uid)."""
        lm = self.lm
        for k, lane in enumerate(path_lanes):
            if s_start[k] > horizon_end:
                break
            if lm.lane_kind[lane] != KIND_PREFAB:
                continue
            item = int(lm.lane_item[lane])
            # a toll plaza's lanes only fan out (no crossing paths), and a lone light or gate may control a plain
            # road piece: those count too (2026-10-29: AutoHaul drove into a toll booth lane at 25 km/h)
            if self.j.is_intersection(item) or int(self.rules.sem_id[lane]) >= 0 or self.rules.toll_plaza(item):
                yield k, lane, item

    def light(self, lane: int, item: int) -> bool:
        """A visible traffic light controls the path (virtual lights only steer the AI: plain give way)."""
        ctrl = self.rules.control(lane, item)
        return bool(ctrl) and ctrl.startswith("traffic light")

    def controls_ahead(self, path_lanes: list[int], s_start: list[float], s_front: float,
                       horizon: float) -> str | None:
        """A barrier, or a traffic light without a light reader, on the path within `horizon`: why to hand
        back, or None."""
        for k, lane, item in self._section(path_lanes, s_start, s_front + horizon):
            ctrl = self.rules.control(lane, item)
            if (ctrl == "virtual traffic light" or (self.lights is not None and self.light(lane, item))
                    or (self.tolls and ctrl == "toll gate")):
                continue
            if ctrl:
                d = s_start[k] - s_front
                return f"{ctrl} {'here' if d <= 0 else f'in {d:.0f} m'} (not handled yet): take over"
        return None

    def _gate_s(self, lane: int, item: int) -> float:
        """Metres along `lane` to the booth: the controlling semaphore nearest the lane, projected onto it."""
        lm = self.lm
        heads = self.rules.semaphores(lane, item)
        f, la = lm.lane_first[lane], lm.lane_last[lane]
        seg = lm.seg[f:la + 1]
        if not len(heads):
            return float(lm.seg_len[f:la + 1].sum())
        a, b = seg[:, 0:2], seg[:, 2:4]
        d = b - a
        l2 = np.maximum((d * d).sum(axis=1), 1e-9)
        best = None
        for h in heads:
            q = self.rules.sem_xyz[h, 0:2]
            t = np.clip(((q - a) * d).sum(axis=1) / l2, 0, 1)
            dist = np.hypot(*(a + d * t[:, None] - q).T)
            i = int(np.argmin(dist))
            if best is None or dist[i] < best[0]:
                best = (dist[i], float(lm.seg_len[f:f + i].sum() + t[i] * lm.seg_len[f + i]))
        return best[1]

    def _toll_gate(self, lane: int, item: int, s_lane: float, s_front: float, v: float, dt: float,
                   out: GiveWayResult, blocked: list) -> None:
        p = self.p
        gs = self._toll.setdefault(item, {"phase": "approach", "stood": 0.0, "since": 1e9, "presses": 0, "t": 0.0})
        stop = s_lane + self._gate_s(lane, item) - p.toll_back_m
        if gs["phase"] in ("approach", "paying"):
            out.points.append((stop - 40.0, p.toll_approach_kmh / 3.6, "toll gate"))
            if self._paid and gs["presses"]:
                self._paid = False
                gs["phase"], gs["t"] = "paid", 0.0
            else:
                blocked.append((max(stop, s_front), "toll gate: paying" if gs["presses"] else "toll gate: stopping at the booth"))
                if v < 0.3 and stop - s_front < 1.5:
                    gs["stood"] += dt
                    gs["since"] += dt
                    if gs["stood"] > p.toll_settle_s and gs["since"] > p.toll_press_every_s:
                        if gs["presses"] >= p.toll_presses:
                            out.handback = (f"toll gate: no payment after pressing activate {gs['presses']} times: "
                                            "pay yourself, then press / to go on")
                        else:
                            out.press.append("activate")
                            gs["presses"] += 1
                            gs["since"] = 0.0
                            gs["phase"] = "paying"
                return
        if gs["phase"] == "paid":
            gs["t"] += dt
            if gs["t"] < p.toll_open_s:
                blocked.append((max(stop, s_front), "toll gate: paid, waiting for the barrier"))
                return
            gs["phase"] = "through"
        out.points.append((max(s_front, stop - 5.0), p.toll_pass_kmh / 3.6, "toll gate"))
        out.points.append((stop + 15.0, p.toll_pass_kmh / 3.6, "toll gate"))

    # ------------------------------------------------------------- traffic
    def _locate(self, traffic: list[dict], x: float, y: float) -> list[_Part]:
        lm, p = self.lm, self.p
        parts = []
        for v in traffic:
            for bx, bz, qw, qx, qy, qz, length, _width in _boxes(v):
                if math.hypot(bx - x, bz - y) > p.traffic_radius_m:
                    continue
                rot = quat_forward_rotation(qw, qx, qy, qz)
                for h in lm.nearest(bx, bz, rot, max_dist=p.match_m, k=3):
                    first = lm.lane_first[h.lane]
                    prog = float(lm.seg_len[first:h.segment].sum() + h.t * lm.seg_len[h.segment])
                    parts.append(_Part(int(v["id"]), h.lane, prog, float(v["speed"]), float(length)))
        return parts

    # ------------------------------------------------------------- step
    def step(self, path_lanes: list[int], s_start: list[float], s_front: float, speed: float,
             traffic: list[dict], own_len: float, truck_xy: tuple[float, float], dt: float,
             horizon: float) -> GiveWayResult:
        """`s_start[k]`: path arc length where path_lanes[k] starts; `s_front`: our front bumper's arc length;
        `own_len`: front bumper to rear (with the trailer)."""
        p, lm, out = self.p, self.lm, GiveWayResult()
        v = max(speed, 0.0)
        on_path = set(path_lanes)
        section = list(self._section(path_lanes, s_start, s_front + horizon))
        if not section:
            self._wait = 0.0
            return out
        parts = self._locate(traffic or [], *truck_xy) if traffic else []
        lines: dict[int, float] = {}
        blocked: list[tuple[float, str]] = []
        committed: list[str] = []
        yielding: set[int] = set()
        evaluated = False
        unseen = False
        green: dict[int, int] = {}                  # junction -> our light's semaphore id, while it's green
        for item in [it for it in self._stopped if it not in {sec[2] for sec in section}]:
            del self._stopped[item]                 # junctions left behind
        for item in [it for it in self._toll if it not in {sec[2] for sec in section}]:
            del self._toll[item]
        self._dark &= {sec[2] for sec in section}
        plazas = {it for _, _, it in section if self.tolls and self.rules.toll_plaza(it)}
        for k, lane, item in section:
            line = lines.setdefault(item, s_start[k])
            line_stop = line - p.line_back_m
            end = s_start[k + 1] if k + 1 < len(s_start) else s_start[k] + float(self.g.length[lane])
            if item not in plazas:                  # a toll plaza's fanning lanes: the booth sets the speed
                for at in np.arange(max(s_start[k], s_front), end + 0.1, 4.0):
                    out.points.append((float(at), p.inside_kmh / 3.6, "intersection"))
            mine = self.rules.priority(lane)
            if self.tolls and self.rules.control(lane, item) == "toll gate":
                self._toll_gate(lane, item, s_start[k], s_front, v, dt, out, blocked)
            dark = item in self._dark
            if self.lights is not None and self.light(lane, item) and dark:
                mine = -1                           # no light to obey: give way to every conflicting path
            elif self.lights is not None and self.light(lane, item) and line_stop <= s_front - p.overshoot_m:
                green[item] = int(self.rules.sem_id[lane])   # inside the junction: entered on green (or amber)
            elif self.lights is not None and self.light(lane, item):
                state = self.lights.state(lane, item)
                stop = line_stop - self.lights.stop_back_m(lane, item)
                if stop < s_front - p.overshoot_m:
                    # past the viewing point: standing, stay (closer, the lamps go out of sight above the cab);
                    # moving, the line itself
                    stop = s_front if v < 0.5 else line_stop
                room = stop - s_front
                need = v * v / (2 * max(room, 0.01)) if v > 0.5 else 0.0
                if state == "green":
                    green[item] = int(self.rules.sem_id[lane])
                elif state == "amber" and need > p.amber_decel:
                    committed.append("committed (amber light: too close to stop)")
                    green[item] = int(self.rules.sem_id[lane])
                elif state in ("red", "amber"):
                    if need > p.max_decel:
                        committed.append(f"committed ({state} light: too close to stop)")
                        green[item] = int(self.rules.sem_id[lane])
                    else:
                        blocked.append((max(stop, s_front), f"{state} light: stopping"))
                else:
                    out.points.append((stop, p.approach_kmh / 3.6, "traffic light"))
                    if need <= p.max_decel:
                        unseen = True
                        if (v < 0.3 and stop - s_front < 3.0 and self._wait > p.light_unseen_s - 0.5
                                and getattr(self.lights, "dark", lambda *_: False)(lane, item)):
                            self._dark.add(item)    # stood here, looked: no lamp lit. Not a light to obey
                        else:
                            blocked.append((max(stop, s_front), "traffic light: can't see it yet, stopping"))
            if self.rules.traffic_rule(lane) == "stop" and line_stop > s_front - 1.0:
                stood = self._stopped.get(item, 0.0)
                if v < 0.3 and line_stop - s_front < 3.0:
                    stood += dt
                    self._stopped[item] = stood
                if stood < p.stop_full_s:
                    blocked.append((line_stop, "stop sign: stopping at the line"))
            for c in self.j.conflicts(lane):
                if c.other in on_path:
                    continue
                s_c = s_start[k] + c.s_mine
                if s_c + own_len + p.clear_m < s_front or s_c - s_front > horizon:
                    continue                        # cleared already, or too far ahead to matter yet
                evaluated = True
                give = self.rules.priority(c.other) >= mine
                other_sid = int(self.rules.sem_id[c.other])
                if item in green and other_sid >= 0 and other_sid != green[item]:
                    give = False                    # its light is red while ours is green
                if give and line_stop > s_front - 0.3 and item not in yielding:
                    yielding.add(item)
                    out.points.append((line_stop, p.approach_kmh / 3.6, "give way"))
                why = self._threat(c, give, s_c - s_front, own_len, v, parts, on_path, item)
                if why is None:
                    continue
                # where to wait: at the line, or (already past it) short of the conflict zone (where the two
                # paths come within a vehicle width or so of each other)
                zone = s_start[k] + c.zone_mine - p.inside_back_m
                wait = min(line_stop, zone) if line_stop > s_front - 0.3 else zone
                room = wait - s_front
                if room < -p.overshoot_m or (v > 0.5 and v * v / (2 * max(room, 0.01)) > p.max_decel):
                    committed.append(f"committed ({why})")
                    continue                        # can't stop before it any more: go on through
                blocked.append((max(wait, s_front), why))      # rolled a little past it: stop right here
        if blocked:
            wait, why = min(blocked)
            out.stop_at = wait
            out.note = why
            self._wait = self._wait + dt if v < 0.3 else 0.0
            limit = (p.light_unseen_s if unseen else p.light_max_wait_s if "light" in why
                     else 1e9 if why.startswith("toll") else p.max_wait_s)
            if self._wait > limit:
                out.handback = f"waited {self._wait:.0f} s at the junction ({why}): take over"
        else:
            self._wait = 0.0
            out.note = (committed[0] if committed else "give way: clear" if yielding
                        else "right of way" if evaluated else "")
        return out

    def _threat(self, c: Conflict, give: bool, d_conf: float, own_len: float, v: float, parts: list[_Part],
                on_path: set[int], item: int) -> str | None:
        """Why the conflict point `d_conf` metres ahead of our front bumper isn't clear, or None."""
        p = self.p
        lead_in = max(p.occupied_m, c.s_mine - c.zone_mine)       # the zone starts this far before the point
        their_in = max(p.occupied_m, c.s_other - c.zone_other)
        t_in = time_to(d_conf - lead_in, v, p.own_accel, max(v, p.junction_speed))
        t_out = time_to(d_conf + own_len + p.clear_m, v, p.own_accel, max(v, p.junction_speed))
        app = self.approach(c.other, c.s_other)
        length = self.g.length
        what = "crossing" if c.kind == "cross" else "merging"
        for part in parts:
            off = app.get(part.lane)
            if off is None or part.lane in on_path:
                continue
            dist = off + float(length[part.lane]) - part.progress
            front, rear = dist - part.length / 2, dist + part.length / 2
            if rear < -p.clear_m:
                continue                            # past the conflict point
            if front <= their_in:
                return f"vehicle {part.vid} in the way on a {what} path"
            if give:
                if part.speed < 0.5 and front > p.stopped_ignore_m:
                    continue
                t_arr = time_to(front - their_in, part.speed, p.other_accel, max(part.speed, p.other_vmax))
                if t_arr < t_out + p.yield_margin_s:
                    return f"giving way to vehicle {part.vid} on a {what} path ({front:.0f} m out)"
            else:
                to_line = front - c.s_other             # its front to the start of its junction path
                if part.speed < 1.5 or (to_line > 0 and part.speed ** 2 / (2 * to_line) < p.other_stop_decel):
                    continue                        # it should give way to us, and it still can
                t_arr = (front - their_in) / part.speed
                t_leave = t_arr + (part.length + their_in + p.occupied_m) / part.speed
                if t_arr < t_out + p.priority_margin_s and t_leave > t_in - p.priority_margin_s:
                    return f"vehicle {part.vid} is crossing without giving way"
        return None
