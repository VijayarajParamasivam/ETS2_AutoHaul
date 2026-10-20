"""Autopilot: lane keeping and speed / following-distance control on the lane map (Phase 2).

One `step()` per telemetry frame: localize on the lane map, build the path ahead, then
  lateral       lane-keeping law -> target wheel angle -> steering servo -> steering axis
  longitudinal  speed planner (limit, curves, lane end) + ACC (lead vehicle) -> pedal controller
The same class drives the simulator (tests, tools/sim_drive.py) and the game (tools/drive.py).
It never decides alone that driving is safe: tools/drive.py runs SafetyMonitor around it, and any
`fault` here makes the drive loop hand control back to the driver.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np

from autohaul.control.lanechange import LaneChange, LaneChangeParams, gap_check, offset_profile
from autohaul.control.path import PathRef
from autohaul.control.speed import KMH, Lead, PedalController, SpeedParams, SpeedPlanner, acc_accel, find_lead
from autohaul.control.steering import AbsoluteServo, LateralController, LateralParams, PulseModulator, RelativeServo
from autohaul.control.vehicle import TruckGeometry, measured_steer_left, pose_from_frame
from autohaul.safety.supervisor import Command
from autohaul.world.lanemap import KIND_PREFAB, LaneMap
from autohaul.world.junctions import Junctions
from autohaul.world.localizer import LaneTracker
from autohaul.world.route import Route, Router


@dataclass
class AutopilotParams:
    path_ahead_m: float = 260.0
    lane_lost_s: float = 0.3
    max_lateral_m: float = 1.6
    min_confidence: float = 0.3
    low_confidence_s: float = 1.0
    engage_max_lateral_m: float = 1.0
    engage_max_heading_deg: float = 12.0
    engage_min_confidence: float = 0.5
    max_steer_units: float = 0.9
    lane_end_margin_m: float = 15.0
    min_turn_radius_m: float = 40.0       # Phase 2 steers through nothing sharper (junctions are Phase 3)
    sharp_turn_lookahead_s: float = 6.0
    sharp_turn_lookahead_m: float = 60.0
    pedal_miss_s: float = 0.5             # fault when the game ignores AutoHaul's pedals this long
    hazard_radius_m: float = 60.0         # toll plaza icons sit within ~50 m of the booths
    hazard_lookahead_s: float = 8.0
    hazard_lookahead_m: float = 120.0
    # Phase 3 routing
    route_min_turn_radius_m: float = 10.0     # planned turns on the route may be this tight
    intersection_lookahead_s: float = 8.0     # hand back this long before a junction with crossing paths
    intersection_lookahead_m: float = 100.0
    lane_change_handback_s: float = 8.0       # ... or before the point where the route needs a lane change
    lane_change_handback_m: float = 150.0
    arrive_m: float = 60.0                    # hand back this close to the destination point
    reroute_interval_s: float = 1.0


@dataclass
class Status:
    fault: str | None = None
    lane: int | None = None
    lane_desc: str = ""
    confidence: float = 0.0
    lateral: float = 0.0              # tracker: origin's offset from the lane centre (+ right)
    ctrl_lateral: float = 0.0         # controller: predicted rear-axle offset
    heading_err_deg: float = 0.0
    path_m: float = 0.0
    steer_target: float = 0.0         # effective steering units (+ left)
    steer_eff: float = 0.0
    steer_cmd: float = 0.0
    servo_gain: float = 0.0
    kappa_ff: float = 0.0
    curve_ratio: float = 1.0
    speed_kmh: float = 0.0
    target_kmh: float = 0.0
    target_reason: str = ""
    limit_kmh: float = 0.0
    lead: Lead | None = None
    a_request: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    emergency: bool = False
    route_km: float | None = None     # remaining driving distance on the route (world km; the GPS shows ~x19)
    on_route: bool = False
    route_note: str = ""
    indicator: str | None = None      # "left" / "right" while AutoHaul signals a lane change
    lane_change: str = ""
    timings_ms: dict = field(default_factory=dict)


class Autopilot:
    def __init__(self, lm: LaneMap, geom: TruckGeometry, steering_mode: str = "relative", *, lateral: bool = True,
                 longitudinal: bool = True, params: AutopilotParams | None = None,
                 lat_params: LateralParams | None = None, speed_params: SpeedParams | None = None,
                 relative_gain: float = 0.42, relative_deadzone: float = 0.0, hazards: np.ndarray | None = None,
                 pulse_steering: bool = False, check_pedals: bool = True,
                 router: Router | None = None, junctions: Junctions | None = None,
                 lane_change: LaneChangeParams | None = None, auto_lane_change: bool = True):
        """`hazards`: (n, 2) map points AutoHaul must not drive through itself, e.g. toll plazas.
        `pulse_steering`: relative steering as single-frame pulses (PulseModulator); off by default (the pedal
        loss it was meant to fix was the game taking one input event per frame, fixed in the bridge DLL).
        `check_pedals`: fault when the game doesn't take AutoHaul's pedals (off in shadow mode).
        `router` + `set_destination()`: follow a planned route; `junctions`: hand back before intersections."""
        self.lm = lm
        self.hazards = np.empty((0, 2)) if hazards is None else np.asarray(hazards, dtype=float).reshape(-1, 2)
        self.geom = geom
        self.lateral_on = lateral
        self.longitudinal_on = longitudinal
        self.p = params or AutopilotParams()
        self.sp = speed_params or SpeedParams()
        self.tracker = LaneTracker(lm)
        self.lat = LateralController(geom, lat_params)
        if steering_mode == "relative":
            self.servo = RelativeServo(gain=relative_gain, deadzone=relative_deadzone,
                                       max_cmd=PulseModulator.MAX_AVERAGE if pulse_steering else 1.0)
        elif steering_mode == "absolute":
            self.servo = AbsoluteServo()
        else:
            raise ValueError(f"unknown steering mode {steering_mode!r}")
        self.planner = SpeedPlanner(self.sp)
        self.pedals = PedalController(self.sp)
        self._lost = 0.0
        self._low_conf = 0.0
        self.pulse = PulseModulator() if pulse_steering and steering_mode == "relative" else None
        self.check_pedals = check_pedals
        self._sent = Command()
        self._pedal_miss = 0.0
        self.path: PathRef | None = None
        self.router = router
        self.junctions = junctions
        self.destination: tuple[float, float] | None = None
        self.route: Route | None = None
        self._last_reroute = -1e9
        self._sim_clock = 0.0
        self.lcp = lane_change or LaneChangeParams()
        self.auto_lane_change = auto_lane_change
        self.lc: LaneChange | None = None
        self._odo = 0.0
        self._trailer = False

    def set_destination(self, x: float, y: float) -> None:
        self.destination = (x, y)
        self.route = None

    def _progress_m(self, st) -> float:
        lm = self.lm
        return float(lm.seg_len[lm.lane_first[st.lane]:st.segment].sum() + st.t * lm.seg_len[st.segment])

    def _plan(self, st) -> bool:
        if self.router is None or self.destination is None:
            return False
        self._last_reroute = self._sim_clock
        self.route = self.router.route(st.lane, self._progress_m(st), self.destination)
        return self.route is not None

    def _choose(self, lane: int, succ: list[tuple[int, float]]) -> int:
        nxt = self.route.next_after(lane) if self.route else None
        for cand, _turn in succ:
            if cand == nxt:
                return cand
        return succ[0][0]

    # ------------------------------------------------------------- engage
    def engage(self, frame) -> str | None:
        """Reset the controllers on the current state. Returns why engaging is refused, or None."""
        self.tracker.reset()
        self.lc = None
        self.lat.reset()
        self.servo.reset()
        if self.pulse:
            self.pulse.reset()
        self._sent = Command()
        self._pedal_miss = 0.0
        self.pedals.reset(max(frame.speed, 0.0))
        self._lost = self._low_conf = 0.0
        if frame.speed < -0.2:
            return "truck is reversing"
        if self.longitudinal_on and frame.parking_brake:
            return "parking brake is on"
        pose = pose_from_frame(frame)
        st = self.tracker.update(pose.x, pose.y, pose.rotation)
        if self.lateral_on or self.longitudinal_on:
            if st is None:
                return "not on a mapped lane (no lane within 4 m in the driving direction)"
            if st.confidence < self.p.engage_min_confidence:
                return f"lane match uncertain (confidence {st.confidence:.2f})"
        if self.lateral_on:
            if abs(st.lateral) > self.p.engage_max_lateral_m:
                return f"{abs(st.lateral):.1f} m off the lane centre; centre the truck first"
            if abs(math.degrees(st.heading_err)) > self.p.engage_max_heading_deg:
                return f"truck is {abs(math.degrees(st.heading_err)):.0f} deg off the lane direction"
        if st is not None and self.destination is not None:
            if not self._plan(st):
                return "no route from this lane to the destination"
        if st is not None and len(self.hazards):
            try:
                path = PathRef(self.tracker.path_ahead(self.p.path_ahead_m, choose=self._choose if self.route else None))
            except ValueError:
                return "no path ahead on the lane map"
            hz = self.hazard_ahead(path, path.project(pose.x, pose.y).s, max(frame.speed, 0.0))
            if hz:
                return hz.replace(": take over", "")
        if st is not None and self.junctions is not None:
            try:
                path = PathRef(self.tracker.path_ahead(self.p.path_ahead_m, choose=self._choose if self.route else None))
            except ValueError:
                return "no path ahead on the lane map"
            jn = self._junction_ahead(path, path.project(pose.x, pose.y).s, max(frame.speed, 0.0))
            if jn:
                return jn.replace(": take over", "")
        return None

    def _junction_ahead(self, path: PathRef, s_truck: float, speed: float) -> str | None:
        """An intersection (junction with crossing paths) on the path within the warning distance."""
        if self.junctions is None:
            return None
        horizon = max(self.p.intersection_lookahead_m, self.p.intersection_lookahead_s * speed)
        for lane in self.tracker.path_lanes:
            if self.lm.lane_kind[lane] != KIND_PREFAB or not self.junctions.is_intersection(int(self.lm.lane_item[lane])):
                continue
            first = self.lm.seg[self.lm.lane_first[lane], 0:2]
            d = path.project(float(first[0]), float(first[1])).s - s_truck
            if d <= horizon:
                where = "here" if d <= 0 else f"in {d:.0f} m"
                return f"intersection {where} (traffic lights / give way not handled yet): take over"
        return None

    def _route_checks(self, st, speed: float, out: Status) -> str | None:
        """Remaining distance, arrival, and lane changes the route needs (not automated yet)."""
        rt, lm = self.route, self.lm
        to_end = float(self.router.g.length[st.lane]) - self._progress_m(st)
        rem = rt.remaining_m(st.lane, to_end)
        if rem is not None:
            out.route_km = rem / 1000
            if rem <= self.p.arrive_m:
                return "destination reached: take over"
        horizon = max(self.p.lane_change_handback_m, self.p.lane_change_handback_s * speed)
        i = rt.lane_index[st.lane]
        dist = to_end                               # metres to the end of route lane j (j = i at first)
        for j in range(i, min(i + 40, len(rt.lanes) - 1)):
            if rt.changes[j + 1]:
                cur, nxt = int(lm.lane_sub[rt.lanes[j]]), int(lm.lane_sub[rt.lanes[j + 1]])
                side = "right" if abs(nxt) > abs(cur) else "left"
                out.route_note = f"lane change {side} needed within {dist:.0f} m"
                if dist <= horizon and self.lateral_on and not self.auto_lane_change:
                    return f"the route needs a lane change to the {side} (within {dist:.0f} m): take over"
                break
            if dist > horizon:
                break
            dist += float(self.router.g.length[rt.lanes[j + 1]])
        return None

    def _upcoming_change(self, st, lookahead_m: float = 3000.0) -> tuple[int, float] | None:
        """The route's next lane change: (side +1 right / -1 left, metres until it must be done)."""
        rt = self.route
        i = rt.lane_index.get(st.lane)
        if i is None:
            return None
        dist = float(self.router.g.length[st.lane]) - self._progress_m(st)
        for j in range(i, len(rt.lanes) - 1):
            if rt.changes[j + 1]:
                cur, nxt = int(self.lm.lane_sub[rt.lanes[j]]), int(self.lm.lane_sub[rt.lanes[j + 1]])
                return (1 if abs(nxt) > abs(cur) else -1), dist
            if dist > lookahead_m:
                return None
            dist += float(self.router.g.length[rt.lanes[j + 1]])
        return None

    def _neighbour(self, lane: int, side: int) -> int | None:
        """The parallel lane of the same road on `side` (+1 right: larger |sub|), if there is one."""
        sub = abs(int(self.lm.lane_sub[lane]))
        for n in self.router.g.neighbours(lane).tolist():
            if (abs(int(self.lm.lane_sub[n])) > sub) == (side > 0):
                return int(n)
        return None

    def _lane_change_step(self, st, pose, traffic, out: Status) -> str | None:
        """Signal, wait for a gap, then move into the lane the route needs (only when AutoHaul steers).
        The change is made as early as possible: into the neighbouring lane of whatever road piece the truck is
        on, then the route is re-planned from there (the planner itself puts changes at the last moment)."""
        lc, p = self.lc, self.lcp
        if lc is None or lc.phase == "signal":
            if not (self.lateral_on and self.auto_lane_change and self.route and self.router):
                return None
            up = self._upcoming_change(st)
            if up is None:
                self.lc = None
                return None
            side, deadline = up
            target = self._neighbour(st.lane, side)
            if lc is None:
                if target is None:
                    return None                 # no lane on that side here yet: wait for one
                lc = self.lc = LaneChange(target, side, self._sim_clock)
            elif target is not None:
                lc.target = target              # the truck moved onto the next road piece while signalling
        out.indicator = lc.side_name
        v = max(pose.speed, 0.0)
        rear = p.trailer_rear_m if self._trailer else p.tractor_rear_m
        if lc.phase == "signal":
            length = max(p.min_len_m, p.len_s * v)
            why = gap_check(pose, lc.side, traffic, self.geom.front_bumper, rear, p)
            signalled = self._sim_clock - lc.started >= p.signal_s
            lc.waiting = why or ("" if signalled else "signalling")
            if deadline < length + p.deadline_margin_m:
                return f"no room left to change lanes to the {lc.side_name} ({why or 'too close to the split'}): take over"
            if not why and signalled and self._neighbour(st.lane, lc.side) is not None:
                hit = [h for h in self.lm.nearest(pose.x, pose.y, pose.rotation, max_dist=8.0, k=8) if h.lane == lc.target]
                if not hit:
                    return "can't find the target lane for the lane change: take over"
                lc.phase, lc.d0, lc.length = "move", self._odo, length
                lc.profile = offset_profile(hit[0].lateral, length)
            out.lane_change = f"{lc.side_name}: {lc.waiting or 'moving'} (needed within {deadline:.0f} m)"
            return None
        d = self._odo - lc.d0
        if d >= lc.length:
            self.lc = None
            out.indicator, out.lane_change = None, "done"
            self._last_reroute = -1e9               # re-plan from the new lane right away
            return None
        out.lane_change = f"{lc.side_name}: moving {d / lc.length * 100:.0f}%"
        return None

    def hazard_ahead(self, path: PathRef, s0: float, speed: float) -> str | None:
        """A toll plaza (or other hazard point) on the path within the warning distance."""
        if not len(self.hazards):
            return None
        horizon = max(self.p.hazard_lookahead_m, self.p.hazard_lookahead_s * speed)
        x0, y0 = path.point(max(s0, 0.0))
        near = self.hazards[np.hypot(self.hazards[:, 0] - x0, self.hazards[:, 1] - y0) < horizon + self.p.hazard_radius_m]
        if not len(near):
            return None
        ss = np.arange(max(s0, 0.0), min(s0 + horizon, path.length) + 1e-9, 5.0)
        pts = np.array([path.point(v) for v in ss])
        d = np.hypot(pts[:, None, 0] - near[None, :, 0], pts[:, None, 1] - near[None, :, 1]).min(axis=1)
        hit = np.flatnonzero(d < self.p.hazard_radius_m)
        if len(hit):
            return f"toll plaza ahead in {ss[hit[0]] - s0:.0f} m: take over"
        return None

    # ------------------------------------------------------------- step
    def step(self, frame, traffic: list[dict] | None, dt: float, clock=None) -> tuple[Command, Status]:
        import time
        clock = clock or time.perf_counter
        t0 = clock()
        out = Status(speed_kmh=frame.speed * 3.6, steer_eff=frame.effective_steering)
        cmd = Command()
        pose = pose_from_frame(frame)
        self._sim_clock += dt
        self._odo += max(frame.speed, 0.0) * dt
        trailers = getattr(frame, "trailers", None)
        self._trailer = bool(trailers is not None and trailers[0].connected)
        prefer = self.tracker.path_lanes
        if self.route and self.tracker.state is not None:
            i = self.route.lane_index.get(self.tracker.state.lane)
            if i is not None:
                prefer = self.route.lanes[i:i + 12]
        moving = self.lc is not None and self.lc.phase == "move"
        st = self.tracker.update(pose.x, pose.y, pose.rotation, prefer=prefer, force=self.lc.target if moving else None)
        t1 = clock()
        if st is None:
            self._lost += dt
            if self._lost > self.p.lane_lost_s:
                out.fault = "lost the lane (no mapped lane under the truck)"
            # hold the wheel (relative mode: command 0 keeps it where it is), coast
            self.pedals.reset(max(frame.speed, 0.0))
            out.timings_ms = {"track": (t1 - t0) * 1e3}
            self._sent = cmd
            return cmd, out
        self._lost = 0.0
        out.lane, out.lane_desc, out.confidence = st.lane, self.lm.describe(st.lane), st.confidence
        out.lateral, out.heading_err_deg = st.lateral, math.degrees(st.heading_err)
        # during a lane change the tracker follows the target lane; measure against the eased line instead
        moving_now = self.lc is not None and self.lc.phase == "move" and self.lc.profile is not None
        off_ref = st.lateral - (self.lc.profile(self._odo - self.lc.d0)[0] if moving_now else 0.0)
        out.lateral = off_ref                       # offset from the line being followed
        self._low_conf = self._low_conf + dt if st.confidence < self.p.min_confidence else 0.0
        if self.destination is not None and self.router is not None:
            if (self.route is None or st.lane not in self.route.lane_index) and                     self._sim_clock - self._last_reroute >= self.p.reroute_interval_s:
                self._plan(st)                      # off the route (or none yet): plan from here
            out.on_route = bool(self.route and st.lane in self.route.lane_index)
        pts = self.tracker.path_ahead(self.p.path_ahead_m, choose=self._choose if self.route else None)
        try:
            path = self.path = PathRef(pts)
        except ValueError:                      # at the very end of a lane with no mapped continuation
            out.fault = "no path ahead on the lane map"
            self._sent = cmd
            return cmd, out
        out.path_m = path.length
        s_truck = path.project(pose.x, pose.y).s
        hz = self.hazard_ahead(path, s_truck, max(frame.speed, 0.0))
        if hz:
            out.fault = hz
        if not out.fault:
            out.fault = self._junction_ahead(path, s_truck, max(frame.speed, 0.0))
        if self.route is not None and out.on_route and not out.fault:
            out.fault = self._route_checks(st, max(frame.speed, 0.0), out)
        if not out.fault and (self.lc is not None or self.route is not None):
            out.fault = self._lane_change_step(st, pose, traffic, out)
        if out.fault and self.lc is not None:
            self.lc = None                          # handing back: AutoHaul stops signalling
        t2 = clock()

        # --- lateral
        rate = (self.servo.gain * self.servo.max_cmd * self.geom.rad_per_unit
                if isinstance(self.servo, RelativeServo) else None)    # fastest the wheel turns, rad/s
        offset = None
        if self.lc is not None and self.lc.phase == "move":
            lc, odo = self.lc, self._odo
            offset = lambda ds: lc.profile(odo - lc.d0 + ds)          # noqa: E731
        lat = self.lat.step(pose, path, dt, measured_steer_left(frame, self.geom), steer_rate=rate, offset=offset)
        out.ctrl_lateral, out.kappa_ff, out.curve_ratio = lat.lateral, lat.kappa_ff, lat.ratio
        target = max(-self.p.max_steer_units, min(self.p.max_steer_units, lat.steer_left / self.geom.rad_per_unit))
        out.steer_target = target
        if self.lateral_on:
            cmd.steer = self.servo.step(target, frame.effective_steering, dt)
            if self.pulse:
                cmd.steer = self.pulse.step(cmd.steer)
            out.steer_cmd = cmd.steer
            out.servo_gain = getattr(self.servo, "gain", 0.0)
            if out.fault:
                pass
            elif abs(off_ref) > self.p.max_lateral_m:
                out.fault = f"{abs(off_ref):.1f} m off the {'lane-change line' if moving_now else 'lane centre'}"
            elif self._low_conf > self.p.low_confidence_s:
                out.fault = f"lane match uncertain for {self._low_conf:.1f} s"
            elif self.servo.fighting:
                out.fault = "the wheel is moving against AutoHaul's steering (someone else steering?)"
            else:
                # Phase 2 keeps lanes; junction turns need the route (Phase 3). Hand back early enough
                # to react when the path ahead turns sharper than a highway ramp.
                s0 = lat.s_rear + self.geom.rear_axle
                horizon = max(self.p.sharp_turn_lookahead_m, self.p.sharp_turn_lookahead_s * max(frame.speed, 0.0))
                ss = np.arange(s0, min(s0 + horizon, path.length), 3.0)
                if len(ss):
                    k = np.abs(np.asarray(path.curvature(ss)))
                    i = int(np.argmax(k))
                    r_min = self.p.route_min_turn_radius_m if out.on_route else self.p.min_turn_radius_m
                    if k[i] > 1.0 / r_min:
                        out.fault = f"sharp turn ahead (radius {1 / k[i]:.0f} m in {ss[i] - s0:.0f} m): take over"
        t3 = clock()

        # --- longitudinal
        if self.longitudinal_on:
            v = max(frame.speed, 0.0)
            s0 = path.project(pose.x, pose.y).s
            limit = self.planner.limit_speed(frame.navigation_speed_limit)
            # a mapped path shorter than requested ends (dead end / gap in the lane graph): stop before it
            stop_at = None
            if path.length < self.p.path_ahead_m - 1:
                stop_at = max(path.length - self.geom.front_bumper - self.p.lane_end_margin_m, s0)
            v_ahead, a_need, d_ahead, kind = self.planner.ahead(
                path, s0, v, stop_at, steer_rate=rate if self.lateral_on else None,   # driver steers: no limit
                wheelbase=self.geom.wheelbase,
                steer_now=-measured_steer_left(frame, self.geom))     # planner: map convention, + = right
            target_v, reason = limit, "limit" if frame.navigation_speed_limit > 0.5 else "no limit known"
            if self.sp.max_speed_kmh * KMH <= target_v:
                reason = "max speed"
            if v_ahead < target_v:
                target_v = v_ahead
                reason = f"curve in {d_ahead:.0f} m" if kind == "curve" else f"mapped lane ends in {d_ahead:.0f} m"
            a_req = self.sp.k_speed * (target_v - v)
            if a_need < -0.15:
                a_req = min(a_req, 1.1 * a_need)
            emergency = False
            sp_lead = self.sp
            if self.lc is not None and self.lc.phase == "move":       # straddling: watch both lanes
                sp_lead = dataclasses.replace(self.sp, lane_half_width=self.sp.lane_half_width + self.lcp.lane_width)
            lead = find_lead(path, s0 + self.geom.front_bumper, traffic, sp_lead)
            if lead is not None:
                a_acc, emergency = acc_accel(lead, v, self.sp)
                if a_acc < a_req:
                    a_req, reason = a_acc, f"following at {lead.gap:.0f} m"
            out.lead, out.emergency = lead, emergency
            out.limit_kmh, out.target_kmh, out.target_reason, out.a_request = limit * 3.6, target_v * 3.6, reason, a_req
            cmd.throttle, cmd.brake = self.pedals.step(a_req, v, dt, emergency, v_cap=target_v)
            out.throttle, out.brake = cmd.throttle, cmd.brake
            # the game must actually take the pedals we sent last frame (it silently ignores them while
            # AutoHaul's steering axis is held non-zero, and controller mixes can cancel them)
            if self.check_pedals:
                sent = max(self._sent.throttle, self._sent.brake)
                seen = frame.input_throttle if self._sent.throttle >= self._sent.brake else frame.input_brake
                self._pedal_miss = self._pedal_miss + dt if sent > 0.2 and seen < 0.5 * sent - 0.05 else 0.0
                if self._pedal_miss > self.p.pedal_miss_s and not out.fault:
                    which = "throttle" if self._sent.throttle >= self._sent.brake else "brake"
                    out.fault = f"the game isn't taking AutoHaul's {which} (sent {sent:.2f}, game shows {seen:.2f})"
            if out.fault and not emergency:
                cmd.throttle = 0.0
        t4 = clock()
        out.timings_ms = {"track": (t1 - t0) * 1e3, "path": (t2 - t1) * 1e3, "lat": (t3 - t2) * 1e3, "long": (t4 - t3) * 1e3}
        self._sent = cmd
        return cmd, out

    def steer_measured(self, frame) -> float:
        return measured_steer_left(frame, self.geom)
