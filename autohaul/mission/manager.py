"""The mission manager: runs the active job on top of the autopilot.

    no job ──(job accepted)──► to trailer ──(near its depot)──► at trailer  (the driver couples: Phase 5)
                                                                     │ (cargo loaded)
           delivered ◄──(job.delivered)── at destination ◄── to destination
    interrupts, from either "to" state:
           to stop ──(stopped at it)──► at stop (refuel / sleep script) ──► back

Every frame `MissionDriver.step` keeps the autopilot's destination on the mission's current target: the next
planned stop (driving up to it and stopping on it), else the job's depot (handing back ~60 m before it, as
the coupling and the parking are still the driver's). Stops are re-planned on the way (StopPlanner) and when
a leg starts. Everything the mission decides goes to the decision log with its reasons.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

from autohaul.drive.autopilot import Autopilot, Status
from autohaul.drive.coupling import Coupling, CouplingParams
from autohaul.mission.actions import ActionParams, ActionStep, QuickPark, Refuel, Sleep
from autohaul.mission.decisions import DecisionLog
from autohaul.mission.economics import EconParams, calibrate
from autohaul.mission.job import Job, JobHistory, clock, duration
from autohaul.mission.planner import Plan, PlanState, StopPlanner
from autohaul.mission.rates import Rates
from autohaul.safety.supervisor import Command
from autohaul.world.bundle import World
from autohaul.world.places import Place
from autohaul.world.services import Services


@dataclass
class MissionParams:
    replan_m: float = 1500.0          # re-plan stops every this many world metres (~28 game km)
    replan_gain_eur: float = 150.0    # ... and switch only to a plan worth this much more (or if the old one fails)
    keep_stop_m: float = 3000.0       # within this of the next stop, keep it (no re-planning)
    fatigue: str = "auto"             # "auto" (the profile's g_fatigue, else detect), "on" or "off"
    quick_park: bool = True           # at the depots: try the game's quick park before handing back
    park_here_m: float = 150.0        # engaged standing this close to the leg's depot: quick park right away
    plan_in_background: bool = False  # plan off the driving loop (the game: a search can take 0.5 s, 11 m blind at
                                      # 80 km/h); the simulator plans in step, so that runs stay repeatable
    plan_every_s: float = 2.0         # (in the background) at most this often
    couple: bool = True               # at the trailer's depot: reverse under the trailer (where the save has it)
    couple_within_m: float = 250.0    # ... when it stands this close (else quick park)


@dataclass
class Target:
    x: float
    y: float
    label: str
    goal: tuple[int, float] | None = None
    stop: bool = False


@dataclass
class MissionStatus:
    state: str = "no job"
    job: str = ""
    target: str = ""
    next_stop: str = ""
    action: str = ""
    note: str = ""
    decision: str = ""


@dataclass
class MissionStep:
    cmd: Command
    status: Status
    mission: MissionStatus
    press: list[str] = field(default_factory=list)       # virtual buttons to pulse this frame
    hold: dict[str, bool] = field(default_factory=dict)  # ... and to hold / release
    fault: str | None = None                             # hand back with this reason
    reverse: bool = False                                # in reverse on purpose (coupling): don't shift to drive


class Mission:
    def __init__(self, world: World, services: Services, decisions: DecisionLog, history: JobHistory,
                 params: MissionParams | None = None, econ: EconParams | None = None, planner_params=None,
                 action_params: ActionParams | None = None):
        self.w, self.services, self.log, self.history = world, services, decisions, history
        self.p = params or MissionParams()
        self.rates = Rates()
        econ, note = calibrate(history.load(), econ or EconParams())
        self.econ_note = note
        self.planner = StopPlanner(world.router, services, self.rates, econ, planner_params)
        self.action_params = action_params or ActionParams()
        self.job: Job | None = None
        self.job_started: int | None = None
        self.state = "no job"
        self.plan: Plan | None = None
        self.leg_after_m = 0.0                # world metres from this leg's end to the destination
        self._since_plan_m = 1e9
        self._metres_at_plan = 0.0
        self.action = None
        self.note = ""
        self.profile_fatigue: bool | None = None       # the game's own setting, when the tool could read it
        self._force_replan = False
        self._plan_gen = 0                     # goes up when the job changes: a plan still being made is then stale
        self._plan_thread: threading.Thread | None = None
        self._plan_result: tuple | None = None
        self._plan_asked = -1e9
        self.trailer_on: bool | None = None            # a trailer is coupled (telemetry); None: not known yet
        self.free_target: Place | None = None          # driven to while there's no job (drive.py --to)

    # ------------------------------------------------------------- job
    def loaded(self) -> bool:
        """On the way to the destination: the job's cargo is loaded AND a trailer is coupled. A freight-market
        job can report its cargo loaded while the trailer still waits at the depot (2026-10-29: AutoHaul set
        off for Duisburg without it)."""
        return bool(self.job is not None and self.job.cargo_loaded and self.trailer_on is not False)

    def leg_state(self) -> str:
        if self.job is not None:
            return "to destination" if self.loaded() else "to trailer"
        return "to place" if self.free_target is not None else "no job"

    def set_free_target(self, place: Place | None) -> None:
        """Drive to `place` (with fuel and rest stops) whenever no job is active."""
        self.free_target = place
        if self.job is None and self.state in ("no job", "to place", "delivered"):
            self.state, self.plan = self.leg_state(), None

    def leg_target(self) -> Place | None:
        if self.job is None:
            return self.free_target
        j = self.job
        if self.loaded():
            return self.w.places.company(j.destination_city, j.destination_company)
        return self.w.places.company(j.source_city, j.source_company)

    def _job_changed(self, job: Job | None, game_time: int) -> None:
        old = self.job
        keep = self.action if isinstance(self.action, (QuickPark, Coupling)) else None   # its success changes the job
        self._plan_gen += 1
        self.job, self.plan, self.action = job, None, keep
        self._since_plan_m = 1e9
        if job is None:
            self.state = self.leg_state()
            return
        if old is None or old.key != job.key:
            self.job_started = game_time
            self.log.add("job", {"cargo": job.cargo, "from": job.source_label, "to": job.destination_label,
                                 "income": job.income, "deadline": clock(job.deadline),
                                 "time_left": duration(job.deadline - game_time), "planned_km": job.planned_km,
                                 "market": job.market, "loaded": job.cargo_loaded, "economics": self.econ_note})
        self.state = self.leg_state()
        self.leg_after_m = self._after_leg_m() if self.state == "to trailer" else 0.0

    def _after_leg_m(self) -> float:
        """World metres from the source depot to the destination (counted into the trailer leg's timing)."""
        j = self.job
        a = self.w.places.company(j.source_city, j.source_company)
        b = self.w.places.company(j.destination_city, j.destination_company)
        if a is None or b is None:
            return 0.0
        starts = self.w.router.goal_lanes(a.x, a.y, 120.0)
        for lane in sorted(starts, key=lambda ln: starts[ln])[:8]:     # some depot lanes are dead ends in the yard
            rt = self.w.router.route(lane, 0.0, (b.x, b.y), max_expand=1_500_000)
            if rt:
                return rt.length_m
        return 0.0

    def update(self, frame, cfg_block, events, fatigue_on: bool | None = None) -> None:
        """Every frame, engaged or not: the job's events, the job, and the learned rates. Events first: a
        delivery arrives together with the job disappearing from the config, and belongs to the old job."""
        for ev in events or ():
            rec = self.history.record_event(self.job, ev, self.job_started)
            if rec is not None:
                kind = rec["event"]
                self.log.add(kind, {k: v for k, v in rec.items() if k != "job"})
                self.state = "delivered" if kind == "delivered" else "no job"
                self.plan = None
                if not isinstance(self.action, (QuickPark, Coupling)):     # they check for this themselves
                    self.action = None
            elif ev.id in ("player.fined", "player.tollgate.paid", "player.use.ferry", "player.use.train"):
                self.log.add("event", {"id": ev.id, "game_time": ev.game_time, **ev.attrs})
        job = Job.from_config(cfg_block)
        changed = (job is None) != (self.job is None) or (
            job is not None and self.job is not None
            and (job.key != self.job.key or job.cargo_loaded != self.job.cargo_loaded))
        if changed:
            delivered = self.state == "delivered"
            self._job_changed(job, int(frame.game_time))
            if delivered and job is None:
                self.state = "delivered"
        trailers = getattr(frame, "trailers", None)
        on = bool(trailers[0].connected) if trailers else None
        if on is not None and on != self.trailer_on:
            self.trailer_on = on
            if self.job is not None and self.state in ("to trailer", "to destination", "at trailer") \
                    and self.state != self.leg_state() and not isinstance(self.action, (QuickPark, Coupling)):
                self.state, self.plan = self.leg_state(), None      # coupled (or dropped) the trailer
                self.leg_after_m = self._after_leg_m() if self.state == "to trailer" else 0.0
        tr = frame.truck_placement
        self.rates.update(tr.x, tr.z, frame.speed, float(frame.fuel), float(frame.game_time),
                          float(frame.next_rest_stop), float(getattr(frame, "fuel_average_consumption", 0.0)),
                          float(getattr(frame, "local_scale", 0.0)))

    def fatigue(self, frame) -> bool:
        if self.p.fatigue in ("on", "off"):
            return self.p.fatigue == "on"
        if self.profile_fatigue is not None:          # the profile's g_fatigue: the timer may run even with it off
            return self.profile_fatigue
        if self.rates.fatigue_on is not None:
            return self.rates.fatigue_on
        return float(frame.next_rest_stop) > 0         # unknown yet: a running timer is taken as fatigue on

    # ------------------------------------------------------------- planning
    def replan_due(self) -> bool:
        if self.state not in ("to trailer", "to destination", "to place", "to stop") or self.action is not None:
            return False
        return self.rates.metres - self._metres_at_plan >= self.p.replan_m or self.plan is None

    def replan(self, frame, lane: int, progress: float, capacity_l: float, near_stop_m: float | None) -> None:
        req = self._plan_request(frame, lane, progress, capacity_l, near_stop_m)
        if req is not None:
            self._apply_plan(req, self._make_plan(req))

    def replan_async(self, frame, lane: int, progress: float, capacity_l: float, near_stop_m: float | None) -> None:
        """Start planning on another thread, unless it's already at it; `replan_poll` picks up the result."""
        now = time.monotonic()
        if self._plan_thread is not None or now - self._plan_asked < self.p.plan_every_s:
            return
        req = self._plan_request(frame, lane, progress, capacity_l, near_stop_m)
        if req is None:
            return
        self._plan_asked = now

        def work():
            try:
                self._plan_result = (req, self._make_plan(req), None)
            except Exception as e:                    # noqa: BLE001 (a thread's exception would just vanish)
                self._plan_result = (req, None, e)

        self._plan_thread = threading.Thread(target=work, name="plan", daemon=True)
        self._plan_thread.start()

    def replan_poll(self) -> bool:
        """Apply a finished plan (on the driving thread). True if there was one."""
        result = self._plan_result
        if result is None:
            return False
        self._plan_result = self._plan_thread = None
        req, new, error = result
        if error is not None:
            self.note = f"planning failed: {error}"
            return True
        if req["gen"] == self._plan_gen and self.action is None:    # (stale otherwise: another job, or stopped already)
            self._apply_plan(req, new)
        return True

    def _plan_request(self, frame, lane, progress, capacity_l, near_stop_m) -> dict | None:
        self._metres_at_plan = self.rates.metres
        target = self.leg_target()
        if target is None:
            return None
        forced, self._force_replan = self._force_replan, False
        if (not forced and near_stop_m is not None and near_stop_m < self.p.keep_stop_m and self.plan
                and self.plan.stops):
            return None                               # committed to the stop just ahead
        job = self.job
        return {"gen": self._plan_gen, "target": target, "forced": forced, "lane": lane, "progress": progress,
                "fuel": float(frame.fuel), "capacity": capacity_l, "game_time": float(frame.game_time),
                "rest": float(frame.next_rest_stop) if self.fatigue(frame) else None,
                "income": float(job.income) if job else 0.0,
                "deadline": None if job is None else job.deadline - self.leg_after_m * self.rates.min_per_m}    # this leg's end

    def _make_plan(self, req: dict) -> Plan | None:
        """The searches: nothing in here changes the mission, so it can run on another thread."""
        target = req["target"]
        rt = self.w.router.route(req["lane"], req["progress"], (target.x, target.y))
        if rt is None:
            return None
        state = PlanState(rt, req["fuel"], req["capacity"], req["game_time"], req["rest"], req["income"], req["deadline"])
        return self.planner.plan(state, req["progress"])

    def _apply_plan(self, req: dict, new: Plan | None) -> None:
        target, forced = req["target"], req["forced"]
        if new is None:
            self.note = f"no route to {target.label}"
            return
        old = self.plan
        sites = lambda pl: [(st.kind, st.point.site) for st in pl.stops] if pl else None     # noqa: E731
        if old is not None and old.problem is None and new.problem is None and sites(new) == sites(old):
            self.plan = new                           # same stops, fresher numbers: nothing new to log
            return
        keep = (not forced and old is not None and old.problem is None and old.stops and new.stops
                and new.value < old.value + self.p.replan_gain_eur
                and old.stops[0].point.site in {c.get("site") for c in new.record.get("candidates", [])})
        if keep:
            return
        self.plan = new
        self.log.add("plan", {"leg": self.state, "to": target.label, **new.record})
        if self.state == "to stop" and not new.stops:
            self.state = self.leg_state()

    def target(self) -> Target | None:
        if self.plan is not None and self.plan.stops and self.state in ("to trailer", "to destination", "to place",
                                                                         "to stop"):
            st = self.plan.stops[0]
            self.state = "to stop"
            sp = st.point
            return Target(sp.x, sp.y, f"{'fuel' if st.kind == 'fuel' else 'sleep'} stop ({sp.country})",
                          goal=(sp.lane, sp.s), stop=True)
        if self.state == "to stop":
            self.state = self.leg_state()
        place = self.leg_target()
        if place is None:
            return None
        return Target(place.x, place.y, place.label)

    # ------------------------------------------------------------- stops
    def begin_action(self, capacity_l: float) -> None:
        st = self.plan.stops[0]
        self.action = Refuel(capacity_l, self.action_params) if st.kind == "fuel" else Sleep(self.action_params)
        self.state = "at stop"

    def finish_action(self, result: ActionStep, frame) -> None:
        st = self.plan.stops.pop(0)
        rec = {"kind": st.kind, "site": st.point.site, "country": st.point.country, "result": result.note,
               "failed": result.failed, "clock": clock(frame.game_time), "fuel_l": round(float(frame.fuel)),
               "rest_left": duration(float(frame.next_rest_stop))}
        if st.kind == "fuel" and isinstance(self.action, Refuel):
            rec.update(litres=round(self.action.litres, 1),
                       cost_eur=round(self.action.litres * self.services.price_at(st.point), 2))
        self.log.add("stop", rec)
        self.action = None
        self.state = self.leg_state()
        self._metres_at_plan = -1e9                    # re-plan from here, from scratch
        self._force_replan = True
        self.plan = None if result.failed else self.plan

    def status(self) -> MissionStatus:
        j = self.job
        ms = MissionStatus(state=self.state, note=self.note)
        if j is not None:
            ms.job = f"{j.cargo}: {j.source_label} -> {j.destination_label}, due {clock(j.deadline)}"
        if self.plan and self.plan.stops:
            ms.next_stop = self.plan.stops[0].describe()
        elif self.plan and self.plan.problem:
            ms.next_stop = self.plan.problem
        last = next((r for r in reversed(self.log.records) if r.get("rationale")), None)
        ms.decision = last.get("rationale", "") if last else ""
        return ms


class MissionDriver:
    """The autopilot plus the mission: one `step` per frame while AutoHaul drives (the game or the simulator)."""

    def __init__(self, ap: Autopilot, mission: Mission, trailer_source=None, hook_back_m: float = 1.57,
                 coupling_params: CouplingParams | None = None):
        self.ap, self.m = ap, mission
        self._target: Target | None = None
        self.trailer_source = trailer_source            # () -> TrailerPose | None: the job trailer, from the save
        self.hook_back_m = hook_back_m                  # the fifth wheel behind the truck's origin
        self.coupling_params = coupling_params

    def engage(self, frame) -> str | None:
        m = self.m
        if m.action is not None or m.state == "at stop":
            # handed back during a stop script: the driver may have refuelled / slept already. Drop it and
            # re-plan from the fuel and rest timer as they are now (never sleep twice).
            m.log.add("stop", {"kind": m.action.name if m.action else "?", "result": "interrupted by the driver",
                               "clock": clock(frame.game_time)})
            m.action, m.plan = None, None
            m.state = m.leg_state()
        if m.state == "to trailer" and abs(frame.speed) < 1.0 and (act := self._coupling(frame)) is not None:
            m.state, m.action = "at trailer", act               # the trailer stands near: couple it
            m.log.add("couple", {"result": "coupling the trailer (engaged near it)", "clock": clock(frame.game_time)})
            return None
        place = m.leg_target() if m.state in ("to trailer", "to destination") else None
        tr = frame.truck_placement
        if (place is not None and m.p.quick_park and abs(frame.speed) < 1.0
                and math.hypot(tr.x - place.x, tr.z - place.y) < m.p.park_here_m):
            # standing in the depot already (the trailer waits here): no yard driving, quick park first
            deliver = m.state == "to destination"
            m.state = "at destination" if deliver else "at trailer"
            m.action = QuickPark("deliver" if deliver else "attach", self._delivered if deliver else self._attached,
                                 m.action_params)
            m.log.add("park", {"what": m.action.what, "result": "trying quick park (engaged in the depot)",
                               "clock": clock(frame.game_time)})
            return None
        self._sync_target(force=True)
        return self.ap.engage(frame)

    def _sync_target(self, force: bool = False) -> None:
        t = self.m.target()
        if t is None:
            return
        if force or self._target is None or (t.x, t.y, t.goal, t.stop) != \
                (self._target.x, self._target.y, self._target.goal, self._target.stop):
            self.ap.set_destination(t.x, t.y, goal=t.goal, stop=t.stop)
            self._target = t

    def _coupling(self, frame) -> Coupling | None:
        """A coupling script for the job trailer, if the save shows it uncoupled within `couple_within_m`."""
        m = self.m
        if not m.p.couple or self.trailer_source is None:
            return None
        t = self.trailer_source()
        tr = frame.truck_placement
        if t is None or t.connected or math.hypot(t.x - tr.x, t.y - tr.z) > m.p.couple_within_m:
            return None
        self.ap.servo.reset()
        return Coupling(self.ap.geom, t, self.hook_back_m, self.coupling_params)

    def step(self, frame, cfg_block, events, traffic, dt: float) -> MissionStep:
        m = self.m
        m.update(frame, cfg_block, events)
        capacity = float(cfg_block.truck.fuel_capacity) if cfg_block is not None else 0.0
        if isinstance(m.action, Coupling):
            act = m.action.step(frame, dt)
            end = act.done or act.failed
            st = Status(speed_kmh=frame.speed * 3.6, steer_eff=frame.effective_steering, target_reason=act.note)
            st.steer_target = act.steer
            steer = 0.0 if end else self.ap.servo.step(act.steer, frame.effective_steering, dt)
            if self.ap.pulse and not end:
                steer = self.ap.pulse.step(steer)
            out = MissionStep(Command(steer, act.throttle, act.brake), st, m.status(), act.press, reverse=act.reverse)
            out.mission.action = act.note
            if end:
                m.action = None
                m.log.add("couple", {"result": act.failed or act.note, "clock": clock(frame.game_time)})
                if act.failed:
                    out.fault = act.failed
                else:
                    m.state = m.leg_state()              # the trailer is on: on to the destination
                    self._sync_target(force=True)
                    self.ap.engage(frame)                # out of the yard (join) and on
            return out
        if isinstance(m.action, QuickPark):
            act = m.action.step(frame, dt)
            st = Status(speed_kmh=frame.speed * 3.6, steer_eff=frame.effective_steering, target_reason=act.note)
            out = MissionStep(Command(0.0, 0.0, act.brake), st, m.status(), act.press, act.hold)
            out.mission.action = act.note
            if act.done or act.failed:
                what = m.action.what
                m.action = None
                m.log.add("park", {"what": what, "result": act.failed or act.note, "clock": clock(frame.game_time)})
                if act.failed:
                    out.fault = act.failed
                elif what == "deliver":
                    out.fault = "at the destination: delivered by quick park (pick the next job)"
                else:
                    m.state = m.leg_state()              # the trailer is on: on to the destination
                    self._sync_target(force=True)
                    self.ap.engage(frame)                # out of the yard (join) and on
            return out
        if m.action is not None:
            act = m.action.step(frame, dt)
            st = Status(speed_kmh=frame.speed * 3.6, steer_eff=frame.effective_steering, target_reason=act.note)
            out = MissionStep(Command(0.0, 0.0, act.brake), st, m.status(), act.press, act.hold)
            out.mission.action = act.note
            if act.done or act.failed:
                m.finish_action(act, frame)
                self._last_stop_m = None
                self._sync_target(force=True)
                if act.failed:
                    out.fault = act.failed
                else:
                    self.ap.engage(frame)                # drive on from the stop
            return out
        self._sync_target()
        if m.replan_poll():
            self._sync_target()
        st0 = self.ap.tracker.state
        if st0 is not None and m.replan_due():
            near = self.ap.route and self._target and self._target.stop
            plan = m.replan_async if m.p.plan_in_background else m.replan
            plan(frame, st0.lane, self.ap._progress_m(st0), capacity, self._last_stop_m if near else None)
            self._sync_target()
        cmd, st = self.ap.step(frame, traffic, dt)
        self._last_stop_m = st.stop_m
        out = MissionStep(cmd, st, m.status())
        if st.arrived and m.state == "to stop":
            m.begin_action(capacity)
            out.mission.state = m.state
        elif st.fault:
            if st.fault.startswith("destination reached") and m.state in ("to trailer", "at trailer") \
                    and (act := self._coupling(frame)) is not None:
                m.state, m.action = "at trailer", act
                m.log.add("couple", {"result": "coupling the trailer", "clock": clock(frame.game_time)})
                out.cmd = Command(0.0, 0.0, m.action_params.hold_brake)
                out.mission.state = m.state
                st.fault = None
                st.target_reason = "at the depot: coupling the trailer"
                return out
            if st.fault.startswith("destination reached") and m.p.quick_park \
                    and m.state in ("to trailer", "to destination", "at trailer", "at destination"):
                deliver = m.state in ("to destination", "at destination")   # (re-engaged at the depot: again)
                m.state = "at destination" if deliver else "at trailer"
                m.action = QuickPark("deliver" if deliver else "attach",
                                     self._delivered if deliver else self._attached, m.action_params)
                m.log.add("park", {"what": m.action.what, "result": "trying quick park", "clock": clock(frame.game_time)})
                out.cmd = Command(0.0, 0.0, m.action_params.hold_brake)
                out.mission.state = m.state
                st.fault = None                          # not a hand-back: drive.py reads the status's fault too
                st.target_reason = "at the depot: quick park"
                return out
            if st.fault.startswith("destination reached") and m.state == "to trailer":
                st.fault = "at the trailer's depot: couple the trailer yourself (Phase 5), then press / to go on"
                m.state = "at trailer"
            elif st.fault.startswith("destination reached") and m.state == "to destination":
                st.fault = "at the destination depot: park and deliver yourself (Phase 5)"
                m.state = "at destination"
            elif st.fault.startswith("destination reached") and m.state == "to place":
                st.fault = f"at the destination: {self._target.label if self._target else 'arrived'}"
                m.state = "arrived"
            out.fault = st.fault
            m.log.add("handback", {"state": m.state, "reason": st.fault, "clock": clock(frame.game_time)})
        return out

    def _delivered(self, frame) -> bool:
        """The job's delivery event came in (Mission.update sets "delivered"), or the job is gone."""
        return self.m.state == "delivered" or self.m.job is None

    def _attached(self, frame) -> bool:
        trailers = getattr(frame, "trailers", None)
        return bool(trailers and trailers[0].connected)

    _last_stop_m: float | None = None
