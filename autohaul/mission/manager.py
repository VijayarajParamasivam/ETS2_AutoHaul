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

from dataclasses import dataclass, field

from autohaul.drive.autopilot import Autopilot, Status
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
        self.free_target: Place | None = None          # driven to while there's no job (drive.py --to)

    # ------------------------------------------------------------- job
    def leg_state(self) -> str:
        if self.job is not None:
            return "to destination" if self.job.cargo_loaded else "to trailer"
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
        if j.cargo_loaded:
            return self.w.places.company(j.destination_city, j.destination_company)
        return self.w.places.company(j.source_city, j.source_company)

    def _job_changed(self, job: Job | None, game_time: int) -> None:
        old = self.job
        keep = self.action if isinstance(self.action, QuickPark) else None   # its success changes the job
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
        self.state = "to destination" if job.cargo_loaded else "to trailer"
        self.leg_after_m = self._after_leg_m() if not job.cargo_loaded else 0.0

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
                if not isinstance(self.action, QuickPark):      # quick park checks for this itself
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
        tr = frame.truck_placement
        self.rates.update(tr.x, tr.z, frame.speed, float(frame.fuel), float(frame.game_time),
                          float(frame.next_rest_stop), float(getattr(frame, "fuel_average_consumption", 0.0)))

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
        self._metres_at_plan = self.rates.metres
        target = self.leg_target()
        if target is None:
            return
        forced, self._force_replan = self._force_replan, False
        if (not forced and near_stop_m is not None and near_stop_m < self.p.keep_stop_m and self.plan
                and self.plan.stops):
            return                                    # committed to the stop just ahead
        rt = self.w.router.route(lane, progress, (target.x, target.y))
        if rt is None:
            self.note = f"no route to {target.label}"
            return
        rest = float(frame.next_rest_stop) if self.fatigue(frame) else None
        job = self.job
        deadline = None
        if job is not None:
            deadline = job.deadline - self.leg_after_m * self.rates.min_per_m    # time this leg must end by
        state = PlanState(rt, float(frame.fuel), capacity_l, float(frame.game_time), rest,
                          float(job.income) if job else 0.0, deadline)
        new = self.planner.plan(state, progress)
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

    def __init__(self, ap: Autopilot, mission: Mission):
        self.ap, self.m = ap, mission
        self._target: Target | None = None

    def engage(self, frame) -> str | None:
        m = self.m
        if m.action is not None or m.state == "at stop":
            # handed back during a stop script: the driver may have refuelled / slept already. Drop it and
            # re-plan from the fuel and rest timer as they are now (never sleep twice).
            m.log.add("stop", {"kind": m.action.name if m.action else "?", "result": "interrupted by the driver",
                               "clock": clock(frame.game_time)})
            m.action, m.plan = None, None
            m.state = m.leg_state()
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

    def step(self, frame, cfg_block, events, traffic, dt: float) -> MissionStep:
        m = self.m
        m.update(frame, cfg_block, events)
        capacity = float(cfg_block.truck.fuel_capacity) if cfg_block is not None else 0.0
        if isinstance(m.action, QuickPark):
            act = m.action.step(frame, dt)
            st = Status(speed_kmh=frame.speed * 3.6, steer_eff=frame.effective_steering, target_reason=act.note)
            out = MissionStep(Command(0.0, 0.0, act.brake), st, m.status(), act.press, act.hold)
            out.mission.action = act.note
            if act.done or act.failed:
                what = m.action.what
                m.action = None
                m.log.add("park", {"what": what, "result": act.note or act.failed, "clock": clock(frame.game_time)})
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
        st0 = self.ap.tracker.state
        if st0 is not None and m.replan_due():
            near = self.ap.route and self._target and self._target.stop
            m.replan(frame, st0.lane, self.ap._progress_m(st0), capacity,
                     self._last_stop_m if near else None)
            self._sync_target()
        cmd, st = self.ap.step(frame, traffic, dt)
        self._last_stop_m = st.stop_m
        out = MissionStep(cmd, st, m.status())
        if st.arrived and m.state == "to stop":
            m.begin_action(capacity)
            out.mission.state = m.state
        elif st.fault:
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
        return bool(trailers is not None and trailers[0].connected) or bool(self.m.job and self.m.job.cargo_loaded)

    _last_stop_m: float | None = None
