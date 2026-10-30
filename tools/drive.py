"""Phase 2: AutoHaul drives on the lane map while you supervise.

    python tools/drive.py --mode speed     # AutoHaul: throttle/brake (limit, curves, following distance); you steer
    python tools/drive.py --mode steer     # AutoHaul: lane keeping; you use the pedals
    python tools/drive.py --mode full      # both
    python tools/drive.py --to berlin:nbfc # Phase 3: follow a route (default: the active job's depot)
    python tools/drive.py --mission        # Phase 4: run the active job: fuel and rest stops, decision log

Drive onto a mapped lane, then press the engage key ("/" by default) in the game. It toggles:
press it again to hand back. You take over instantly, and AutoHaul disengages, when you
  * press a driving key for an axis AutoHaul controls (A/D/arrows for steering, W/S/arrows/Space for
    pedals), move the controller stick or a trigger,
  * press the kill key (backslash or Pause) or BACK+START on the controller,
and it hands back by itself on any fault (lane lost, uncertain lane, off centre, someone else steering,
pause, stale telemetry). Beeps: rising = engaged, falling = handed back, rapid = fault.
If another window takes the focus while AutoHaul drives, it brings the game straight back to the front: the
game ignores all input while it isn't focused and keeps the last throttle and steering. Press the engage key
to hand back before you use another window.
Everything is recorded (frames + autopilot notes) to <cache>/recordings/.
"""

from __future__ import annotations

import _thread
import argparse
import dataclasses
import json
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.control.giveway import GiveWayParams  # noqa: E402
from autohaul.control.speed import SpeedParams  # noqa: E402
from autohaul.control.vehicle import TruckGeometry  # noqa: E402
from autohaul.drive.autopilot import Autopilot  # noqa: E402
from autohaul.drive.indicators import IndicatorControl  # noqa: E402
from autohaul.drive.keeper import Keeper, KeeperParams  # noqa: E402
from autohaul.drive.wipers import WiperControl, picture_features  # noqa: E402
from autohaul.infra.overlay_feed import OverlayFeed, explain  # noqa: E402
from autohaul.learn.learner import Learner  # noqa: E402
from autohaul.drive.lights import Headlights  # noqa: E402
from autohaul.infra.gameenv import active_controls_file, controls_constants, profile_setting, steering_mode  # noqa: E402
from autohaul.infra import saves  # noqa: E402
from autohaul.infra.paths import cache_dir, logs_dir, map_dir  # noqa: E402
from autohaul.infra.session import Session, enable_ansi  # noqa: E402
from autohaul.io import _layout as L  # noqa: E402
from autohaul.io.gamedata import GameDataReader  # noqa: E402
from autohaul.io.telemetry import text  # noqa: E402
from autohaul.io.win32 import bring_to_front, foreground_pid  # noqa: E402
from autohaul.vision.camera import Camera  # noqa: E402
from autohaul.vision.lights import LightVision  # noqa: E402
from autohaul.vision.screen import GameScreen  # noqa: E402
from autohaul.mission.decisions import DecisionLog  # noqa: E402
from autohaul.mission.job import JobHistory  # noqa: E402
from autohaul.mission.manager import Mission, MissionDriver  # noqa: E402
from autohaul.mission.settings import mission_settings  # noqa: E402
from autohaul.safety.keys import VK_NAME, vk  # noqa: E402
from autohaul.safety.mp_guard import multiplayer_reasons  # noqa: E402
from autohaul.safety.supervisor import Command  # noqa: E402
from autohaul.world.bundle import load_world  # noqa: E402
from autohaul.world.lanemap import LaneMap  # noqa: E402
from autohaul.world.pois import toll_plazas  # noqa: E402
from autohaul.world.places import Places  # noqa: E402
from autohaul.world.services import Services  # noqa: E402


def beep(pattern: str, enabled: bool = True) -> None:
    if not enabled:
        return
    tones = {"on": [(880, 90), (1320, 120)], "off": [(990, 90), (660, 160)],
             "fault": [(1400, 70), (0, 40)] * 4, "refuse": [(330, 250)]}[pattern]

    def play():
        import winsound
        for f, ms in tones:
            if f:
                winsound.Beep(f, ms)
            else:
                time.sleep(ms / 1000)

    threading.Thread(target=play, daemon=True).start()


def speed_params(cfg) -> SpeedParams:
    d = cfg.get("drive", {})
    names = {f.name for f in dataclasses.fields(SpeedParams)}
    return SpeedParams(**{k: v for k, v in d.items() if k in names})


def give_way_params(cfg) -> GiveWayParams:
    d = cfg.get("drive", {}).get("give_way", {})
    names = {f.name for f in dataclasses.fields(GiveWayParams)}
    return GiveWayParams(**{k: v for k, v in d.items() if k in names})


def preflight(s: Session, longitudinal: bool) -> list[str]:
    """Problems that make driving impossible (empty list = OK)."""
    problems = []
    f = s.reader.wait_frame(500) or s.reader.read_latest()
    hdr = s.reader.header()
    if f is None:
        return ["no telemetry frames: load your profile and get into the truck"]
    if not hdr.input_registered:
        problems.append("the bridge's input device is not registered (check game.log.txt for [autohaul] lines)")
    cfgb = s.reader.read_config()
    shifter = text(cfgb.controls.shifter_type) if cfgb and cfgb.controls.present else ""
    if longitudinal and shifter != "automatic":
        problems.append(f"transmission is '{shifter or 'unknown'}': AutoHaul's pedal control needs "
                        "Options -> Gameplay -> Transmission type -> Automatic")
    if steering_mode(s.cfg) is None:
        problems.append("could not read the steering mode (c_relatsteer) from your profile's controls.sii")
    mp = multiplayer_reasons(f, hdr.game_pid, s.cfg.get("safety.mp_guard_modules"))
    if mp:
        problems.append("multiplayer detected: " + "; ".join(mp))
    if s.keyboard is None:
        problems.append("keyboard hook unavailable: AutoHaul can't see the engage key or your driving keys")
    return problems


class Driver:
    def __init__(self, s: Session, ap: Autopilot, lateral: bool, longitudinal: bool, places=None,
                 fixed_target=None, fallback=None, mission: MissionDriver | None = None):
        self.s, self.ap, self.md = s, ap, mission
        self.lateral, self.longitudinal = lateral, longitudinal
        self.headlights = Headlights() if s.cfg.get("drive", {}).get("auto_headlights", True) else None
        kp = {k: v for k, v in s.cfg.get("drive", {}).get("keeper", {}).items() if hasattr(KeeperParams, k)}
        self.keeper = Keeper(KeeperParams(**kp)) if longitudinal else None
        if self.keeper is not None:
            ap.p.ready_check = False             # the keeper releases the parking brake and shifts to drive
        self._unfocused_at: float | None = None  # engaged and another window in front since
        self._refocus_until = 0.0                # after a hand-back: keep pulling the game to the front until
        self._refocus_tried = -1e9
        self._kept_hold: str | None = None       # a button the keeper is holding (the engine starter)
        self._kept_msg = False                   # the status message is the keeper's
        self.mission_status = None
        self.places, self.fixed_target, self.fallback = places, fixed_target, fallback
        self.ind = IndicatorControl(lambda name: self.s.controls.press(name))
        self._adaptive_at = self._saved_at = time.monotonic()
        d = s.cfg.get("drive", {})
        self.beeps = d.get("beeps", True)
        self.require_traffic = d.get("require_traffic_data", True)
        self.engage_vk = vk(d.get("engage_key", "SLASH"))
        self.status_period = 1.0 / d.get("status_hz", 5)
        self.stale_abort_s = s.cfg.get("safety.telemetry_stale_abort_ms", 1000) / 1000
        self.refocus = d.get("refocus_game", True)
        self.refocus_grace_s = d.get("refocus_grace_s", 0.5)
        self.gd = GameDataReader()
        self.engaged = False
        self.last_cmd = Command()
        self.message = f"standby: press {VK_NAME.get(self.engage_vk, '?')} in the game to engage"
        self._presses = s.keyboard.state.press_count(self.engage_vk)
        self._stale_since: float | None = None
        self._last_sim_us: int | None = None
        self._last_note = 0.0
        self._last_status = 0.0
        self.stats = {"engagements": 0, "engaged_s": 0.0, "engaged_km": 0.0, "faults": {}}

    # ------------------------------------------------------------- engage / disengage
    learner = feed = wipers = grab_view = None      # set in main()
    _paused_at: tuple | None = None                 # paused while engaged: where the truck stood
    slow_step_ms = 100.0
    capacity_l = 0.0
    _last_lane: int | None = None
    FEED_EVENTS = {"engaged": "engaged", "keeper": "truck: ", "press": "pressed ", "toll_paid": "toll paid",
                   "gps_reroute": "the game's GPS re-routed", "light": "light: ", "toll_booth_avoided": "avoiding a toll booth",
                   "engage_refused": "can't engage: ", "paused": "game paused: still in charge"}

    def note(self, **fields) -> None:
        if self.s.recorder:
            self.s.recorder.note(**fields)
        e = fields.get("event")
        if self.feed is not None and e in self.FEED_EVENTS:
            detail = fields.get("reason") or fields.get("action") or fields.get("button") or fields.get("colour") or ""
            self.feed.event(f"{self.FEED_EVENTS[e]}{detail}")

    def hold_paused(self, f, cfgb, events, now: float) -> None:
        """The game is paused: nothing moves, and AutoHaul stays in charge. It used to hand back here, and after the
        pause the truck rolled on at speed with nobody steering (2026-10-30: 4 s at 78 km/h, then the rail)."""
        ctl = self.s.controls
        if self._paused_at is None:
            self._paused_at = (f.truck_placement.x, f.truck_placement.z)
            self.note(event="paused", keeps_driving=True)
            self.message = "ENGAGED (game paused: carries on when you resume; / or a driving key takes over)"
        verdict = self.s.safety.check(f, None)
        if not verdict.ok and verdict.kind not in ("paused", "override", "stale"):
            self.disengage(verdict.reason, fault=verdict.kind != "kill")
            return
        if self.md is not None:
            self.md.m.update(f, cfgb, events)
        ctl.neutral()
        ctl.commit()                                # the heartbeat
        self.feed_overlay(f, None, now)
        if now - self._last_status > self.status_period:
            self._last_status = now
            self.print_status(f, None)

    def resume_after_pause(self, f) -> bool:
        """The pause is over. Carry on, unless the truck was moved meanwhile (a save loaded, a ferry). Keys pressed in
        the menus don't count as taking over; a driving key still down does."""
        x, y = self._paused_at
        self._paused_at = None
        if math.hypot(f.truck_placement.x - x, f.truck_placement.z - y) > 30.0:
            self.disengage("the truck was moved while the game was paused", fault=False)
            return False
        self.s.safety.reset()
        self.ap.servo.reset()
        self.last_cmd = Command()
        self.message = "ENGAGED"
        self.note(event="resumed")
        if self.feed is not None:
            self.feed.event("game resumed: still driving")
        return True

    def adaptive(self, f, st, now: float, dt: float) -> None:
        """Once a second: the wipers, and saving what's been learned now and then."""
        if now - self._adaptive_at < 1.0:
            return
        step, self._adaptive_at = now - self._adaptive_at, now
        if self.wipers is not None and self.grab_view is not None:
            img = self.grab_view() if foreground_pid() == self.s.reader.header().game_pid and not f.paused else None
            self.wipers.step(bool(f.wipers), None if img is None else picture_features(img), now, min(step, 2.0), self.engaged)
        if now - self._saved_at > 300.0:
            self._saved_at = now
            if self.learner is not None:
                self.learner.save()
            if self.wipers is not None:
                self.wipers.save()

    def feed_overlay(self, f, st, now: float) -> None:
        if self.feed is None or not self.feed.due(now):
            return
        ms = None
        if self.md is not None:
            ms = self.mission_status if self.engaged and self.mission_status else self.md.m.status()
        extras: dict = {}
        if self.wipers is not None:
            extras["rain"] = self.wipers.describe()
        if self.learner is not None:
            extras["learned"] = self.learner.summary()
        self.feed.send(explain(f, st if self.engaged else None, ms, self.message, self.engaged, self.capacity_l, extras))

    def try_engage(self, frame) -> None:
        s, ctl = self.s, self.s.controls
        hdr = s.reader.header()
        reason = None
        if foreground_pid() != hdr.game_pid:
            reason = "the game window is not focused"
        elif frame.paused:
            reason = "the game is paused"
        elif self.longitudinal and self.require_traffic and self.gd.traffic() is None:
            reason = "no traffic data from the ETS2LA plugin, so no lead-vehicle detection (see docs/PHASE2.md)"
        else:
            down = s.keyboard.state.any_down(s.safety.override_vks)
            if down is not None:
                reason = f"release the {VK_NAME.get(down, hex(down))} key first"
        if reason is None and self.md is not None:
            if self.md.m.target() is None:
                reason = "no active job (accept one, or start with --to CITY to drive somewhere without one)"
            else:
                reason = self.md.engage(frame)
        elif reason is None:
            self.update_destination()
            reason = self.ap.engage(frame)
        if reason is None:
            # re-arm the bridge (clears a kill latch from earlier) and wait for the DLL to confirm
            ctl.neutral()
            ctl.set_enabled(False)
            ctl.rearm()
            end = time.monotonic() + 1.5
            while s.reader.header().bridge_flags & L.BRIDGE_FLAG_KILL_LATCHED:
                if time.monotonic() > end:
                    reason = "the bridge kept its kill latch (is a kill key held?)"
                    break
                ctl.commit()
                s.reader.wait_frame(50)
        if reason:
            self.message = f"NOT engaged: {reason}"
            self.note(event="engage_refused", reason=reason)
            beep("refuse", self.beeps)
            return
        s.safety.reset()
        ctl.set_enabled(True)
        self.engaged = True
        if self.keeper is not None:
            self.keeper.reset()
        self.ind.reset()
        self.last_cmd = Command()
        self.stats["engagements"] += 1
        self.message = "ENGAGED"
        self.note(event="engaged", mode=self.mode_name)
        beep("on", self.beeps)

    def update_destination(self) -> None:
        """Pick the destination when engaging: --to if given, else the active job's depot (the job can change
        during a session), else --fallback-to."""
        if self.places is None:
            return
        target = self.fixed_target or self.places.job_target(self.s.reader.read_config()) or self.fallback
        if target is None:
            if self.ap.destination is not None:
                self.ap.destination, self.ap.route = None, None
            return
        if self.ap.destination != (target.x, target.y):
            self.ap.set_destination(target.x, target.y)
            self.note(event="destination", label=target.label, x=target.x, y=target.y)
            print(f"\ndestination: {target.label}")

    def keep_focus(self, now: float) -> bool:
        """Engaged: True while the game window is in front, or was brought back within the grace time. The game
        reads no input while another window is in front and keeps the last throttle and steering, so a click on
        another window would leave the truck at full throttle (the crash of 2026-10-23)."""
        game = self.s.reader.header().game_pid
        if foreground_pid() == game:
            if self._unfocused_at is not None:
                gone = now - self._unfocused_at
                self._unfocused_at = None
                self.note(event="refocused", after_s=round(gone, 3))
                self.message = "ENGAGED (brought the game back to the front: press / to hand back first)"
            return True
        if not self.refocus:
            return False
        if self._unfocused_at is None:
            self._unfocused_at = now
            self.note(event="focus_lost", foreground_pid=foreground_pid())
        if now - self._refocus_tried > 0.1:
            self._refocus_tried = now
            bring_to_front(game)
        return foreground_pid() == game or now - self._unfocused_at < self.refocus_grace_s

    def refocus_after_handback(self, now: float) -> None:
        """After a focus-loss hand-back the released controls reach the game only once it's in front again."""
        if now >= self._refocus_until or now - self._refocus_tried < 0.2:
            return
        game = self.s.reader.header().game_pid
        if foreground_pid() == game:
            self._refocus_until = 0.0
            return
        self._refocus_tried = now
        bring_to_front(game)

    def keep_truck_ready(self, f, cmd, dt: float, now: float, reversing: bool = False):
        """Engine on, parking brake off, gearbox in drive (autohaul/drive/keeper.py). Returns the command to send
        (held on the brake while something isn't ready) and a failure to hand back with, or None."""
        if self.keeper is None:
            return cmd, None
        k = self.keeper.step(f, cmd.throttle > 0.2, dt, now, reversing)
        ctl = self.s.controls
        if k.press:
            ctl.press(k.press)
            self.note(event="keeper", action=k.press, reason=k.problem)
        if k.hold and k.hold != self._kept_hold:
            self.note(event="keeper", action=f"hold {k.hold}", reason=k.problem)
        if self._kept_hold and self._kept_hold != k.hold:
            ctl.hold(self._kept_hold, False)
        if k.hold:
            ctl.hold(k.hold, True)
        self._kept_hold = k.hold
        if k.problem:
            self.message = f"ENGAGED ({k.problem})"
            cmd = dataclasses.replace(cmd, throttle=0.0, brake=max(cmd.brake, 0.3))
        elif self.message.startswith("ENGAGED (") and self._kept_msg:
            self.message = "ENGAGED"                 # the keeper's problem is solved
        self._kept_msg = bool(k.problem)
        return cmd, k.failure

    def disengage(self, reason: str, fault: bool) -> None:
        ctl = self.s.controls
        if self.learner is not None:
            f = self.s.reader.read_latest()
            self.learner.on_handback(reason, float(f.speed) if f is not None else 0.0, self._last_lane)
        if self.feed is not None:
            self.feed.event(f"handed back: {reason}")
        failed = getattr(self.ap.gw, "failed_toll_lane", None) if self.ap.gw is not None else None
        if reason.startswith("toll gate: no payment") and failed is not None:
            remember_bad_toll(self.s.cfg, self.ap.lm, failed)    # the router avoids this booth from now on
            self.note(event="toll_booth_avoided", lane=self.ap.lm.describe(failed))
        if self.ind.mine:                           # switch AutoHaul's indicator off while controls are live
            side = self.ind.mine
            ctl.neutral()
            end = time.monotonic() + self.ind.settle_s(time.monotonic())
            while time.monotonic() < end:           # a press just sent must show in telemetry first
                ctl.commit()
                self.s.reader.wait_frame(30)
            f = self.s.reader.read_latest()
            if f is not None and ((side == "left" and f.lblinker) or (side == "right" and f.rblinker)):
                ctl.press(f"{side[0]}blinker")
                end = time.monotonic() + 0.15
                while time.monotonic() < end:
                    ctl.commit()
                    self.s.reader.wait_frame(30)
            self.ind.reset()
        if self._kept_hold:
            ctl.hold(self._kept_hold, False)
            self._kept_hold = None
        ctl.neutral()
        ctl.set_enabled(False)
        ctl.commit()
        self.engaged = False
        self._unfocused_at = None
        self.last_cmd = Command()
        self.message = f"handed back: {reason}"
        if fault:
            self.stats["faults"][reason.split(" (")[0]] = self.stats["faults"].get(reason.split(" (")[0], 0) + 1
        self.note(event="disengaged", reason=reason, fault=fault)
        beep("fault" if fault else "off", self.beeps)

    @property
    def mode_name(self) -> str:
        return "full" if self.lateral and self.longitudinal else ("steer" if self.lateral else "speed")

    # ------------------------------------------------------------- loop
    def run(self, minutes: float = 0.0) -> None:
        s = self.s
        end = time.monotonic() + minutes * 60 if minutes else None
        while end is None or time.monotonic() < end:
            f = s.reader.wait_frame(100)
            now = time.monotonic()
            presses = s.keyboard.state.press_count(self.engage_vk)
            toggle = presses != self._presses
            self._presses = presses
            if f is None:
                if self.engaged:
                    self._stale_since = self._stale_since or now
                    if now - self._stale_since > self.stale_abort_s:
                        self.disengage("telemetry stopped", fault=True)
                    else:
                        s.controls.commit()            # keep the heartbeat while the game stutters
                continue
            self._stale_since = None
            s.record_frame(f)
            self.watch_gps(f)
            dt = 1 / 60
            if self._last_sim_us is not None and f.simulation_time_us > self._last_sim_us:
                dt = min(0.1, max(0.001, (f.simulation_time_us - self._last_sim_us) / 1e6))
            self._last_sim_us = f.simulation_time_us
            events = s.reader.poll_events()
            for ev in events:
                if ev.id == "player.tollgate.paid":
                    self.ap.toll_paid()
                    self.note(event="toll_paid", **ev.attrs)
                elif ev.id == "player.fined":
                    offence = ev.attrs.get("fine.offence", "?")
                    if self.learner is not None:
                        self.learner.on_fine(offence, float(f.speed), self._last_lane)
                    if self.feed is not None:
                        self.feed.event(f"fined: {offence} ({ev.attrs.get('fine.amount', '?')} EUR)")
            cfgb = s.reader.read_config() if self.md is not None else None
            if self.md is not None and not self.engaged:
                self.md.m.update(f, cfgb, events)        # the job, its events and the rates, while you drive
            if toggle:
                if self.engaged:
                    self.disengage(f"{VK_NAME.get(self.engage_vk, 'engage')} pressed", fault=False)
                else:
                    self.try_engage(f)
            st = None
            if not self.engaged:
                self._paused_at = None
                self.refocus_after_handback(now)
            if self.engaged and f.paused:
                self.hold_paused(f, cfgb, events, now)
                continue
            if self.engaged and self._paused_at is not None and not self.resume_after_pause(f):
                continue
            if self.engaged:
                traffic = self.gd.traffic() if self.longitudinal else None
                out = None
                step_at = time.perf_counter()
                if self.md is not None:
                    out = self.md.step(f, cfgb, events, traffic, dt)
                    cmd, st = out.cmd, out.status
                    self.mission_status = out.mission
                    if out.fault:
                        st.fault = out.fault
                else:
                    cmd, st = self.ap.step(f, traffic, dt)
                took_ms = (time.perf_counter() - step_at) * 1e3
                if took_ms > self.slow_step_ms:                # the truck drives on blind during this: never expected
                    self.note(event="slow_step", ms=round(took_ms), timings=st.timings_ms, lane=st.lane)
                    if self.feed is not None:
                        self.feed.event(f"slow step: {took_ms:.0f} ms")
                verdict = s.safety.check(f, self.last_cmd)
                if not verdict.ok and verdict.kind != "stale":
                    self.disengage(verdict.reason, fault=verdict.kind not in ("override", "kill", "paused"))
                elif st.fault:
                    self.disengage(st.fault, fault=not st.fault.startswith("at the"))
                elif not self.keep_focus(now):
                    self.disengage("the game window lost focus and AutoHaul couldn't bring it back", fault=True)
                    self._refocus_until = now + 10.0       # until the game takes the released controls
                elif (ready := self.keep_truck_ready(f, cmd, dt, now, bool(out and out.reverse)))[1]:
                    self.disengage(ready[1], fault=True)
                else:
                    cmd = ready[0]
                    self.ap.sent(cmd)
                    ctl = s.controls
                    ctl.steer(cmd.steer if self.lateral else 0.0)
                    ctl.throttle(cmd.throttle if self.longitudinal else 0.0)
                    ctl.brake(cmd.brake if self.longitudinal else 0.0)
                    self.ind.step(f.lblinker, f.rblinker, st.indicator, now)
                    if self.headlights is not None:
                        button = self.headlights.step(f.game_time, bool(f.light_low_beam), bool(f.light_parking), now)
                        if button:
                            ctl.press(button)
                    for name in st.press:                # paying at a toll gate
                        ctl.press(name)
                        self.note(event="press", button=name, reason=st.junction)
                    if out is not None:                  # a stop script's buttons (refuel, sleep, quick park)
                        for name in out.press:
                            ctl.press(name)
                            self.note(event="press", button=name, reason=out.mission.action)
                        for name, on in out.hold.items():
                            ctl.hold(name, on)
                    ctl.commit()
                    self.last_cmd = Command(cmd.steer if self.lateral else 0.0,
                                            cmd.throttle if self.longitudinal else 0.0,
                                            cmd.brake if self.longitudinal else 0.0)
                    self.stats["engaged_s"] += dt
                    self.stats["engaged_km"] += max(f.speed, 0.0) * dt / 1000
                lights = getattr(self.ap.gw, "lights", None) if self.ap.gw is not None else None
                for ch in getattr(lights, "changes", ())[:]:
                    self.note(event="light", **ch)
                if lights is not None and hasattr(lights, "changes"):
                    lights.changes.clear()
                if now - self._last_note > 0.1:
                    self._last_note = now
                    fields = self.note_fields(st)
                    self.note(event="ap", **fields)
                    if self.learner is not None and self.engaged:
                        trailers = getattr(f, "trailers", None)
                        self.learner.observe(time.time(), f, fields, True, bool(trailers and trailers[0].connected))
                if st.lane is not None:
                    self._last_lane = st.lane
            self.adaptive(f, st, now, dt)
            self.feed_overlay(f, st, now)
            if now - self._last_status > self.status_period:
                self._last_status = now
                self.print_status(f, st)

    def watch_gps(self, f) -> None:
        """The game's GPS distance falls at local_scale x speed while the truck follows the GPS route; a jump
        up means the game re-routed (AutoHaul left the game's route, or the GPS target changed)."""
        nav = f.navigation_distance
        prev = getattr(self, "_gps_prev", None)
        self._gps_prev = nav
        if prev is None or nav <= 0 or prev <= 0:
            return
        if nav - prev > 300.0:
            self.note(event="gps_reroute", before_m=round(prev), after_m=round(nav))
            if self.engaged:
                self.message = f"ENGAGED (game GPS re-routed: {prev / 1000:.1f} -> {nav / 1000:.1f} km)"

    @staticmethod
    def note_fields(st) -> dict:
        d = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in dataclasses.asdict(st).items()
             if k not in ("lead", "timings_ms")}
        if st.lead:
            d["lead_gap"], d["lead_speed"] = round(st.lead.gap, 1), round(st.lead.speed, 2)
        d["ms"] = {k: round(v, 2) for k, v in st.timings_ms.items()}
        return d

    def print_status(self, f, st) -> None:
        parts = [f"[{self.mode_name}] {self.message}"[:60].ljust(60), f"{f.speed * 3.6:5.1f} km/h"]
        if st is not None:
            parts.append(f"lane {st.confidence:.2f} lat {st.lateral:+.2f} m")
            if self.lateral:
                parts.append(f"steer {st.steer_target:+.3f}/{st.steer_eff:+.3f}")
            if self.longitudinal:
                lead = f" lead {st.lead.gap:4.0f} m" if st.lead else ""
                parts.append(f"target {st.target_kmh:3.0f} ({st.target_reason}){lead} thr {st.throttle:.2f} brk {st.brake:.2f}")
            if st.lane_change:
                parts.append(f"lane change {st.lane_change}")
            if st.junction:
                parts.append(f"junction: {st.junction}")
            if st.route_km is not None:
                parts.append(f"route {st.route_km:.2f} km{' ' + st.route_note if st.route_note else ''}"
                             f"{'' if st.on_route else ' (off route)'}")
        if self.md is not None:
            ms = self.mission_status if self.engaged and self.mission_status else self.md.m.status()
            parts.append(f"mission: {ms.state}" + (f", next: {ms.next_stop}" if ms.next_stop else "")
                         + (f" ({ms.action})" if ms.action else ""))
        print("\r" + " | ".join(parts)[:240].ljust(240), end="", flush=True)


BAD_TOLLS = "toll_booths_no_payment.json"


def bad_tolls(cfg) -> list[tuple[int, int]]:
    """Toll booth paths where pressing activate took no payment (learned while driving; the router avoids
    them). The map doesn't say which booths have a payment machine."""
    path = logs_dir(cfg) / BAD_TOLLS
    try:
        return [(int(item, 16), int(sub)) for item, sub in json.loads(path.read_text(encoding="utf-8"))]
    except (OSError, ValueError):
        return []


def remember_bad_toll(cfg, lm, lane: int) -> None:
    path = logs_dir(cfg) / BAD_TOLLS
    known = [[f"{i:x}", s] for i, s in bad_tolls(cfg)]
    entry = [f"{int(lm.lane_item[lane]):x}", int(lm.lane_sub[lane])]
    if entry not in known:
        path.write_text(json.dumps(known + [entry]), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["speed", "steer", "full"], default="full")
    parser.add_argument("--max-kmh", type=float, help="override drive.max_speed_kmh")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--minutes", type=float, default=0.0, help="quit after this long (0 = until Ctrl+C)")
    parser.add_argument("--to", help="destination: CITY, CITY:COMPANY or X,Y (default: the active job's depot)")
    parser.add_argument("--fallback-to", help="destination when no job is active at engage time")
    parser.add_argument("--no-route", action="store_true", help="lane following only (Phase 2 behaviour)")
    parser.add_argument("--no-rules", action="store_true",
                        help="hand back before every intersection instead of giving way by the map's rules")
    parser.add_argument("--mission", action="store_true",
                        help="Phase 4: run the active job (fuel and rest stops, decisions to logs/decisions.jsonl)")
    parser.add_argument("--no-learning", action="store_true", help="drive with the defaults: learn nothing, apply nothing")
    parser.add_argument("--stdin-quit", action="store_true",
                        help="quit (as Ctrl+C: hand back first) when stdin closes or reads 'quit' (tools/launcher.py)")
    args = parser.parse_args()
    if args.stdin_quit:
        def watch_stdin():
            for line in sys.stdin:
                if line.strip() == "quit":
                    break
            _thread.interrupt_main()

        threading.Thread(target=watch_stdin, name="stdin-quit", daemon=True).start()
    lateral = args.mode in ("steer", "full")
    longitudinal = args.mode in ("speed", "full")

    enable_ansi()
    with Session("drive", controls=True, record=not args.no_record) as s:
        s.connect()
        problems = preflight(s, longitudinal)
        if problems:
            print("Can't drive:\n  - " + "\n  - ".join(problems))
            return 1
        cfg = s.cfg
        d = cfg.get("drive", {})
        print("loading the lane map ...", flush=True)
        lm = LaneMap.load_or_build(map_dir(cfg))
        cfgb = s.reader.read_config()
        geom = TruckGeometry.from_config(cfgb.truck, math.radians(d.get("steer_deg_per_unit", 38.0)),
                                         d.get("front_overhang_m", 1.45))
        mode = steering_mode(cfg)
        consts = controls_constants(active_controls_file(cfg))
        sp = speed_params(cfg)
        if args.max_kmh:
            sp.max_speed_kmh = args.max_kmh
        router = junctions = rules = target = fallback = places = world = None
        if args.mission and (args.no_route or args.mode != "full"):
            print("--mission needs routing and --mode full")
            return 1
        if not args.no_route:
            places = Places(map_dir(cfg))
            target = places.resolve(args.to) if args.to else None
            fallback = places.resolve(args.fallback_to) if args.fallback_to else None
            for spec, got in ((args.to, target), (args.fallback_to, fallback)):
                if spec and got is None:
                    print(f"unknown destination {spec!r} (use CITY, CITY:COMPANY or X,Y with the game's tokens)")
                    return 1
            world = load_world(map_dir(cfg), lm, rules=not args.no_rules, avoid=bad_tolls(cfg))
            router, junctions, rules = world.router, world.junctions, world.rules
        ap = Autopilot(lm, geom, mode, lateral=lateral, longitudinal=longitudinal, speed_params=sp,
                       relative_gain=consts.get("c_rsteersens", 0.42),
                       relative_deadzone=d.get("relative_deadzone", 0.0), hazards=toll_plazas(map_dir(cfg)),
                       router=router, junctions=junctions, rules=rules, give_way=give_way_params(cfg))
        if rules is not None and d.get("traffic_lights", True):
            screen, cam_reader = GameScreen(s.reader.header().game_pid), GameDataReader()

            def camera():
                c, client = cam_reader.camera(), screen.client()
                return Camera.from_block(c, client[2], client[3], vertical_fov=False) if c and client else None

            ap.set_lights(LightVision(rules, lm, screen.grab, camera, time.monotonic))
            print("traffic lights: read from the screen (stop on red and amber, go on green)")
        if rules is not None and d.get("tolls", True):
            ap.set_tolls(True)
            print("toll gates: stops at the booth and pays (activate)")
        if router is not None:
            print("destination: " + (target.label if target else "the active job's depot when you engage"
                                     + (f", else {fallback.label}" if fallback else " (none: lane following)")))
        md = None
        if args.mission:
            services = Services(lm, map_dir(cfg), no_go=rules.cars_only() if rules is not None else None)
            mp, pp, ep, actp = mission_settings(cfg)
            mp.plan_in_background = True
            sys.setswitchinterval(0.001)                # the driving loop gets its turn quickly while a plan is made
            decisions = DecisionLog(logs_dir(cfg) / "decisions.jsonl")
            mission = Mission(world, services, decisions, JobHistory(logs_dir(cfg) / "jobs.jsonl"), mp, ep, pp, actp)
            g_fatigue = profile_setting(cfg, "g_fatigue")
            mission.profile_fatigue = None if g_fatigue is None else g_fatigue.strip() not in ("0", "")
            profile = active_controls_file(cfg).parent

            def job_trailer():
                try:
                    return saves.job_trailer(profile)
                except Exception as e:                  # noqa: BLE001 (no pycryptodome, a save being written)
                    print(f"couldn't read the job trailer from the save: {e}")
                    return None

            hook = abs(float(cfgb.truck.hook_position.z)) or 1.57
            md = MissionDriver(ap, mission, trailer_source=job_trailer, hook_back_m=hook if hook < 4 else 1.57)
            if target is not None:
                mission.set_free_target(target)         # driven to while no job is active
            mission.update(s.reader.read_latest(), s.reader.read_config(), [])
            job = mission.status().job or "no job yet (accept one; AutoHaul picks it up)"
            fat = {None: "unknown (watching the rest timer)", True: "on", False: "off (no sleep stops)"}
            print(f"mission: {job}; fatigue {fat[mission.profile_fatigue]}; economics: {mission.econ_note}; "
                  f"decisions -> {decisions.path}")
        s.safety.set_axes(steering=lateral, pedals=longitudinal)
        s.safety.reset()
        if world is not None:
            ap.entrances = world.entrances
        s.reader.poll_events()                      # the game keeps old events (the last job's delivery): skip them
        learned_dir = cache_dir(cfg) / "learned"
        learner = None
        if not args.no_learning:
            learner = Learner(learned_dir / "learned.json", lm)
            learner.attach(sp, ap.lat, router)
            ap.caution = learner.caution
            print("learning: on (" + "; ".join(learner.summary()[:2]) + f"), {learned_dir}")
        drv = Driver(s, ap, lateral, longitudinal, places, target, fallback, mission=md)
        drv.learner, drv.feed = learner, OverlayFeed()
        drv.capacity_l = float(cfgb.truck.fuel_capacity)
        if lateral and longitudinal:
            view = GameScreen(s.reader.header().game_pid)
            drv.wipers, drv.grab_view = WiperControl(lambda name: s.controls.press(name), learned_dir / "rain.json"), view.grab_all
        s.recorder and s.recorder.note(event="start", mode=args.mode, steering_mode=mode,
                                       geometry=dataclasses.asdict(geom), speed=dataclasses.asdict(sp))
        print(f"truck {text(cfgb.truck.brand)} {text(cfgb.truck.name)}: wheelbase {geom.wheelbase:.2f} m; "
              f"steering {mode} (rate {ap.servo.gain if hasattr(ap.servo, 'prior') else '-'}); max {sp.max_speed_kmh:.0f} km/h")
        keys = ", ".join(VK_NAME.get(v, hex(v)) for v in s.safety.override_vks)
        print(f"mode {args.mode}: press {d.get('engage_key', 'SLASH')} in the game to engage / hand back. "
              f"Taking over: {keys}, controller, kill key. Ctrl+C here to quit.\n")
        try:
            drv.run(args.minutes)
        except KeyboardInterrupt:
            pass
        finally:
            if drv.engaged:
                drv.disengage("AutoHaul stopped (time limit / quit)", fault=True)
                time.sleep(0.6)                      # let the alarm beep play
            if learner is not None:
                card = learner.end_session()
                if card:
                    print(f"\nlearned: {card}")
            if drv.wipers is not None:
                drv.wipers.save()
            st = drv.stats
            print(f"\n\nengaged {st['engagements']}x, {st['engaged_s'] / 60:.1f} min, {st['engaged_km']:.2f} km; "
                  f"faults: {st['faults'] or 'none'}")
            if s.recording_path:
                print(f"recording: {s.recording_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
