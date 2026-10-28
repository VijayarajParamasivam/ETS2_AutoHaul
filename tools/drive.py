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

import argparse
import dataclasses
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
from autohaul.drive.keeper import Keeper, KeeperParams  # noqa: E402
from autohaul.drive.lights import Headlights  # noqa: E402
from autohaul.infra.gameenv import active_controls_file, controls_constants, profile_setting, steering_mode  # noqa: E402
from autohaul.infra.paths import logs_dir, map_dir  # noqa: E402
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
        self._ind_on: str | None = None         # indicator AutoHaul switched on ("left"/"right")
        self._ind_pressed = 0.0
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
    def note(self, **fields) -> None:
        if self.s.recorder:
            self.s.recorder.note(**fields)

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

    def indicate(self, f, want: str | None, now: float) -> None:
        """Match the game's indicator to what the autopilot wants. The game's buttons are toggles, so compare
        with telemetry and press only on a mismatch (at most every 0.6 s, so a press is seen before the next).
        Only an indicator AutoHaul switched on is switched off again."""
        if now - self._ind_pressed < 0.6:
            return
        state = {"left": bool(f.lblinker), "right": bool(f.rblinker)}
        for side in ("left", "right"):
            on = state[side]
            if want == side and not on:
                self.s.controls.press(f"{side[0]}blinker")
                self._ind_on, self._ind_pressed = side, now
                return
            if want != side and on and self._ind_on == side:
                self.s.controls.press(f"{side[0]}blinker")
                self._ind_pressed = now
                if not want:
                    self._ind_on = None
                return
        if want is None and self._ind_on and not state[self._ind_on]:
            self._ind_on = None

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

    def keep_truck_ready(self, f, cmd, dt: float, now: float):
        """Engine on, parking brake off, gearbox in drive (autohaul/drive/keeper.py). Returns the command to send
        (held on the brake while something isn't ready) and a failure to hand back with, or None."""
        if self.keeper is None:
            return cmd, None
        k = self.keeper.step(f, cmd.throttle > 0.2, dt, now)
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
        if self._ind_on:                            # switch AutoHaul's indicator off while controls are live
            f = self.s.reader.read_latest()
            if f is not None and ((self._ind_on == "left" and f.lblinker) or (self._ind_on == "right" and f.rblinker)):
                ctl.neutral()
                ctl.press(f"{self._ind_on[0]}blinker")
                end = time.monotonic() + 0.15
                while time.monotonic() < end:
                    ctl.commit()
                    self.s.reader.wait_frame(30)
            self._ind_on = None
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
                self.refocus_after_handback(now)
            if self.engaged:
                traffic = self.gd.traffic() if self.longitudinal else None
                out = None
                if self.md is not None:
                    out = self.md.step(f, cfgb, events, traffic, dt)
                    cmd, st = out.cmd, out.status
                    self.mission_status = out.mission
                    if out.fault:
                        st.fault = out.fault
                else:
                    cmd, st = self.ap.step(f, traffic, dt)
                verdict = s.safety.check(f, self.last_cmd)
                if not verdict.ok and verdict.kind != "stale":
                    self.disengage(verdict.reason, fault=verdict.kind not in ("override", "kill", "paused"))
                elif st.fault:
                    self.disengage(st.fault, fault=not st.fault.startswith("at the"))
                elif not self.keep_focus(now):
                    self.disengage("the game window lost focus and AutoHaul couldn't bring it back", fault=True)
                    self._refocus_until = now + 10.0       # until the game takes the released controls
                elif (ready := self.keep_truck_ready(f, cmd, dt, now))[1]:
                    self.disengage(ready[1], fault=True)
                else:
                    cmd = ready[0]
                    self.ap.sent(cmd)
                    ctl = s.controls
                    ctl.steer(cmd.steer if self.lateral else 0.0)
                    ctl.throttle(cmd.throttle if self.longitudinal else 0.0)
                    ctl.brake(cmd.brake if self.longitudinal else 0.0)
                    self.indicate(f, st.indicator, now)
                    if self.headlights is not None:
                        button = self.headlights.step(f.game_time, bool(f.light_low_beam), bool(f.light_parking), now)
                        if button:
                            ctl.press(button)
                    for name in st.press:                # paying at a toll gate
                        ctl.press(name)
                        self.note(event="press", button=name, reason=st.junction)
                    if out is not None:                  # a stop script's buttons (refuel, sleep)
                        for name in out.press:
                            ctl.press(name)
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
                    self.note(event="ap", **self.note_fields(st))
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
    args = parser.parse_args()
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
            world = load_world(map_dir(cfg), lm, rules=not args.no_rules)
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
            decisions = DecisionLog(logs_dir(cfg) / "decisions.jsonl")
            mission = Mission(world, services, decisions, JobHistory(logs_dir(cfg) / "jobs.jsonl"), mp, ep, pp, actp)
            g_fatigue = profile_setting(cfg, "g_fatigue")
            mission.profile_fatigue = None if g_fatigue is None else g_fatigue.strip() not in ("0", "")
            md = MissionDriver(ap, mission)
            if target is not None:
                mission.set_free_target(target)         # driven to while no job is active
            mission.update(s.reader.read_latest(), s.reader.read_config(), [])
            job = mission.status().job or "no job yet (accept one; AutoHaul picks it up)"
            fat = {None: "unknown (watching the rest timer)", True: "on", False: "off (no sleep stops)"}
            print(f"mission: {job}; fatigue {fat[mission.profile_fatigue]}; economics: {mission.econ_note}; "
                  f"decisions -> {decisions.path}")
        s.safety.set_axes(steering=lateral, pedals=longitudinal)
        s.safety.reset()
        drv = Driver(s, ap, lateral, longitudinal, places, target, fallback, mission=md)
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
            st = drv.stats
            print(f"\n\nengaged {st['engagements']}x, {st['engaged_s'] / 60:.1f} min, {st['engaged_km']:.2f} km; "
                  f"faults: {st['faults'] or 'none'}")
            if s.recording_path:
                print(f"recording: {s.recording_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
