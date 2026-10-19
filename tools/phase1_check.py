"""Phase 1 in-game checks. The truck must be PARKED with the PARKING BRAKE ON.

    python tools/phase1_check.py preflight   # read-only: connection, parked, single-player
    python tools/phase1_check.py wiggle      # steering sine, blinkers, hazards, lights, wipers, horn, pedals
    python tools/phase1_check.py kill        # you press the kill key during a steering sine; measures release
    python tools/phase1_check.py watchdog    # AutoHaul stops its heartbeat on purpose; measures release
    python tools/phase1_check.py override    # you touch the stick / a driving key; measures hand-back
    python tools/phase1_check.py calibrate   # steering command -> input / effective / wheel angle
    python tools/phase1_check.py all

Every test sends nothing until you press ENTER, then counts down 5 s so you can switch to the
game window. Any safety trip (kill key, your input, pause, stale telemetry, multiplayer) releases
all controls immediately. Results: <cache>/logs/phase1-*.json, recording in <cache>/recordings/.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.paths import cache_dir, logs_dir  # noqa: E402
from autohaul.infra.session import Session  # noqa: E402
from autohaul.io import _layout as L  # noqa: E402
from autohaul.io.telemetry import text  # noqa: E402
from autohaul.io.win32 import qpc  # noqa: E402
from autohaul.safety.keys import VK_NAME  # noqa: E402
from autohaul.safety.mp_guard import multiplayer_reasons  # noqa: E402
from autohaul.safety.supervisor import Command, Verdict  # noqa: E402

PARKED_SPEED = 0.5  # m/s


@dataclass
class Result:
    test: str
    status: str = "PASS"
    details: list[str] = field(default_factory=list)
    data: dict = field(default_factory=dict)

    def fail(self, msg: str) -> None:
        self.status = "FAIL"
        self.details.append(msg)

    def warn(self, msg: str) -> None:
        if self.status == "PASS":
            self.status = "WARN"
        self.details.append(msg)

    def info(self, msg: str) -> None:
        self.details.append(msg)


class Aborted(Exception):
    def __init__(self, verdict: Verdict):
        super().__init__(verdict.reason)
        self.verdict = verdict


class StaleGuard:
    """Stale telemetry is tolerated briefly (autosave stutters freeze the game too); abort if it lasts."""

    def __init__(self, s: Session):
        self.limit_s = s.cfg.get("safety.telemetry_stale_abort_ms", 1000) / 1000.0
        self.since: float | None = None

    def must_abort(self, verdict: Verdict) -> bool:
        if verdict.ok:
            self.since = None
            return False
        if verdict.kind != "stale":
            return True
        now = time.monotonic()
        self.since = self.since or now
        return now - self.since > self.limit_s


# ---------------------------------------------------------------- helpers

def latest(s: Session) -> L.TelemetryFrame:
    f = s.reader.wait_frame(500)
    if f is None:
        f = s.reader.read_latest()
    if f is None:
        raise RuntimeError("no telemetry frames: is the game running and not in the main menu?")
    return f


def preflight(s: Session, r: Result) -> bool:
    f = latest(s)
    hdr = s.reader.header()
    tv = hdr.telemetry_game_version
    r.info(f"game {text(hdr.game_id)}, telemetry game version {tv >> 16}.{tv & 0xFFFF}, bridge {text(hdr.bridge_version)}")
    if not hdr.input_registered:
        r.fail("the bridge's input device is not registered (check game.log.txt for [autohaul] errors)")
    if f.paused:
        r.fail("game is paused / in a menu: return to driving")
    if abs(f.speed) > PARKED_SPEED:
        r.fail(f"truck is moving ({f.speed * 3.6:.1f} km/h): stop and engage the parking brake")
    if not f.parking_brake:
        r.fail("parking brake is off: engage it (Space by default)")
    mp = multiplayer_reasons(f, hdr.game_pid, s.cfg.get("safety.mp_guard_modules"))
    if mp:
        r.fail("multiplayer detected: " + "; ".join(mp))
    if hdr.bridge_flags & L.BRIDGE_FLAG_KILL_LATCHED:
        r.info("bridge kill latch is set from earlier; it is re-armed when a test starts")
    if s.keyboard is None:
        r.warn("keyboard hook unavailable: kill keys still work inside the game (DLL), but not in Python")
    return r.status != "FAIL"


def countdown(msg: str) -> None:
    input(f"\n{msg}\nPress ENTER to start, then switch to the game window ... ")
    for i in range(5, 0, -1):
        print(f"  starting in {i} ...", end="\r", flush=True)
        time.sleep(1)
    print(" " * 30, end="\r")


def drive(s: Session, duration_s: float, fn: Callable[[float, L.TelemetryFrame], Command],
          actions: Callable[[float, L.TelemetryFrame], None] | None = None,
          until: Callable[[float, L.TelemetryFrame], bool] | None = None) -> list[tuple[float, L.TelemetryFrame, Command]]:
    """Run a control function every frame under the SafetyMonitor. Raises Aborted on any safety trip."""
    ctl = s.controls
    assert ctl is not None
    ctl.rearm()
    s.safety.reset()
    ctl.set_enabled(True)
    trace = []
    guard = StaleGuard(s)
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            if t > duration_s:
                break
            f = s.reader.wait_frame(100)
            if f is None:
                f = s.reader.read_latest()
            cmd = fn(t, f) if f is not None else Command()
            ctl.steer(cmd.steer)
            ctl.throttle(cmd.throttle)
            ctl.brake(cmd.brake)
            if actions and f is not None:
                actions(t, f)
            ctl.commit()
            verdict = s.safety.check(f, cmd)
            if s.recorder and f is not None:
                s.recorder.frame(f)
                s.recorder.controls(ctl.snapshot())
            if guard.must_abort(verdict):
                ctl.release()
                raise Aborted(verdict)
            if f is not None:
                trace.append((t, f, cmd))
                if until and until(t, f):
                    break
    finally:
        ctl.release()
    return trace


def fit_gain(xs: list[float], ys: list[float]) -> float:
    sxx = sum(x * x for x in xs)
    return sum(x * y for x, y in zip(xs, ys)) / sxx if sxx else float("nan")


def wait_field(s: Session, pred: Callable[[L.TelemetryFrame], bool], timeout_s: float = 1.5,
               cmd: Command | None = None) -> bool:
    """Keep the heartbeat alive (and the safety monitor running) until pred(frame) or timeout.
    `cmd` must match the axes currently commanded, for the telemetry override cross-check."""
    end = time.monotonic() + timeout_s
    guard = StaleGuard(s)
    while time.monotonic() < end:
        assert s.controls is not None
        s.controls.commit()
        f = s.reader.wait_frame(50)
        verdict = s.safety.check(f, cmd or Command())
        if guard.must_abort(verdict):
            s.controls.release()
            raise Aborted(verdict)
        if f is not None and pred(f):
            return True
    return False


def sine(amplitude: float, period: float) -> Callable[[float, L.TelemetryFrame], Command]:
    return lambda t, f: Command(steer=amplitude * math.sin(2 * math.pi * t / period))


# ---------------------------------------------------------------- tests

def test_wiggle(s: Session, r: Result) -> None:
    w = s.cfg.section("tools.wiggle")
    countdown("WIGGLE: AutoHaul will turn the steering wheel left and right, then toggle blinkers, hazards, "
              "lights, wipers, sound the horn and touch the pedals.")
    trace = drive(s, w["duration_s"], sine(w["steering_amplitude"], w["steering_period_s"]))
    cmds = [c.steer for _, _, c in trace[5:]]
    ins = [f.input_steering for _, f, _ in trace[5:]]
    gain = fit_gain(cmds, ins)
    r.data["steering_input_gain"] = gain
    expected = s.cfg.get("safety.steering_input_gain")
    if math.isnan(gain) or abs(gain - expected) > 0.25:
        r.fail(f"truck.input.steering / command = {gain:.3f}, expected about {expected} - the game did not follow")
    else:
        r.info(f"steering follows the command: input gain {gain:+.3f} (expected {expected:+.1f})")
    wheel = max((abs(f.wheel_steering[0]) for _, f, _ in trace), default=0.0) * 360
    r.info(f"front wheel max angle {wheel:.1f}°")

    ctl = s.controls
    assert ctl is not None
    ctl.rearm()
    s.safety.reset()
    ctl.set_enabled(True)
    ctl.commit()

    def toggle(name: str, field_name: str, label: str) -> None:
        f0 = latest(s)
        before = getattr(f0, field_name)
        ctl.press(name)
        ctl.commit()
        if not wait_field(s, lambda f: getattr(f, field_name) != before):
            r.fail(f"{label}: no change after pressing '{name}'")
            return
        ctl.press(name)
        ctl.commit()
        if wait_field(s, lambda f: getattr(f, field_name) == before):
            r.info(f"{label}: toggled on and back off")
        else:
            r.warn(f"{label}: changed but did not return to the original state")

    try:
        toggle("lblinker", "lblinker", "left blinker")
        toggle("rblinker", "rblinker", "right blinker")
        toggle("flasher4way", "hazard_warning", "hazard lights")

        f0 = latest(s)
        start = (f0.light_parking, f0.light_low_beam)
        ctl.press("light")
        ctl.commit()
        if wait_field(s, lambda f: (f.light_parking, f.light_low_beam) != start):
            r.info("lights: 'light' cycles the headlights")
        else:
            r.fail("lights: no change after pressing 'light'")
        for _ in range(4):
            if (latest(s).light_parking, latest(s).light_low_beam) == start:
                break
            ctl.press("light")
            ctl.commit()
            wait_field(s, lambda f: False, 0.8)
        if (latest(s).light_parking, latest(s).light_low_beam) != start:
            r.warn("lights: could not restore the original light state - check them manually")

        f0 = latest(s)
        ctl.press("wipers1" if not f0.wipers else "wipers0")
        ctl.commit()
        if wait_field(s, lambda f: f.wipers != f0.wipers):
            r.info("wipers: direct position input works")
            ctl.press("wipers0" if not f0.wipers else "wipers1")
            ctl.commit()
            wait_field(s, lambda f: f.wipers == f0.wipers)
        else:
            r.fail("wipers: no change")

        ctl.hold("horn")
        wait_field(s, lambda f: False, 0.4)
        ctl.hold("horn", False)
        ctl.commit()
        r.info("horn: held 0.4 s (no telemetry for the horn: listen for it)")

        for axis, field_name, value in (("abackward", "input_brake", 0.3), ("aforward", "input_throttle", 0.2)):
            held = Command(brake=value) if axis == "abackward" else Command(throttle=value)
            ctl.axis(axis, value)
            ctl.commit()
            ok = wait_field(s, lambda f, fn=field_name, v=value: abs(getattr(f, fn) - v) < 0.05, 1.0, held)
            ctl.axis(axis, 0.0)
            ctl.commit()
            wait_field(s, lambda f, fn=field_name: getattr(f, fn) < 0.05, 1.0, held)
            (r.info if ok else r.fail)(f"{axis}: {'input follows' if ok else 'input did not reach'} {value}")
    finally:
        ctl.release()


def kill_vk_press_qpc(s: Session) -> int:
    if not s.keyboard:
        return 0
    return max((s.keyboard.state.last_press_qpc(v) for v in s.safety.kill_vks), default=0)


def test_kill(s: Session, r: Result) -> None:
    countdown("KILL SWITCH: AutoHaul will steer left and right for up to 30 s. "
              f"Press your kill key ({' or '.join(s.cfg.get('safety.kill_keys'))}) in the game at any moment.")
    try:
        drive(s, 30, sine(0.5, 3.0))
        r.fail("no kill key press seen within 30 s")
        return
    except Aborted as a:
        if a.verdict.kind != "kill":
            r.fail(f"stopped by {a.verdict.kind}: {a.verdict.reason}")
            return
        r.info(f"kill detected: {a.verdict.reason}")
    press = kill_vk_press_qpc(s)
    # Python already released; measure when the game's steering input reached zero.
    t_zero = None
    end = time.monotonic() + 2
    while time.monotonic() < end:
        f = s.reader.wait_frame(100)
        if f is not None and abs(f.input_steering) < 0.02:
            t_zero = f.publish_qpc
            break
    hdr = s.reader.header()
    if press and t_zero:
        ms = (t_zero - press) * 1000 / s.reader.qpc_freq
        r.data["kill_to_zero_ms"] = ms
        (r.info if ms < 100 else r.warn)(f"steering input back to 0 {ms:.0f} ms after the key press")
    if hdr.bridge_flags & L.BRIDGE_FLAG_KILL_LATCHED:
        r.info(f"the DLL latched the kill itself (kill count {hdr.kill_count}) - independent of Python")
    else:
        r.warn("the DLL did not latch the kill itself (the game may not have been polling its input device at "
               "that moment). Python's hook caught it, so the truck was still released.")


def test_watchdog(s: Session, r: Result) -> None:
    countdown("WATCHDOG: AutoHaul steers for 3 s, then deliberately stops sending heartbeats (simulated crash). "
              "The bridge must release the steering by itself.")
    ctl = s.controls
    assert ctl is not None
    trips0 = s.reader.header().watchdog_trips
    drive(s, 3.0, lambda t, f: Command(steer=0.4))
    # drive() released at the end; re-apply a hold without further commits to simulate the hang.
    ctl.rearm()
    ctl.set_enabled(True)
    ctl.steer(0.4)
    ctl.commit()
    hang = qpc()
    t_zero = None
    end = time.monotonic() + 2.0
    saw_steer = False
    while time.monotonic() < end:
        f = s.reader.wait_frame(100)
        if f is None:
            continue
        if abs(f.input_steering) > 0.2:
            saw_steer = True
        if saw_steer and abs(f.input_steering) < 0.02:
            t_zero = f.publish_qpc
            break
    ctl.release()
    hdr = s.reader.header()
    if t_zero is None:
        r.fail("steering was not released within 2 s of the heartbeat stopping")
        return
    ms = (t_zero - hang) * 1000 / s.reader.qpc_freq
    r.data["watchdog_release_ms"] = ms
    limit = hdr.watchdog_ms + 60
    (r.info if ms <= limit else r.fail)(f"released {ms:.0f} ms after the last heartbeat (watchdog {hdr.watchdog_ms} ms)")
    if hdr.watchdog_trips <= trips0:
        r.warn("watchdog trip counter did not increase")


def test_override(s: Session, r: Result) -> None:
    countdown("MANUAL OVERRIDE: AutoHaul steers gently for up to 30 s. Take over by moving the left stick, "
              "pressing a trigger, or pressing W/A/S/D.")
    try:
        drive(s, 30, sine(0.3, 4.0))
        r.fail("no manual input detected within 30 s")
    except Aborted as a:
        if a.verdict.kind == "override":
            r.info(f"control handed back: {a.verdict.reason}")
            if s.keyboard and "driving key" in (a.verdict.reason or ""):
                name = a.verdict.reason.split()[-1]
                vk = next((k for k, v in VK_NAME.items() if v == name), None)
                if vk:
                    ms = (qpc() - s.keyboard.state.last_press_qpc(vk)) * 1000 / s.reader.qpc_freq
                    r.data["override_detect_ms"] = ms
        else:
            r.fail(f"stopped by {a.verdict.kind}: {a.verdict.reason}")


def test_calibrate(s: Session, r: Result) -> None:
    c = s.cfg.section("tools.calibrate_steering")
    countdown("STEERING CALIBRATION: AutoHaul holds the wheel at several positions for a few seconds each.")
    cfg = s.reader.read_config()
    steerable = [i for i in range(cfg.truck.wheel_count) if cfg.truck.wheel_steerable[i]] if cfg else [0, 1]
    rows = []
    for level in c["levels"]:
        trace = drive(s, c["settle_s"] + 0.5, lambda t, f, v=level: Command(steer=v))
        tail = [f for t, f, _ in trace if t > c["settle_s"]]
        if not tail:
            continue
        inp = sum(f.input_steering for f in tail) / len(tail)
        eff = sum(f.effective_steering for f in tail) / len(tail)
        wheel = sum(sum(f.wheel_steering[i] for i in steerable) / max(1, len(steerable)) for f in tail) / len(tail) * 360
        rows.append({"command": level, "input": inp, "effective": eff, "wheel_deg": wheel})
        print(f"  command {level:+.2f}  input {inp:+.3f}  effective {eff:+.3f}  wheel {wheel:+6.2f}°")
    if not rows:
        r.fail("no samples")
        return
    xs = [row["command"] for row in rows]
    gains = {k: fit_gain(xs, [row[k] for row in rows]) for k in ("input", "effective", "wheel_deg")}
    r.data.update({"rows": rows, "gains": gains})
    r.info(f"input gain {gains['input']:+.3f}, effective gain {gains['effective']:+.3f}, "
           f"wheel {gains['wheel_deg']:+.2f}°/unit")
    out = cache_dir(s.cfg) / "calibration"
    out.mkdir(parents=True, exist_ok=True)
    (out / "steering.json").write_text(json.dumps({"time": time.time(), "rows": rows, "gains": gains}, indent=2))
    r.info(f"saved {out / 'steering.json'}")
    if abs(gains["input"] - s.cfg.get("safety.steering_input_gain")) > 0.1:
        r.warn(f"set safety.steering_input_gain = {gains['input']:.3f} in config/local.toml")


TESTS = {"wiggle": test_wiggle, "kill": test_kill, "watchdog": test_watchdog, "override": test_override,
         "calibrate": test_calibrate}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("test", choices=["preflight", *TESTS, "all"])
    args = parser.parse_args()

    names = list(TESTS) if args.test == "all" else ([] if args.test == "preflight" else [args.test])
    results: list[Result] = []
    with Session("phase1", controls=bool(names), record=bool(names)) as s:
        s.connect()
        pre = Result("preflight")
        ok = preflight(s, pre)
        results.append(pre)
        print(f"preflight: {pre.status}")
        for d in pre.details:
            print(f"  - {d}")
        if ok:
            for name in names:
                r = Result(name)
                try:
                    TESTS[name](s, r)
                except Aborted as a:
                    r.fail(f"aborted by safety: {a.verdict.kind}: {a.verdict.reason}")
                except KeyboardInterrupt:
                    r.fail("interrupted")
                    results.append(r)
                    break
                results.append(r)
                print(f"{name}: {r.status}")
                for d in r.details:
                    print(f"  - {d}")
                if name != names[-1]:
                    if not preflight(s, Result("recheck")):
                        print("stopping: the truck is no longer parked safely")
                        break
        rec = s.recording_path
    report = {"time": time.time(), "results": [r.__dict__ for r in results], "recording": str(rec) if rec else None}
    out = logs_dir(s.cfg) / f"phase1-{time.strftime('%Y%m%d-%H%M%S')}.json"
    out.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
    print(f"\nreport: {out}")
    if rec:
        print(f"recording: {rec}")
    return 1 if any(r.status == "FAIL" for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
