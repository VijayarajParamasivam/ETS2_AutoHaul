"""Measure how the game turns the wheel for AutoHaul's steering axis in RELATIVE steering mode.

The truck must be PARKED with the PARKING BRAKE ON. AutoHaul turns the wheel at several small and
medium rates (both directions) for about a second each, re-centring in between, and fits
    wheel rate = gain x (|axis| - deadzone) / (1 - deadzone)
A deadzone matters: small lane-keeping corrections below it would do nothing. Kill key and your
driving keys stop it at once, as in Phase 1.

    python tools/steer_ident.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from phase1_check import Aborted, countdown, drive, latest  # noqa: E402
from autohaul.control.steering import RelativeServo  # noqa: E402
from autohaul.infra.gameenv import active_controls_file, controls_constants, steering_mode  # noqa: E402
from autohaul.infra.paths import logs_dir  # noqa: E402
from autohaul.infra.session import Session  # noqa: E402
from autohaul.io.win32 import foreground_pid  # noqa: E402
from autohaul.safety.supervisor import Command, Verdict  # noqa: E402

LEVELS = [0.04, 0.08, 0.12, 0.16, 0.20, 0.30, 0.50]


def recentre(s: Session, gain: float) -> float:
    servo = RelativeServo(gain=gain, adapt=False)
    last = [0.0]

    def fn(t, f):
        last[0] = f.effective_steering
        return Command(steer=servo.step(0.0, f.effective_steering, 1 / 60) if abs(f.effective_steering) > 0.004 else 0.0)

    drive(s, 8.0, fn, until=lambda t, f: abs(f.effective_steering) < 0.004 and t > 0.3)
    return last[0]


def fit(cmds: np.ndarray, rates: np.ndarray) -> tuple[float, float, float]:
    """(gain, deadzone, rms error) minimizing squared error over a deadzone grid."""
    best = None
    for dz in np.arange(0.0, 0.31, 0.01):
        x = np.maximum(cmds - dz, 0.0) / (1 - dz)
        g = float((x * rates).sum() / max((x * x).sum(), 1e-9))
        err = float(np.sqrt(np.mean((g * x - rates) ** 2)))
        if best is None or err < best[2] - 1e-6:
            best = (g, float(dz), err)
    return best


def not_ready(s: Session, f) -> list[str]:
    problems = []
    if abs(f.speed) > 0.5 or not f.parking_brake:
        problems.append("parking brake on, truck standing still")
    if f.paused:
        problems.append("game unpaused")
    if foreground_pid() != s.reader.header().game_pid:
        problems.append("game window focused")
    return problems


def wait_ready(s: Session, timeout_s: float) -> bool:
    """Start by itself once the truck is parked and the game has focus (no console needed)."""
    end = time.time() + timeout_s
    last = None
    while time.time() < end:
        f = s.reader.wait_frame(200)
        if f is None:
            continue
        missing = not_ready(s, f)
        if missing != last:
            print("waiting for: " + ", ".join(missing) if missing else "ready: starting in 2 s", flush=True)
            last = missing
        if not missing:
            time.sleep(2.0)
            return not not_ready(s, latest(s))
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wait-s", type=float, default=0.0,
                        help="instead of ENTER + countdown, start once parked and the game has focus (wait up to N s)")
    args = parser.parse_args()
    with Session("steer-ident", controls=True, record=True) as s:
        if not s.connect(10):
            print("game not running")
            return 1
        if steering_mode(s.cfg) != "relative":
            print("Can't run: this test is for RELATIVE steering mode (c_relatsteer 1)")
            return 1
        prior = controls_constants(active_controls_file(s.cfg)).get("c_rsteersens", 0.42)
        s.safety.set_axes(steering=True, pedals=False)
        if args.wait_s:
            if not wait_ready(s, args.wait_s):
                print("gave up waiting; nothing was sent to the game")
                return 1
        else:
            missing = not_ready(s, latest(s))
            missing = [m for m in missing if m != "game window focused"]
            if missing:
                print("Can't run, needs: " + ", ".join(missing))
                return 1
            countdown("STEERING RATE TEST: the wheel turns left and right at different speeds for ~1 s each, "
                      "re-centring in between (about 40 s).")
        rows = []
        try:
            recentre(s, prior)
            for level in LEVELS:
                for sign in (1, -1):
                    if foreground_pid() != s.reader.header().game_pid:
                        raise Aborted(Verdict(False, "the game window lost focus", "override", True))
                    trace = drive(s, 1.2, lambda t, f, v=sign * level: Command(steer=v))
                    pts = [(t, fr.effective_steering) for t, fr, _ in trace if t > 0.3]
                    if len(pts) > 10:
                        t_, e_ = np.array(pts).T
                        slope = float(np.polyfit(t_, e_, 1)[0])
                        rate = -slope / sign                       # + = turning the way the axis asks
                        rows.append({"axis": sign * level, "rate": rate})
                        print(f"  axis {sign * level:+.2f}: wheel rate {rate:+.4f} units/s ({rate / level:+.3f} per unit)")
                    recentre(s, prior)
        except Aborted as exc:
            print(f"\nstopped: {exc.verdict.reason}")
            if not rows:
                return 1
        cmds = np.array([abs(r["axis"]) for r in rows])
        rates = np.array([max(r["rate"], 0.0) for r in rows])
        gain, dz, err = fit(cmds, rates)
        out = {"time": time.time(), "rows": rows, "gain": round(gain, 4), "deadzone": round(dz, 3),
               "rms_error": round(err, 5), "c_rsteersens": prior}
        print(f"\nfit: wheel rate = {gain:.3f} x (|axis| - {dz:.2f}) / (1 - {dz:.2f})   (rms error {err:.4f})"
              f"\nc_rsteersens in your controls.sii: {prior}")
        if dz >= 0.02:
            print(f"\n-> add to config/local.toml:\n[drive]\nrelative_deadzone = {dz:.2f}")
        else:
            print("\n-> no deadzone on AutoHaul's axis: nothing to configure")
        path = logs_dir(s.cfg) / f"steer-ident-{time.strftime('%Y%m%d-%H%M%S')}.json"
        path.write_text(json.dumps(out, indent=2))
        print(f"report: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
