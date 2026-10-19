"""Parked check: how much throttle/brake survives when AutoHaul steers in short pulses?

On 1.53.2.1 (relative steering) the game ignores AutoHaul's pedal axes in every frame where AutoHaul's
steering axis is non-zero (tools/pedal_check.py). Relative steering holds the wheel at axis 0, so steering
can be delivered as short pulses with the pedals free in between. Each phase holds throttle (or brake)
0.3 for 3 s with a steering pulse pattern (alternating direction, so the wheel stays put) and reports the
fraction of frames the game saw the pedal and the mean effective value.

    python tools/pedal_pulse_check.py --wait-s 600
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

from phase1_check import Aborted, drive  # noqa: E402
from steer_ident import recentre, wait_ready  # noqa: E402
from autohaul.infra.gameenv import active_controls_file, controls_constants  # noqa: E402
from autohaul.infra.paths import logs_dir  # noqa: E402
from autohaul.infra.session import Session  # noqa: E402
from autohaul.safety.supervisor import Command  # noqa: E402


def pattern(every: int, amp: float, pedal: str):
    state = {"n": 0}

    def fn(t, f):
        state["n"] += 1
        n = state["n"]
        steer = 0.0
        if every and n % every == 0:
            steer = amp if (n // every) % 2 else -amp          # alternate: the wheel doesn't wander
        return Command(steer=steer, throttle=0.3 if pedal == "throttle" else 0.0, brake=0.3 if pedal == "brake" else 0.0)
    return fn


PHASES = [
    ("no steering (reference)", 0, 0.0, "throttle"),
    ("1 pulse every 6 frames", 6, 0.3, "throttle"),
    ("1 pulse every 3 frames", 3, 0.3, "throttle"),
    ("1 pulse every 2 frames", 2, 0.3, "throttle"),
    ("tiny constant 0.003", -1, 0.003, "throttle"),
    ("brake, 1 pulse every 6 frames", 6, 0.3, "brake"),
    ("brake, no steering (reference)", 0, 0.0, "brake"),
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wait-s", type=float, default=600.0)
    args = parser.parse_args()
    with Session("pedal-pulse-check", controls=True, record=True) as s:
        if not s.connect(10):
            print("game not running")
            return 1
        prior = controls_constants(active_controls_file(s.cfg)).get("c_rsteersens", 0.42)
        s.safety.set_axes(steering=True, pedals=True)
        if not wait_ready(s, args.wait_s):
            print("gave up waiting; nothing was sent to the game")
            return 1
        results = []
        try:
            for name, every, amp, pedal in PHASES:
                fn = (lambda t, f, a=amp, p=pedal: Command(steer=-a, throttle=0.3 if p == "throttle" else 0.0,
                                                           brake=0.3 if p == "brake" else 0.0)) if every < 0 \
                    else pattern(every, amp, pedal)
                trace = drive(s, 3.0, fn)
                tail = [fr for t, fr, _ in trace if t > 1.0]
                inp = np.array([getattr(fr, f"input_{pedal}") for fr in tail])
                eff = np.array([getattr(fr, f"effective_{pedal}") for fr in tail])
                seen = float(np.mean(inp > 0.2)) if len(inp) else float("nan")
                row = {"phase": name, "pedal": pedal, "frames": len(inp), "pedal_seen_fraction": round(seen, 3),
                       "input_mean": round(float(inp.mean()), 3) if len(inp) else None,
                       "effective_mean": round(float(eff.mean()), 3) if len(eff) else None,
                       "rpm_mean": round(float(np.mean([fr.engine_rpm for fr in tail])), 0) if tail else None}
                results.append(row)
                print(f"  {name:32s} game saw the {pedal} in {seen * 100:5.1f}% of frames; input mean {row['input_mean']:.2f}, "
                      f"effective mean {row['effective_mean']:.2f}, rpm {row['rpm_mean']:.0f}")
            recentre(s, prior)
        except Aborted as exc:
            print(f"\nstopped: {exc.verdict.reason}")
        path = logs_dir(s.cfg) / f"pedal-pulse-check-{time.strftime('%Y%m%d-%H%M%S')}.json"
        path.write_text(json.dumps(results, indent=2))
        print(f"report: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
