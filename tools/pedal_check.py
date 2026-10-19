"""Does the game take AutoHaul's throttle/brake while AutoHaul also steers? (parked check)

The truck must be PARKED with the PARKING BRAKE ON (the engine only revs). Each phase holds a pedal for
2 s with a different steering pattern and records what the game reports as its pedal input:
    A  throttle 0.3, steering 0
    B  throttle 0.3, steering held at a small constant value
    C  throttle 0.3, steering changing every frame (as the lane-keeping servo does)
    D  throttle 0.3, steering 0 again
    E  brake 0.3, steering changing every frame
Your controller's trigger readings are logged too (keep your hands off it). The wheel is re-centred at the end.

    python tools/pedal_check.py --wait-s 300
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
from autohaul.safety import xinput  # noqa: E402
from autohaul.safety.supervisor import Command  # noqa: E402


def phases():
    alt = lambda t, f: 0.05 if int(t * 60) % 2 else -0.05          # noqa: E731  (changes every frame)
    return [
        ("A throttle, steering 0", lambda t, f: Command(throttle=0.3), "input_throttle"),
        ("B throttle, steering constant", lambda t, f: Command(steer=-0.05, throttle=0.3), "input_throttle"),
        ("C throttle, steering changing", lambda t, f: Command(steer=alt(t, f), throttle=0.3), "input_throttle"),
        ("D throttle, steering 0 again", lambda t, f: Command(throttle=0.3), "input_throttle"),
        ("E brake, steering changing", lambda t, f: Command(steer=alt(t, f), brake=0.3), "input_brake"),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wait-s", type=float, default=300.0)
    args = parser.parse_args()
    with Session("pedal-check", controls=True, record=True) as s:
        if not s.connect(args.wait_s):              # waits for the game to start, too
            print("game not running")
            return 1
        prior = controls_constants(active_controls_file(s.cfg)).get("c_rsteersens", 0.42)
        s.safety.set_axes(steering=True, pedals=True)
        if not wait_ready(s, args.wait_s):
            print("gave up waiting; nothing was sent to the game")
            return 1
        results = []
        h0 = s.reader.header()
        c0 = (h0.input_frames, h0.input_calls, h0.input_events_delivered, h0.input_events_requeued)
        try:
            for name, fn, field in phases():
                pads = []
                trace = drive(s, 2.0, lambda t, f, fn=fn: (pads.append(xinput.poll()), fn(t, f))[1])
                tail = [fr for t, fr, _ in trace if t > 1.0]
                got = float(np.mean([getattr(fr, field) for fr in tail])) if tail else float("nan")
                eff = float(np.mean([getattr(fr, field.replace("input_", "effective_")) for fr in tail])) if tail else float("nan")
                trig = [(p.left_trigger, p.right_trigger) for ps in pads for p in ps]
                lt = max((x[0] for x in trig), default=0.0)
                rt = max((x[1] for x in trig), default=0.0)
                results.append({"phase": name, "field": field, "sent": 0.3, "game_input": round(got, 3),
                                "game_effective": round(eff, 3), "pad_lt_max": round(lt, 2), "pad_rt_max": round(rt, 2)})
                print(f"  {name:32s} sent 0.30 -> game {field} {got:.2f} (effective {eff:.2f})   pad LT/RT max {lt:.2f}/{rt:.2f}")
            recentre(s, prior)
        except Aborted as exc:
            print(f"\nstopped: {exc.verdict.reason}")
        h1 = s.reader.header()
        frames, calls, delivered, requeued = (h1.input_frames - c0[0], h1.input_calls - c0[1],
                                              h1.input_events_delivered - c0[2], h1.input_events_requeued - c0[3])
        print(f"bridge counters: {frames} input frames, {calls} callback calls ({calls / max(frames, 1):.2f}/frame), "
              f"{delivered} events delivered, {requeued} offered but not fetched in their frame")
        results.append({"input_frames": frames, "callback_calls": calls, "delivered": delivered, "requeued": requeued})
        path = logs_dir(s.cfg) / f"pedal-check-{time.strftime('%Y%m%d-%H%M%S')}.json"
        path.write_text(json.dumps(results, indent=2))
        print(f"report: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
