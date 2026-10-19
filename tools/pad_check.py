"""Controller (XInput) safety check. Read-only: needs no game and sends nothing.

    python tools/pad_check.py               # waits up to 120 s for each action below
    python tools/pad_check.py --seconds 60

While it runs, on the Xbox controller:
  1. push the left stick to the LEFT, then to the RIGHT  -> must be seen as manual override
  2. squeeze the LEFT trigger, then the RIGHT trigger     -> must be seen as manual override
  3. hold BACK + START together                           -> must be seen as the kill switch
It uses the same SafetyMonitor methods the control loops use, so this verifies the real detection path.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import logs_dir  # noqa: E402
from autohaul.safety import xinput  # noqa: E402
from autohaul.safety.supervisor import SafetyMonitor  # noqa: E402

STEPS = ["stick_left", "stick_right", "left_trigger", "right_trigger", "kill_chord"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=120.0)
    args = parser.parse_args()

    cfg = load_config()
    monitor = SafetyMonitor(cfg, reader=None, keyboard=None, use_xinput=True)  # only the pad methods are used
    seen: dict[str, str] = {}
    pads_seen: set[int] = set()
    print("Controller check: stick LEFT, stick RIGHT, LEFT trigger, RIGHT trigger, then hold BACK+START.", flush=True)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end and len(seen) < len(STEPS):
        for p in xinput.poll():
            pads_seen.add(p.index)
            reason = monitor.pad_override(p)
            if reason and "stick" in reason:
                key = "stick_left" if p.left_x < 0 else "stick_right"
                seen.setdefault(key, reason)
            if reason and "trigger" in reason:
                if p.left_trigger > monitor.trigger_th:
                    seen.setdefault("left_trigger", f"controller {p.index} left trigger {p.left_trigger:.2f}")
                if p.right_trigger > monitor.trigger_th:
                    seen.setdefault("right_trigger", f"controller {p.index} right trigger {p.right_trigger:.2f}")
            reason = monitor.pad_kill(p)
            if reason:
                seen.setdefault("kill_chord", reason)
        for key in STEPS:
            if key in seen and not seen[key].startswith("  "):
                print(f"  detected {key:13s} -> {seen[key]}", flush=True)
                seen[key] = "  " + seen[key]
        time.sleep(0.02)

    results = {k: ("PASS", seen[k].strip()) if k in seen else ("FAIL", "not seen") for k in STEPS}
    print()
    if not pads_seen:
        print("FAIL  no XInput controller connected")
    for k, (status, detail) in results.items():
        print(f"{status:4s}  {k:13s}  {detail}")
    out = logs_dir(cfg) / f"padcheck-{time.strftime('%Y%m%d-%H%M%S')}.json"
    out.write_text(json.dumps({"time": time.time(), "controllers": sorted(pads_seen),
                               "results": {k: {"status": s, "detail": d} for k, (s, d) in results.items()}}, indent=2))
    print(f"\nreport: {out}")
    return 0 if pads_seen and all(s == "PASS" for s, _ in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
