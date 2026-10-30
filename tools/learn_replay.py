"""Run the learner (autohaul/learn) over recorded drives, without driving anything.

    python tools/learn_replay.py 20261029 --dry     # what it would learn from drives since then; nothing saved
    python tools/learn_replay.py 20261029           # ... and start from that (only if nothing is learned yet, or --add)
    python tools/learn_replay.py --report           # what's learned now, and the scorecards
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import cache_dir, map_dir, recordings_dir  # noqa: E402
from autohaul.io.recorder import KIND_EVENT, KIND_FRAME, KIND_NOTE, RecordingReader  # noqa: E402
from autohaul.learn.learner import Learner  # noqa: E402
from autohaul.world.lanemap import LaneMap  # noqa: E402


MEASURED = ("thr_accel", "brake_decel", "c_us")


def replay(learner: Learner, path: Path) -> None:
    frame, engaged, lane = None, False, None
    seen = 0.0
    for r in RecordingReader(path):
        if r.kind == KIND_FRAME:
            frame = r.frame()
            if not engaged and not frame.paused and r.t - seen >= 0.1:
                seen = r.t
                learner.observe_driver(r.t, frame)
        elif r.kind == KIND_EVENT and frame is not None:
            ev = r.json()
            if ev.get("id") == "player.fined":
                learner.on_fine(ev.get("attrs", {}).get("fine.offence", "?"), float(frame.speed), lane)
            elif ev.get("id") == "player.tollgate.paid" and not engaged:
                learner.on_toll_paid(frame)
        elif r.kind == KIND_NOTE and frame is not None:
            n = r.json()
            e = n.get("event")
            if e == "engaged":
                engaged = True
                learner.end_lesson(frame)
            elif e == "disengaged":
                engaged = False
                reason = n.get("reason", "")
                learner.on_handback(reason, float(frame.speed), lane)
                if not reason.startswith(("game paused", "AutoHaul stopped")):
                    learner.start_lesson(r.t, frame, reason)
            elif e == "ap" and engaged:
                if n.get("lane") is not None:
                    lane = n["lane"]
                trailers = getattr(frame, "trailers", None)
                learner.clock = lambda t=r.t: t
                learner.observe(r.t, frame, n, True, bool(trailers and trailers[0].connected))


def report(learner: Learner) -> None:
    print("values:", json.dumps(learner.values, indent=1))
    hot = {k: v for k, v in learner.spots.items() if v["n"] >= learner.p.spot_slow_after}
    print(f"trouble spots: {len(learner.spots)} seen, {len(hot)} slowing")
    for k, v in sorted(hot.items(), key=lambda kv: -kv[1]["n"])[:12]:
        print(f"  {k}: {v['n']:.0f}x at {v['speed'] * 3.6:.0f} km/h ({v['why']})")
    print(f"route corrections: {len(learner.route_cost)}", dict(list(learner.route_cost.items())[:8]))
    print(f"lessons from you: {len(learner.lessons)} lanes; toll booths: {learner.toll_lanes}")
    for k, v in sorted(learner.lessons.items(), key=lambda kv: -kv[1]["n"])[:12]:
        speeds = [row[0] for row in v["bins"] if row[0] is not None]
        lats = [row[1] for row in v["bins"] if row[1] is not None]
        print(f"  {k}: {v['n']}x, {len(v['bins'])} bins, " + (f"{min(speeds) * 3.6:.0f}-{max(speeds) * 3.6:.0f} km/h, " if speeds else "no speed, ")
              + (f"line {min(lats):+.1f}..{max(lats):+.1f} m" if lats else "") + f" ({v['why']})")
    print("scorecards:")
    for c in learner.scorecards[-12:]:
        print(" ", c)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("since", nargs="?", help="recordings from this date on: 20261029 or 20261029-132607")
    ap.add_argument("--dry", action="store_true", help="don't save")
    ap.add_argument("--add", action="store_true", help="add to what's learned already")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--lessons", action="store_true", help="only add what you showed it (lessons, toll booths) to what's learned")
    args = ap.parse_args()
    cfg = load_config()
    store = cache_dir(cfg) / "learned" / "learned.json"
    lm = LaneMap.load_or_build(map_dir(cfg), log=lambda *a: None)
    if args.report:
        report(Learner(store, lm))
        return 0
    if not args.since:
        ap.error("give a date, or --report")
    if store.exists() and not (args.add or args.dry or args.lessons):
        print(f"{store} exists: --add to add to it")
        return 1
    learner = Learner(None if args.dry or args.lessons else store, lm)
    learner.p.gps_rise_m = float("inf")      # those drives followed an older router: no route lessons from them
    for path in sorted(recordings_dir(cfg).glob("*-drive.ahrec")):
        if path.stem >= args.since and path.stat().st_size > 500_000:
            replay(learner, path)
            print(path.name, learner.end_session() or "(too short for a scorecard)")
    if args.lessons:
        keep = Learner(store, lm)
        keep.lessons.update(learner.lessons)
        keep.toll_lanes.update(learner.toll_lanes)
        keep.save()
        report(keep)
        return 0
    # a recording can't show what a changed gain would have done: the nudges just pile up. Keep what it really
    # measured about the truck; the steering and braking gains are learned on the road, where each change is tried.
    for ctx in learner.values.values():
        for name in [n for n in ctx if n not in MEASURED]:
            del ctx[name]
    if not args.dry:
        learner.save()
    report(learner)
    return 0


if __name__ == "__main__":
    sys.exit(main())
