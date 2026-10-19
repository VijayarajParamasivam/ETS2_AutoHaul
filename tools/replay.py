"""Inspect or export an AutoHaul recording (.ahrec).

    python tools/replay.py                               # summary of the newest recording
    python tools/replay.py path\\to\\run.ahrec            # summary of a given recording
    python tools/replay.py run.ahrec --csv out.csv       # export frames (default fields below)
    python tools/replay.py run.ahrec --csv out.csv --fields speed,input_steering,truck_placement.x

Later phases feed recordings through the planners via autohaul.io.recorder.RecordingReader.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import recordings_dir  # noqa: E402
from autohaul.infra.stats import Rolling  # noqa: E402
from autohaul.io import recorder as R  # noqa: E402

DEFAULT_FIELDS = ["frame_index", "game_time", "speed", "navigation_speed_limit", "input_steering", "effective_steering",
                  "input_throttle", "input_brake", "engine_rpm", "displayed_gear", "fuel", "next_rest_stop",
                  "truck_placement.x", "truck_placement.z", "truck_placement.heading", "paused"]


def get(obj, dotted: str):
    for part in dotted.split("."):
        if "[" in part:
            name, idx = part[:-1].split("[")
            obj = getattr(obj, name)[int(idx)]
        else:
            obj = getattr(obj, part)
    return obj


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", nargs="?")
    parser.add_argument("--csv")
    parser.add_argument("--fields", default=",".join(DEFAULT_FIELDS))
    args = parser.parse_args()

    if args.path:
        path = Path(args.path)
    else:
        files = sorted(recordings_dir(load_config()).glob("*.ahrec"), key=lambda p: p.stat().st_mtime)
        if not files:
            print("no recordings found")
            return 1
        path = files[-1]
    rec = R.RecordingReader(path)
    print(f"{path}\nmeta: {rec.meta}")

    kinds = Counter()
    intervals = Rolling(10**7)
    gaps = 0
    first = last = None
    prev_idx = None
    prev_t = None
    speed_max = 0.0
    events = []
    writer = None
    fields = args.fields.split(",")
    out = open(args.csv, "w", newline="") if args.csv else None
    if out:
        writer = csv.writer(out)
        writer.writerow(["wall_time", *fields])
    for r in rec:
        kinds[r.kind] += 1
        if r.kind == R.KIND_FRAME:
            f = r.frame()
            first = first or f
            last = f
            if prev_idx is not None and f.frame_index != prev_idx + 1:
                gaps += 1
            if prev_t is not None:
                intervals.add((r.t - prev_t) * 1000)
            prev_idx, prev_t = f.frame_index, r.t
            speed_max = max(speed_max, abs(f.speed))
            if writer:
                writer.writerow([f"{r.t:.6f}", *(get(f, k) for k in fields)])
        elif r.kind == R.KIND_EVENT:
            events.append(r.json())
    if out:
        out.close()
        print(f"wrote {args.csv}")

    names = {R.KIND_FRAME: "frames", R.KIND_CONFIG: "config", R.KIND_EVENT: "events", R.KIND_CONTROLS: "controls",
             R.KIND_NOTE: "notes"}
    print("records: " + ", ".join(f"{names.get(k, k)} {n}" for k, n in sorted(kinds.items())))
    if first is not None:
        st = intervals.summary()
        print(f"frames {first.frame_index}..{last.frame_index}, {gaps} gaps (frames the recorder skipped)")
        print(f"recorded interval mean {st['mean']:.2f} ms, p95 {st['p95']:.2f}, max {st['max']:.2f}")
        print(f"max speed {speed_max * 3.6:.1f} km/h, game time {first.game_time}..{last.game_time} min")
    for e in events:
        print(f"event: {e['id']} {e['attrs']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
