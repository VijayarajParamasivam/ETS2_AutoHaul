"""In-game check of the mission's refuel script (Phase 4), at a fuel pump.

    python tools/stop_check.py --find            # the nearest truck pumps, with a console teleport command
    python tools/stop_check.py --wait-s 300      # run the refuel script once the truck stands at a pump

The script is the mission's own (autohaul/mission/actions.py Refuel): brake held, press `activate`, and if no
fuel flows, hold it. It runs under the safety monitor (your keys or controller abort it) and reports the fuel
level frame by frame to logs/stop-check-*.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from phase1_check import Aborted, drive  # noqa: E402
from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import logs_dir, map_dir  # noqa: E402
from autohaul.infra.session import Session  # noqa: E402
from autohaul.io.telemetry import TelemetryReader  # noqa: E402
from autohaul.io.win32 import foreground_pid  # noqa: E402
from autohaul.mission.actions import ActionParams, Refuel  # noqa: E402
from autohaul.mission.settings import mission_settings  # noqa: E402
from autohaul.safety.supervisor import Command  # noqa: E402
from autohaul.world.bundle import load_world  # noqa: E402
from autohaul.world.services import Services  # noqa: E402


def find(n: int) -> int:
    r = TelemetryReader("")
    if not r.connect():
        print("game not running")
        return 1
    f = r.wait_frame(1000) or r.read_latest()
    x, y = f.truck_placement.x, f.truck_placement.z
    md = map_dir(load_config())
    w = load_world(md, log=lambda *_: None)
    sv = Services(w.lm, md, no_go=w.rules.cars_only() if w.rules else None, log=lambda *_: None)
    pumps = sorted(sv.fuel, key=lambda p: math.hypot(p.x - x, p.y - y))[:n]
    for p in pumps:
        lm = w.lm
        first = lm.lane_first[p.lane]
        cum = 0.0
        seg = first
        while seg < lm.lane_last[p.lane] and cum + lm.seg_len[seg] < p.s:
            cum += lm.seg_len[seg]
            seg += 1
        t = (p.s - cum) / max(lm.seg_len[seg], 1e-6)
        z = lm.seg[seg, 4] + t * (lm.seg[seg, 5] - lm.seg[seg, 4])
        rot = math.atan2(lm.seg[seg, 3] - lm.seg[seg, 1], lm.seg[seg, 2] - lm.seg[seg, 0])
        a = (-math.pi / 2 - rot + math.pi) % (2 * math.pi) - math.pi
        print(f"{math.hypot(p.x - x, p.y - y) / 1000:6.1f} km  {p.country:10s} {sv.price_at(p):.2f} EUR/l  site {p.site}  "
              f"goto {p.x:.2f};{z + 1.0:.2f};{p.y:.2f};{a:.4f};0")
    return 0


def main() -> int:
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--find", type=int, nargs="?", const=5, help="list the nearest N truck pumps")
    a.add_argument("--wait-s", type=float, default=300.0)
    a.add_argument("--max-s", type=float, default=150.0, help="give up after this long")
    args = a.parse_args()
    if args.find:
        return find(args.find)
    with Session("stop-check", controls=True, record=True) as s:
        if not s.connect(10):
            print("game not running")
            return 1
        cfgb = s.reader.read_config()
        capacity = float(cfgb.truck.fuel_capacity)
        _, _, _, ap = mission_settings(s.cfg)
        s.safety.set_axes(steering=True, pedals=True)
        end = time.time() + args.wait_s
        print("waiting for: truck standing at the pump, game focused and unpaused", flush=True)
        while time.time() < end:
            f = s.reader.wait_frame(200)
            if f is not None and abs(f.speed) < 0.3 and not f.paused and foreground_pid() == s.reader.header().game_pid:
                break
        else:
            print("gave up waiting; nothing was sent to the game")
            return 1
        f0 = s.reader.read_latest()
        print(f"start: fuel {f0.fuel:.1f} / {capacity:.0f} l", flush=True)
        act = Refuel(capacity, ap if isinstance(ap, ActionParams) else ActionParams())
        log, state = [], {"step": None, "prev_t": None}

        def fn(t, f):
            dt = 1 / 60 if state["prev_t"] is None else max(1e-3, min(0.1, t - state["prev_t"]))
            state["prev_t"] = t
            st = act.step(f, dt)
            state["step"] = st
            log.append({"t": round(t, 2), "fuel": round(float(f.fuel), 2), "phase": act.phase, "note": st.note,
                        "press": st.press, "hold": st.hold})
            if st.note and (len(log) < 2 or log[-2]["note"] != st.note):
                print(f"  t={t:5.1f}s fuel {f.fuel:7.1f} l  {act.phase:6s} {st.note}", flush=True)
            return Command(brake=st.brake)

        def actions(t, f):
            st = state["step"]
            for name in st.press:
                s.controls.press(name)
            for name, on in st.hold.items():
                s.controls.hold(name, on)

        outcome = "time up"
        try:
            drive(s, args.max_s, fn, actions, until=lambda t, f: bool(state["step"] and
                                                                       (state["step"].done or state["step"].failed)))
            st = state["step"]
            outcome = st.failed or st.note if st else "no frames"
        except Aborted as exc:
            outcome = f"stopped: {exc.verdict.reason}"
        f1 = s.reader.read_latest()
        print(f"\n{outcome}\nfuel {f0.fuel:.1f} -> {f1.fuel:.1f} l (+{f1.fuel - f0.fuel:.1f} l), money check: see the game")
        path = logs_dir(s.cfg) / f"stop-check-{time.strftime('%Y%m%d-%H%M%S')}.json"
        path.write_text(json.dumps({"outcome": outcome, "fuel_start": f0.fuel, "fuel_end": f1.fuel,
                                    "capacity": capacity, "log": log}, indent=1))
        print(f"report: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
