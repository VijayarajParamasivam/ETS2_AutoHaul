"""Shadow mode: run the autopilot on live telemetry while YOU drive. Sends nothing to the game.

    python tools/shadow_drive.py --seconds 120

It reports what AutoHaul would have done: its wheel target next to yours, how centred you drive per
the lane map, the lead vehicle it would follow, the speed it would choose, the truck's measured
curve response (yaw rate vs wheel angle), and per-frame compute time. Notes go to a recording.
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

import numpy as np  # noqa: E402

from drive import speed_params  # noqa: E402
from autohaul.control.vehicle import TruckGeometry  # noqa: E402
from autohaul.drive.autopilot import Autopilot  # noqa: E402
from autohaul.infra.gameenv import active_controls_file, controls_constants, steering_mode  # noqa: E402
from autohaul.infra.paths import logs_dir, map_dir  # noqa: E402
from autohaul.infra.session import Session  # noqa: E402
from autohaul.io.gamedata import GameDataReader  # noqa: E402
from autohaul.world.lanemap import LaneMap  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=120.0)
    args = parser.parse_args()
    with Session("shadow", controls=False, record=True, keyboard=False) as s:
        if not s.connect(10):
            print("game not running")
            return 1
        cfg = s.cfg
        d = cfg.get("drive", {})
        lm = LaneMap.load_or_build(map_dir(cfg))
        cfgb = s.reader.read_config()
        geom = TruckGeometry.from_config(cfgb.truck, math.radians(d.get("steer_deg_per_unit", 38.0)),
                                         d.get("front_overhang_m", 1.45))
        consts = controls_constants(active_controls_file(cfg))
        ap = Autopilot(lm, geom, steering_mode(cfg), speed_params=speed_params(cfg),
                       relative_gain=consts.get("c_rsteersens", 0.42))
        gd = GameDataReader()
        rows, faults, ms = [], {}, []
        engaged = False
        last_us = None
        end = time.time() + args.seconds
        print(f"shadowing for {args.seconds:.0f} s; drive normally ...", flush=True)
        while time.time() < end:
            f = s.reader.wait_frame(200)
            if f is None or f.paused:
                continue
            s.record_frame(f)
            dt = 1 / 60
            if last_us is not None and f.simulation_time_us > last_us:
                dt = min(0.1, max(0.001, (f.simulation_time_us - last_us) / 1e6))
            last_us = f.simulation_time_us
            if not engaged:
                engaged = ap.engage(f) is None and f.speed > 3
                continue
            cmd, st = ap.step(f, gd.traffic(), dt)
            ms.append(sum(st.timings_ms.values()))
            if st.fault:
                key = st.fault.split(" (")[0].split(" for ")[0]
                faults[key] = faults.get(key, 0) + 1
                engaged = False                 # re-engage when the lane match is good again
                continue
            rows.append((f.speed * 3.6, st.lateral, st.steer_target, f.effective_steering, st.target_kmh,
                         st.lead.gap if st.lead else np.nan, st.curve_ratio, st.confidence,
                         f.navigation_speed_limit * 3.6))
            if s.recorder and len(rows) % 6 == 0:
                s.recorder.note(event="shadow", speed_kmh=round(f.speed * 3.6, 1), lateral=round(st.lateral, 3),
                                steer_target=round(st.steer_target, 4), steer_eff=round(f.effective_steering, 4),
                                target_kmh=round(st.target_kmh, 1), reason=st.target_reason,
                                lead_gap=round(st.lead.gap, 1) if st.lead else None, ratio=round(st.curve_ratio, 3))
        if not rows:
            print("no usable samples (were you driving on a mapped lane?)")
            return 1
        a = np.array(rows)
        moving = a[:, 0] > 20
        out = {"samples": len(a), "faults": faults, "learned_curve_ratio": round(float(a[-1, 6]), 3)}
        lat = np.abs(a[moving, 1])
        diff = a[moving, 2] - a[moving, 3]
        out["your_lateral_median_m"] = round(float(np.median(lat)), 2) if lat.size else None
        out["your_lateral_p90_m"] = round(float(np.percentile(lat, 90)), 2) if lat.size else None
        out["steer_target_minus_yours_rms"] = round(float(np.sqrt(np.mean(diff ** 2))), 4) if diff.size else None
        out["speed_kmh_mean"] = round(float(a[:, 0].mean()), 1)
        out["autopilot_target_kmh_mean"] = round(float(a[:, 4].mean()), 1)
        out["lead_seen_fraction"] = round(float(np.mean(~np.isnan(a[:, 5]))), 2)
        out["compute_ms_p99"] = round(float(np.percentile(ms, 99)), 2)
        print(json.dumps(out, indent=2))
        path = logs_dir(cfg) / f"shadow-{time.strftime('%Y%m%d-%H%M%S')}.json"
        path.write_text(json.dumps(out, indent=2))
        print(f"report: {path}\nrecording: {s.recording_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
