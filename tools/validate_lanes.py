"""Check the lane map against live AI traffic: AI vehicles drive on lane centrelines, so their
lateral distance to the nearest same-direction lane measures how accurate the lane map is.
Read-only: sends nothing to the game.

    python tools/validate_lanes.py --seconds 30
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import logs_dir, map_dir  # noqa: E402
from autohaul.io.gamedata import GameDataReader  # noqa: E402
from autohaul.io.telemetry import TelemetryReader  # noqa: E402
from autohaul.world.lanemap import KIND_ROAD, LaneMap, heading_to_rotation, quat_forward_rotation, wrap  # noqa: E402


def dist_to_polyline(p: np.ndarray, poly: np.ndarray) -> float:
    a, b = poly[:-1], poly[1:]
    d = b - a
    l2 = (d * d).sum(axis=1)
    l2[l2 == 0] = 1e-9
    t = np.clip(((p - a) * d).sum(axis=1) / l2, 0, 1)
    proj = a + d * t[:, None]
    return float(np.min(np.hypot(*(proj - p).T)))


def predict_paths(lm: LaneMap, reader: TelemetryReader, gd: GameDataReader, seconds: float) -> dict:
    """Predict each AI vehicle's path 80 m ahead from the lane map, then measure how far its real
    positions stray from it while it covers the next 60 m."""
    from autohaul.world.localizer import LaneTracker
    trackers: dict[int, LaneTracker] = {}
    active: dict[int, tuple[np.ndarray, np.ndarray]] = {}   # id -> (predicted polyline, start position)
    errors, switches, travelled, junction_misses = [], 0, 0.0, 0
    last: dict[int, np.ndarray] = {}
    end = time.time() + seconds
    while time.time() < end:
        reader.wait_frame(200)
        for v in gd.traffic() or []:
            if v["speed"] < 3.0:
                continue
            vid = v["id"]
            pos = np.array([v["wx"], v["wz"]])
            rot = quat_forward_rotation(v["qw"], v["qx"], v["qy"], v["qz"])
            tr = trackers.setdefault(vid, LaneTracker(lm))
            st = tr.update(pos[0], pos[1], rot)
            if st is None:
                continue
            if st.switched:
                switches += 1
            if vid in last:
                travelled += float(np.hypot(*(pos - last[vid])))
            last[vid] = pos
            if vid in active:
                poly, start = active[vid]
                if np.hypot(*(pos - start)) > 60:
                    del active[vid]
                else:
                    e = dist_to_polyline(pos, poly)
                    if e > 3.0:
                        junction_misses += 1   # took another branch than "straight on" (expected at junctions)
                        del active[vid]
                    else:
                        errors.append(e)
            if vid not in active:
                poly = tr.path_ahead(80.0)
                if len(poly) >= 2:
                    active[vid] = (poly, pos)
        time.sleep(0.2)
    out = {"samples": len(errors), "junction_branch_changes": junction_misses,
           "lane_switches": switches, "km_tracked": round(travelled / 1000, 2)}
    if errors:
        e = np.array(errors)
        out.update({"median_m": round(float(np.median(e)), 2), "p90_m": round(float(np.percentile(e, 90)), 2)})
        print(f"\npath prediction: {len(e)} samples, distance from predicted path median {out['median_m']:.2f} m, "
              f"p90 {out['p90_m']:.2f} m; {junction_misses} vehicles took a different branch than straight on; "
              f"tracker lane switches {switches} over {out['km_tracked']} km of AI driving")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--predict", type=float, default=0.0,
                        help="also run N seconds of path prediction: predicted lane path vs. where AI vehicles go")
    args = parser.parse_args()

    cfg = load_config()
    lm = LaneMap.load_or_build(map_dir(cfg))
    reader = TelemetryReader()
    if not reader.connect():
        print("game not running")
        return 1
    gd = GameDataReader()

    lat_road, lat_prefab, unmatched, heading_dev = [], [], 0, []
    last_pos: dict[int, tuple[float, float, float]] = {}
    examples = []
    truck = []
    end = time.time() + args.seconds
    while time.time() < end:
        f = reader.wait_frame(200)
        veh = gd.traffic() or []
        now = time.time()
        for v in veh:
            if v["speed"] < 1.0:          # parked / stopped vehicles may sit off-lane
                continue
            rot = quat_forward_rotation(v["qw"], v["qx"], v["qy"], v["qz"])
            prev = last_pos.get(v["id"])
            if prev and now - prev[2] > 0.15:
                mx, my = v["wx"] - prev[0], v["wz"] - prev[1]
                if math.hypot(mx, my) > 1.0:
                    heading_dev.append(abs(float(wrap(np.array(rot - math.atan2(my, mx))))))
            last_pos[v["id"]] = (v["wx"], v["wz"], now)
            hits = lm.nearest(v["wx"], v["wz"], rot)
            if not hits:
                unmatched += 1
                continue
            h = hits[0]
            (lat_road if lm.lane_kind[h.lane] == KIND_ROAD else lat_prefab).append(h.lateral)
            if len(examples) < 8 and abs(h.lateral) > 1.0:
                examples.append((round(h.lateral, 2), lm.describe(h.lane), round(v["speed"] * 3.6)))
        if f is not None:
            hits = lm.nearest(f.truck_placement.x, f.truck_placement.z, heading_to_rotation(f.truck_placement.heading))
            truck.append(hits[0].lateral if hits else None)
        time.sleep(0.25)

    report_track = {}
    if args.predict:
        report_track = predict_paths(lm, reader, gd, args.predict)

    def stats(name, xs):
        if not xs:
            return {"n": 0}
        a = np.abs(np.array(xs))
        s = {"n": len(xs), "median_abs_m": round(float(np.median(a)), 2), "p90_abs_m": round(float(np.percentile(a, 90)), 2),
             "within_1m": round(float(np.mean(a < 1.0)), 3), "mean_signed_m": round(float(np.mean(xs)), 2)}
        print(f"{name:13s} n={s['n']:5d}  median |lateral| {s['median_abs_m']:.2f} m  p90 {s['p90_abs_m']:.2f} m  "
              f"within 1 m {s['within_1m'] * 100:.0f}%  mean signed {s['mean_signed_m']:+.2f} m")
        return s

    print()
    report = {"road": stats("road lanes", lat_road), "prefab": stats("prefab lanes", lat_prefab)}
    total = len(lat_road) + len(lat_prefab) + unmatched
    print(f"unmatched (no lane within 12 m, same direction): {unmatched}/{total}")
    if heading_dev:
        hd = np.degrees(np.array(heading_dev))
        print(f"quaternion heading vs. actual movement: median {np.median(hd):.1f} deg, p90 {np.percentile(hd, 90):.1f} deg")
        report["heading_dev_deg_median"] = round(float(np.median(hd)), 2)
    tv = [t for t in truck if t is not None]
    print(f"truck: matched {len(tv)}/{len(truck)} samples" + (f", lateral median {np.median(tv):+.2f} m" if tv else ""))
    if examples:
        print("examples > 1 m off:", examples)
    report.update({"unmatched": unmatched, "examples": examples, "time": time.time(), "prediction": report_track})
    out = logs_dir(cfg) / f"lanecheck-{time.strftime('%Y%m%d-%H%M%S')}.json"
    out.write_text(json.dumps(report, indent=2, default=str))
    print(f"report: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
