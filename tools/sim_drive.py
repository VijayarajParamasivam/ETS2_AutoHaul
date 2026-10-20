"""Drive the simulated truck on the REAL lane map (no game needed): real junctions, exits, curves.

    python tools/sim_drive.py --from-recording latest --minutes 5       # start where a recording starts
    python tools/sim_drive.py --x -27900 --y 36080 --rot -34 --minutes 3 # start pose (map x, y, deg)

Reports faults with their positions, lateral error to the lane centre, speed vs. target, and compute time.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from autohaul.control.speed import SpeedParams  # noqa: E402
from autohaul.control.vehicle import TruckGeometry  # noqa: E402
from autohaul.drive.autopilot import Autopilot  # noqa: E402
from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import map_dir, recordings_dir  # noqa: E402
from autohaul.io.recorder import RecordingReader  # noqa: E402
from autohaul.sim.truck import SimParams, TruckSim  # noqa: E402
from autohaul.world.lanemap import LaneMap, heading_to_rotation  # noqa: E402
from autohaul.world.pois import toll_plazas  # noqa: E402
from autohaul.world.graph import LaneGraph  # noqa: E402
from autohaul.world.junctions import Junctions  # noqa: E402
from autohaul.world.places import Places  # noqa: E402
from autohaul.world.route import Router  # noqa: E402


def start_from_recording(spec: str) -> tuple[float, float, float, float]:
    cfg = load_config()
    path = Path(spec) if spec != "latest" else max(recordings_dir(cfg).glob("*.ahrec"), key=lambda p: p.stat().st_mtime)
    for _, f in RecordingReader(path).frames():
        if f.speed > 5 and not f.paused:
            p = f.truck_placement
            return p.x, p.z, heading_to_rotation(p.heading), f.speed
    raise SystemExit(f"{path}: no moving frame")


def main() -> int:
    ap_ = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap_.add_argument("--from-recording")
    ap_.add_argument("--x", type=float)
    ap_.add_argument("--y", type=float)
    ap_.add_argument("--rot", type=float, help="degrees, map rotation")
    ap_.add_argument("--kmh", type=float, default=60.0, help="start speed")
    ap_.add_argument("--limit-kmh", type=float, default=80.0, help="simulated posted limit")
    ap_.add_argument("--minutes", type=float, default=5.0)
    ap_.add_argument("--mass", type=float, default=30000.0)
    ap_.add_argument("--to", help="route to CITY, CITY:COMPANY or X,Y (Phase 3)")
    args = ap_.parse_args()
    if args.from_recording:
        x, y, rot, v0 = start_from_recording(args.from_recording)
    else:
        x, y, rot, v0 = args.x, args.y, math.radians(args.rot), args.kmh / 3.6
    cfg = load_config()
    t0 = time.time()
    lm = LaneMap.load_or_build(map_dir(cfg))
    print(f"lane map loaded in {time.time() - t0:.1f} s")
    geom = TruckGeometry()
    sim = TruckSim(geom, SimParams(mass=args.mass), x=x, y=y, rotation=rot, speed=v0)
    router = junctions = None
    if args.to:
        target = Places(map_dir(cfg)).resolve(args.to)
        if target is None:
            print(f"unknown destination {args.to!r}")
            return 1
        router, junctions = Router(lm, LaneGraph.load_or_build(lm, map_dir(cfg))), Junctions(lm)
    ap = Autopilot(lm, geom, "relative", speed_params=SpeedParams(max_speed_kmh=args.limit_kmh),
                   hazards=toll_plazas(map_dir(cfg)), router=router, junctions=junctions)
    if args.to:
        ap.set_destination(target.x, target.y)
        print(f"destination: {target.label}")
    frame = sim.frame(args.limit_kmh / 3.6)
    reason = ap.engage(frame)
    if reason:
        print(f"engage refused at the start: {reason}")
        return 1
    dt = sim.p.dt
    lat, speed, target, ms, faults = [], [], [], [], []
    dist = 0.0
    last_print = 0.0
    last_lc, changes_done = "", 0
    for _ in range(int(args.minutes * 60 / dt)):
        frame = sim.frame(args.limit_kmh / 3.6)
        cmd, st = ap.step(frame, [], dt)
        if st.fault:
            faults.append((round(sim.t, 1), st.fault, round(sim.s.x, 1), round(sim.s.y, 1), st.lane_desc))
            print(f"  t={sim.t:6.1f}s FAULT {st.fault} at ({sim.s.x:.1f}, {sim.s.y:.1f}) lane {st.lane_desc}")
            if len(faults) > 5:
                break
            reason = ap.engage(sim.frame(args.limit_kmh / 3.6))      # try to continue, like a driver re-engaging
            if reason:
                print(f"  can't re-engage: {reason}")
                break
            continue
        if st.lane_change and st.lane_change != last_lc:
            print(f"  t={sim.t:6.1f}s lane change {st.lane_change}")
            changes_done += st.lane_change == "done"
        last_lc = st.lane_change
        lat.append(abs(st.lateral))
        speed.append(sim.s.speed * 3.6)
        target.append(st.target_kmh)
        ms.append(sum(st.timings_ms.values()))
        sim.step(cmd.steer, cmd.throttle, cmd.brake)
        dist += sim.s.speed * dt
        if sim.t - last_print > 20:
            last_print = sim.t
            route = f"  route {st.route_km:.2f} km left{' ' + st.route_note if st.route_note else ''}" if st.route_km is not None else ""
            print(f"  t={sim.t:6.0f}s {dist / 1000:5.2f} km  {sim.s.speed * 3.6:5.1f} km/h (target {st.target_kmh:4.0f}: "
                  f"{st.target_reason})  lat {st.lateral:+.2f} m  {st.lane_desc}{route}")
        if sim.s.speed < 0.2 and st.target_kmh < 1:
            print(f"  stopped: {st.target_reason}")
            break
    a = np.array(lat)
    print(f"\n{dist / 1000:.2f} km in {sim.t:.0f} s simulated; faults {len(faults)}; lane changes done {changes_done}")
    if a.size:
        print(f"lateral to lane centre: median {np.median(a):.2f} m, p95 {np.percentile(a, 95):.2f} m, max {a.max():.2f} m")
        print(f"speed mean {np.mean(speed):.1f} km/h, target mean {np.mean(target):.1f}; compute p99 {np.percentile(ms, 99):.2f} ms")
    return 0 if not faults else 2


if __name__ == "__main__":
    sys.exit(main())
