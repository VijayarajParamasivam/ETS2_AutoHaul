"""1.53 compatibility self-test: checks the bridge DLL and each block of the pinned ETS2LA plugin
against live game data. Read-only: sends no inputs.

    python tools/selftest_gamedata.py              # 20 s sample (drive around: traffic, lights, a GPS route)
    python tools/selftest_gamedata.py --seconds 60

Best results: have a job or GPS destination set, and drive past traffic and a signalled junction
during the sample. Writes a JSON report next to the logs.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.paths import game_log, logs_dir  # noqa: E402
from autohaul.infra.session import Session  # noqa: E402
from autohaul.infra.stats import Rolling  # noqa: E402
from autohaul.io import _layout as L  # noqa: E402
from autohaul.io.gamedata import GameDataReader  # noqa: E402
from autohaul.io.telemetry import channel_seen, text  # noqa: E402

# Channels ETS2 1.53.2.1 doesn't provide (game.log: "lookup_channel: channel ... not found").
EXPECTED_MISSING_ON_153 = {"mandatory.break", "truck.adblue.consumption.average"}


GUARD_RE = re.compile(r"AutoHaul guard: (\w+) reader (faulted.*|took over.*)")
# Which self-test check each guarded plugin reader feeds.
READER_CHECK = {"camera": "ets2la: camera", "ai_traffic": "ets2la: traffic",
                "traffic_objects": "ets2la: traffic lights", "navigation": "ets2la: route"}


def plugin_log(cfg) -> list[str]:
    try:
        text_ = game_log(cfg).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return [f"(cannot read game.log.txt: {exc})"]
    return [ln.strip() for ln in text_.splitlines() if "[autohaul]" in ln or "[ets2_la_plugin]" in ln]


def guard_events(lines: list[str]) -> dict[str, str]:
    """reader name -> why the AutoHaul guard in the ETS2LA plugin switched it off."""
    out = {}
    for ln in lines:
        m = GUARD_RE.search(ln)
        if m:
            out[m.group(1)] = m.group(2)
        elif "traffic_objects reader disabled" in ln:
            out.setdefault("traffic_objects", "disabled by design: crashes the game on 1.53.2.1")
    return out


@dataclass
class Check:
    name: str
    status: str   # PASS WARN FAIL SKIP
    detail: str


def dist(ax: float, az: float, bx: float, bz: float) -> float:
    return math.hypot(ax - bx, az - bz)


def qnorm(r: dict) -> float:
    return math.sqrt(r["qw"] ** 2 + r["qx"] ** 2 + r["qy"] ** 2 + r["qz"] ** 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()

    checks: list[Check] = []
    with Session("selftest", keyboard=False) as s:
        print("Connecting to the bridge ...")
        if not s.connect(timeout_s=60):
            print("FAIL: autohaul_bridge.dll is not running (no telemetry mapping). Is it installed and the game started?")
            return 1
        hdr = s.reader.header()
        tv = hdr.telemetry_game_version
        checks.append(Check("bridge: layout", "PASS", f"v{hdr.layout_version} 0x{hdr.layout_hash:016x}, bridge {text(hdr.bridge_version)}"))
        checks.append(Check("bridge: telemetry game version", "PASS" if tv == (1 << 16 | 18) else "WARN",
                            f"{tv >> 16}.{tv & 0xFFFF} (1.53 reports 1.18)"))
        checks.append(Check("bridge: input device", "PASS" if hdr.input_registered else "FAIL",
                            f"registered={hdr.input_registered} active={hdr.input_active} inputs={hdr.input_count}"))

        gd = GameDataReader()
        avail = gd.available()
        print(f"ETS2LA blocks present: {avail}")
        print(f"Sampling for {args.seconds:.0f} s - drive normally (traffic, a signalled junction, a GPS route) ...")

        intervals = Rolling(100000)
        seen_bits = [0] * L.CHANNEL_SEEN_WORDS
        last_qpc = 0
        frames = 0
        cam_ok, cam_bad, cam_samples = 0, [], 0
        traffic_max, traffic_bad, traffic_ids = 0, [], set()
        sem_max, sem_bad, sem_states = 0, [], set()
        route_max, route_vs_nav = 0, []
        moved = 0.0
        prev_pos = None
        t_end = time.time() + args.seconds
        last_sample = 0.0
        last_f = None
        while time.time() < t_end:
            f = s.reader.wait_frame(200)
            if f is None:
                continue
            frames += 1
            last_f = f
            if last_qpc:
                intervals.add((f.publish_qpc - last_qpc) * 1000.0 / s.reader.qpc_freq)
            last_qpc = f.publish_qpc
            for i in range(L.CHANNEL_SEEN_WORDS):
                seen_bits[i] |= f.channel_seen[i]
            tx, tz = f.truck_placement.x, f.truck_placement.z
            if prev_pos:
                moved += dist(tx, tz, *prev_pos)
            prev_pos = (tx, tz)
            now = time.time()
            if now - last_sample < 0.2:
                continue
            last_sample = now
            cam = gd.camera()
            if cam is not None:
                cam_samples += 1
                d = dist(cam["wx"], cam["wz"], tx, tz)
                if 10 <= cam["fov"] <= 150 and d < 60 and abs(qnorm(cam) - 1) < 0.1:
                    cam_ok += 1
                elif len(cam_bad) < 3:
                    cam_bad.append(f"fov={cam['fov']:.1f} dist={d:.1f}m |q|={qnorm(cam):.2f}")
            veh = gd.traffic()
            if veh is not None:
                traffic_max = max(traffic_max, len(veh))
                for v in veh:
                    traffic_ids.add(v["id"])
                    d = dist(v["wx"], v["wz"], tx, tz)
                    if not (d < 1500 and 1.0 < v["length"] < 40 and abs(qnorm(v) - 1) < 0.1) and len(traffic_bad) < 3:
                        traffic_bad.append(f"dist={d:.0f}m len={v['length']:.1f} |q|={qnorm(v):.2f}")
            sems = gd.semaphores()
            if sems is not None:
                sem_max = max(sem_max, len(sems))
                for m in sems:
                    sem_states.add(m["state_name"])
                    d = dist(m["wx"], m["wz"], tx, tz)
                    if (d > 1500 or m["state_name"].startswith("unknown")) and len(sem_bad) < 3:
                        sem_bad.append(f"dist={d:.0f}m state={m['state_name']}")
            route = gd.route()
            if route is not None:
                route_max = max(route_max, len(route))
                if route and f.navigation_distance > 0:
                    route_vs_nav.append((route[0]["distance_to_end"], f.navigation_distance))

        # ---- bridge results
        st = intervals.summary()
        hz = 1000.0 / st["mean"] if st["n"] else 0.0
        checks.append(Check("bridge: frame rate", "PASS" if hz >= 55 else ("WARN" if hz >= 30 else "FAIL"),
                            f"{hz:.1f} Hz over {frames} frames, interval p95 {st['p95']:.1f} ms, max {st['max']:.1f} ms"))
        never, explained = [], []
        if last_f is not None:
            cfg = s.reader.read_config()
            wheels = cfg.truck.wheel_count if cfg else 0
            has_trailer = bool(cfg and cfg.trailers[0].present) or bool(last_f.trailers[0].connected)
            has_job = bool(cfg and cfg.job.present)
            hshifter = bool(cfg and text(cfg.controls.shifter_type) == "hshifter")
            for cid, sdk, index, _t, _field in L.CHANNELS:
                if seen_bits[cid // 64] & (1 << (cid % 64)):
                    continue
                if index is not None and index >= wheels:
                    continue
                if sdk.startswith("trailer.") and not sdk.startswith("trailer.0."):
                    continue
                name = sdk if index is None else f"{sdk}[{index}]"
                if (sdk in EXPECTED_MISSING_ON_153 or (sdk.startswith("trailer.0.") and not has_trailer)
                        or (sdk == "job.cargo.damage" and not has_job) or (sdk == "truck.hshifter.slot" and not hshifter)):
                    explained.append(name)
                else:
                    never.append(name)
        checks.append(Check("bridge: channels", "PASS" if not never else "WARN",
                            f"never received: {', '.join(never) or 'none'}"
                            + (f" | expected now (not in 1.53, no trailer/job, or no H-shifter): {len(explained)}" if explained else "")))
        cfg = s.reader.read_config()
        checks.append(Check("bridge: truck config", "PASS" if cfg and cfg.truck.present else "FAIL",
                            f"{text(cfg.truck.brand)} {text(cfg.truck.name)}, {cfg.truck.wheel_count} wheels" if cfg else "no config"))
        checks.append(Check("bridge: job config", "PASS" if cfg and cfg.job.present else "WARN",
                            f"{text(cfg.job.cargo)} to {text(cfg.job.destination_city)}" if cfg and cfg.job.present
                            else "no active job (take one to test job/deadline fields)"))

        # ---- ETS2LA results
        if not any(avail.values()):
            checks.append(Check("ets2la: plugin", "FAIL", "no ETS2LA shared memory found: install with "
                                "`python tools/install_plugin.py --with-ets2la` (game closed), then restart the game"))
        else:
            if cam_samples == 0:
                checks.append(Check("ets2la: camera", "FAIL", "block missing or never readable"))
            else:
                rate = cam_ok / cam_samples
                checks.append(Check("ets2la: camera", "PASS" if rate > 0.9 else "FAIL",
                                    f"{cam_ok}/{cam_samples} plausible samples" + (f"; bad: {cam_bad}" if cam_bad else "")))
            checks.append(Check("ets2la: traffic",
                                "FAIL" if traffic_bad else ("PASS" if traffic_max > 0 else "WARN"),
                                f"max {traffic_max} vehicles at once, {len(traffic_ids)} distinct ids"
                                + (f"; implausible: {traffic_bad}" if traffic_bad else "")
                                + ("" if traffic_max else "; none seen - drive near AI traffic and re-run")))
            checks.append(Check("ets2la: traffic lights",
                                "FAIL" if sem_bad else ("PASS" if sem_max > 0 else "WARN"),
                                f"max {sem_max} at once, states seen {sorted(sem_states)}"
                                + (f"; implausible: {sem_bad}" if sem_bad else "")
                                + ("" if sem_max else "; none seen - drive towards a signalled junction and re-run")))
            if route_max == 0 and last_f is not None and last_f.navigation_distance > 0:
                checks.append(Check("ets2la: route", "FAIL",
                                    f"empty although the game's GPS has a route ({last_f.navigation_distance / 1000:.1f} km): "
                                    "the reader can't locate the route on this build -> use AutoHaul's own routing"))
            elif route_max == 0:
                checks.append(Check("ets2la: route", "WARN", "empty - set a GPS destination or take a job and re-run"))
            else:
                diffs = [abs(a - b) for a, b in route_vs_nav]
                rel = [abs(a - b) / max(b, 1.0) for a, b in route_vs_nav]
                good = sum(1 for d, r in zip(diffs, rel) if d < 1000 or r < 0.15)
                checks.append(Check("ets2la: route", "PASS" if route_vs_nav and good / len(route_vs_nav) > 0.8 else "WARN",
                                    f"{route_max} route items; first item distance vs telemetry navigation distance "
                                    f"agree in {good}/{len(route_vs_nav)} samples"))
        gd.close()

    # ---- game log (+ the AutoHaul guard inside the ETS2LA plugin)
    log_lines = plugin_log(s.cfg)
    for reader, why in guard_events(log_lines).items():
        name = READER_CHECK.get(reader)
        for i, c in enumerate(checks):
            if c.name == name:
                status = "SKIP" if why.startswith("disabled by design") else "FAIL"
                checks[i] = Check(c.name, status, f"reader switched off by the AutoHaul guard: {why}")
    plugin_errors = [ln for ln in log_lines if ("error" in ln.lower() or "could not" in ln.lower())
                     and "AutoHaul guard" not in ln]
    checks.append(Check("game.log", "WARN" if plugin_errors else "PASS",
                        f"{len(log_lines)} plugin lines" + (f"; problems: {plugin_errors[:3]}" if plugin_errors else "")))

    width = max(len(c.name) for c in checks)
    print()
    for c in checks:
        print(f"{c.status:4s}  {c.name:{width}s}  {c.detail}")
    report = {"time": time.time(), "seconds": args.seconds, "moved_m": round(moved, 1), "checks": [asdict(c) for c in checks],
              "game_log": log_lines[-40:]}
    out = logs_dir(s.cfg) / f"selftest-{time.strftime('%Y%m%d-%H%M%S')}.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nreport: {out}")
    if moved < 50:
        print("note: the truck barely moved during the sample; traffic/lights/route checks are more meaningful while driving.")
    return 1 if any(c.status == "FAIL" for c in checks) else 0


if __name__ == "__main__":
    sys.exit(main())
