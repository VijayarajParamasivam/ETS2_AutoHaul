"""Live telemetry dashboard: frame rate / jitter, truck state, job, fatigue, safety status.

    python tools/dashboard.py            # read-only, sends no inputs
    python tools/dashboard.py --record   # also save a recording for tools/replay.py

Ctrl+C to quit. The dashboard never controls the truck.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.session import Session, enable_ansi  # noqa: E402
from autohaul.infra.stats import Rolling  # noqa: E402
from autohaul.io import _layout as L  # noqa: E402
from autohaul.io.gamedata import GameDataReader  # noqa: E402
from autohaul.io.telemetry import channel_seen, text  # noqa: E402

FLAGS = [("TEL", L.BRIDGE_FLAG_TELEMETRY_OK), ("INPUT", L.BRIDGE_FLAG_INPUT_OK), ("ALIVE", L.BRIDGE_FLAG_CONTROL_ALIVE),
         ("ACTIVE", L.BRIDGE_FLAG_CONTROL_ACTIVE), ("KILL", L.BRIDGE_FLAG_KILL_LATCHED)]


def hhmm(minutes: int) -> str:
    sign = "-" if minutes < 0 else ""
    m = abs(minutes)
    return f"{sign}{m // 60}:{m % 60:02d}"


def game_clock(t: int) -> str:
    days = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"]
    return f"{days[(t // 1440) % 7]} {(t // 60) % 24:02d}:{t % 60:02d}"


def render(s: Session, f: L.TelemetryFrame, intervals: Rolling, ages: Rolling, events: list[str],
           gd: GameDataReader, gd_counts: dict) -> str:
    hdr = s.reader.header()
    cfg = s.reader.read_config()
    flags = " ".join(n for n, bit in FLAGS if hdr.bridge_flags & bit) or "-"
    tv = hdr.telemetry_game_version
    fps = 1000.0 / intervals.mean if len(intervals) and intervals.mean > 0 else 0.0
    lines = [
        f"AutoHaul dashboard  ·  {text(hdr.game_name)}  ·  telemetry game v{tv >> 16}.{tv & 0xFFFF}  ·  bridge {text(hdr.bridge_version)}",
        f"bridge flags: {flags:32s} watchdog {hdr.watchdog_ms} ms  trips {hdr.watchdog_trips}  kills {hdr.kill_count}  "
        f"input frames {hdr.input_frames}  {'PAUSED' if f.paused else 'running'}",
        f"frames: {f.frame_index:>8d}  {fps:5.1f} Hz  interval p50 {intervals.percentile(50):5.2f} ms  "
        f"p95 {intervals.percentile(95):5.2f}  max {intervals.max:6.2f}  |  age p50 {ages.percentile(50):4.2f} ms  "
        f"p95 {ages.percentile(95):4.2f}  torn {s.reader.torn_reads}",
        "",
        f"speed {f.speed * 3.6:6.1f} km/h   limit {f.navigation_speed_limit * 3.6:5.0f} km/h   cruise {f.cruise_control * 3.6:5.0f}   "
        f"gear {f.displayed_gear:+3d} ({f.engine_gear:+3d})   rpm {f.engine_rpm:6.0f}",
        f"pos x {f.truck_placement.x:11.2f}  y {f.truck_placement.y:8.2f}  z {f.truck_placement.z:11.2f}   "
        f"heading {f.truck_placement.heading * 360:6.1f}°  pitch {f.truck_placement.pitch * 360:5.1f}°",
        f"input  steer {f.input_steering:+.3f} thr {f.input_throttle:.3f} brk {f.input_brake:.3f}   "
        f"effective steer {f.effective_steering:+.3f} thr {f.effective_throttle:.3f} brk {f.effective_brake:.3f}",
        f"fuel {f.fuel:6.1f} L  range {f.fuel_range:6.0f} km  avg {f.fuel_average_consumption * 100:5.1f} L/100km   "
        f"air {f.brake_air_pressure:5.1f} psi  park brake {'ON' if f.parking_brake else 'off'}  "
        f"engine {'on' if f.engine_enabled else 'off'}",
        f"lights low {f.light_low_beam} high {f.light_high_beam} park {f.light_parking}  blinkers L{f.lblinker} R{f.rblinker} "
        f"hazard {f.hazard_warning}  wipers {f.wipers}  retarder {f.retarder_level}  engine brake {f.motor_brake}",
        "",
        f"game time {game_clock(f.game_time)}   rest needed in {hhmm(f.next_rest_stop)}   "
        f"mandatory break {'n/a (1.53)' if not channel_seen(f, 'mandatory.break') else hhmm(f.next_mandatory_break)}   "
        f"scale {f.local_scale:.1f}",
        f"navigation {f.navigation_distance / 1000:7.1f} km  {f.navigation_time / 3600:5.2f} h",
    ]
    if cfg and cfg.job.present:
        left = cfg.job.delivery_time - f.game_time
        lines.append(f"job: {text(cfg.job.cargo)} {cfg.job.cargo_mass / 1000:.1f} t  {text(cfg.job.source_city)} -> "
                     f"{text(cfg.job.destination_city)} ({text(cfg.job.destination_company)})  €{cfg.job.income}  "
                     f"deadline in {hhmm(left)}  damage {f.job_cargo_damage * 100:.1f}%")
    else:
        lines.append("job: none")
    lines.append(f"trailer 0: {'connected' if f.trailers[0].connected else 'not connected'}"
                 + (f"  ({text(cfg.trailers[0].id)})" if cfg and cfg.trailers[0].present else ""))
    if cfg and cfg.truck.present:
        lines.append(f"truck: {text(cfg.truck.brand)} {text(cfg.truck.name)}  tank {cfg.truck.fuel_capacity:.0f} L  "
                     f"gears {cfg.truck.forward_gear_count}F/{cfg.truck.reverse_gear_count}R  "
                     f"retarder steps {cfg.truck.retarder_step_count}  shifter {text(cfg.controls.shifter_type)}")
    lines.append("")
    lines.append("ETS2LA memory plugin: " + "  ".join(f"{k} {v}" for k, v in gd_counts.items()))
    verdict = s.safety.check(f, None)
    lines.append(f"safety: {'OK' if verdict.ok else verdict.kind.upper() + ' - ' + (verdict.reason or '')}")
    lines.append("")
    lines.append("recent gameplay events:")
    if events:
        lines.extend(f"  {e}" for e in events[-5:])
    else:
        lines.append("  (none)")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--refresh-hz", type=float, default=5.0)
    args = parser.parse_args()

    enable_ansi()
    with Session("dashboard", record=args.record) as s:
        s.connect()
        gd = GameDataReader()
        intervals, ages = Rolling(600), Rolling(600)
        last_qpc = 0
        events: list[str] = []
        next_draw = 0.0
        gd_counts: dict = {}
        print("\x1b[2J", end="")
        try:
            while True:
                f = s.reader.wait_frame(200)
                if f is None:
                    continue
                if last_qpc:
                    intervals.add((f.publish_qpc - last_qpc) * 1000.0 / s.reader.qpc_freq)
                last_qpc = f.publish_qpc
                ages.add(s.reader.frame_age_ms(f))
                s.record_frame(f)
                for ev in s.reader.poll_events():
                    events.append(f"{game_clock(ev.game_time)}  {ev.id}  {ev.attrs}")
                    if s.recorder:
                        s.recorder.event(ev)
                now = time.monotonic()
                if now >= next_draw:
                    next_draw = now + 1.0 / args.refresh_hz
                    t, sem, r = gd.traffic(), gd.semaphores(), gd.route()
                    gd_counts = {"traffic": "-" if t is None else len(t), "lights": "-" if sem is None else len(sem),
                                 "route": "-" if r is None else len(r), "camera": "-" if gd.camera() is None else "ok"}
                    print("\x1b[H\x1b[J" + render(s, f, intervals, ages, events, gd, gd_counts), flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            gd.close()
    if s.recording_path:
        print(f"recording: {s.recording_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
