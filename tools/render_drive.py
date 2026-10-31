"""Draw a stretch of a recorded drive as a clip: the map's lanes, the truck, and what AutoHaul read and decided.

Everything in the picture comes from the recording and the lane map, so it's the real drive but not game footage (other
traffic isn't recorded, only what AutoHaul said about it). Needs `pip install pillow imageio-ffmpeg`.

    python tools/render_drive.py 20261029-020945 372 449 --speed 2 --out docs/media/drive
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import imageio_ffmpeg  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from autohaul.control.vehicle import TruckGeometry, heading_to_rotation  # noqa: E402
from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import map_dir  # noqa: E402
from autohaul.io.recorder import KIND_FRAME, KIND_NOTE, RecordingReader  # noqa: E402
from autohaul.world.lanemap import LaneMap  # noqa: E402

W, H, SS = 960, 540, 2              # the clip's size; the map is drawn this many times larger, then scaled down
FPS = 30
PANEL_W = 300
BG, ROAD, ROAD_FAR, CENTRE = (15, 18, 25), (62, 69, 85), (34, 38, 48), (98, 107, 126)
ROUTE, TRAIL, CAB, TRAILER = (64, 196, 180), (255, 176, 59), (240, 240, 245), (150, 160, 178)
RED, AMBER, GREEN, TEXT, DIM = (240, 70, 70), (255, 176, 59), (90, 210, 130), (235, 238, 244), (140, 148, 164)
CAB_LEN, TRAILER_LEN, TRUCK_W = 6.2, 13.6, 2.55


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype("seguisb.ttf" if bold else "segoeui.ttf", size)


def load_stretch(name: str, a: float, b: float) -> list[tuple[float, object, dict]]:
    """(time, frame, the latest note) from `a` to `b` seconds into the recording."""
    out, t0, f, note = [], None, None, {}
    for r in RecordingReader(Path.home() / "AutoHaul" / "recordings" / f"{name}-drive.ahrec"):
        t0 = r.t if t0 is None else t0
        t = r.t - t0
        if t > b:
            break
        if r.kind == KIND_FRAME:
            f = r.frame()
            if t >= a and note:
                out.append((t - a, f, note))
        elif r.kind == KIND_NOTE:
            n = r.json()
            if n.get("event") == "ap":
                note = n
    return out


class Map:
    """The lanes around the stretch, split into polylines per lane."""

    def __init__(self, lm: LaneMap, xs: np.ndarray, ys: np.ndarray, height: float, margin: float = 320.0):
        s = lm.seg
        keep = ((s[:, 0] > xs.min() - margin) & (s[:, 0] < xs.max() + margin)
                & (s[:, 1] > ys.min() - margin) & (s[:, 1] < ys.max() + margin))
        idx = np.flatnonzero(keep)
        self.lines = []
        for part in np.split(idx, np.flatnonzero(np.diff(lm.seg_lane[idx]) != 0) + 1):
            if len(part):
                pts = np.vstack([s[part, 0:2], s[part[-1], 2:4]])
                self.lines.append((pts, pts.mean(axis=0), float(np.hypot(*(pts.max(axis=0) - pts.min(axis=0)))), int(lm.seg_lane[part[0]]), float(s[part, 4].mean())))


def view(x0: float, y0: float, rot: float, scale: float, cx: float, cy: float):
    """World (map) points to screen pixels: heading up, the truck at (cx, cy)."""
    fx, fy, rx, ry = math.cos(rot), math.sin(rot), -math.sin(rot), math.cos(rot)

    def to_screen(p: np.ndarray) -> np.ndarray:
        d = np.asarray(p, dtype=float) - (x0, y0)
        return np.stack([cx + scale * (d[..., 0] * rx + d[..., 1] * ry), cy - scale * (d[..., 0] * fx + d[..., 1] * fy)], axis=-1)
    return to_screen


def box(front: np.ndarray, rot: float, length: float, width: float) -> np.ndarray:
    u, r = np.array([math.cos(rot), math.sin(rot)]), np.array([-math.sin(rot), math.cos(rot)])
    return np.array([front + r * width / 2, front - r * width / 2, front - r * width / 2 - u * length, front + r * width / 2 - u * length])


def polyline(d: ImageDraw.ImageDraw, pts: np.ndarray, fill, width: int) -> None:
    d.line([tuple(p) for p in pts.tolist()], fill=fill, width=width, joint="curve")


def panel(img: Image.Image, t: float, f, n: dict, total: float, speed: float) -> None:
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    x0, m = W - PANEL_W, 18
    d.rectangle([x0, 0, W, H], fill=(11, 13, 19, 228))
    d.line([x0, 0, x0, H], fill=(60, 66, 80, 255))
    d.text((x0 + m, 14), "AutoHaul", font=font(24, True), fill=TEXT)
    d.text((W - m, 22), "driving", font=font(14, True), fill=GREEN, anchor="ra")

    v, tgt, lim = f.speed * 3.6, float(n.get("target_kmh") or 0), float(n.get("limit_kmh") or 0)
    d.text((x0 + m, 52), f"{max(v, 0):.0f}", font=font(64, True), fill=TEXT)
    d.text((x0 + m + 96, 88), "km/h", font=font(16), fill=DIM)
    if lim > 0:
        d.ellipse([W - m - 52, 66, W - m, 118], fill=(245, 245, 245), outline=RED, width=6)
        d.text((W - m - 26, 92), f"{lim:.0f}", font=font(18, True), fill=(20, 20, 20), anchor="mm")

    # speed bar: now, and where AutoHaul is taking it
    y, full = 146, PANEL_W - 2 * m
    d.rounded_rectangle([x0 + m, y, x0 + m + full, y + 8], 4, fill=(44, 50, 62))
    top = 100.0
    d.rounded_rectangle([x0 + m, y, x0 + m + full * min(max(v, 0) / top, 1), y + 8], 4, fill=ROUTE)
    mark = x0 + m + full * min(tgt / top, 1)
    d.rectangle([mark - 1, y - 4, mark + 1, y + 12], fill=TEXT)

    brake, throttle = float(n.get("brake") or 0), float(n.get("throttle") or 0)
    if brake > 0.03 or v - tgt > 3:
        doing, col = f"Slowing to {tgt:.0f} km/h", AMBER
    elif v < 0.5 and tgt < 0.5:
        doing, col = "Waiting", RED
    elif tgt - v > 3:
        doing, col = f"Speeding up to {tgt:.0f} km/h", GREEN
    else:
        doing, col = f"Holding {tgt:.0f} km/h", TEXT

    def heading(y: int, s: str) -> None:
        d.text((x0 + m, y), s, font=font(12, True), fill=DIM)
    heading(170, "DECISION")
    d.text((x0 + m, 186), doing, font=font(19, True), fill=col)
    heading(222, "WHY")
    why = str(n.get("target_reason") or "")
    why = {"limit": "the speed limit", "max speed": "the top speed set for the truck"}.get(why, why)
    for i, line in enumerate(textwrap.wrap(why, 33)[:3]):
        d.text((x0 + m, 238 + 21 * i), line, font=font(16), fill=TEXT)

    heading(312, "READS")
    rows = [("Off the lane line", f"{float(n.get('ctrl_lateral') or 0):+.2f} m"),
            ("Lane match", f"{float(n.get('confidence') or 0) * 100:.0f} %"),
            ("Steering", f"{float(n.get('steer_target') or 0):+.2f}")]
    junction = str(n.get("junction") or "")
    rows.append(("Junction", junction.split(" (")[0][:24] if junction else "none near"))
    for i, (a, b) in enumerate(rows):
        d.text((x0 + m, 330 + 23 * i), a, font=font(14), fill=DIM)
        d.text((W - m, 330 + 23 * i), b, font=font(14, True), fill=TEXT, anchor="ra")

    # indicators, as the truck's own lamps were
    for lamp, side, xc in ((f.light_lblinker, "left", x0 + m + 16), (f.light_rblinker, "right", W - m - 16)):
        s = 1 if side == "right" else -1
        col = AMBER if lamp else (44, 50, 62)
        y = 448
        d.polygon([(xc + s * 14, y), (xc - s * 2, y - 12), (xc - s * 2, y - 5), (xc - s * 14, y - 5), (xc - s * 14, y + 5), (xc - s * 2, y + 5), (xc - s * 2, y + 12)], fill=col)
    d.text((x0 + PANEL_W / 2, 448), "indicators", font=font(12), fill=DIM, anchor="mm")

    d.text((x0 + m, 484), "A real drive with a loaded trailer," if f.trailers[0].connected else "A real drive,", font=font(13), fill=DIM)
    d.text((x0 + m, 502), f"drawn from its recording. {speed:g}x speed.", font=font(13), fill=DIM)
    d.rectangle([x0, H - 3, x0 + PANEL_W * min(t / total, 1), H], fill=ROUTE)
    img.alpha_composite(ov)


def render(args) -> None:
    cfg = load_config()
    lm = LaneMap.load_or_build(map_dir(cfg), log=lambda *a: None)
    rows = load_stretch(args.name, args.start, args.end)
    if not rows:
        raise SystemExit("nothing recorded in that stretch")
    total = rows[-1][0]
    ts = np.array([r[0] for r in rows])
    xs = np.array([r[1].truck_placement.x for r in rows])
    ys = np.array([r[1].truck_placement.z for r in rows])
    rots = np.unwrap([heading_to_rotation(r[1].truck_placement.heading) for r in rows])
    speeds = np.array([r[1].speed for r in rows])
    driven = []
    for r in rows:
        ln = r[2].get("lane")
        if ln is not None and (not driven or driven[-1] != int(ln)):
            driven.append(int(ln))
    world = Map(lm, xs, ys, float(rows[0][1].truck_placement.y))
    route = [lm.lane_points(ln) for ln in driven]
    geom = TruckGeometry()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    mp4 = out.with_suffix(".mp4")
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    enc = subprocess.Popen([ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                            "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "slow", "-movflags", "+faststart", str(mp4)],
                           stdin=subprocess.PIPE)
    n_frames = int(total / args.speed * FPS)
    cx, cy = (W - PANEL_W) / 2 * SS, H * 0.68 * SS
    # the camera eases: a smoothed heading, and a zoom that opens up with speed
    smooth = int(max(1, 0.6 * len(ts) / max(total, 1e-3)))
    kernel = np.ones(smooth) / smooth
    cam = np.convolve(np.pad(rots, smooth, mode="edge"), kernel, mode="same")[smooth:-smooth]
    zoom = np.convolve(np.pad(np.interp(speeds, [3.0, 23.0], [7.0, 3.1]), smooth * 3, mode="edge"), np.ones(smooth * 3) / (smooth * 3), mode="same")[smooth * 3:-smooth * 3]
    for k in range(n_frames):
        t = k / FPS * args.speed
        i = min(int(np.searchsorted(ts, t)), len(rows) - 1)
        _, f, n = rows[i]
        x, y, rot, scale = xs[i], ys[i], float(cam[i]), float(zoom[i]) * SS
        to = view(x, y, rot, scale, cx, cy)
        img = Image.new("RGB", (W * SS, H * SS), BG)
        d = ImageDraw.Draw(img)
        reach = 1.3 * W * SS / scale
        here = float(f.truck_placement.y)
        near = [ln for ln in world.lines if math.hypot(*(ln[1] - (x, y))) < reach + ln[2] / 2]
        # lanes at another height (a bridge over, a road under) stay in the background
        for far in (True, False):
            for pts, _, _, _, h in near:
                if (abs(h - here) > 4.0) == far:
                    polyline(d, to(pts), ROAD_FAR if far else ROAD, int(4.4 * scale))
        for pts, _, _, _, h in near:
            if abs(h - here) <= 4.0:
                polyline(d, to(pts), CENTRE, max(1, int(0.12 * scale)))
        for pts in route:
            polyline(d, to(pts), ROUTE, max(2, int(0.7 * scale)))
        trail = np.column_stack([xs[max(0, i - 6000):i + 1], ys[max(0, i - 6000):i + 1]])[::4]
        if len(trail) > 1:
            polyline(d, to(trail), TRAIL, max(2, int(0.8 * scale)))

        # the truck
        here_rot = float(rots[i])
        front = np.array([x, y]) + geom.front_bumper * np.array([math.cos(here_rot), math.sin(here_rot)])
        tr = f.trailers[0]
        cab = box(front, here_rot, CAB_LEN, TRUCK_W)
        back = cab[2:4]
        if tr.connected:
            rt = heading_to_rotation(tr.placement.heading)
            pin = np.array([x, y]) - geom.rear_axle * np.array([math.cos(here_rot), math.sin(here_rot)])
            body = box(pin + 1.2 * np.array([math.cos(rt), math.sin(rt)]), rt, TRAILER_LEN, TRUCK_W)
            d.polygon([tuple(p) for p in to(body).tolist()], fill=TRAILER)
            back = body[2:4]
        d.polygon([tuple(p) for p in to(cab).tolist()], fill=CAB)
        if f.light_brake:
            for p in back:
                q = to(p)
                r = 0.75 * scale
                d.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], fill=RED)
        for lamp, a, b in ((f.light_lblinker, cab[1], cab[2]), (f.light_rblinker, cab[0], cab[3])):
            if lamp:
                for p in (a, b):
                    q = to(p)
                    r = 0.8 * scale
                    d.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], fill=AMBER)

        frame = img.resize((W, H), Image.LANCZOS).convert("RGBA")
        panel(frame, t, f, n, total, args.speed)
        enc.stdin.write(frame.convert("RGB").tobytes())
        if k % 150 == 0:
            print(f"{k}/{n_frames}", flush=True)
        if k == int(n_frames * 0.5):
            frame.convert("RGB").save(out.with_suffix(".still.png"))
    enc.stdin.close()
    enc.wait()
    # the README plays only a GIF inline; two passes for its palette keep it small and clean
    filt = f"fps={args.gif_fps},scale={args.gif_width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle"
    subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(mp4), "-filter_complex", filt, str(out.with_suffix(".gif"))], check=True)
    for p in (mp4, out.with_suffix(".gif")):
        print(p, f"{p.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("name", help="recording, e.g. 20261029-020945")
    ap.add_argument("start", type=float, help="seconds into the recording")
    ap.add_argument("end", type=float)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--out", default="docs/media/drive", help="path without extension: writes .mp4, .gif and a .still.png")
    ap.add_argument("--gif-fps", type=int, default=12)
    ap.add_argument("--gif-width", type=int, default=720)
    render(ap.parse_args())
