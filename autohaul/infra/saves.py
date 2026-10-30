"""The job trailer's position, from the game's save.

Telemetry knows a trailer only once it's coupled, and the ETS2LA traffic data has AI vehicles only. The game writes
a save when a job is accepted (`autosave_job`) and every few minutes while driving; its `player` unit holds the
assigned trailer's world placement (`trailer_placement`: position, then rotation as a quaternion) and whether it's
coupled (`assigned_trailer_connected`).

1.53 encrypts saves (even with `g_save_format 2`): "ScsC", a 32-byte HMAC, a 16-byte IV, the 4-byte plain size,
then AES-256-CBC over zlib-compressed text. The key is the game's fixed one (public in SII tools). Only the
newest save of the active profile is read, never written.
"""

from __future__ import annotations

import math
import re
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

from autohaul.world.lanemap import quat_forward_rotation

KEY = bytes([0x2a, 0x5f, 0xcb, 0x17, 0x91, 0xd2, 0x2f, 0xb6, 0x02, 0x45, 0xb3, 0xd8, 0x36, 0x9e, 0xd0, 0xb2,
             0xc2, 0x73, 0x71, 0x56, 0x3f, 0xbf, 0x1f, 0x3c, 0x9e, 0xdf, 0x6b, 0x11, 0x82, 0x5a, 0x5d, 0x0a])


@dataclass
class TrailerPose:
    x: float                # map plane (game x, z)
    y: float
    rotation: float         # map rotation of the trailer's forward direction (towards its kingpin)
    connected: bool
    saved_at: float         # the save's file time (s since epoch)


def decrypt(raw: bytes) -> bytes:
    if raw[:4] != b"ScsC":
        return raw
    from Crypto.Cipher import AES
    iv = raw[36:52]
    size = struct.unpack("<I", raw[52:56])[0]
    out = zlib.decompress(AES.new(KEY, AES.MODE_CBC, iv).decrypt(raw[56:]))
    if len(out) != size:
        raise ValueError("save decrypted to the wrong size")
    return out


def newest_save(profile_dir: Path) -> Path | None:
    files = list((Path(profile_dir) / "save").glob("*/game.sii"))
    return max(files, key=lambda p: p.stat().st_mtime) if files else None


def _num(s: str) -> float:
    s = s.strip()
    return struct.unpack(">f", bytes.fromhex(s[1:]))[0] if s.startswith("&") else float(s)


def parse_trailer(text: str, saved_at: float = 0.0) -> TrailerPose | None:
    """The assigned trailer's placement from a save's text, or None (no job trailer)."""
    i = text.find("\nplayer : ")
    if i < 0:
        return None
    block = text[i:text.find("\n}", i)]
    m = re.search(r"\n trailer_placement: \(([^)]*)\) \(([^;)]*);([^)]*)\)", block)
    if not m:
        return None
    pos = [_num(v) for v in m.group(1).split(",")]
    quat = [_num(m.group(2))] + [_num(v) for v in m.group(3).split(",")]
    if not any(pos):
        return None
    c = re.search(r"\n assigned_trailer_connected: (\w+)", block)
    return TrailerPose(pos[0], pos[2], quat_forward_rotation(*quat), bool(c and c.group(1) == "true"), saved_at)


def job_trailer(profile_dir: Path | None) -> TrailerPose | None:
    """The job trailer from the profile's newest save, or None."""
    if profile_dir is None:
        return None
    path = newest_save(profile_dir)
    if path is None:
        return None
    try:
        text = decrypt(path.read_bytes()).decode("utf-8", errors="replace")
    except (OSError, ValueError, zlib.error):
        return None
    return parse_trailer(text, path.stat().st_mtime)


def kingpin(t: TrailerPose, ahead_m: float) -> tuple[float, float]:
    """The kingpin: `ahead_m` in front of the trailer's origin (5 m on a 1.53 box trailer)."""
    return t.x + ahead_m * math.cos(t.rotation), t.y + ahead_m * math.sin(t.rotation)
