"""Facts about the user's game setup that affect whether AutoHaul can drive:
the active profile's steering mode, and whether the game runs elevated (as administrator)."""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import re
import winreg
from pathlib import Path

from autohaul.infra.config import Config

_k32 = ctypes.WinDLL("kernel32", use_last_error=True)
_adv = ctypes.WinDLL("advapi32", use_last_error=True)
_k32.OpenProcess.argtypes = [wt.DWORD, wt.BOOL, wt.DWORD]
_k32.OpenProcess.restype = wt.HANDLE
_k32.CloseHandle.argtypes = [wt.HANDLE]
_adv.OpenProcessToken.argtypes = [wt.HANDLE, wt.DWORD, ctypes.POINTER(wt.HANDLE)]
_adv.OpenProcessToken.restype = wt.BOOL
_adv.GetTokenInformation.argtypes = [wt.HANDLE, ctypes.c_int, ctypes.c_void_p, wt.DWORD, ctypes.POINTER(wt.DWORD)]
_adv.GetTokenInformation.restype = wt.BOOL

PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
TOKEN_QUERY = 0x0008
TOKEN_ELEVATION = 20
COMPAT_LAYERS = r"Software\Microsoft\Windows NT\CurrentVersion\AppCompatFlags\Layers"


def is_elevated(pid: int) -> bool | None:
    """True if the process runs elevated (as administrator); None if it can't be inspected."""
    h = _k32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
    if not h:
        return None
    try:
        tok = wt.HANDLE()
        if not _adv.OpenProcessToken(h, TOKEN_QUERY, ctypes.byref(tok)):
            return None
        try:
            value, size = wt.DWORD(), wt.DWORD()
            if not _adv.GetTokenInformation(tok, TOKEN_ELEVATION, ctypes.byref(value), 4, ctypes.byref(size)):
                return None
            return bool(value.value)
        finally:
            _k32.CloseHandle(tok)
    finally:
        _k32.CloseHandle(h)


def run_as_admin_flag(cfg: Config) -> str | None:
    """'HKCU' / 'HKLM' if eurotrucks2.exe has the RUNASADMIN compatibility flag, else None."""
    exe = str(Path(cfg.get("game.install_dir")) / "bin" / "win_x64" / "eurotrucks2.exe").lower()
    for name, hive in (("HKCU", winreg.HKEY_CURRENT_USER), ("HKLM", winreg.HKEY_LOCAL_MACHINE)):
        try:
            with winreg.OpenKey(hive, COMPAT_LAYERS) as key:
                i = 0
                while True:
                    try:
                        value_name, data, _ = winreg.EnumValue(key, i)
                    except OSError:
                        break
                    i += 1
                    if value_name.lower() == exe and "RUNASADMIN" in str(data).upper():
                        return name
        except OSError:
            continue
    return None


def active_controls_file(cfg: Config) -> Path | None:
    """controls.sii of the most recently used profile."""
    profiles = Path(cfg.get("game.home_dir")) / "profiles"
    files = list(profiles.glob("*/controls.sii")) if profiles.exists() else []
    return max(files, key=lambda p: p.stat().st_mtime) if files else None


def controls_constants(path: Path) -> dict[str, float]:
    text = path.read_text(encoding="utf-8", errors="replace")
    return {m.group(1): float(m.group(2)) for m in re.finditer(r'"constant (\w+) ([-\d.]+)"', text)}


def steering_mode(cfg: Config) -> str | None:
    """'relative' (steering input = turn rate) or 'absolute' (input = wheel position); None if unknown.

    In relative mode the game integrates every steering input, AutoHaul's included, so a command
    turns the wheel at a speed instead of holding it at an angle (seen on 1.53.2.1, 2026-10-19).
    """
    path = active_controls_file(cfg)
    if path is None:
        return None
    value = controls_constants(path).get("c_relatsteer")
    if value is None:
        return None
    return "relative" if value >= 0.5 else "absolute"
