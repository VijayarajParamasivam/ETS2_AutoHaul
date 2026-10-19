"""Multiplayer guard: AutoHaul is single-player only and refuses to drive in Convoy or TruckersMP."""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt

from autohaul.io import _layout as L
from autohaul.io.telemetry import channel_seen

kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
PROCESS_QUERY_INFORMATION = 0x0400
PROCESS_VM_READ = 0x0010
LIST_MODULES_ALL = 0x03

kernel32.OpenProcess.argtypes = [wt.DWORD, wt.BOOL, wt.DWORD]
kernel32.OpenProcess.restype = wt.HANDLE
kernel32.K32EnumProcessModulesEx.argtypes = [wt.HANDLE, ctypes.POINTER(wt.HMODULE), wt.DWORD,
                                             ctypes.POINTER(wt.DWORD), wt.DWORD]
kernel32.K32EnumProcessModulesEx.restype = wt.BOOL
kernel32.K32GetModuleBaseNameW.argtypes = [wt.HANDLE, wt.HMODULE, wt.LPWSTR, wt.DWORD]
kernel32.K32GetModuleBaseNameW.restype = wt.DWORD
kernel32.CloseHandle.argtypes = [wt.HANDLE]


def process_modules(pid: int) -> set[str] | None:
    """Lower-cased base names of the DLLs loaded in `pid`, or None if the process can't be inspected."""
    h = kernel32.OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, False, pid)
    if not h:
        return None
    try:
        needed = wt.DWORD()
        arr = (wt.HMODULE * 1024)()
        if not kernel32.K32EnumProcessModulesEx(h, arr, ctypes.sizeof(arr), ctypes.byref(needed), LIST_MODULES_ALL):
            return None
        count = min(needed.value // ctypes.sizeof(wt.HMODULE), 1024)
        names = set()
        buf = ctypes.create_unicode_buffer(260)
        for i in range(count):
            if kernel32.K32GetModuleBaseNameW(h, arr[i], buf, 260):
                names.add(buf.value.lower())
        return names
    finally:
        kernel32.CloseHandle(h)


def multiplayer_reasons(frame: L.TelemetryFrame | None, game_pid: int, modules: list[str]) -> list[str]:
    """Empty list = single-player. Otherwise human-readable reasons."""
    reasons = []
    if frame is not None and channel_seen(frame, "multiplayer.time.offset") and frame.multiplayer_time_offset != 0:
        reasons.append(f"Convoy session (multiplayer.time.offset={frame.multiplayer_time_offset})")
    if game_pid:
        loaded = process_modules(game_pid)
        if loaded is not None:
            for m in modules:
                if m.lower() in loaded:
                    reasons.append(f"{m} loaded in the game process (TruckersMP)")
    return reasons
