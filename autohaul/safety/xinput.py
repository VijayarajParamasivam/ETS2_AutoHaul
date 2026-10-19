"""XInput polling of the physical controllers (independent of the game's own input mixing)."""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
from dataclasses import dataclass

BUTTONS = {
    "DPAD_UP": 0x0001, "DPAD_DOWN": 0x0002, "DPAD_LEFT": 0x0004, "DPAD_RIGHT": 0x0008,
    "START": 0x0010, "BACK": 0x0020, "LEFT_THUMB": 0x0040, "RIGHT_THUMB": 0x0080,
    "LEFT_SHOULDER": 0x0100, "RIGHT_SHOULDER": 0x0200, "A": 0x1000, "B": 0x2000, "X": 0x4000, "Y": 0x8000,
}
ERROR_SUCCESS = 0


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", wt.WORD), ("bLeftTrigger", ctypes.c_ubyte), ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short), ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short), ("sThumbRY", ctypes.c_short)]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wt.DWORD), ("Gamepad", XINPUT_GAMEPAD)]


def _load():
    for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            dll = ctypes.WinDLL(name)
            dll.XInputGetState.argtypes = [wt.DWORD, ctypes.POINTER(XINPUT_STATE)]
            dll.XInputGetState.restype = wt.DWORD
            return dll
        except OSError:
            continue
    return None


_xinput = _load()


@dataclass
class PadState:
    index: int
    buttons: int
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    left_trigger: float
    right_trigger: float

    def pressed(self, name: str) -> bool:
        return bool(self.buttons & BUTTONS[name])


def chord_mask(names: list[str]) -> int:
    mask = 0
    for n in names:
        mask |= BUTTONS[n.upper()]
    return mask


def poll() -> list[PadState]:
    """States of all connected XInput controllers (empty if XInput is unavailable)."""
    if _xinput is None:
        return []
    out = []
    st = XINPUT_STATE()
    for i in range(4):
        if _xinput.XInputGetState(i, ctypes.byref(st)) != ERROR_SUCCESS:
            continue
        g = st.Gamepad
        out.append(PadState(i, g.wButtons, g.sThumbLX / 32767.0, g.sThumbLY / 32767.0,
                            g.sThumbRX / 32767.0, g.sThumbRY / 32767.0,
                            g.bLeftTrigger / 255.0, g.bRightTrigger / 255.0))
    return out
