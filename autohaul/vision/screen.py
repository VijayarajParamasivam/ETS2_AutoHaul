"""Screen capture of the game window with plain GDI (ctypes + numpy, no extra packages).

The game runs windowed, so its picture is on the desktop: BitBlt from the screen DC copies any rectangle of it.
Coordinates are physical pixels (the process is made DPI aware), relative to the game window's client area.
"""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import struct
import zlib

import numpy as np

from autohaul.io.win32 import foreground_pid, main_window

user32 = ctypes.WinDLL("user32", use_last_error=True)
gdi32 = ctypes.WinDLL("gdi32", use_last_error=True)
try:                                                    # physical pixels, not scaled ones
    ctypes.WinDLL("shcore").SetProcessDpiAwareness(2)
except (OSError, AttributeError):
    user32.SetProcessDPIAware()

user32.GetDC.argtypes = [wt.HWND]
user32.GetDC.restype = wt.HDC
user32.ReleaseDC.argtypes = [wt.HWND, wt.HDC]
user32.GetClientRect.argtypes = [wt.HWND, ctypes.POINTER(wt.RECT)]
user32.ClientToScreen.argtypes = [wt.HWND, ctypes.POINTER(wt.POINT)]
gdi32.CreateCompatibleDC.argtypes = [wt.HDC]
gdi32.CreateCompatibleDC.restype = wt.HDC
gdi32.CreateCompatibleBitmap.argtypes = [wt.HDC, ctypes.c_int, ctypes.c_int]
gdi32.CreateCompatibleBitmap.restype = wt.HBITMAP
gdi32.SelectObject.argtypes = [wt.HDC, wt.HGDIOBJ]
gdi32.SelectObject.restype = wt.HGDIOBJ
gdi32.BitBlt.argtypes = [wt.HDC, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, wt.HDC, ctypes.c_int,
                         ctypes.c_int, wt.DWORD]
gdi32.GetDIBits.argtypes = [wt.HDC, wt.HBITMAP, wt.UINT, wt.UINT, ctypes.c_void_p, ctypes.c_void_p, wt.UINT]
gdi32.DeleteObject.argtypes = [wt.HGDIOBJ]
gdi32.DeleteDC.argtypes = [wt.HDC]
SRCCOPY = 0x00CC0020


class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [("biSize", wt.DWORD), ("biWidth", ctypes.c_long), ("biHeight", ctypes.c_long),
                ("biPlanes", wt.WORD), ("biBitCount", wt.WORD), ("biCompression", wt.DWORD),
                ("biSizeImage", wt.DWORD), ("biXPelsPerMeter", ctypes.c_long), ("biYPelsPerMeter", ctypes.c_long),
                ("biClrUsed", wt.DWORD), ("biClrImportant", wt.DWORD)]


class GameScreen:
    def __init__(self, pid: int):
        self.pid = pid
        self.hwnd = main_window(pid)

    def client(self) -> tuple[int, int, int, int] | None:
        """(left, top, width, height) of the game's picture on the screen, physical pixels."""
        if not self.hwnd:
            self.hwnd = main_window(self.pid)
            if not self.hwnd:
                return None
        r = wt.RECT()
        if not user32.GetClientRect(self.hwnd, ctypes.byref(r)):
            self.hwnd = 0
            return None
        pt = wt.POINT(0, 0)
        user32.ClientToScreen(self.hwnd, ctypes.byref(pt))
        return pt.x, pt.y, r.right - r.left, r.bottom - r.top

    def grab(self, x: int, y: int, w: int, h: int) -> np.ndarray | None:
        """(h, w, 3) RGB uint8 of a rectangle of the game's picture (client coordinates), or None. None too while
        another window is in front: the screen then shows that window, not the game."""
        if foreground_pid() != self.pid:
            return None
        c = self.client()
        if c is None or w <= 0 or h <= 0:
            return None
        sdc = user32.GetDC(None)
        mdc = gdi32.CreateCompatibleDC(sdc)
        bmp = gdi32.CreateCompatibleBitmap(sdc, w, h)
        old = gdi32.SelectObject(mdc, bmp)
        try:
            gdi32.BitBlt(mdc, 0, 0, w, h, sdc, c[0] + x, c[1] + y, SRCCOPY)
            bi = BITMAPINFOHEADER(ctypes.sizeof(BITMAPINFOHEADER), w, -h, 1, 32, 0, 0, 0, 0, 0, 0)
            buf = (ctypes.c_ubyte * (w * h * 4))()
            if not gdi32.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bi), 0):
                return None
            a = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)
            return a[:, :, 2::-1].copy()             # BGRA -> RGB
        finally:
            gdi32.SelectObject(mdc, old)
            gdi32.DeleteObject(bmp)
            gdi32.DeleteDC(mdc)
            user32.ReleaseDC(None, sdc)

    def grab_all(self) -> np.ndarray | None:
        c = self.client()
        return None if c is None else self.grab(0, 0, c[2], c[3])


def write_png(path, rgb: np.ndarray) -> None:
    """Minimal PNG writer (for debug pictures)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[i].tobytes() for i in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
