"""See-through overlay on the game window: what AutoHaul reads, what it's doing, and why.

    python tools/overlay.py [--corner tl|tr|bl|br] [--alpha 0.78] [--port 47821]

The launcher starts it for you ("Show overlay"). It's a separate process listening to tools/drive.py's feed
(autohaul/infra/overlay_feed.py), so it can't affect the driving. Clicks go through it and it never takes the focus
(AutoHaul hands back if the game loses focus); it's hidden while the game isn't in front, and left out of screen
captures so the traffic-light reader doesn't see it (--capture leaves it in: for screenshots of the overlay).
"""

from __future__ import annotations

import argparse
import ctypes
import json
import socket
import sys
import time
import tkinter as tk
from ctypes import wintypes as wt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autohaul.infra.overlay_feed import DEFAULT_PORT  # noqa: E402

GAME_TITLE = "Euro Truck Simulator 2"
GWL_EXSTYLE = -20
WS_EX = 0x00080000 | 0x00000020 | 0x08000000 | 0x00000080    # layered, click-through, no-activate, tool window
WDA_EXCLUDEFROMCAPTURE = 0x11
GA_ROOT = 2
WIDTH = 430
BG, FG, DIM = "#0d1117", "#e6edf3", "#8b949e"
HEAD = {"reads": "#58a6ff", "doing": "#3fb950", "why": "#d29922", "learned": "#bc8cff", "events": "#8b949e"}

user32 = ctypes.windll.user32
user32.GetAncestor.restype = wt.HWND
user32.GetAncestor.argtypes = [wt.HWND, wt.UINT]
user32.GetWindowLongPtrW.restype = ctypes.c_ssize_t
user32.GetWindowLongPtrW.argtypes = [wt.HWND, ctypes.c_int]
user32.SetWindowLongPtrW.restype = ctypes.c_ssize_t
user32.SetWindowLongPtrW.argtypes = [wt.HWND, ctypes.c_int, ctypes.c_ssize_t]
user32.FindWindowW.restype = wt.HWND
user32.GetForegroundWindow.restype = wt.HWND


def game_rect(hwnd) -> tuple[int, int, int, int] | None:
    """The game's client area on the screen: (x, y, w, h)."""
    r = wt.RECT()
    if not user32.GetClientRect(hwnd, ctypes.byref(r)):
        return None
    pt = wt.POINT(0, 0)
    user32.ClientToScreen(hwnd, ctypes.byref(pt))
    return pt.x, pt.y, r.right, r.bottom


class Overlay:
    def __init__(self, root: tk.Tk, args):
        self.root, self.args = root, args
        self.state: dict = {}
        self.last_state = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", args.port))
        self.sock.setblocking(False)
        root.withdraw()
        root.overrideredirect(True)
        root.attributes("-topmost", True)
        root.attributes("-alpha", args.alpha)
        root.configure(bg=BG)
        self.text = tk.Text(root, bg=BG, fg=FG, bd=0, highlightthickness=0, wrap="word", padx=8, pady=5,
                            font=("Segoe UI", 9), cursor="arrow", width=1, height=1)
        self.text.pack(fill="both", expand=True)
        self.text.tag_configure("mode", font=("Segoe UI Semibold", 10))
        self.text.tag_configure("label", foreground=DIM)
        for k, c in HEAD.items():
            self.text.tag_configure(k, foreground=c, font=("Segoe UI Semibold", 8), spacing1=3)
        self.text.tag_configure("event", foreground=DIM, font=("Segoe UI", 8))
        self.text.tag_configure("on", foreground="#3fb950")
        self.text.tag_configure("off", foreground="#d29922")
        root.update_idletasks()
        self.hwnd = user32.GetAncestor(root.winfo_id(), GA_ROOT)
        user32.SetWindowLongPtrW(self.hwnd, GWL_EXSTYLE, user32.GetWindowLongPtrW(self.hwnd, GWL_EXSTYLE) | WS_EX)
        if not args.capture:
            user32.SetWindowDisplayAffinity(self.hwnd, WDA_EXCLUDEFROMCAPTURE)
        self.shown = False
        self._drawn = ""
        self._geom = ""
        self._h = 0
        self._shrink_since = None
        self.shrink_after_s = 3.0
        self.tick()

    def receive(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except OSError:
                return
            try:
                self.state, self.last_state = json.loads(data.decode("utf-8")), time.monotonic()
            except ValueError:
                pass

    def section(self, key: str, title: str, rows) -> None:
        if not rows:
            return
        t = self.text
        t.insert("end", f"{title}\n", key)
        for row in rows:
            if isinstance(row, str):
                t.insert("end", row + "\n", "event" if key == "events" else ())
            else:
                t.insert("end", f"{row[0]}  ", "label")
                t.insert("end", f"{row[1]}\n")

    def draw(self) -> None:
        s = self.state
        stale = time.monotonic() - self.last_state > 3.0
        if stale:
            s = {"mode": "AutoHaul isn't running (start it from the launcher)"}
        key = json.dumps(s, sort_keys=True)
        if key == self._drawn:
            return
        self._drawn = key
        t = self.text
        t.configure(state="normal")
        t.delete("1.0", "end")
        t.insert("end", "● ", "on" if s.get("engaged") else "off")
        t.insert("end", (s.get("mode") or "") + "\n", "mode")
        self.section("reads", "WHAT IT READS", s.get("reads"))
        self.section("doing", "WHAT IT'S DOING", s.get("doing"))
        self.section("why", "WHY", s.get("why"))
        self.section("learned", "WHAT IT HAS LEARNED", s.get("learned"))
        self.section("events", "RECENTLY", s.get("events"))
        t.configure(state="disabled")

    def settled_height(self, need: int, now: float) -> int:
        """Grows at once, shrinks only after `shrink_after_s` with less to show: a line that comes and goes (a lead
        vehicle, a lane change) doesn't make the panel jump."""
        if need >= self._h:
            self._h, self._shrink_since = need, None
        elif self._shrink_since is None:
            self._shrink_since = now
        elif now - self._shrink_since >= self.shrink_after_s:
            self._h, self._shrink_since = need, None
        return self._h

    def place(self) -> None:
        game = user32.FindWindowW(None, GAME_TITLE)
        fg = user32.GetForegroundWindow()
        if not game or (fg != game and not self.args.always):
            if self.shown:
                self.root.withdraw()
                self.shown = False
            return
        rect = game_rect(game)
        if rect is None:
            return
        gx, gy, gw, gh = rect
        w = min(WIDTH, gw - 24)
        if self._h == 0:                                   # the first time: it can only be measured once it's shown
            self.root.geometry(f"{w}x200+{gx + 12}+{gy + 12}")
            self.root.deiconify()
            self.shown = True
            self.root.update()
        need = int(self.text.tk.call(self.text._w, "count", "-update", "-ypixels", "1.0", "end")) + 12
        h = self.settled_height(min(gh - 24, need), time.monotonic())
        c = self.args.corner
        x = gx + 12 if c[1] == "l" else gx + gw - w - 12
        y = gy + 12 if c[0] == "t" else gy + gh - h - 12
        geom = f"{w}x{h}+{x}+{y}"
        if geom != self._geom:                             # only when it really changed: no flicker
            self.root.geometry(geom)
            self._geom = geom
        if not self.shown:
            self.root.deiconify()
            self.shown = True

    def tick(self) -> None:
        self.receive()
        self.draw()
        self.place()
        self.root.after(200, self.tick)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--corner", default="tl", choices=["tl", "tr", "bl", "br"])
    ap.add_argument("--alpha", type=float, default=0.72)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--capture", action="store_true", help="leave it in screen captures")
    ap.add_argument("--always", action="store_true", help="show it even when the game isn't in front")
    args = ap.parse_args()
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)     # the game's rectangle is in physical pixels
    except OSError:
        pass
    root = tk.Tk()
    root.title("AutoHaul overlay")
    Overlay(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
