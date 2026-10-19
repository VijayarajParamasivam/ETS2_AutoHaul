"""Global low-level keyboard monitor (WH_KEYBOARD_LL) for the kill switch and manual override.

The hook only observes: every event is passed on to the game unchanged. It runs on its own
thread with a message loop (required by Windows for LL hooks) and must return quickly, so the
callback only updates counters.
"""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import threading

from autohaul.io.win32 import qpc

user32 = ctypes.WinDLL("user32", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

WH_KEYBOARD_LL = 13
HC_ACTION = 0
WM_KEYDOWN, WM_KEYUP, WM_SYSKEYDOWN, WM_SYSKEYUP = 0x0100, 0x0101, 0x0104, 0x0105
WM_QUIT = 0x0012
LLKHF_INJECTED = 0x10

LRESULT = ctypes.c_ssize_t


class KBDLLHOOKSTRUCT(ctypes.Structure):
    _fields_ = [("vkCode", wt.DWORD), ("scanCode", wt.DWORD), ("flags", wt.DWORD), ("time", wt.DWORD),
                ("dwExtraInfo", ctypes.c_size_t)]


HOOKPROC = ctypes.WINFUNCTYPE(LRESULT, ctypes.c_int, wt.WPARAM, wt.LPARAM)
user32.SetWindowsHookExW.argtypes = [ctypes.c_int, HOOKPROC, wt.HINSTANCE, wt.DWORD]
user32.SetWindowsHookExW.restype = wt.HHOOK
user32.CallNextHookEx.argtypes = [wt.HHOOK, ctypes.c_int, wt.WPARAM, wt.LPARAM]
user32.CallNextHookEx.restype = LRESULT
user32.UnhookWindowsHookEx.argtypes = [wt.HHOOK]
user32.GetMessageW.argtypes = [ctypes.POINTER(wt.MSG), wt.HWND, wt.UINT, wt.UINT]
user32.PostThreadMessageW.argtypes = [wt.DWORD, wt.UINT, wt.WPARAM, wt.LPARAM]
kernel32.GetModuleHandleW.argtypes = [wt.LPCWSTR]
kernel32.GetModuleHandleW.restype = wt.HMODULE
kernel32.GetCurrentThreadId.restype = wt.DWORD


class KeyState:
    """Thread-safe key state: what is held now, and how many presses each key has seen."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._down: set[int] = set()
        self._presses: dict[int, int] = {}
        self._last_press_qpc: dict[int, int] = {}

    def on_event(self, vk: int, down: bool) -> None:
        with self._lock:
            if down:
                if vk not in self._down:
                    self._presses[vk] = self._presses.get(vk, 0) + 1
                    self._last_press_qpc[vk] = qpc()
                self._down.add(vk)
            else:
                self._down.discard(vk)

    def is_down(self, vk: int) -> bool:
        with self._lock:
            return vk in self._down

    def any_down(self, vks: tuple[int, ...]) -> int | None:
        with self._lock:
            for v in vks:
                if v in self._down:
                    return v
        return None

    def press_count(self, vk: int) -> int:
        with self._lock:
            return self._presses.get(vk, 0)

    def press_counts(self, vks: tuple[int, ...]) -> tuple[int, ...]:
        with self._lock:
            return tuple(self._presses.get(v, 0) for v in vks)

    def last_press_qpc(self, vk: int) -> int:
        """QueryPerformanceCounter timestamp of the most recent press of `vk` (0 = never)."""
        with self._lock:
            return self._last_press_qpc.get(vk, 0)


class KeyboardMonitor:
    """Starts the LL hook thread. Use .state (KeyState) to query keys."""

    def __init__(self) -> None:
        self.state = KeyState()
        self._thread: threading.Thread | None = None
        self._thread_id = 0
        self._started = threading.Event()
        self._hook = None
        self._proc = HOOKPROC(self._callback)  # keep a reference: ctypes callbacks must not be collected
        self.error: str | None = None

    def _callback(self, code: int, wparam: int, lparam: int) -> int:
        if code == HC_ACTION:
            kb = ctypes.cast(lparam, ctypes.POINTER(KBDLLHOOKSTRUCT)).contents
            if wparam in (WM_KEYDOWN, WM_SYSKEYDOWN):
                self.state.on_event(kb.vkCode, True)
            elif wparam in (WM_KEYUP, WM_SYSKEYUP):
                self.state.on_event(kb.vkCode, False)
        return user32.CallNextHookEx(None, code, wparam, lparam)

    def _run(self) -> None:
        self._thread_id = kernel32.GetCurrentThreadId()
        self._hook = user32.SetWindowsHookExW(WH_KEYBOARD_LL, self._proc, kernel32.GetModuleHandleW(None), 0)
        if not self._hook:
            self.error = f"SetWindowsHookExW failed ({ctypes.get_last_error()})"
            self._started.set()
            return
        self._started.set()
        msg = wt.MSG()
        while user32.GetMessageW(ctypes.byref(msg), None, 0, 0) > 0:
            pass
        user32.UnhookWindowsHookEx(self._hook)
        self._hook = None

    def start(self, timeout: float = 2.0) -> bool:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="keyboard-hook", daemon=True)
            self._thread.start()
            self._started.wait(timeout)
        return self.error is None and self._hook is not None

    def stop(self) -> None:
        if self._thread is not None:
            user32.PostThreadMessageW(self._thread_id, WM_QUIT, 0, 0)
            self._thread.join(timeout=2.0)
            self._thread = None
