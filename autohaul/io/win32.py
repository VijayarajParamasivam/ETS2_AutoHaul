"""Small ctypes wrappers for the Win32 calls AutoHaul needs (no pywin32 dependency)."""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import mmap

kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

SYNCHRONIZE = 0x00100000
WAIT_OBJECT_0 = 0
WAIT_TIMEOUT = 0x102

kernel32.OpenEventW.argtypes = [wt.DWORD, wt.BOOL, wt.LPCWSTR]
kernel32.OpenEventW.restype = wt.HANDLE
kernel32.WaitForSingleObject.argtypes = [wt.HANDLE, wt.DWORD]
kernel32.WaitForSingleObject.restype = wt.DWORD
kernel32.CloseHandle.argtypes = [wt.HANDLE]
kernel32.CloseHandle.restype = wt.BOOL
kernel32.QueryPerformanceCounter.argtypes = [ctypes.POINTER(ctypes.c_int64)]
kernel32.QueryPerformanceFrequency.argtypes = [ctypes.POINTER(ctypes.c_int64)]
kernel32.GetCurrentProcessId.restype = wt.DWORD


def qpc() -> int:
    v = ctypes.c_int64()
    kernel32.QueryPerformanceCounter(ctypes.byref(v))
    return v.value


def qpc_frequency() -> int:
    v = ctypes.c_int64()
    kernel32.QueryPerformanceFrequency(ctypes.byref(v))
    return v.value


def current_pid() -> int:
    return int(kernel32.GetCurrentProcessId())


class NamedEvent:
    """Waits on a named event created by the bridge DLL. Missing event -> wait() just sleeps."""

    def __init__(self, name: str):
        self.name = name
        self.handle = None

    def _open(self) -> bool:
        if self.handle is None:
            h = kernel32.OpenEventW(SYNCHRONIZE, False, self.name)
            self.handle = h or None
        return self.handle is not None

    def wait(self, timeout_ms: int) -> bool:
        if not self._open():
            kernel32.Sleep(max(1, timeout_ms))
            return False
        return kernel32.WaitForSingleObject(self.handle, max(0, timeout_ms)) == WAIT_OBJECT_0

    def close(self) -> None:
        if self.handle:
            kernel32.CloseHandle(self.handle)
        self.handle = None


kernel32.Sleep.argtypes = [wt.DWORD]

FILE_MAP_READ = 0x0004
kernel32.OpenFileMappingW.argtypes = [wt.DWORD, wt.BOOL, wt.LPCWSTR]
kernel32.OpenFileMappingW.restype = wt.HANDLE
kernel32.MapViewOfFile.argtypes = [wt.HANDLE, wt.DWORD, wt.DWORD, wt.DWORD, ctypes.c_size_t]
kernel32.MapViewOfFile.restype = ctypes.c_void_p
kernel32.UnmapViewOfFile.argtypes = [ctypes.c_void_p]
kernel32.UnmapViewOfFile.restype = wt.BOOL


def create_mapping(name: str, size: int) -> mmap.mmap:
    """Open or create a named pagefile-backed mapping (used by writers)."""
    return mmap.mmap(-1, size, tagname=name, access=mmap.ACCESS_WRITE)


class ReadOnlyMapping:
    """Read-only view of a mapping someone else created. `open()` returns False if it doesn't exist."""

    def __init__(self, name: str, size: int):
        self.name = name
        self.size = size
        self.handle = None
        self.address: int | None = None

    def open(self) -> bool:
        if self.address is not None:
            return True
        h = kernel32.OpenFileMappingW(FILE_MAP_READ, False, self.name)
        if not h:
            return False
        addr = kernel32.MapViewOfFile(h, FILE_MAP_READ, 0, 0, self.size)
        if not addr:
            kernel32.CloseHandle(h)
            return False
        self.handle, self.address = h, addr
        return True

    def read(self, offset: int, n: int) -> bytes:
        assert self.address is not None and offset + n <= self.size
        return ctypes.string_at(self.address + offset, n)

    def read_into(self, cls, offset: int = 0):
        """Copy a ctypes structure out of the view."""
        assert self.address is not None
        out = cls()
        ctypes.memmove(ctypes.addressof(out), self.address + offset, ctypes.sizeof(cls))
        return out

    def u64(self, offset: int) -> int:
        assert self.address is not None
        return ctypes.c_uint64.from_address(self.address + offset).value

    def close(self) -> None:
        if self.address is not None:
            kernel32.UnmapViewOfFile(self.address)
        if self.handle:
            kernel32.CloseHandle(self.handle)
        self.handle = None
        self.address = None

user32 = ctypes.WinDLL("user32", use_last_error=True)
user32.GetForegroundWindow.restype = wt.HWND
user32.GetWindowThreadProcessId.argtypes = [wt.HWND, ctypes.POINTER(wt.DWORD)]
user32.GetWindowThreadProcessId.restype = wt.DWORD


def foreground_pid() -> int:
    """Process id owning the focused window (0 if none). The game reads input only while focused."""
    hwnd = user32.GetForegroundWindow()
    if not hwnd:
        return 0
    pid = wt.DWORD()
    user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    return int(pid.value)


WNDENUMPROC = ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
user32.EnumWindows.argtypes = [WNDENUMPROC, wt.LPARAM]
user32.IsWindowVisible.argtypes = [wt.HWND]
user32.GetWindow.argtypes = [wt.HWND, wt.UINT]
user32.GetWindow.restype = wt.HWND
user32.IsIconic.argtypes = [wt.HWND]
user32.ShowWindow.argtypes = [wt.HWND, ctypes.c_int]
user32.SetForegroundWindow.argtypes = [wt.HWND]
user32.BringWindowToTop.argtypes = [wt.HWND]
user32.AttachThreadInput.argtypes = [wt.DWORD, wt.DWORD, wt.BOOL]
user32.SwitchToThisWindow.argtypes = [wt.HWND, wt.BOOL]
user32.SwitchToThisWindow.restype = None
user32.keybd_event.argtypes = [wt.BYTE, wt.BYTE, wt.DWORD, ctypes.c_size_t]
user32.keybd_event.restype = None
kernel32.GetCurrentThreadId.restype = wt.DWORD
GW_OWNER = 4
SW_RESTORE = 9
VK_MENU = 0x12
KEYEVENTF_KEYUP = 0x0002


def main_window(pid: int) -> int:
    """The visible top-level, unowned window of process `pid` (0 if none)."""
    found = []

    def visit(hwnd, _):
        p = wt.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(p))
        if p.value == pid and user32.IsWindowVisible(hwnd) and not user32.GetWindow(hwnd, GW_OWNER):
            found.append(hwnd)
            return False
        return True

    user32.EnumWindows(WNDENUMPROC(visit), 0)
    return found[0] if found else 0


def bring_to_front(pid: int, alt_trick: bool = True) -> bool:
    """Put process `pid`'s window back in front so it gets input again. Windows only lets a process take the
    foreground in some cases, so this tries the usual ways in turn: plain SetForegroundWindow, joining the
    foreground thread's input queue, SwitchToThisWindow, and (alt_trick) a tap of Alt, which lifts the lock.
    True if the window is in front afterwards."""
    hwnd = main_window(pid)
    if not hwnd:
        return False
    if user32.IsIconic(hwnd):
        user32.ShowWindow(hwnd, SW_RESTORE)
    if user32.SetForegroundWindow(hwnd) and foreground_pid() == pid:
        return True
    fg = user32.GetForegroundWindow()
    fg_thread = user32.GetWindowThreadProcessId(fg, None) if fg else 0
    me = kernel32.GetCurrentThreadId()
    if fg_thread and fg_thread != me and user32.AttachThreadInput(me, fg_thread, True):
        try:
            user32.BringWindowToTop(hwnd)
            user32.SetForegroundWindow(hwnd)
        finally:
            user32.AttachThreadInput(me, fg_thread, False)
        if foreground_pid() == pid:
            return True
    user32.SwitchToThisWindow(hwnd, True)
    if foreground_pid() == pid:
        return True
    if alt_trick:
        user32.keybd_event(VK_MENU, 0, 0, 0)
        user32.keybd_event(VK_MENU, 0, KEYEVENTF_KEYUP, 0)
        user32.SetForegroundWindow(hwnd)
    return foreground_pid() == pid
