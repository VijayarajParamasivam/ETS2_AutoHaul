"""AutoHaul launcher: pick a mode, start and stop tools/drive.py, watch its status and log.

    AutoHaul.bat (double-click)   or   .venv\\Scripts\\pythonw.exe tools\\launcher.py [--start]

drive.py runs as a hidden child process with --stdin-quit: Stop (or closing the window) asks it to quit the same
way Ctrl+C does, so it hands the truck back before it exits. The last settings are kept in logs/launcher.json.
"Show overlay" also runs tools/overlay.py on top of the game; "Learn while driving" lets autohaul/learn adjust the
driving; "Reset learning" forgets it.
"""

from __future__ import annotations

import json
import os
import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import map_dir  # noqa: E402
from autohaul.world.place_search import Option, PlaceIndex, exact, search  # noqa: E402

PYTHON = ROOT / ".venv" / "Scripts" / "python.exe"
SETTINGS = ROOT / "logs" / "launcher.json"
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

CORNERS = {"top left": "tl", "top right": "tr", "bottom left": "bl", "bottom right": "br"}
MODES = [
    ("mission", "Job (full mission)",
     "Runs the active job: to the trailer, couples it, to the destination, quick park; fuel and rest stops."),
    ("to", "Drive to a place",
     "Follows the route to the city or company you choose below (no job needed)."),
    ("road", "Follow the road",
     "Lane keeping and speed, no route: hands back before junctions it can't decide."),
    ("steer", "Steer only",
     "AutoHaul steers along the route; you use the pedals."),
    ("speed", "Speed only",
     "AutoHaul does throttle and brake (limits, curves, following distance); you steer."),
]


def drive_args(mode: str, place: str, max_kmh: int, record: bool, learn: bool = True) -> list[str]:
    """drive.py's command-line arguments for a launcher mode."""
    args = {"mission": ["--mission"], "to": ["--to", place.strip()], "road": ["--no-route"],
            "steer": ["--mode", "steer"], "speed": ["--mode", "speed"]}[mode]
    args += ["--max-kmh", str(max_kmh)]
    if not record:
        args.append("--no-record")
    if not learn:
        args.append("--no-learning")
    return args + ["--stdin-quit"]


def place_spec(city: Option | None, company: Option | None) -> str:
    """drive.py's --to for a choice: CITY:COMPANY (that depot), CITY (its centre)."""
    if city is None:
        return ""
    return f"{city.token}:{company.token}" if company is not None else city.token


class SuggestEntry(ttk.Frame):
    """A text box that suggests as you type. Up and down move through the list, Enter or Tab or a click takes one, Esc
    closes it. `options()` gives what it can suggest."""

    MAX_ROWS = 8

    def __init__(self, parent, options, width: int = 32, on_change=None, on_pick=None, open_on_focus: bool = False):
        super().__init__(parent)
        self.options, self.on_change, self.on_pick = options, on_change, on_pick
        self.open_on_focus = open_on_focus
        self.text = tk.StringVar()
        self.entry = ttk.Entry(self, textvariable=self.text, width=width)
        self.entry.pack(fill="x")
        self.popup: tk.Toplevel | None = None
        self.listbox: tk.Listbox | None = None
        self.shown: list[Option] = []
        self.picked: Option | None = None
        self.entry.bind("<KeyRelease>", self._typed)
        self.entry.bind("<Down>", lambda e: self._move(1))
        self.entry.bind("<Up>", lambda e: self._move(-1))
        self.entry.bind("<Return>", self._accept)
        self.entry.bind("<Tab>", self._accept_tab)
        self.entry.bind("<Escape>", lambda e: self.close())
        self.entry.bind("<FocusIn>", self._focused)
        self.entry.bind("<FocusOut>", lambda e: self.after(150, self._maybe_close))

    def set(self, option: Option | None) -> None:
        self.picked = option
        self.text.set(option.label if option else "")

    def choice(self) -> Option | None:
        """What the box says, if it names one of the options."""
        return exact(self.text.get(), self.options())

    def enable(self, on: bool) -> None:
        self.entry.configure(state="normal" if on else "disabled")
        if not on:
            self.close()

    # ------------------------------------------------------------- the list
    def _typed(self, event) -> None:
        if event.keysym in ("Up", "Down", "Return", "Tab", "Escape", "Shift_L", "Shift_R", "Control_L", "Control_R", "Alt_L", "Alt_R", "Left", "Right", "Home", "End"):
            return
        self.picked = None
        self.refresh()
        if self.on_change:
            self.on_change()

    def _focused(self, event) -> None:
        if self.open_on_focus and str(self.entry.cget("state")) == "normal":
            self.refresh()

    def refresh(self) -> None:
        self.shown = search(self.text.get(), self.options(), self.MAX_ROWS)
        only = len(self.shown) == 1 and self.shown[0].label == self.text.get()
        if not self.shown or only:
            self.close()
            return
        self._open()
        self.listbox.delete(0, "end")
        for o in self.shown:
            self.listbox.insert("end", o.label)
        self.listbox.configure(height=len(self.shown))
        self.listbox.selection_clear(0, "end")
        self.listbox.selection_set(0)
        self.listbox.activate(0)
        self.popup.update_idletasks()
        self.popup.geometry(f"{self.entry.winfo_width()}x{self.listbox.winfo_reqheight()}"
                            f"+{self.entry.winfo_rootx()}+{self.entry.winfo_rooty() + self.entry.winfo_height()}")

    def _open(self) -> None:
        if self.popup is not None:
            return
        self.popup = tk.Toplevel(self)
        self.popup.overrideredirect(True)
        self.popup.attributes("-topmost", True)
        self.listbox = tk.Listbox(self.popup, activestyle="none", exportselection=False, borderwidth=1, relief="solid")
        self.listbox.pack(fill="both", expand=True)
        self.listbox.bind("<ButtonRelease-1>", self._clicked)
        self.winfo_toplevel().bind("<Configure>", self._window_moved, add="+")

    def close(self) -> None:
        if self.popup is not None:
            self.popup.destroy()
            self.popup = self.listbox = None

    def _maybe_close(self) -> None:
        if self.focus_get() is not self.entry:
            self.close()

    def _window_moved(self, event) -> None:
        if event.widget is self.winfo_toplevel():
            self.close()

    def _move(self, step: int) -> str:
        if self.popup is None:
            self.refresh()
            return "break"
        sel = self.listbox.curselection()
        i = max(0, min(len(self.shown) - 1, (sel[0] if sel else -1) + step))
        self.listbox.selection_clear(0, "end")
        self.listbox.selection_set(i)
        self.listbox.see(i)
        return "break"

    def _take(self, option: Option) -> None:
        self.set(option)
        self.close()
        self.entry.icursor("end")
        if self.on_pick:
            self.on_pick()

    def _clicked(self, event) -> None:
        i = self.listbox.nearest(event.y)
        if 0 <= i < len(self.shown):
            self._take(self.shown[i])
            self.entry.focus_set()

    def _accept(self, event=None) -> str:
        if self.popup is not None and self.listbox.curselection():
            self._take(self.shown[self.listbox.curselection()[0]])
        elif self.on_pick:
            self.on_pick()
        return "break"

    def _accept_tab(self, event=None) -> None:
        if self.popup is not None and self.text.get().strip() and self.listbox.curselection():
            self._take(self.shown[self.listbox.curselection()[0]])
        self.close()                                   # and Tab moves on as usual


def other_autohaul() -> list[int]:
    """PIDs of drive.py processes already running (started from a terminal or by an earlier launcher)."""
    cmd = ("Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | "
           "Where-Object { $_.CommandLine -like '*tools*drive.py*' } | ForEach-Object { $_.ProcessId }")
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command", cmd], capture_output=True, text=True,
                             timeout=15, creationflags=NO_WINDOW).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [int(x) for x in out.split() if x.isdigit() and int(x) != os.getpid()]


class Launcher:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.proc: subprocess.Popen | None = None
        self.overlay_proc: subprocess.Popen | None = None
        self.lines: queue.Queue[tuple[str, str]] = queue.Queue()
        saved = self._load()
        root.title("AutoHaul")
        root.minsize(620, 520)
        root.protocol("WM_DELETE_WINDOW", self.close)

        frame = ttk.Frame(root, padding=12)
        frame.pack(fill="both", expand=True)

        modes = ttk.LabelFrame(frame, text="Mode", padding=8)
        modes.pack(fill="x")
        self.mode = tk.StringVar(value=saved.get("mode", "mission"))
        for key, label, desc in MODES:
            row = ttk.Frame(modes)
            row.pack(fill="x", pady=1)
            ttk.Radiobutton(row, text=label, value=key, variable=self.mode, command=self._mode_changed,
                            width=20).pack(side="left")
            ttk.Label(row, text=desc, foreground="#555", wraplength=420).pack(side="left", fill="x")

        opts = ttk.Frame(frame, padding=(0, 8))
        opts.pack(fill="x")
        try:
            self.index: PlaceIndex | None = PlaceIndex(map_dir(load_config()))
        except (OSError, ValueError, KeyError):
            self.index = None                     # no parsed map: "Drive to a place" can't work anyway
        ttk.Label(opts, text="City:").grid(row=0, column=0, sticky="w")
        self.city = SuggestEntry(opts, lambda: self.index.cities if self.index else [], on_change=self._city_changed,
                                 on_pick=self._city_picked)
        self.city.grid(row=0, column=1, sticky="w", padx=6)
        ttk.Label(opts, text="Company:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.company = SuggestEntry(opts, self._city_companies, open_on_focus=True, on_change=self._show_destination,
                                    on_pick=self._show_destination)
        self.company.grid(row=1, column=1, sticky="w", padx=6, pady=(4, 0))
        self.destination = tk.StringVar()
        ttk.Label(opts, textvariable=self.destination, foreground="#555").grid(row=1, column=2, sticky="w")
        self._load_place(saved)
        ttk.Label(opts, text="Max speed (km/h):").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.max_kmh = tk.IntVar(value=int(saved.get("max_kmh", 80)))
        ttk.Spinbox(opts, from_=20, to=90, increment=5, textvariable=self.max_kmh, width=6).grid(
            row=2, column=1, sticky="w", padx=6, pady=(6, 0))
        self.record = tk.BooleanVar(value=bool(saved.get("record", True)))
        ttk.Checkbutton(opts, text="Record the drive (for fixing problems later)", variable=self.record).grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(6, 0))
        self.overlay = tk.BooleanVar(value=bool(saved.get("overlay", True)))
        ttk.Checkbutton(opts, text="Show overlay on the game", variable=self.overlay,
                        command=self._overlay_changed).grid(row=4, column=0, columnspan=3, sticky="w")
        self.corner = tk.StringVar(value=saved.get("corner", "top left"))
        ttk.Combobox(opts, textvariable=self.corner, values=list(CORNERS), state="readonly", width=12).grid(
            row=4, column=2, sticky="w")
        self.learn = tk.BooleanVar(value=bool(saved.get("learn", True)))
        ttk.Checkbutton(opts, text="Learn while driving", variable=self.learn).grid(row=5, column=0, columnspan=2, sticky="w")
        ttk.Button(opts, text="Reset learning", command=self.reset_learning).grid(row=5, column=2, sticky="w")

        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", pady=(4, 8))
        self.start_btn = ttk.Button(buttons, text="Start", command=self.start)
        self.start_btn.pack(side="left")
        self.stop_btn = ttk.Button(buttons, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.pack(side="left", padx=6)
        ttk.Label(buttons, text="Then click the game and press / to engage; / again (or any driving key) to take over.",
                  foreground="#555").pack(side="left", padx=6)

        self.status = tk.StringVar(value="stopped")
        ttk.Label(frame, textvariable=self.status, font=("Consolas", 10, "bold"), wraplength=600).pack(
            fill="x", pady=(0, 6))
        self.log = ScrolledText(frame, height=14, font=("Consolas", 9), state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True)

        self._mode_changed()
        root.after(100, self._pump)

    # ------------------------------------------------------------- settings
    @staticmethod
    def _load() -> dict:
        try:
            return json.loads(SETTINGS.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _save(self, max_kmh: int) -> None:
        try:
            SETTINGS.parent.mkdir(parents=True, exist_ok=True)
            SETTINGS.write_text(json.dumps({"mode": self.mode.get(), "place": self._spec(),
                                            "max_kmh": max_kmh, "record": self.record.get(),
                                            "overlay": self.overlay.get(), "corner": self.corner.get(),
                                            "learn": self.learn.get()}), encoding="utf-8")
        except OSError:
            pass

    def _mode_changed(self) -> None:
        self.city.enable(self.mode.get() == "to")
        self._city_changed()

    # ------------------------------------------------------------- where to
    def _city_companies(self) -> list[Option]:
        city = self.city.choice()
        return self.index.companies(city.token) if self.index and city else []

    def _load_place(self, saved: dict) -> None:
        """The last choice ("city" or "city:company", tokens as drive.py takes them)."""
        token, _, company = str(saved.get("place", "")).strip().lower().partition(":")
        if not self.index or not token:
            return
        city = next((c for c in self.index.cities if c.token == token), None)
        self.city.set(city)
        if city and company:
            self.company.set(next((c for c in self.index.companies(city.token) if c.token == company), None))

    def _city_changed(self) -> None:
        city = self.city.choice()
        self.company.enable(self.mode.get() == "to" and city is not None)
        known = self._city_companies()
        if self.company.text.get() and exact(self.company.text.get(), known) is None and self.company.picked is not None:
            self.company.set(None)                # that company isn't in this city
        self._show_destination()

    def _city_picked(self) -> None:
        self._city_changed()
        if self.city.choice() is not None:
            self.company.entry.focus_set()

    def _show_destination(self) -> None:
        city, typed = self.city.choice(), self.company.text.get().strip()
        company = self.company.choice()
        if city is None:
            self.destination.set("")
        elif typed and company is None:           # still typing, if there's something to suggest
            matches = search(typed, self._city_companies())
            self.destination.set("" if matches else f"no company like that in {city.name}")
        elif company is not None:
            self.destination.set(f"to {company.name} in {city.name}")
        else:
            n = len(self._city_companies())
            self.destination.set(f"to the centre of {city.name} ({n} companies there)")

    def _spec(self) -> str:
        return place_spec(self.city.choice(), self.company.choice())

    # ------------------------------------------------------------- the drive process
    def start(self) -> None:
        if self.proc is not None:
            return
        if self.mode.get() == "to":
            if self.city.choice() is None:
                messagebox.showwarning("AutoHaul", "Choose a city from the list first.")
                return
            if self.company.text.get().strip() and self.company.choice() is None:
                messagebox.showwarning("AutoHaul", f"There's no company like that in {self.city.choice().name}: choose one "
                                                   "from the list, or leave the box empty for the city centre.")
                return
        others = other_autohaul()
        if others:
            if not messagebox.askyesno("AutoHaul", f"AutoHaul is already running (process {', '.join(map(str, others))})."
                                       "\n\nStop it and start this one?"):
                return
            for pid in others:
                subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True,   # (the venv's
                               creationflags=NO_WINDOW)                                     # python.exe is a stub)
        try:
            max_kmh = max(20, min(90, int(self.max_kmh.get())))
        except (tk.TclError, ValueError):
            max_kmh = 80
        self._save(max_kmh)
        args = [str(PYTHON), "-u", str(ROOT / "tools" / "drive.py")] + drive_args(
            self.mode.get(), self._spec(), max_kmh, self.record.get(), self.learn.get())
        self._append(f"> {' '.join(args[2:])}\n")
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        try:
            self.proc = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, env=env, creationflags=NO_WINDOW)
        except OSError as e:
            messagebox.showerror("AutoHaul", f"Couldn't start drive.py: {e}")
            return
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()
        self.start_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.status.set("starting ...")
        self._overlay_changed()

    def _overlay_changed(self) -> None:
        running = self.overlay_proc is not None and self.overlay_proc.poll() is None
        want = self.overlay.get() and self.proc is not None
        if want and not running:
            self.overlay_proc = subprocess.Popen([str(PYTHON), str(ROOT / "tools" / "overlay.py"), "--corner",
                                                  CORNERS.get(self.corner.get(), "tl")], cwd=ROOT,
                                                 creationflags=NO_WINDOW)
        elif running and not want:
            self.overlay_proc.terminate()
            self.overlay_proc = None

    def reset_learning(self) -> None:
        folder = Path.home() / "AutoHaul" / "learned"
        if self.proc is not None:
            messagebox.showinfo("AutoHaul", "Stop AutoHaul first.")
            return
        if not messagebox.askyesno("AutoHaul", f"Forget everything AutoHaul has learned?\n\n{folder}"):
            return
        for name in ("learned.json",):
            (folder / name).unlink(missing_ok=True)
        self._append("[learning reset]\n")

    def stop(self) -> None:
        p = self.proc
        if p is None:
            return
        self.status.set("stopping (handing back) ...")
        try:
            p.stdin.write(b"quit\n")
            p.stdin.close()
        except OSError:
            pass
        self.root.after(6000, lambda: p.poll() is None and p.kill())

    def close(self) -> None:
        if self.proc is not None:
            if not messagebox.askyesno("AutoHaul", "AutoHaul is running. Stop it and close?"):
                return
            self.stop()
            try:
                self.proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None
        self._overlay_changed()
        self.root.destroy()

    def _read(self, proc: subprocess.Popen) -> None:
        """Split drive.py's output: lines for the log, the \\r-refreshed status line for the status label."""
        buf = ""
        fd = proc.stdout.fileno()
        while True:
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            buf = (buf + ANSI.sub("", chunk.decode("utf-8", errors="replace"))).replace("\r\n", "\n")
            while True:
                i = min((k for k in (buf.find("\r"), buf.find("\n")) if k >= 0), default=-1)
                if i < 0 or (buf[i] == "\r" and i == len(buf) - 1):
                    break                                   # (a lone \r at the end may be half of \r\n)
                piece, sep, buf = buf[:i], buf[i], buf[i + 1:]
                if piece.startswith("[") or (sep == "\r" and piece.strip()):
                    self.lines.put(("status", piece.strip()))
                elif sep == "\n":
                    self.lines.put(("log", piece + "\n"))
            if buf.startswith("["):
                self.lines.put(("status", buf.strip()))
        if buf.strip():
            self.lines.put(("log", buf + "\n"))
        self.lines.put(("exit", str(proc.wait())))

    def _pump(self) -> None:
        try:
            while True:
                kind, text = self.lines.get_nowait()
                if kind == "status":
                    self.status.set(re.sub(r"\s{2,}", "  ", text))
                elif kind == "log":
                    self._append(text)
                else:
                    self._append(f"[AutoHaul exited ({text})]\n")
                    self.status.set("stopped")
                    self.proc = None
                    self._overlay_changed()
                    self.start_btn.configure(state="normal")
                    self.stop_btn.configure(state="disabled")
        except queue.Empty:
            pass
        self.root.after(100, self._pump)

    def _append(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")


def main() -> int:
    if not PYTHON.exists():
        print(f"no virtual environment at {PYTHON.parent.parent}: see README.md (setup)")
        return 1
    root = tk.Tk()
    try:
        ttk.Style(root).theme_use("vista")
    except tk.TclError:
        pass
    ui = Launcher(root)
    if "--start" in sys.argv[1:]:                # start the last-used mode right away
        root.after(300, ui.start)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
