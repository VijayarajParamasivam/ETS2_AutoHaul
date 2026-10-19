"""Install / remove AutoHaul's game plugins in <game>\\bin\\win_x64\\plugins.

    python tools/install_plugin.py                  # install autohaul_bridge.dll only (step 1)
    python tools/install_plugin.py --with-ets2la    # also the read-only ETS2LA memory plugin (step 2)
    python tools/install_plugin.py --remove ets2la  # remove just the ETS2LA plugin
    python tools/install_plugin.py --remove all     # remove everything AutoHaul installed
    python tools/install_plugin.py --status

The game must be closed: Windows locks loaded DLLs.
"""

from __future__ import annotations

import argparse
import filecmp
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import build_dir, game_plugins_dir  # noqa: E402

FILES = {"bridge": ["autohaul_bridge.dll"], "ets2la": ["ets2_la_plugin.dll"]}


def game_running() -> bool:
    out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq eurotrucks2.exe", "/NH"], capture_output=True, text=True).stdout
    return "eurotrucks2.exe" in out.lower()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--with-ets2la", action="store_true")
    parser.add_argument("--remove", choices=["bridge", "ets2la", "all"])
    parser.add_argument("--status", action="store_true")
    args = parser.parse_args()

    cfg = load_config()
    plugins = game_plugins_dir(cfg)
    out = build_dir(cfg) / "out"

    if args.status:
        print(f"plugins dir: {plugins} ({'exists' if plugins.exists() else 'missing'})")
        for group, names in FILES.items():
            for n in names:
                print(f"  {group:7s} {n:22s} {'installed' if (plugins / n).exists() else '-'}")
        return 0

    running = game_running()
    if running and args.remove:
        print("ETS2 is running: close the game first (loaded DLLs are locked).")
        return 1

    if args.remove:
        groups = FILES if args.remove == "all" else {args.remove: FILES[args.remove]}
        for names in groups.values():
            for n in names:
                p = plugins / n
                if p.exists():
                    p.unlink()
                    print(f"removed {p}")
        return 0

    groups = ["bridge"] + (["ets2la"] if args.with_ets2la else [])
    plugins.mkdir(parents=True, exist_ok=True)
    for g in groups:
        for n in FILES[g]:
            src = out / n
            dst = plugins / n
            if not src.exists():
                print(f"{src} not built: run `python tools/build_native.py`")
                return 1
            if dst.exists() and filecmp.cmp(src, dst, shallow=False):
                print(f"already installed {dst}")
                continue
            if running and dst.exists():
                # The game has this DLL loaded (locked); a new file it hasn't loaded yet is safe to add.
                print(f"ETS2 is running and {n} is loaded: close the game first to replace it.")
                return 1
            shutil.copy2(src, dst)
            print(f"installed {dst}")
    if running:
        print("Done. ETS2 is running: plugins load at startup, so restart the game to activate new ones.")
    else:
        print("Done. Start ETS2. If it warns that SDK plugins / advanced SDK features are in use, confirm with OK.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
