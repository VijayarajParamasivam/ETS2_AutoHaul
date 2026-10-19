"""Extract map data from the game files with the pinned TruckSim Maps parser.

    python tools/extract_map.py              # full parse -> <cache>/map/<version>/raw
    python tools/extract_map.py --dry-run    # parse only, write nothing (smoke test)
    python tools/extract_map.py --only-defs  # just /def data (fast)

Steps: npm ci (if needed) -> build the parser's native addon with MSVC (if needed) -> run parser.
Writes <out>/extract_report.json with timing, exit code and per-file entry counts.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.config import load_config  # noqa: E402
from autohaul.infra.paths import map_dir, third_party_dir  # noqa: E402

from _buildenv import msvc_env, tool  # noqa: E402


def run(cmd: list[str], cwd: Path, env: dict[str, str], log: Path) -> int:
    print(f"$ {' '.join(cmd)}")
    with open(log, "a", encoding="utf-8") as f:
        f.write(f"\n$ {' '.join(cmd)}\n")
        f.flush()
        proc = subprocess.run(cmd, cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT)
    return proc.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="parse but don't write map files")
    parser.add_argument("--only-defs", action="store_true", help="only parse /def data")
    parser.add_argument("--heap-mb", type=int, default=8192, help="Node max old-space size")
    args = parser.parse_args()

    cfg = load_config()
    src = third_party_dir(cfg) / "trucksim_maps"
    if not (src / "package.json").exists():
        sys.exit(f"{src} missing: run `python tools/fetch_deps.py trucksim_maps` first")
    game_dir = Path(cfg.get("game.install_dir"))
    out = map_dir(cfg) / "raw"
    out.mkdir(parents=True, exist_ok=True)
    log = out / "extract.log"
    log.write_text("", encoding="utf-8")

    env = msvc_env()
    npm = tool("npm.cmd", env)
    npx = tool("npx.cmd", env)

    if not (src / "node_modules").exists():
        # --ignore-scripts: upstream postinstall uses `ln -sf` (Unix only) and husky; neither is needed.
        if run([npm, "ci", "--ignore-scripts", "--no-audit", "--no-fund"], src, env, log):
            sys.exit(f"npm ci failed, see {log}")

    release = src / "packages/clis/parser/build/Release"
    if not all((release / f"{name}.node").exists() for name in ("cityhash", "gdeflate")):
        if run([npm, "run", "build", "-w", "packages/clis/parser"], src, env, log):
            sys.exit(f"native addon build failed, see {log}")

    env["NODE_OPTIONS"] = f"--max-old-space-size={args.heap_mb}"
    cmd = [npx, "tsx", "packages/clis/parser/index.ts", "-i", str(game_dir), "-o", str(out)]
    if args.dry_run:
        cmd.append("--dryRun")
    if args.only_defs:
        cmd.append("--onlyDefs")

    t0 = time.time()
    code = run(cmd, src, env, log)
    elapsed = time.time() - t0

    counts = {}
    for f in sorted(out.glob("*.json")):
        if f.name == "extract_report.json":
            continue
        try:
            counts[f.name] = len(json.loads(f.read_text(encoding="utf-8")))
        except (ValueError, TypeError):
            counts[f.name] = "unreadable"
    report = {
        "game_dir": str(game_dir),
        "expected_version": cfg.get("game.expected_version"),
        "parser_commit": json.loads((src / ".autohaul_pin.json").read_text())["commit"],
        "dry_run": args.dry_run,
        "only_defs": args.only_defs,
        "exit_code": code,
        "elapsed_s": round(elapsed, 1),
        "files": counts,
    }
    (out / "extract_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"log: {log}")
    return code


if __name__ == "__main__":
    sys.exit(main())
