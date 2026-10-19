"""Fetch the pinned third-party dependencies listed in deps.lock.toml.

Each dependency is cloned into <cache_dir>/third_party/<name>, checked out at the pinned
commit, and the listed patches are applied with `git apply`. A marker file records the
commit + patch hashes so re-running is a no-op unless something changed.

    python tools/fetch_deps.py                  # all non-optional deps
    python tools/fetch_deps.py ets2la_plugin    # just one
    python tools/fetch_deps.py --with-optional  # include trucklib
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.config import PROJECT_ROOT, load_config  # noqa: E402
from autohaul.infra.paths import third_party_dir  # noqa: E402

LOCK_FILE = PROJECT_ROOT / "deps.lock.toml"
MARKER = ".autohaul_pin.json"


def git(*args: str, cwd: Path | None = None) -> str:
    result = subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def patch_digest(patches: list[str]) -> list[str]:
    out = []
    for rel in patches:
        data = (PROJECT_ROOT / rel).read_bytes()
        out.append(f"{rel}:{hashlib.sha256(data).hexdigest()[:16]}")
    return out


def fetch(name: str, spec: dict, root: Path, force: bool) -> None:
    dest = root / name
    patches = spec.get("patches", [])
    wanted = {"commit": spec["commit"], "patches": patch_digest(patches)}
    marker = dest / MARKER
    if not force and marker.exists() and json.loads(marker.read_text()) == wanted:
        print(f"[{name}] up to date @ {spec['commit'][:8]}")
        return

    if not (dest / ".git").exists():
        print(f"[{name}] cloning {spec['repo']}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        git("clone", "--filter=blob:none", "--no-checkout", spec["repo"], str(dest))
        # Keep upstream line endings byte-for-byte so patches apply regardless of global autocrlf.
        git("config", "core.autocrlf", "false", cwd=dest)
    else:
        git("config", "core.autocrlf", "false", cwd=dest)
        git("fetch", "--quiet", "origin", cwd=dest)

    print(f"[{name}] checkout {spec['commit'][:8]}")
    git("checkout", "--force", "--detach", spec["commit"], cwd=dest)
    git("reset", "--hard", spec["commit"], cwd=dest)
    if spec.get("submodules"):
        print(f"[{name}] submodules")
        git("submodule", "update", "--init", "--recursive", "--force", cwd=dest)

    for rel in patches:
        print(f"[{name}] apply {rel}")
        git("apply", "--whitespace=nowarn", str(PROJECT_ROOT / rel), cwd=dest)

    marker.write_text(json.dumps(wanted, indent=2))
    print(f"[{name}] ready at {dest}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="*", help="dependency names (default: all non-optional)")
    parser.add_argument("--with-optional", action="store_true")
    parser.add_argument("--force", action="store_true", help="re-checkout and re-apply patches")
    args = parser.parse_args()

    cfg = load_config()
    lock = tomllib.loads(LOCK_FILE.read_text(encoding="utf-8"))
    names = args.names or [n for n, s in lock.items() if args.with_optional or not s.get("optional")]
    unknown = [n for n in names if n not in lock]
    if unknown:
        parser.error(f"unknown dependencies: {', '.join(unknown)}")

    root = third_party_dir(cfg)
    for name in names:
        try:
            fetch(name, lock[name], root, args.force)
        except subprocess.CalledProcessError as exc:
            print(f"[{name}] FAILED: {' '.join(exc.cmd)}\n{exc.stderr}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
