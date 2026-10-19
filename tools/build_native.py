"""Build the native DLLs with MSVC + CMake + Ninja (CMake/Ninja come from the project venv).

    python tools/build_native.py               # bridge + ets2la_plugin
    python tools/build_native.py bridge        # only autohaul_bridge.dll (+ test harness)
    python tools/build_native.py ets2la_plugin

Outputs are copied to <cache_dir>/build/out/.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autohaul.infra.config import PROJECT_ROOT, load_config  # noqa: E402
from autohaul.infra.paths import build_dir, third_party_dir  # noqa: E402

from _buildenv import msvc_env, tool  # noqa: E402


def cmake_build(name: str, src: Path, bld: Path, env: dict[str, str], extra: list[str] | None = None) -> None:
    bld.mkdir(parents=True, exist_ok=True)
    print(f"[{name}] configure {src}")
    cmake = tool("cmake", env)
    subprocess.run(
        [cmake, "-G", "Ninja", "-S", str(src), "-B", str(bld), "-DCMAKE_BUILD_TYPE=Release",
         f"-DCMAKE_MAKE_PROGRAM={tool('ninja', env)}", "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", *(extra or [])],
        env=env, check=True,
    )
    print(f"[{name}] build")
    subprocess.run([cmake, "--build", str(bld), "--config", "Release"], env=env, check=True)


def copy_outputs(bld: Path, out: Path, patterns: list[str]) -> list[Path]:
    out.mkdir(parents=True, exist_ok=True)
    copied = []
    for pattern in patterns:
        for f in bld.rglob(pattern):
            if "CMakeFiles" in f.parts:
                continue
            dst = out / f.name
            shutil.copy2(f, dst)
            copied.append(dst)
    return copied


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("targets", nargs="*", choices=["bridge", "ets2la_plugin"], default=[])
    args = parser.parse_args()
    targets = args.targets or ["bridge", "ets2la_plugin"]

    cfg = load_config()
    env = msvc_env()
    root = build_dir(cfg)
    out = root / "out"

    if "bridge" in targets:
        bld = root / "bridge"
        cmake_build("bridge", PROJECT_ROOT / "native", bld, env)
        for f in copy_outputs(bld, out, ["autohaul_bridge.dll", "autohaul_bridge.pdb", "fake_game.exe"]):
            print(f"  -> {f}")

    if "ets2la_plugin" in targets:
        src = third_party_dir(cfg) / "ets2la_plugin"
        if not (src / "CMakeLists.txt").exists():
            sys.exit(f"{src} missing: run `python tools/fetch_deps.py ets2la_plugin` first")
        bld = root / "ets2la_plugin"
        cmake_build("ets2la_plugin", src, bld, env)
        for f in copy_outputs(bld, out, ["ets2_la_plugin.dll", "ets2_la_plugin.pdb"]):
            print(f"  -> {f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
