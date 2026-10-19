"""Shared helpers for build tools: MSVC developer environment and tool lookup."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

VSWHERE = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"


def msvc_env() -> dict[str, str]:
    """Return os.environ extended with the MSVC x64 developer environment (vcvars64)."""
    if not VSWHERE.exists():
        sys.exit("vswhere.exe not found: install Visual Studio Build Tools with the C++ workload")
    install = subprocess.run(
        [str(VSWHERE), "-all", "-products", "*", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
         "-property", "installationPath"],
        capture_output=True, text=True, check=True,
    ).stdout.strip().splitlines()
    if not install:
        sys.exit("No Visual Studio installation with MSVC x64 tools found")
    vcvars = Path(install[0]) / "VC/Auxiliary/Build/vcvars64.bat"
    out = subprocess.run(f'cmd /s /c ""{vcvars}" >nul && set"', capture_output=True, text=True, shell=True, check=True).stdout
    # Windows env var names are case-insensitive; normalize so "Path" and "PATH" don't both exist.
    env = {k.upper(): v for k, v in os.environ.items()}
    for line in out.splitlines():
        key, sep, value = line.partition("=")
        if sep:
            env[key.upper()] = value
    # Put the venv's cmake/ninja first.
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    return env


def tool(name: str, env: dict[str, str]) -> str:
    """Resolve an executable against env["PATH"] (subprocess would use the parent's PATH)."""
    found = shutil.which(name, path=env["PATH"])
    if not found:
        sys.exit(f"{name} not found on PATH")
    return found
