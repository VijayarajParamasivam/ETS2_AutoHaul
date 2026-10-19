"""Configuration loading: config/autohaul.toml, then config/local.toml merged on top.

String values have Windows-style environment variables (%LOCALAPPDATA%) expanded.
"""

from __future__ import annotations

import os
import tomllib
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "autohaul.toml"
LOCAL_CONFIG = PROJECT_ROOT / "config" / "local.toml"

_MISSING = object()


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def _expand(value: Any) -> Any:
    if isinstance(value, str):
        return os.path.expandvars(value)
    if isinstance(value, list):
        return [_expand(v) for v in value]
    if isinstance(value, dict):
        return {k: _expand(v) for k, v in value.items()}
    return value


class Config:
    """Read-only nested config with dotted-path access: cfg.get("safety.kill_keys")."""

    def __init__(self, data: dict[str, Any]):
        self._data = data

    def get(self, dotted: str, default: Any = _MISSING) -> Any:
        node: Any = self._data
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                if default is _MISSING:
                    raise KeyError(f"config key not found: {dotted}")
                return default
            node = node[part]
        return node

    def section(self, dotted: str) -> dict[str, Any]:
        value = self.get(dotted)
        if not isinstance(value, dict):
            raise KeyError(f"config key is not a section: {dotted}")
        return value

    def as_dict(self) -> dict[str, Any]:
        return self._data


def load_config(path: Path | None = None, local: Path | None = LOCAL_CONFIG) -> Config:
    with open(path or DEFAULT_CONFIG, "rb") as f:
        data = tomllib.load(f)
    if local is not None and local.exists():
        with open(local, "rb") as f:
            data = _merge(data, tomllib.load(f))
    return Config(_expand(data))
