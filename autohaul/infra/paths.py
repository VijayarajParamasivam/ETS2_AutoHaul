"""Well-known directories. Large generated data lives under the cache dir (outside OneDrive)."""

from __future__ import annotations

from pathlib import Path

from autohaul.infra.config import PROJECT_ROOT, Config


def cache_dir(cfg: Config) -> Path:
    path = Path(cfg.get("paths.cache_dir"))
    path.mkdir(parents=True, exist_ok=True)
    return path


def third_party_dir(cfg: Config) -> Path:
    return cache_dir(cfg) / "third_party"


def build_dir(cfg: Config) -> Path:
    return cache_dir(cfg) / "build"


def recordings_dir(cfg: Config) -> Path:
    path = cache_dir(cfg) / "recordings"
    path.mkdir(parents=True, exist_ok=True)
    return path


def logs_dir(cfg: Config) -> Path:
    path = cache_dir(cfg) / "logs"
    path.mkdir(parents=True, exist_ok=True)
    return path


def map_dir(cfg: Config, game_version: str | None = None) -> Path:
    return cache_dir(cfg) / "map" / (game_version or cfg.get("game.expected_version"))


def game_plugins_dir(cfg: Config) -> Path:
    return Path(cfg.get("game.install_dir")) / "bin" / "win_x64" / "plugins"


def game_log(cfg: Config) -> Path:
    return Path(cfg.get("game.home_dir")) / "game.log.txt"


def project_root() -> Path:
    return PROJECT_ROOT
