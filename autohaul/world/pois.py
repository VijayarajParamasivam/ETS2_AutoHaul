"""Points of interest from the parsed map that the drivers care about."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def toll_plazas(map_dir: Path) -> np.ndarray:
    """(n, 2) map positions of toll plaza icons (342 on 1.53). Empty if the map data has none."""
    path = Path(map_dir) / "raw" / "europe-pois.json"
    if not path.exists():
        return np.empty((0, 2))
    with open(path, encoding="utf-8") as f:
        pois = json.load(f)
    pts = [(p["x"], p["y"]) for p in pois if p.get("icon") == "toll_ico"]
    return np.array(pts, dtype=float).reshape(-1, 2)
