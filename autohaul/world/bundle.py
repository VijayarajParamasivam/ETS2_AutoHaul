"""Everything the drive and mission tools load from the extracted map, in one place."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from autohaul.world.graph import LaneGraph
from autohaul.world.junctions import Junctions
from autohaul.world.lanemap import LaneMap
from autohaul.world.places import Places
from autohaul.world.pois import toll_plazas
from autohaul.world.route import Router
from autohaul.world.rules import JunctionRules, MapTooOld


@dataclass
class World:
    map_dir: Path
    lm: LaneMap
    graph: LaneGraph
    router: Router
    junctions: Junctions
    rules: JunctionRules | None       # None: the map has no junction rules (or they were turned off)
    places: Places
    hazards: np.ndarray               # (n, 2) toll plazas


CITY_SCALE, ROAD_SCALE = 3.0, 19.0     # game metres per world metre in city areas / elsewhere


def city_weights(lm: LaneMap, raw_dir: Path) -> np.ndarray:
    """Per lane: CITY_SCALE / ROAD_SCALE when its middle lies in a city area (the map's city rectangles, from
    their corner; checked against the game's telemetry scale: 6,415 of 6,415 samples inside read 3), else 1."""
    try:
        with open(Path(raw_dir) / "europe-cities.json", encoding="utf-8") as f:
            cities = json.load(f)
    except OSError:
        return np.ones(len(lm.lane_kind))
    mid = (lm.lane_first + lm.lane_last) // 2
    mx, my = lm.seg[mid, 0], lm.seg[mid, 1]
    inside = np.zeros(len(mid), dtype=bool)
    order = np.argsort(mx)
    xs = mx[order]
    for c in cities:
        for a in c.get("areas", []):
            lo, hi = np.searchsorted(xs, a["x"]), np.searchsorted(xs, a["x"] + a["width"], "right")
            idx = order[lo:hi]
            inside[idx[(my[idx] >= a["y"]) & (my[idx] <= a["y"] + a["height"])]] = True
    return np.where(inside, CITY_SCALE / ROAD_SCALE, 1.0)


def load_world(map_dir: Path, lm: LaneMap | None = None, rules: bool = True, log=print) -> World:
    """`rules=False`: no junction rules (AutoHaul then hands back before every intersection)."""
    map_dir = Path(map_dir)
    lm = lm or LaneMap.load_or_build(map_dir, log=log)
    graph = LaneGraph.load_or_build(lm, map_dir, log=log)
    jr = None
    if rules:
        try:
            jr = JunctionRules.load_or_build(lm, map_dir, log=log)
        except MapTooOld as exc:
            log(f"no junction rules ({exc}): AutoHaul hands back before every intersection")
    router = Router(lm, graph, no_go=jr.cars_only() if jr is not None else None,
                    weight=city_weights(lm, map_dir / "raw"))
    return World(map_dir, lm, graph, router, Junctions(lm), jr, Places(map_dir), toll_plazas(map_dir))
