"""Fuel stations and places to sleep, from the map's points of interest, tied to the lanes a truck stops on.

  fuel   `gas_ico` points: the pumps of a fuel station (usually two per station). A pump counts only if a lane a
         truck may use passes within `fuel_max_off_m`, so car-only pumps drop out.
  sleep  `parking_ico` points: the game's parking triggers, where the activate key offers sleep (rest areas,
         fuel stations, hotels). The truck stops on the nearest lane within `sleep_max_off_m`; the trigger
         areas are large, so that is inside them in practice (checked in-game: the action reports it).
Fuel prices are per country (patched map parser); a station's country is that of the nearest city.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from autohaul.world.lanemap import LaneMap


@dataclass(frozen=True)
class ServicePoint:
    kind: str               # "fuel" or "sleep"
    x: float                # the pump / parking point
    y: float
    lane: int               # the lane the truck stops on
    s: float                # metres along that lane to the stop (its closest point to x, y)
    off_m: float            # the point's distance from that lane
    site: str               # the station / rest area (prefab or trigger uid): its pumps and parking share it
    country: str            # country token
    price: float | None     # EUR per litre (fuel), when the map has it


class Services:
    def __init__(self, lm: LaneMap, map_dir: Path, no_go: np.ndarray | None = None, fuel_max_off_m: float = 8.0,
                 sleep_max_off_m: float = 30.0, log=print):
        self.lm = lm
        raw = Path(map_dir) / "raw"
        pois = _load(raw / "europe-pois.json")
        cities = _load(raw / "europe-cities.json")
        countries = {c["token"]: c for c in _load(raw / "europe-countries.json")}
        self._city_xy = np.array([[c["x"], c["y"]] for c in cities]) if cities else np.zeros((0, 2))
        self._city_country = [c.get("countryToken", "") for c in cities]
        self._price = {t: c.get("fuelPrice") for t, c in countries.items()}
        self._no_go = np.zeros(len(lm.lane_kind), bool) if no_go is None else np.asarray(no_go, bool)
        self.fuel = self._attach(pois, "gas_ico", "fuel", fuel_max_off_m)
        self.sleep = self._attach(pois, "parking_ico", "sleep", sleep_max_off_m)
        known = [p for p in self._price.values() if p]
        self.default_price = float(np.median(known)) if known else 1.5
        log(f"services: {len(self.fuel)} fuel pumps, {len(self.sleep)} places to sleep "
            f"({sum(1 for p in self._price.values() if p)} countries with fuel prices)")

    @classmethod
    def from_points(cls, lm: LaneMap, points: list[ServicePoint], default_price: float = 1.5) -> "Services":
        """Hand-made services (tests, synthetic maps)."""
        sv = cls.__new__(cls)
        sv.lm = lm
        sv._city_xy, sv._city_country, sv._price = np.zeros((0, 2)), [], {}
        sv._no_go = np.zeros(len(lm.lane_kind), bool)
        sv.fuel = [p for p in points if p.kind == "fuel"]
        sv.sleep = [p for p in points if p.kind == "sleep"]
        sv.default_price = default_price
        return sv

    def points(self, kind: str) -> list[ServicePoint]:
        return self.fuel if kind == "fuel" else self.sleep

    def price_at(self, sp: ServicePoint) -> float:
        return sp.price if sp.price else self.default_price

    def country_at(self, x: float, y: float) -> str:
        if not len(self._city_xy):
            return ""
        i = int(np.argmin(np.hypot(self._city_xy[:, 0] - x, self._city_xy[:, 1] - y)))
        return self._city_country[i]

    def _attach(self, pois: list[dict], icon: str, kind: str, max_off: float) -> list[ServicePoint]:
        out = []
        for p in pois:
            if p.get("icon") != icon:
                continue
            x, y = float(p["x"]), float(p["y"])
            best = None
            for h in self.lm.nearest(x, y, None, max_dist=max_off, k=8):
                if not self._no_go[h.lane]:
                    best = h
                    break
            if best is None:
                continue
            first = self.lm.lane_first[best.lane]
            s = float(self.lm.seg_len[first:best.segment].sum() + best.t * self.lm.seg_len[best.segment])
            site = str(p.get("prefabUid") or p.get("uid") or f"{round(x)},{round(y)}")
            country = self.country_at(x, y)
            out.append(ServicePoint(kind, x, y, best.lane, s, best.dist, site, country,
                                    self._price.get(country) if kind == "fuel" else None))
        return out

    def along(self, kind: str, pts: np.ndarray, corridor_m: float) -> list[tuple[ServicePoint, float, float]]:
        """Points of `kind` within `corridor_m` of polyline `pts` (n, 2): (point, metres along the polyline to
        its closest point, distance from it), in route order. One per site: its closest point."""
        cand = self.points(kind)
        if not cand or len(pts) < 2:
            return []
        xy = np.array([[c.x, c.y] for c in cand])
        lo, hi = pts.min(axis=0) - corridor_m, pts.max(axis=0) + corridor_m
        idx = np.flatnonzero((xy[:, 0] >= lo[0]) & (xy[:, 0] <= hi[0]) & (xy[:, 1] >= lo[1]) & (xy[:, 1] <= hi[1]))
        if not len(idx):
            return []
        a, b = pts[:-1], pts[1:]
        ab = b - a
        l2 = np.maximum((ab * ab).sum(axis=1), 1e-9)
        cum = np.concatenate([[0.0], np.cumsum(np.sqrt(l2))])
        best: dict[str, tuple[ServicePoint, float, float]] = {}
        for i in idx.tolist():
            p = xy[i]
            t = np.clip(((p - a) * ab).sum(axis=1) / l2, 0.0, 1.0)
            d = np.hypot(*(a + t[:, None] * ab - p).T)
            j = int(np.argmin(d))
            if d[j] > corridor_m:
                continue
            c = cand[i]
            along = float(cum[j] + t[j] * math.sqrt(l2[j]))
            if c.site not in best or d[j] < best[c.site][2]:
                best[c.site] = (c, along, float(d[j]))
        return sorted(best.values(), key=lambda r: r[1])


def _load(path: Path) -> list:
    if not path.exists():
        return []
    with open(path, encoding="utf-8") as f:
        return json.load(f)
