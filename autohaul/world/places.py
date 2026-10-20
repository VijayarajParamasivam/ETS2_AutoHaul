"""Cities and company depots from the parsed map, and where the current job wants the truck to go."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from autohaul.io.telemetry import text


@dataclass
class Place:
    x: float
    y: float
    label: str


class Places:
    def __init__(self, map_dir: Path):
        raw = Path(map_dir) / "raw"
        with open(raw / "europe-companies.json", encoding="utf-8") as f:
            self.companies = json.load(f)
        with open(raw / "europe-cities.json", encoding="utf-8") as f:
            self.cities = {c["token"]: c for c in json.load(f)}

    def company(self, city: str, company: str | None = None) -> Place | None:
        """A company depot in `city` (any company if none given). Tokens as the game uses them (e.g.
        'nurnberg', 'kathode')."""
        for c in self.companies:
            if c["cityToken"] == city and (company is None or c["token"] == company):
                name = self.cities.get(city, {}).get("name", city)
                return Place(c["x"], c["y"], f"{c['token']} in {name}")
        return None

    def city(self, city: str) -> Place | None:
        c = self.cities.get(city)
        return Place(c["x"], c["y"], c.get("name", city)) if c else None

    def resolve(self, spec: str) -> Place | None:
        """'CITY' (city centre), 'CITY:COMPANY' (that depot) or 'X,Y' (map coordinates)."""
        if "," in spec:
            x, y = (float(v) for v in spec.split(","))
            return Place(x, y, f"({x:.0f}, {y:.0f})")
        if ":" in spec:
            city, company = spec.split(":", 1)
            return self.company(city.strip().lower(), company.strip().lower())
        return self.city(spec.strip().lower())

    def job_target(self, cfg_block) -> Place | None:
        """Where the active job wants the truck: the destination depot once the cargo is loaded,
        otherwise the source depot (where the trailer waits)."""
        if cfg_block is None or not cfg_block.job.present:
            return None
        j = cfg_block.job
        if j.cargo_loaded:
            return self.company(text(j.destination_city_id), text(j.destination_company_id))
        return self.company(text(j.source_city_id), text(j.source_company_id))
