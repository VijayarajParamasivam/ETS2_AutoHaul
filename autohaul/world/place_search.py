"""Finding a city or a company by what someone types: the launcher's suggestions.

Names are matched without accents and case, so "nurn" finds Nürnberg and "lodz" finds Łódź. The game's own tokens match too.
Reads only the parsed map's JSON, which takes milliseconds.
"""

from __future__ import annotations

import json
import unicodedata
from dataclasses import dataclass
from pathlib import Path

# letters that have no accent to strip
_LETTERS = str.maketrans({"ł": "l", "đ": "d", "ø": "o", "æ": "ae", "œ": "oe", "ß": "ss", "ı": "i", "þ": "th", "ð": "d"})
_COUNTRIES = {"uk": "UK", "usa": "USA"}


def fold(text: str) -> str:
    """Lower case, without accents."""
    s = unicodedata.normalize("NFKD", text.casefold().translate(_LETTERS))
    return "".join(c for c in s if not unicodedata.combining(c)).strip()


@dataclass(frozen=True)
class Option:
    token: str              # what the game (and drive.py --to) calls it
    name: str               # what people call it
    label: str              # what the box shows


def search(query: str, options: list[Option], limit: int = 8) -> list[Option]:
    """The options matching `query`, best first: the name itself, then how it starts, a later word's start, the token,
    anywhere in the name. Nothing typed: all of them, by name."""
    q = fold(query)
    if not q:
        return sorted(options, key=lambda o: fold(o.name))[:limit]
    found = []
    for o in options:
        name, token = fold(o.name), o.token.casefold()
        if q == name or q == token or q == fold(o.label):
            rank = 0
        elif name.startswith(q):
            rank = 1
        elif any(word.startswith(q) for word in name.replace("-", " ").split()):
            rank = 2
        elif token.startswith(q):
            rank = 3
        elif q in name or q in fold(o.label):
            rank = 4
        else:
            continue
        found.append((rank, name, o))
    found.sort(key=lambda row: row[:2])
    return [o for _, _, o in found[:limit]]


def exact(text: str, options: list[Option]) -> Option | None:
    """The option this text names exactly (its label, name or token), or None."""
    q = fold(text)
    if not q:
        return None
    for o in options:
        if q in (fold(o.label), fold(o.name), o.token.casefold()):
            return o
    return None


class PlaceIndex:
    def __init__(self, map_dir: Path):
        raw = Path(map_dir) / "raw"
        with open(raw / "europe-cities.json", encoding="utf-8") as f:
            cities = json.load(f)
        with open(raw / "europe-companies.json", encoding="utf-8") as f:
            depots = json.load(f)
        with open(raw / "europe-companyDefs.json", encoding="utf-8") as f:
            names = {d["token"]: d.get("name") or d["token"] for d in json.load(f)}
        self.cities = [Option(c["token"], c.get("name") or c["token"], self._city_label(c)) for c in cities]
        by_city: dict[str, dict[str, Option]] = {}
        for d in depots:
            name = names.get(d["token"], d["token"])
            by_city.setdefault(d["cityToken"], {})[d["token"]] = Option(d["token"], name, name)
        self._companies = {city: list(found.values()) for city, found in by_city.items()}

    @staticmethod
    def _city_label(c: dict) -> str:
        name, country = c.get("name") or c["token"], str(c.get("countryToken") or "")
        if not country:
            return name
        return f"{name}, {_COUNTRIES.get(country, country.replace('_', ' ').title())}"

    def companies(self, city_token: str) -> list[Option]:
        """The companies with a depot in this city."""
        return self._companies.get(city_token, [])
