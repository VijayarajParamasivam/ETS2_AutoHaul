"""Fuel and rest stops for the route ahead: whether one is needed, where, and why.

  needs     fuel: the litres to the destination plus a reserve (`reserve_km` game km) are more than in the tank.
            rest: with the game's fatigue on, the drive to the destination ends less than `rest_buffer_min`
            before the rest timer runs out.
  where     fuel stations and places to sleep within `corridor_m` of the route: stations reachable keeping half
            the reserve, places to sleep reachable `rest_buffer_min` before the rest timer runs out. A few of
            each (the farthest ones, the cheapest stations, and sites with both) get a detour measured on the
            lane graph: off the route to the stop, and back to the route `rejoin_m` further on.
  which     every combination that covers the needs, scored by economics.value (lateness, fuel price and
            detour, time). The best one wins; the decision record lists every candidate and option.
Distances are world metres (the lane map's); the record also shows game km (x19, the open-road map scale).
"""

from __future__ import annotations

import itertools
from dataclasses import asdict, dataclass, field

import numpy as np

from autohaul.mission import economics
from autohaul.mission.economics import EconParams
from autohaul.mission.job import clock, duration
from autohaul.mission.rates import OPEN_ROAD_SCALE, Rates
from autohaul.world.route import Route, Router
from autohaul.world.services import ServicePoint, Services


@dataclass
class PlannerParams:
    reserve_km: float = 150.0          # game km of fuel always kept
    arrive_min_share: float = 0.2      # ... and at least this share of the tank left on arrival: no ending a job
                                       # near empty because the job itself was just covered (2026-10-30)
    rest_buffer_min: float = 60.0      # arrive at a place to sleep this long before the rest timer runs out
    corridor_m: float = 400.0          # world metres from the route (~8 game km)
    fuel_corridors_m: tuple = (400.0, 800.0, 1600.0)   # no pump in reach: look further off the route (2026-10-30: 1 pump
                                       # within 400 m on 465 game km, out of reach, so "none possible" from 165 l to 67 l)
    fuel_detour_max_km: float = 40.0   # game km: a pump further out of the way isn't a good answer while wider looks may
                                       # find a better one (an 8 km ahead one was a 306 km detour, with one at 280 km)
    min_ahead_m: float = 150.0         # stops closer than this are behind us by the time we could turn off
    branch_back_m: float = 900.0       # detours are measured from this far before the stop ...
    rejoin_m: float = 2500.0           # ... to this far after it, along the route
    candidates: int = 4                # farthest-first candidates of each kind (plus cheapest, plus combined)
    refuel_min: float = 15.0           # game minutes a refuel stop takes
    sleep_min: float = 540.0           # ... and a sleep
    fuel_target: float = 0.98          # fill to this share of the tank


@dataclass
class Stop:
    kind: str                          # "fuel" or "sleep"
    point: ServicePoint
    along_m: float                     # where the route passes it
    detour_m: float
    eta_min: float                     # game minutes from now to reach it
    litres: float = 0.0                # planned purchase (fuel)
    price: float = 0.0

    def describe(self) -> str:
        where = f"{self.along_m * OPEN_ROAD_SCALE / 1000:.0f} game km ahead, {self.point.country or '?'}"
        if self.kind == "fuel":
            return f"refuel ~{self.litres:.0f} l at {self.price:.2f} EUR/l ({where})"
        return f"sleep ({where})"


@dataclass
class Plan:
    stops: list[Stop] = field(default_factory=list)
    value: float = 0.0
    record: dict = field(default_factory=dict)
    problem: str | None = None         # no acceptable option: why (the driver must step in)


@dataclass
class PlanState:
    route: Route                       # to the job's current target, from the truck
    fuel_l: float
    capacity_l: float
    game_time: float
    next_rest: float | None            # game minutes until a rest is due; None when the game's fatigue is off
    income: float = 0.0                # 0 without a job (nothing to lose by lateness)
    deadline: float | None = None


class StopPlanner:
    def __init__(self, router: Router, services: Services, rates: Rates, econ: EconParams | None = None,
                 params: PlannerParams | None = None):
        self.router, self.services, self.rates = router, services, rates
        self.econ = econ or EconParams()
        self.p = params or PlannerParams()
        self.lm = router.lm

    # ------------------------------------------------------------- route geometry
    def _polyline(self, rt: Route, start_progress: float) -> tuple[np.ndarray, list[tuple[float, int, float]]]:
        """The route as a polyline from the truck, and (metres along it, lane, metres into that lane) for every
        lane's start. A lane the route changes out of is skipped: the change happens alongside."""
        pts, starts, along = [], [], 0.0
        prev = None
        for i, lane in enumerate(rt.lanes):
            if i + 1 < len(rt.lanes) and rt.changes[i + 1]:
                continue
            p = self.lm.lane_points(lane)
            into = 0.0
            if i == 0 or (i == 1 and rt.changes[1]):
                d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
                into = min(start_progress, float(d[-1]))
                k = int(np.searchsorted(d, into))
                p = np.vstack([np.interp(into, d, p[:, 0]), np.interp(into, d, p[:, 1])]).T.tolist() + p[k:].tolist()
                p = np.array(p)
            if prev is not None:
                along += float(np.hypot(*(p[0] - prev)))
            starts.append((along, lane, into))
            seg = np.hypot(*np.diff(p, axis=0).T)
            pts.append(p)
            along += float(seg.sum())
            prev = p[-1]
        poly = np.vstack(pts) if pts else np.zeros((0, 2))
        keep = np.concatenate([[True], np.hypot(*np.diff(poly, axis=0).T) > 1e-6]) if len(poly) else []
        return poly[keep], starts

    @staticmethod
    def _at(starts, along: float) -> tuple[int, float]:
        """(lane, metres into it) where the route is `along` metres from the truck."""
        k = max(0, int(np.searchsorted([s[0] for s in starts], along, "right")) - 1)
        a0, lane, into = starts[k]
        return lane, into + (along - a0)

    def _point(self, poly: np.ndarray, along: float) -> tuple[float, float]:
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(poly, axis=0).T))])
        return float(np.interp(along, d, poly[:, 0])), float(np.interp(along, d, poly[:, 1]))

    def detour(self, sp: ServicePoint, along: float, poly: np.ndarray, starts, total: float) -> float | None:
        """Extra world metres to stop at `sp` instead of driving on: off the route before it, back on after."""
        b = max(0.0, along - self.p.branch_back_m)
        r = min(total, along + self.p.rejoin_m)
        bl, bp = self._at(starts, b)
        rl, rp = self._at(starts, r)
        rx, ry = self._point(poly, r)
        there = self.router.route(bl, bp, (sp.x, sp.y), goal=(sp.lane, sp.s), max_expand=200_000)
        back = self.router.route(sp.lane, sp.s, (rx, ry), goal=(rl, rp), max_expand=200_000)
        if there is None or back is None:
            return None
        return max(0.0, there.length_m + back.length_m - (r - b))

    # ------------------------------------------------------------- planning
    def plan(self, s: PlanState, start_progress: float) -> Plan:
        p, rates = self.p, self.rates
        fuel_m = rates.fuel_per_m()
        min_m = rates.min_per_m
        game_km = OPEN_ROAD_SCALE / 1000.0
        poly, starts = self._polyline(s.route, start_progress)
        total = s.route.length_m
        scaled = s.route.scaled_m or total
        reserve_l = max(p.reserve_km / game_km * fuel_m, p.arrive_min_share * s.capacity_l)
        need_l = scaled * fuel_m + reserve_l
        drive_min = scaled * min_m
        fuel_needed = s.fuel_l < need_l
        rest_needed = s.next_rest is not None and s.next_rest - p.rest_buffer_min < drive_min
        rec: dict = {
            "game_time": round(s.game_time), "clock": clock(s.game_time),
            "route_game_km": round(scaled * game_km), "drive": duration(drive_min),
            "fuel_l": round(s.fuel_l), "fuel_needed_l": round(need_l), "reserve_l": round(reserve_l),
            "litres_per_game_km": round(fuel_m / game_km, 3), "game_min_per_game_km": round(min_m / game_km, 3),
            "rest_left": None if s.next_rest is None else duration(s.next_rest),
            "deadline": None if s.deadline is None else clock(s.deadline),
            "needs": {"fuel": fuel_needed, "rest": rest_needed}, "econ": asdict(self.econ),
        }
        if not fuel_needed and not rest_needed:
            arrive = s.game_time + drive_min
            rec.update(choice="no stop", rationale=self._why_none(s, arrive, need_l))
            return Plan([], self._ev(s, total, 0.0, 0.0, []), rec)

        fuel_c, sleep_c = [], []
        known: dict[str, float | None] = {}                    # detours worked out already (they take a while)
        if fuel_needed:
            margin = p.reserve_km / game_km * fuel_m / 2       # arrive at the pump with this left, if one allows it
            longest = p.fuel_detour_max_km / game_km
            seen: dict[str, tuple] = {}                        # site -> (candidate, detour)
            for corridor in p.fuel_corridors_m:                # the narrowest that offers a pump with a sane detour
                near = [(sp, along) for sp, along, off in self.services.along("fuel", poly, corridor)
                        if along >= p.min_ahead_m and along * fuel_m <= 0.95 * s.fuel_l]
                roomy = [c for c in near if c[1] * fuel_m <= s.fuel_l - margin]
                for c in self._pick(roomy, cheap=True) + self._pick(near, cheap=True):
                    if c[0].site not in seen:
                        known[c[0].site] = d = self.detour(c[0], c[1], poly, starts, total)
                        seen[c[0].site] = (c, d)
                fair = [c for c, d in seen.values() if d is not None and d <= longest]
                if fair:                                       # with margin at the pump, if any of them gives that
                    fuel_c = [c for c in fair if c[1] * fuel_m <= s.fuel_l - margin] or fair
                    break
            else:                                              # nothing sane anywhere: the least bad one
                ranked = sorted(((d, c) for c, d in seen.values() if d is not None), key=lambda x: x[0])
                fuel_c = [c for _, c in ranked[:1]]
        if rest_needed:
            limit = s.next_rest - p.rest_buffer_min
            for sp, along, off in self.services.along("sleep", poly, p.corridor_m):
                if along >= p.min_ahead_m and along * min_m <= limit:
                    sleep_c.append((sp, along))
        fuel_c = self._pick(fuel_c, cheap=True)
        sleep_c = self._pick(sleep_c, sites={c[0].site for c in fuel_c})
        cands = []
        for kind, group in (("fuel", fuel_c), ("sleep", sleep_c)):
            for sp, along in group:
                d = known[sp.site] if kind == "fuel" and sp.site in known else self.detour(sp, along, poly, starts, total)
                stop = None if d is None else self._stop(kind, sp, along, d, s, fuel_m, min_m)
                cands.append(stop)
                rec.setdefault("candidates", []).append({
                    "kind": kind, "site": sp.site, "country": sp.country, "game_km_ahead": round(along * game_km),
                    "detour_game_km": None if d is None else round(d * game_km, 1),
                    "price": round(self.services.price_at(sp), 3) if kind == "fuel" else None,
                    "eta": clock(s.game_time + along * min_m), "reachable": d is not None})
        fuels = [c for c in cands if c and c.kind == "fuel"]
        sleeps = [c for c in cands if c and c.kind == "sleep"]
        options: list[list[Stop]] = []
        if fuel_needed and rest_needed:
            options = [sorted(pair, key=lambda c: c.along_m) for pair in itertools.product(fuels, sleeps)]
        elif fuel_needed:
            options = [[f] for f in fuels]
        else:
            options = [[sl] for sl in sleeps]
        scored = []
        for stops in options:
            ev, info = self._score(s, total, stops, fuel_m, min_m)
            scored.append((ev, stops, info))
        rec["options"] = [dict(info, stops=[st.describe() for st in stops]) for _, stops, info in scored]
        if not scored:
            plan = Plan([], 0.0, rec, problem=self._why_stuck(fuel_needed, rest_needed, fuels, sleeps))
            rec.update(choice="none possible", rationale=plan.problem)
            return plan
        ev, stops, info = max(scored, key=lambda t: t[0])
        rec.update(choice=[st.describe() for st in stops], value=round(ev),
                   rationale=self._why(stops, info, len(fuels), len(sleeps), s))
        return Plan(stops, ev, rec)

    def _pick(self, cands: list[tuple[ServicePoint, float]], cheap: bool = False,
              sites: set[str] | None = None) -> list[tuple[ServicePoint, float]]:
        """The farthest few (stop as late as allowed), the cheapest few, and sites that also sell fuel."""
        far = sorted(cands, key=lambda c: -c[1])[: self.p.candidates]
        keep = {id(c): c for c in far}
        if cheap:
            for c in sorted(cands, key=lambda c: self.services.price_at(c[0]))[:2]:
                keep[id(c)] = c
        if sites:
            for c in cands:
                if c[0].site in sites:
                    keep[id(c)] = c
        return sorted(keep.values(), key=lambda c: c[1])

    def _stop(self, kind, sp, along, detour, s: PlanState, fuel_m, min_m) -> Stop:
        eta = (along + detour / 2) * min_m
        if kind == "fuel":
            at_pump = s.fuel_l - (along + detour / 2) * fuel_m
            litres = max(0.0, self.p.fuel_target * s.capacity_l - at_pump)
            return Stop(kind, sp, along, detour, eta, litres, self.services.price_at(sp))
        return Stop(kind, sp, along, detour, eta)

    def _ev(self, s: PlanState, total, detour, stop_min, stops) -> float:
        hours = (total + detour) * self.rates.min_per_m / 60.0 + stop_min / 60.0
        late = 0.0 if s.deadline is None else s.game_time + hours * 60.0 - s.deadline
        avg = self.services.default_price
        extra = detour * self.rates.fuel_per_m() * avg + sum(st.litres * (st.price - avg) for st in stops)
        return economics.value(s.income, late, extra, hours, self.econ)

    def _score(self, s: PlanState, total, stops: list[Stop], fuel_m, min_m) -> tuple[float, dict]:
        detour = sum(st.detour_m for st in stops)
        stop_min = sum(self.p.refuel_min if st.kind == "fuel" else self.p.sleep_min for st in stops)
        ev = self._ev(s, total, detour, stop_min, stops)
        arrive = s.game_time + (total + detour) * min_m + stop_min
        late = None if s.deadline is None else arrive - s.deadline
        avg = self.services.default_price
        info = {"value": round(ev), "arrive": clock(arrive), "late": None if late is None else duration(late),
                "detour_game_km": round(detour * OPEN_ROAD_SCALE / 1000, 1),
                "fuel_extra_eur": round(detour * fuel_m * avg + sum(st.litres * (st.price - avg) for st in stops)),
                "stop_time": duration(stop_min)}
        return ev, info

    def _why_none(self, s: PlanState, arrive: float, need_l: float) -> str:
        parts = [f"{s.fuel_l:.0f} l covers the {need_l:.0f} l needed (reserve included)"]
        if s.next_rest is not None:
            parts.append(f"the rest timer ({duration(s.next_rest)}) outlasts the drive")
        else:
            parts.append("fatigue is off")
        if s.deadline is not None:
            parts.append(f"arrive {clock(arrive)}, {duration(s.deadline - arrive)} before the deadline")
        return "no stop needed: " + "; ".join(parts)

    def _why(self, stops: list[Stop], info: dict, n_fuel: int, n_sleep: int, s: PlanState) -> str:
        parts = []
        for st in stops:
            if st.kind == "fuel":
                parts.append(f"{st.describe()}: best of {n_fuel} reachable stations")
            else:
                parts.append(f"{st.describe()}: best of {n_sleep} places to sleep before the rest limit")
        tail = f"arrive {info['arrive']}"
        if info.get("late"):
            tail += f" ({info['late']} after the deadline)" if not info["late"].startswith("-") else \
                    f" ({info['late'][1:]} before the deadline)"
        return "; ".join(parts) + "; " + tail

    @staticmethod
    def _why_stuck(fuel_needed, rest_needed, fuels, sleeps) -> str:
        if fuel_needed and not fuels:
            return "no fuel station reachable on the route with the fuel left: take over"
        if rest_needed and not sleeps:
            return "no place to sleep on the route before the rest limit: take over"
        return "no plan covers both fuel and rest: take over"
