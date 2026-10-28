"""What a plan is worth: income kept against fuel, time and lateness.

  EV = income x late_factor(lateness) - fuel_extra - time_value x hours
    late_factor  1 on time; late: 1 - late_fixed - late_per_hour x hours late, never below late_floor. The game's
                 own formula isn't documented, so these are settings, refitted from your delivered jobs
                 (jobs.jsonl: revenue / income against lateness) once there are a few late ones.
    fuel_extra   what this plan's fuel costs beyond buying it at the average price: the detour's fuel, plus the
                 price difference on the litres bought (fuel bought at a cheap station is a saving)
    time_value   what a game hour of the truck is worth (income per game hour of past jobs, when known)
Options that break a rule (running dry, driving past the rest limit) are dropped before scoring.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np


@dataclass
class EconParams:
    late_fixed: float = 0.2           # share of income lost for being late at all
    late_per_hour: float = 0.05       # ... and per game hour late
    late_floor: float = 0.3
    time_value_eur_h: float = 600.0   # EUR per game hour of the truck's time
    min_jobs_for_fit: int = 3


def late_factor(late_min: float, p: EconParams) -> float:
    if late_min <= 0:
        return 1.0
    return max(p.late_floor, 1.0 - p.late_fixed - p.late_per_hour * late_min / 60.0)


def value(income: float, late_min: float, fuel_extra: float, hours: float, p: EconParams) -> float:
    return income * late_factor(late_min, p) - fuel_extra - p.time_value_eur_h * hours


def calibrate(history: list[dict], p: EconParams) -> tuple[EconParams, str]:
    """Refit the time value and the lateness penalty from delivered jobs. Returns (params, what changed)."""
    done = [h for h in history if h.get("event") == "delivered" and h.get("job")]
    notes = []
    timed = [h for h in done if h.get("job_minutes", 0) > 30]
    if len(timed) >= p.min_jobs_for_fit:
        eur_h = [h["revenue"] / (h["job_minutes"] / 60.0) for h in timed[-20:]]
        p = replace(p, time_value_eur_h=float(np.median(eur_h)))
        notes.append(f"time value {p.time_value_eur_h:.0f} EUR/h from {len(eur_h)} jobs")
    late = [h for h in done if h.get("late_min", 0) > 0 and h["job"].get("income")]
    if len(late) >= p.min_jobs_for_fit:
        hours = np.array([h["late_min"] / 60.0 for h in late])
        lost = np.array([1.0 - h["revenue"] / h["job"]["income"] for h in late])
        a = np.vstack([np.ones_like(hours), hours]).T
        (fixed, per_h), *_ = np.linalg.lstsq(a, lost, rcond=None)
        p = replace(p, late_fixed=float(np.clip(fixed, 0.0, 0.9)), late_per_hour=float(np.clip(per_h, 0.0, 0.5)))
        notes.append(f"late penalty {p.late_fixed:.0%} + {p.late_per_hour:.0%}/h from {len(late)} late jobs")
    return p, "; ".join(notes) or "defaults (not enough delivered jobs to fit)"
