"""The active job (from the bridge's job configuration), the in-game clock, and the history of finished jobs.

In-game time is in minutes since Monday 00:00 of week 1 (telemetry `game_time`); the job's deadline is in the
same clock. Finished jobs (the `job.delivered` / `job.cancelled` gameplay events) go to `jobs.jsonl` so the
economics can learn how lateness and time pay (see economics.calibrate).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

from autohaul.io.telemetry import text

DAYS = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")


def clock(minutes: float) -> str:
    """In-game minutes -> 'Tue 05:46'."""
    m = int(round(minutes))
    return f"{DAYS[(m // 1440) % 7]} {(m // 60) % 24:02d}:{m % 60:02d}"


def duration(minutes: float) -> str:
    """Minutes -> '7h 14min' (negative: '-1h 05min')."""
    sign = "-" if minutes < 0 else ""
    m = int(round(abs(minutes)))
    return f"{sign}{m // 60}h {m % 60:02d}min"


@dataclass(frozen=True)
class Job:
    cargo: str
    cargo_loaded: bool
    source_city: str            # tokens (city / company ids), as Places.company() takes them
    source_company: str
    destination_city: str
    destination_company: str
    source_label: str           # what the game calls them
    destination_label: str
    income: int                 # EUR
    deadline: int               # in-game minutes (absolute)
    planned_km: int             # game km
    market: str
    special: bool
    mass_kg: float

    @classmethod
    def from_config(cls, cfg_block) -> "Job | None":
        if cfg_block is None or not cfg_block.job.present:
            return None
        j = cfg_block.job
        return cls(text(j.cargo) or text(j.cargo_id), bool(j.cargo_loaded),
                   text(j.source_city_id), text(j.source_company_id),
                   text(j.destination_city_id), text(j.destination_company_id),
                   f"{text(j.source_company)} in {text(j.source_city)}",
                   f"{text(j.destination_company)} in {text(j.destination_city)}",
                   int(j.income), int(j.delivery_time), int(j.planned_distance_km), text(j.job_market),
                   bool(j.special_job), float(j.cargo_mass))

    @property
    def key(self) -> tuple:
        """Identifies the job across config updates (the loaded flag changes; the job doesn't)."""
        return (self.source_city, self.source_company, self.destination_city, self.destination_company,
                self.cargo, self.deadline)


class JobHistory:
    """Finished jobs, one JSON record per line."""

    def __init__(self, path: Path):
        self.path = Path(path)

    def load(self) -> list[dict]:
        if not self.path.exists():
            return []
        out = []
        with open(self.path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        continue
        return out

    def add(self, record: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")

    def record_event(self, job: Job | None, event, started_game_time: int | None) -> dict | None:
        """Store a job.delivered / job.cancelled event with what the job promised. Returns the record."""
        if event.id not in ("job.delivered", "job.cancelled"):
            return None
        rec = {"event": event.id.split(".")[1], "game_time": event.game_time, "attrs": dict(event.attrs)}
        if job is not None:
            rec.update(job=asdict(job), late_min=event.game_time - job.deadline)
            if started_game_time is not None:
                rec["job_minutes"] = event.game_time - started_game_time
        if event.id == "job.delivered":
            rec["revenue"] = event.get_int("revenue")
        else:
            rec["penalty"] = event.get_int("cancel.penalty")
        self.add(rec)
        return rec
