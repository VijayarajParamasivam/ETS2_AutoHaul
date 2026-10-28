"""The decision log: one JSON record per mission decision (logs/decisions.jsonl), with the inputs, every
candidate and option with its numbers, the choice, and a one-line rationale."""

from __future__ import annotations

import json
import time
from pathlib import Path


class DecisionLog:
    def __init__(self, path: Path | None):
        self.path = Path(path) if path else None
        self.records: list[dict] = []          # this session's, for the status line and tests

    def add(self, kind: str, record: dict) -> dict:
        rec = {"t": round(time.time(), 3), "decision": kind, **record}
        self.records.append(rec)
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, default=str) + "\n")
        return rec

    @property
    def last(self) -> dict | None:
        return self.records[-1] if self.records else None
