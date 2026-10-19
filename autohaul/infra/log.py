"""Structured logging: JSON lines to <cache>/logs/<run_id>/<name>.jsonl plus a short console format.

Usage:
    log = get_logger("safety")
    log.info("kill_switch", extra=fields(source="keyboard", key="BACKSLASH"))
"""

from __future__ import annotations

import json
import logging
import sys
import time
from pathlib import Path
from typing import Any

_FIELDS_ATTR = "ah_fields"
_configured = False


def fields(**kwargs: Any) -> dict[str, Any]:
    return {_FIELDS_ATTR: kwargs}


class _JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        entry = {
            "t": round(record.created, 6),
            "level": record.levelname.lower(),
            "logger": record.name,
            "event": record.getMessage(),
        }
        extra = getattr(record, _FIELDS_ATTR, None)
        if extra:
            entry.update(extra)
        if record.exc_info:
            entry["exc"] = self.formatException(record.exc_info)
        return json.dumps(entry, default=str)


class _ConsoleFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        extra = getattr(record, _FIELDS_ATTR, None) or {}
        tail = " ".join(f"{k}={v}" for k, v in extra.items())
        stamp = time.strftime("%H:%M:%S", time.localtime(record.created))
        return f"{stamp} {record.levelname[0]} {record.name}: {record.getMessage()} {tail}".rstrip()


def setup_logging(log_dir: Path | None, run_id: str | None = None, level: int = logging.INFO) -> str:
    """Configure root logging once. Returns the run id."""
    global _configured
    run_id = run_id or time.strftime("%Y%m%d-%H%M%S")
    root = logging.getLogger("autohaul")
    if _configured:
        return run_id
    root.setLevel(level)
    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(_ConsoleFormatter())
    root.addHandler(console)
    if log_dir is not None:
        run_dir = log_dir / run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        fh = logging.FileHandler(run_dir / "autohaul.jsonl", encoding="utf-8")
        fh.setFormatter(_JsonFormatter())
        root.addHandler(fh)
    _configured = True
    return run_id


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(f"autohaul.{name}")
