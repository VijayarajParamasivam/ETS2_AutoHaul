"""Mission settings from config/autohaul.toml: [mission] (MissionParams and PlannerParams fields),
[mission.economics] (EconParams) and [mission.actions] (ActionParams). Any field of those can be set."""

from __future__ import annotations

import dataclasses

from autohaul.mission.actions import ActionParams
from autohaul.mission.economics import EconParams
from autohaul.mission.manager import MissionParams
from autohaul.mission.planner import PlannerParams


def _fill(cls, d: dict):
    names = {f.name for f in dataclasses.fields(cls)}
    return cls(**{k: v for k, v in d.items() if k in names})


def mission_settings(cfg) -> tuple[MissionParams, PlannerParams, EconParams, ActionParams]:
    m = cfg.get("mission", {}) or {}
    return (_fill(MissionParams, m), _fill(PlannerParams, m), _fill(EconParams, m.get("economics", {}) or {}),
            _fill(ActionParams, m.get("actions", {}) or {}))
