"""Junction rules from the map: right of way, traffic rules and traffic lights for every junction path.

The game's map data gives each path through a junction (a prefab "nav curve"):
  priority      flags bits 16-19. Where two paths of a junction cross or merge, the lower one gives way;
                the game's AI drives by the same numbers
  blinker       flags bits 2-4: the indicator the AI uses on the path
  traffic rule  a token from the game's world/traffic_rules.sii, '' for none
  semaphore id  the traffic light or barrier controlling the path, -1 for none. All of the junction's
                semaphores with that id (one per light head or barrier arm) switch together
Needs the map extracted with patches/trucksim_maps/0001. Cached next to the lane map (`lanes_rules.npz`),
keyed by the lane map's build time.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from autohaul.world.lanemap import KIND_PREFAB, LaneMap

RULES_VERSION = 2          # 2: junctions whose semaphores couldn't be placed are listed

PRIORITY_SHIFT, PRIORITY_MASK = 16, 0xF
FORCE_NO_BLINKER, RIGHT_BLINKER, LEFT_BLINKER = 0x04, 0x08, 0x10
SMALL_VEHICLES, LARGE_VEHICLES = 0x20, 0x40

# semaphore types (SCS Blender Tools, PrefabLocators.TST)
SEM_PROFILE, SEM_MODEL_ONLY = 0, 1
SEM_LIGHTS = {2: "traffic light", 3: "traffic light (minor road)", 4: "traffic light (major road)",
              7: "traffic light (blockable)", 9: "virtual traffic light"}
SEM_BARRIERS = {5: "barrier (timed)", 6: "barrier (distance)", 8: "barrier (fuel station)",
                10: "barrier (automatic)"}


def profile_kind(name: str) -> str | None:
    """What a semaphore profile (the game's semaphore_profile tokens) controls. Measured on 1.53: toll booths use
    *_toll profiles (ab_toll, es_toll, pt_toll, at_toll, rus_toll_b, tol_g_l_sp), borders bd_* / border_* /
    ab_*bd*, gates br_* / bar_* / b_gen_*, fuel stations gas_* (their own pump signals: nothing to stop for)."""
    n = name.lower()
    if "toll" in n or n.startswith("tol_"):
        return "toll gate"
    if n.startswith(("bd_", "border")) or (n.startswith("ab_") and "bd" in n):
        return "border check"
    if n.startswith("weigh"):
        return "weigh station"
    if n.startswith(("br_", "bar_", "b_gen")):
        return "barrier"
    if n.startswith("gas_"):
        return None
    return "traffic light"


class MapTooOld(RuntimeError):
    """The extracted map has no junction rules: re-extract with the patched parser."""


class JunctionRules:
    def __init__(self, flags: np.ndarray, rule: np.ndarray, sem_id: np.ndarray, rule_names: list[str],
                 sem_item: np.ndarray, sem_sid: np.ndarray, sem_xyz: np.ndarray, sem_rot: np.ndarray,
                 sem_type: np.ndarray, sem_profile: np.ndarray, profile_names: list[str], sem_cycle: np.ndarray,
                 sem_intervals: np.ndarray, meta: dict):
        self.flags = flags                  # (L,) uint32 nav-curve flags (0 for road lanes)
        self.rule = rule                    # (L,) int16 index into rule_names, -1 for none
        self.sem_id = sem_id                # (L,) int16 semaphore id within the junction, -1 for none
        self.rule_names = rule_names
        self.sem_item = sem_item            # (S,) uint64 junction uid, sorted
        self.sem_sid = sem_sid              # (S,) int16 semaphore id
        self.sem_xyz = sem_xyz              # (S, 3) world x, y (map plane), z (height)
        self.sem_rot = sem_rot              # (S,) world rotation, same convention as the lane map
        self.sem_type = sem_type            # (S,) int8 SEM_* type
        self.sem_profile = sem_profile      # (S,) int16 index into profile_names
        self.profile_names = profile_names
        self.sem_cycle = sem_cycle          # (S,) s
        self.sem_intervals = sem_intervals  # (S, 4) s
        self.meta = meta
        self._lane_item: np.ndarray | None = None
        # junctions with semaphores the build couldn't place: their lights exist but aren't known
        self.misfit = {int(i, 16) for i in meta.get("misfit_items", [])}

    # ------------------------------------------------------------- queries
    def priority(self, lane: int) -> int:
        return int(self.flags[lane]) >> PRIORITY_SHIFT & PRIORITY_MASK

    def blinker(self, lane: int) -> str | None:
        f = int(self.flags[lane])
        return "left" if f & LEFT_BLINKER else "right" if f & RIGHT_BLINKER else None

    def cars_only(self) -> np.ndarray:
        """(L,) bool: junction paths the map opens to small vehicles only (no trucks), e.g. car pumps."""
        f = self.flags
        return ((f & SMALL_VEHICLES) != 0) & ((f & LARGE_VEHICLES) == 0)

    def traffic_rule(self, lane: int) -> str:
        i = int(self.rule[lane])
        return self.rule_names[i] if i >= 0 else ""

    def semaphores(self, lane: int, item: int) -> np.ndarray:
        """Indices of the semaphores (light heads / barrier arms) controlling `lane` of junction `item`."""
        sid = int(self.sem_id[lane])
        if sid < 0:
            return np.empty(0, dtype=np.int64)
        lo = np.searchsorted(self.sem_item, np.uint64(item), "left")
        hi = np.searchsorted(self.sem_item, np.uint64(item), "right")
        return lo + np.flatnonzero(self.sem_sid[lo:hi] == sid)

    def unpayable_toll_lanes(self, lane_item: np.ndarray) -> np.ndarray:
        """(L,) bool: toll booth paths controlled by an `ab_toll_no` semaphore in a plaza that also has paying
        booths (20 plazas on 1.53). The router keeps to the paying ones."""
        out = np.zeros(len(self.sem_id), dtype=bool)
        names = self.profile_names
        no = {i for i, n in enumerate(names) if n == "ab_toll_no"}
        if not no:
            return out
        items_no, items_pay = set(), set()
        for it, pi in zip(self.sem_item.tolist(), self.sem_profile.tolist()):
            if pi in no:
                items_no.add(int(it))
            elif 0 <= pi < len(names) and profile_kind(names[pi]) == "toll gate":
                items_pay.add(int(it))
        mixed = items_no & items_pay
        for ln in np.flatnonzero(self.sem_id >= 0):
            item = int(lane_item[ln])
            if item in mixed:
                heads = self.semaphores(int(ln), item)
                if len(heads) and all(int(self.sem_profile[h]) in no for h in heads):
                    out[ln] = True
        return out

    def toll_plaza(self, item: int) -> bool:
        """The junction has a toll booth (any of its semaphores has a toll profile)."""
        cache = self.__dict__.setdefault("_plaza", {})
        if item not in cache:
            lo = np.searchsorted(self.sem_item, np.uint64(item), "left")
            hi = np.searchsorted(self.sem_item, np.uint64(item), "right")
            cache[item] = any(int(t) == SEM_PROFILE and profile_kind(self.profile_names[int(i)]) == "toll gate"
                              for i, t in zip(self.sem_profile[lo:hi], self.sem_type[lo:hi])
                              if 0 <= int(i) < len(self.profile_names))
        return cache[item]

    def control(self, lane: int, item: int) -> str | None:
        """What controls the path besides right of way: "traffic light...", "toll gate", "border check",
        "barrier...", "weigh station", or None (none, decoration only, or a fuel station's own signals)."""
        if int(self.sem_id[lane]) < 0:
            return None
        heads = self.semaphores(lane, item)
        if not len(heads) and item not in self.misfit:
            # the path has a signal id but the junction defines no semaphore: signals for the AI only, no light
            # to stop for (2026-10-29: AutoHaul stood at a country junction "waiting" for a light that isn't there)
            return None
        kinds = {profile_kind(self.profile_names[int(i)]) for i, t in zip(self.sem_profile[heads], self.sem_type[heads])
                 if int(t) == SEM_PROFILE and 0 <= int(i) < len(self.profile_names)}
        for k in ("toll gate", "border check", "weigh station", "barrier"):
            if k in kinds:
                return k
        if kinds == {None}:
            return None
        types = {int(t) for t in self.sem_type[heads]}
        for t in sorted(types):
            if t in SEM_BARRIERS:
                return SEM_BARRIERS[t]
        for t in sorted(types):
            if t in SEM_LIGHTS:
                return SEM_LIGHTS[t]
        if types and types <= {SEM_MODEL_ONLY}:
            return None                     # decoration only
        return "traffic light"              # a profile semaphore, or the id has no semaphore: assume a light

    # ------------------------------------------------------------- build
    @classmethod
    def build(cls, lm: LaneMap, raw_dir: Path, log=print) -> "JunctionRules":
        t0 = time.time()

        def load(name):
            with open(Path(raw_dir) / f"europe-{name}.json", encoding="utf-8") as f:
                return json.load(f)

        log("loading prefab descriptions ...")
        descs = {d["token"]: d for d in load("prefabDescriptions")}
        if not any("flags" in cv for d in descs.values() for cv in d.get("navCurves", [])[:1]):
            raise MapTooOld(f"{raw_dir}: no nav-curve flags; run tools/fetch_deps.py and tools/extract_map.py")
        tokens = {int(p["uid"], 16): p["token"] for p in load("prefabs") if not p.get("hidden")}

        n = len(lm.lane_kind)
        flags = np.zeros(n, dtype=np.uint32)
        rule = np.full(n, -1, dtype=np.int16)
        sem_id = np.full(n, -1, dtype=np.int16)
        rule_names: list[str] = []
        rule_index: dict[str, int] = {}
        profile_names: list[str] = []
        profile_index: dict[str, int] = {}
        sems: list[tuple] = []
        misfit = 0
        misfit_items: list[int] = []

        prefab = np.flatnonzero(lm.lane_kind == KIND_PREFAB)
        order = prefab[np.argsort(lm.lane_item[prefab], kind="stable")]
        items = lm.lane_item[order]
        cuts = np.flatnonzero(np.diff(items)) + 1
        for group in np.split(order, cuts):
            item = int(lm.lane_item[group[0]])
            desc = descs.get(tokens.get(item, ""))
            if desc is None:
                continue
            curves = desc["navCurves"]
            for ln in group.tolist():
                cv = curves[int(lm.lane_sub[ln])]
                flags[ln] = cv.get("flags", 0)
                tr = cv.get("trafficRule", "")
                if tr:
                    if tr not in rule_index:
                        rule_index[tr] = len(rule_names)
                        rule_names.append(tr)
                    rule[ln] = rule_index[tr]
                sem_id[ln] = cv.get("semaphoreId", -1)
            if not desc.get("semaphores") or not (sem_id[group] >= 0).any():
                continue
            # the junction's placement, from its lanes (prefab coordinates -> world): rotation + translation
            loc, wld, dz = [], [], []
            for ln in group.tolist():
                cv = curves[int(lm.lane_sub[ln])]
                f, la = lm.lane_first[ln], lm.lane_last[ln]
                loc += [(cv["start"]["x"], cv["start"]["y"]), (cv["end"]["x"], cv["end"]["y"])]
                wld += [tuple(lm.seg[f, 0:2]), tuple(lm.seg[la, 2:4])]
                dz.append(lm.seg[f, 4] - cv["start"].get("z", 0.0))
            P, W = np.asarray(loc), np.asarray(wld)
            pc, wc = P.mean(axis=0), W.mean(axis=0)
            a, b = P - pc, W - wc
            th = math.atan2(float((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum()), float((a * b).sum()))
            c, s = math.cos(th), math.sin(th)
            fit = np.hypot(*(a @ np.array([[c, s], [-s, c]]) - b).T).max()
            if fit > 1.0:
                misfit += 1
                misfit_items.append(item)
                continue
            z0 = float(np.median(dz))
            for sm in desc["semaphores"]:
                lx, ly = sm["x"] - pc[0], sm["y"] - pc[1]
                prof = sm.get("profile", "")
                if prof not in profile_index:
                    profile_index[prof] = len(profile_names)
                    profile_names.append(prof)
                sid = sm["id"] if sm["id"] < 0x8000 else -1
                sems.append((item, sid, wc[0] + lx * c - ly * s, wc[1] + lx * s + ly * c, z0 + sm["z"],
                             sm["rotation"] + th, sm["type"], profile_index[prof], sm["cycle"],
                             *sm["intervals"]))
        sem_item = np.array([sm[0] for sm in sems], dtype=np.uint64)
        o = np.argsort(sem_item, kind="stable")
        sem_item = sem_item[o]
        arr = np.array([sm[1:] for sm in sems], dtype=float).reshape(-1, 12)[o]
        meta = {"rules_version": RULES_VERSION, "lanes_built": lm.meta.get("built"), "semaphores": len(arr),
                "misfit_junctions": misfit, "misfit_items": [f"{i:x}" for i in misfit_items],
                "build_s": round(time.time() - t0, 1)}
        log(f"junction rules: {int((flags != 0).sum())} paths with flags, {int((rule >= 0).sum())} with a traffic "
            f"rule, {int((sem_id >= 0).sum())} with a semaphore; {len(arr)} semaphores in {meta['build_s']} s")
        return cls(flags, rule, sem_id, rule_names, sem_item, arr[:, 0].astype(np.int16), arr[:, 1:4].copy(),
                   arr[:, 4].copy(), arr[:, 5].astype(np.int8), arr[:, 6].astype(np.int16), profile_names,
                   arr[:, 7].copy(), arr[:, 8:12].copy(), meta)

    # ------------------------------------------------------------- persistence
    def save(self, path: Path) -> None:
        meta = dict(self.meta, rule_names=self.rule_names, profile_names=self.profile_names)
        np.savez_compressed(path, flags=self.flags, rule=self.rule, sem_id=self.sem_id, sem_item=self.sem_item,
                            sem_sid=self.sem_sid, sem_xyz=self.sem_xyz, sem_rot=self.sem_rot,
                            sem_type=self.sem_type, sem_profile=self.sem_profile, sem_cycle=self.sem_cycle,
                            sem_intervals=self.sem_intervals,
                            meta=np.frombuffer(json.dumps(meta).encode(), dtype=np.uint8))

    @classmethod
    def load(cls, path: Path) -> "JunctionRules":
        z = np.load(path)
        meta = json.loads(bytes(z["meta"]).decode())
        rule_names, profile_names = meta.pop("rule_names"), meta.pop("profile_names")
        return cls(z["flags"], z["rule"], z["sem_id"], rule_names, z["sem_item"], z["sem_sid"], z["sem_xyz"],
                   z["sem_rot"], z["sem_type"], z["sem_profile"], profile_names, z["sem_cycle"],
                   z["sem_intervals"], meta)

    @classmethod
    def load_or_build(cls, lm: LaneMap, map_dir: Path, log=print) -> "JunctionRules":
        path = Path(map_dir) / "lanes_rules.npz"
        if path.exists():
            r = cls.load(path)
            if r.meta.get("rules_version") == RULES_VERSION and r.meta.get("lanes_built") == lm.meta.get("built"):
                return r
            log("junction rules are out of date; rebuilding")
        r = cls.build(lm, Path(map_dir) / "raw", log=log)
        r.save(path)
        return r

    @classmethod
    def empty(cls, lm: LaneMap) -> "JunctionRules":
        """No rules known (synthetic maps in tests): every path priority 0, no lights."""
        n = len(lm.lane_kind)
        z = np.zeros(0)
        return cls(np.zeros(n, dtype=np.uint32), np.full(n, -1, dtype=np.int16), np.full(n, -1, dtype=np.int16),
                   [], z.astype(np.uint64), z.astype(np.int16), np.zeros((0, 3)), z, z.astype(np.int8),
                   z.astype(np.int16), [], z, np.zeros((0, 4)), {"rules_version": RULES_VERSION})
