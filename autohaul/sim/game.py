"""A stand-in for the game around TruckSim, for mission tests: the in-game clock, fuel, the rest timer, the job
(as the bridge's config block shows it), gameplay events, and the interactions AutoHaul scripts.

  clock    game minutes advance `scale` x real time (the map's time compression)
  fuel     used per world metre driven (`fuel_l_per_m`, i.e. per game km x scale)
  rest     the timer runs down with the clock when `fatigue` is on
  refuel   at a pump (within `pump_m`, standing): the flow runs while `activate` is held ("hold", as 1.53.2.1
           does) or a press starts it ("press"); `refuel_lps` litres per real second
  sleep    in a parking place (within `parking_m`, standing): `activate` opens the sleep screen, `selectfcs`
           sleeps: the clock jumps `sleep_min`, the rest timer resets to `rest_after_sleep_min`
  deliver  `deliver()` emits job.delivered as the game would once the driver has parked the trailer
  gears    `gearreverse` / `geardrive` select reverse / drive (standing only, as the automatic gearbox does)
  couple   with `trailer_pose` (a TrailerPose, uncoupled): `attach` couples it when the fifth wheel
           (`hook_back_m` behind the truck's origin) is within `couple_m` of the kingpin and the truck faces
           within `couple_deg` of the trailer's heading; the job's cargo counts as loaded from then on
"""

from __future__ import annotations

import math
from types import SimpleNamespace

from autohaul.io.telemetry import GameplayEvent
from autohaul.sim.truck import TruckSim


class SimGame:
    def __init__(self, sim: TruckSim, *, job: dict | None, fuel_l: float, capacity_l: float = 900.0,
                 game_time: float = 8 * 60, next_rest_min: float = 11 * 60, fatigue: bool = True,
                 scale: float = 19.0, fuel_l_per_m: float = 0.33 * 19 / 1000, pumps=(), parkings=(),
                 refuel_mode: str = "hold", refuel_lps: float = 15.0, sleep_min: float = 540.0,
                 rest_after_sleep_min: float = 660.0, pump_m: float = 4.0, parking_m: float = 35.0,
                 quick_park: bool = True, trailer: bool | None = None, trailer_pose=None,
                 hook_back_m: float = 1.57, kingpin_ahead_m: float = 5.0, couple_m: float = 1.2,
                 couple_deg: float = 15.0):
        self.sim = sim
        self.job = dict(job) if job else None
        self.fuel, self.capacity = fuel_l, capacity_l
        self.game_time, self.next_rest, self.fatigue = game_time, next_rest_min, fatigue
        self.scale, self.fuel_l_per_m = scale, fuel_l_per_m
        self.pumps, self.parkings = list(pumps), list(parkings)
        self.refuel_mode, self.refuel_lps = refuel_mode, refuel_lps
        self.sleep_min, self.rest_after = sleep_min, rest_after_sleep_min
        self.pump_m, self.parking_m = pump_m, parking_m
        self.quick_park = quick_park                # the game's quick park answers `quickpark` at the depots
        self.trailer = bool(job and job.get("loaded")) if trailer is None else trailer   # coupled
        self.trailer_pose = trailer_pose
        self.hook_back, self.kingpin_ahead = hook_back_m, kingpin_ahead_m
        self.couple_m, self.couple_deg = couple_m, couple_deg
        self.held: set[str] = set()
        self._pumping = False
        self.flowing = False
        self.sleep_screen = False
        self.log: list[tuple[float, str]] = []           # (sim time, what happened)
        self._events: list[GameplayEvent] = []
        self._seq = 0
        self._xy = (sim.s.x, sim.s.y)
        self.cfg_seq = 1

    # ------------------------------------------------------------- what the bridge would show
    def frame(self, speed_limit: float = 0.0) -> SimpleNamespace:
        f = self.sim.frame(speed_limit)
        f.fuel = self.fuel
        f.trailers = [SimpleNamespace(connected=self.trailer)]
        f.fuel_average_consumption = self.fuel_l_per_m * 1000 / self.scale
        f.fuel_range = self.fuel / max(f.fuel_average_consumption, 1e-6)
        f.game_time = int(self.game_time)
        f.next_rest_stop = int(self.next_rest)
        f.local_scale = self.scale
        f.navigation_distance = 0.0
        return f

    def config(self) -> SimpleNamespace:
        truck = SimpleNamespace(present=1, fuel_capacity=self.capacity)
        j = self.job
        if j is None:
            job = SimpleNamespace(present=0, cargo_loaded=0)
        else:
            b = lambda s: str(s).encode()                       # noqa: E731
            job = SimpleNamespace(
                present=1, cargo_loaded=int(j.get("loaded", True)), special_job=0, cargo=b(j.get("cargo", "cargo")),
                cargo_id=b(j.get("cargo", "cargo")), cargo_mass=float(j.get("mass", 20000)),
                delivery_time=int(j["deadline"]), planned_distance_km=int(j.get("planned_km", 0)),
                income=int(j["income"]), job_market=b(j.get("market", "freight_market")),
                source_city_id=b(j["source_city"]), source_company_id=b(j["source_company"]),
                source_city=b(j["source_city"]), source_company=b(j["source_company"]),
                destination_city_id=b(j["destination_city"]), destination_company_id=b(j["destination_company"]),
                destination_city=b(j["destination_city"]), destination_company=b(j["destination_company"]))
        return SimpleNamespace(truck=truck, job=job, trailers=[], controls=SimpleNamespace(present=1))

    def events(self) -> list[GameplayEvent]:
        out, self._events = self._events, []
        return out

    def _emit(self, eid: str, **attrs) -> None:
        self._seq += 1
        self._events.append(GameplayEvent(self._seq, eid, int(self.game_time), {k.replace("_", "."): str(v)
                                                                                 for k, v in attrs.items()}))

    # ------------------------------------------------------------- inputs
    def buttons(self, press=(), hold=None) -> None:
        for name, on in (hold or {}).items():
            (self.held.add if on else self.held.discard)(name)
        standing = abs(self.sim.s.speed) < 0.3
        for name in press:
            if name == "activate" and standing and self._near(self.pumps, self.pump_m):
                if self.refuel_mode == "press":
                    self.flowing = True
            elif name == "activate" and standing and self._near(self.parkings, self.parking_m):
                self.sleep_screen = True
                self.log.append((self.sim.t, "sleep screen"))
            elif name == "quickpark" and standing and self.job is not None and self.quick_park:
                if self.trailer and self.job.get("loaded"):
                    self.log.append((self.sim.t, "quick park: delivered"))
                    self.deliver()
                    self.trailer = False
                else:
                    self.job["loaded"] = self.trailer = True
                    self.log.append((self.sim.t, "quick park: trailer attached"))
            elif name == "gearreverse" and standing:
                self.sim.s.gear = -1
            elif name == "geardrive" and standing:
                self.sim.s.gear = 1
            elif name == "attach" and not self.trailer and self.trailer_pose is not None and self._at_kingpin():
                self.trailer = True
                if self.job is not None:
                    self.job["loaded"] = True
                self.log.append((self.sim.t, "trailer coupled"))
            elif name == "selectfcs" and self.sleep_screen:
                self.sleep_screen = False
                self.game_time += self.sleep_min
                self.next_rest = self.rest_after
                self.log.append((self.sim.t, f"slept {self.sleep_min / 60:.0f} h"))

    def hook_error(self) -> tuple[float, float]:
        """The fifth wheel's distance from the kingpin (m) and the heading difference (deg)."""
        t, s = self.trailer_pose, self.sim.s
        kx = t.x + self.kingpin_ahead * math.cos(t.rotation)
        ky = t.y + self.kingpin_ahead * math.sin(t.rotation)
        hx, hy = s.x - self.hook_back * math.cos(s.rotation), s.y - self.hook_back * math.sin(s.rotation)
        dh = math.degrees(math.atan2(math.sin(s.rotation - t.rotation), math.cos(s.rotation - t.rotation)))
        return math.hypot(hx - kx, hy - ky), dh

    def _at_kingpin(self) -> bool:
        d, dh = self.hook_error()
        return d <= self.couple_m and abs(dh) <= self.couple_deg

    def _near(self, points, radius) -> bool:
        x, y = self.sim.s.x, self.sim.s.y
        return any(math.hypot(p[0] - x, p[1] - y) <= radius for p in points)

    # ------------------------------------------------------------- time
    def step(self, steer: float, throttle: float, brake: float) -> None:
        dt = self.sim.p.dt
        self.sim.step(steer, throttle, brake)
        x, y = self.sim.s.x, self.sim.s.y
        d = math.hypot(x - self._xy[0], y - self._xy[1])
        self._xy = (x, y)
        self.fuel = max(0.0, self.fuel - d * self.fuel_l_per_m)
        minutes = dt * self.scale / 60.0
        self.game_time += minutes
        if self.fatigue:
            self.next_rest -= minutes
        pumping = (self.flowing or (self.refuel_mode == "hold" and "activate" in self.held
                                    and self._near(self.pumps, self.pump_m)))
        if pumping and abs(self.sim.s.speed) < 0.3:
            if not self._pumping:
                self.log.append((self.sim.t, "refuel started"))
            self.fuel = min(self.capacity, self.fuel + self.refuel_lps * dt)
            if self.fuel >= self.capacity:
                self.flowing = False
        elif self.flowing:
            self.flowing = False                               # drove off: the pump stops
        self._pumping = bool(pumping and abs(self.sim.s.speed) < 0.3 and self.fuel < self.capacity)

    def deliver(self) -> None:
        j = self.job
        late = self.game_time - j["deadline"]
        revenue = j["income"] if late <= 0 else int(j["income"] * 0.7)
        self._emit("job.delivered", revenue=revenue, earned_xp=500, cargo_damage=0.0,
                   distance_km=j.get("planned_km", 0))
        self.job = None
