"""What the overlay (tools/overlay.py) shows: what AutoHaul reads, what it's doing, and why, in plain words.

Sent as one JSON datagram several times a second to a local UDP port. If no overlay is running the datagrams go
nowhere: nothing here can slow down or stop the driving.
"""

from __future__ import annotations

import json
import re
import socket
import time
from collections import deque

DEFAULT_PORT = 47821
MAX_DATAGRAM = 60000


class OverlayFeed:
    def __init__(self, port: int = DEFAULT_PORT, period_s: float = 0.2, events: int = 4):
        self.addr = ("127.0.0.1", port)
        self.period = period_s
        self.events: deque[str] = deque(maxlen=events)
        self._last = 0.0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def event(self, text: str) -> None:
        self.events.append(f"{time.strftime('%H:%M:%S')}  {text}")

    def due(self, now: float) -> bool:
        if now - self._last < self.period:
            return False
        self._last = now
        return True

    def send(self, state: dict) -> None:
        state = dict(state, events=list(self.events))
        try:
            self._sock.sendto(json.dumps(state).encode("utf-8")[:MAX_DATAGRAM], self.addr)
        except OSError:
            pass


def why_speed(reason: str, target_kmh: float) -> str:
    """The speed decision in plain words."""
    r = reason or ""
    if r.startswith("max speed"):
        return f"Nothing in the way: cruising at the {target_kmh:.0f} km/h you allowed"
    if r.startswith("curve"):
        m = re.search(r"in (\d+) m", r)
        return f"Slowing to {target_kmh:.0f} km/h for a curve {m.group(1)} m ahead" if m else f"Curve: {target_kmh:.0f} km/h"
    if r.startswith("following"):
        m = re.search(r"at (\d+) m", r)
        return f"Keeping distance from the vehicle {m.group(1)} m ahead" if m else "Keeping distance from the vehicle ahead"
    if "limit" in r:
        return f"Speed limit: {target_kmh:.0f} km/h"
    return f"{r} ({target_kmh:.0f} km/h)" if r else ""


def _gear(g: int) -> str:
    return "R" if g < 0 else "N" if g == 0 else f"D{g}"


def explain(f, st, mission, message: str, engaged: bool, capacity_l: float = 0.0, extras: dict | None = None) -> dict:
    """The overlay's content from a telemetry frame, the autopilot's Status (or None) and the MissionStatus (or None)."""
    extras = extras or {}
    reads: list[list[str]] = []
    doing: list[list[str]] = []
    why: list[str] = []
    speed = f.speed * 3.6
    limit = float(getattr(f, "navigation_speed_limit", 0.0)) * 3.6
    reads.append(["Speed", f"{speed:.0f} km/h" + (f"  (limit {limit:.0f})" if limit > 1 else "")])
    reads.append(["Truck", f"gear {_gear(int(f.displayed_gear))}, engine {'on' if f.engine_enabled else 'OFF'}"
                  + (", parking brake ON" if f.parking_brake else "")
                  + (", wipers on" if getattr(f, "wipers", 0) else "")])
    fuel = float(f.fuel)
    share = f"  ({fuel / capacity_l * 100:.0f} %)" if capacity_l > 0 else ""
    reads.append(["Fuel", f"{fuel:.0f} L{share}, range ~{float(getattr(f, 'fuel_range', 0.0)):.0f} km"])
    nav = float(getattr(f, "navigation_distance", 0.0))
    if st is not None:
        kind = (st.lane_desc or "not matched").split()[0]
        reads.append(["Lane", f"{kind}, match {st.confidence * 100:.0f} %, "
                      f"{abs(st.lateral):.2f} m {'right' if st.lateral > 0 else 'left'} of centre"])
        if st.lead is not None:
            reads.append(["Ahead", f"vehicle at {st.lead.gap:.0f} m doing {st.lead.speed * 3.6:.0f} km/h"])
        else:
            reads.append(["Ahead", "clear"])
        if st.junction:
            reads.append(["Junction", st.junction])
        route = f"route {st.route_km * 19:.0f} km" if st.route_km is not None else ""
        gps = f"GPS {nav / 1000:.0f} km" if nav > 0 else ""
        if route or gps:
            reads.append(["Distance", ", ".join(x for x in (gps, route) if x) + ("" if st.on_route else "  (off route)")])
    elif nav > 0:
        reads.append(["Distance", f"GPS {nav / 1000:.0f} km"])
    if "rain" in extras:
        reads.append(["Rain", extras["rain"]])

    if st is not None and engaged:
        doing.append(["Target", f"{st.target_kmh:.0f} km/h, throttle {st.throttle * 100:.0f} %, brake {st.brake * 100:.0f} %"])
        side = "left" if st.steer_target > 0.02 else "right" if st.steer_target < -0.02 else "straight"
        doing.append(["Steering", f"{side} ({abs(st.steer_target) * 100:.0f} % of lock)"])
        if st.lane_change:
            doing.append(["Lane change", st.lane_change])
        if st.indicator:
            doing.append(["Indicator", st.indicator])
        text = why_speed(st.target_reason, st.target_kmh)
        if text:
            why.append(text)
        if st.lane_change:
            m = re.search(r"needed within (\d+) m", st.lane_change)
            if m:
                why.append(f"The route needs this lane within {m.group(1)} m")
        if st.route_note and "driving out" not in st.route_note and not st.lane_change:
            why.append(st.route_note[0].upper() + st.route_note[1:])
        if st.junction:
            why.append(f"Junction rule: {st.junction}")
    if mission is not None:
        doing.append(["Mission", mission.state + (f": {mission.action}" if mission.action else "")])
        if mission.job:
            reads.append(["Job", mission.job])
        if mission.next_stop:
            doing.append(["Next stop", mission.next_stop])
        if mission.decision:
            why.append("Stops: " + mission.decision)
        if mission.note:
            why.append(mission.note)
    for line in extras.get("why", ()):
        why.append(line)
    return {"mode": message, "engaged": bool(engaged), "reads": reads, "doing": doing, "why": why,
            "learned": list(extras.get("learned", ()))}
