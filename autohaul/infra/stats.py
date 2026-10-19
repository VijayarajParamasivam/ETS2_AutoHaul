"""Rolling timing statistics (frame intervals, latencies)."""

from __future__ import annotations

from collections import deque


class Rolling:
    def __init__(self, size: int = 600):
        self.values: deque[float] = deque(maxlen=size)

    def add(self, v: float) -> None:
        self.values.append(v)

    def __len__(self) -> int:
        return len(self.values)

    def percentile(self, p: float) -> float:
        if not self.values:
            return float("nan")
        s = sorted(self.values)
        k = min(len(s) - 1, max(0, round(p / 100.0 * (len(s) - 1))))
        return s[k]

    @property
    def mean(self) -> float:
        return sum(self.values) / len(self.values) if self.values else float("nan")

    @property
    def max(self) -> float:
        return max(self.values) if self.values else float("nan")

    def summary(self) -> dict[str, float]:
        return {"n": len(self.values), "mean": self.mean, "p50": self.percentile(50),
                "p95": self.percentile(95), "p99": self.percentile(99), "max": self.max}
