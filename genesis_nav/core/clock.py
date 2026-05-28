"""Runtime clock used by deterministic scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SimulationMode(str, Enum):
    FAST = "fast"
    REALTIME = "realtime"
    LOCKSTEP = "lockstep"
    REPLAY = "replay"


@dataclass
class RuntimeClock:
    step_sec: float = 0.02
    mode: SimulationMode = SimulationMode.FAST
    sim_time_sec: float = 0.0

    def step(self, steps: int = 1) -> float:
        if steps < 1:
            raise ValueError("steps must be >= 1")
        self.sim_time_sec += self.step_sec * steps
        return self.sim_time_sec

    def reset(self) -> None:
        self.sim_time_sec = 0.0
