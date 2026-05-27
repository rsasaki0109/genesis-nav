"""Embodiment adapter boundary and deterministic v0.1 fallback."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from genesis_nav.core.command_gate import RuntimeCommand


@runtime_checkable
class EmbodimentAdapter(Protocol):
    """Minimal contract between the runtime and a simulator or robot."""

    agent_id: str

    def read_pose(self) -> tuple[float, float, float]: ...

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None: ...

    def stop(self, reason: str) -> None: ...


@dataclass
class DiffDriveKinematics:
    """Deterministic in-memory differential-drive integrator.

    Used by the v0.1 CLI and unit tests when the real Genesis adapter is not
    available. The state is intentionally simple so that scenarios remain
    reproducible from seed and scenario file alone.
    """

    agent_id: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0

    def read_pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None:
        if dt_sec <= 0:
            return
        self.linear_x = float(command.linear_x)
        self.angular_z = float(command.angular_z)
        self.yaw = _wrap_angle(self.yaw + self.angular_z * dt_sec)
        self.x += self.linear_x * math.cos(self.yaw) * dt_sec
        self.y += self.linear_x * math.sin(self.yaw) * dt_sec

    def stop(self, reason: str) -> None:
        del reason
        self.linear_x = 0.0
        self.angular_z = 0.0


def _wrap_angle(theta: float) -> float:
    return ((theta + math.pi) % (2.0 * math.pi)) - math.pi
