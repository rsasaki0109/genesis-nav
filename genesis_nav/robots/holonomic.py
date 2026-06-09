"""Holonomic / omnidirectional in-memory kinematics."""

from __future__ import annotations

import math
from dataclasses import dataclass

from genesis_nav.core.command_gate import RuntimeCommand


def _wrap_angle(theta: float) -> float:
    return ((theta + math.pi) % (2.0 * math.pi)) - math.pi


@dataclass
class HolonomicKinematics:
    """Deterministic holonomic integrator for ``(vx, vy, wz)`` body-frame commands."""

    agent_id: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    def read_pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None:
        if dt_sec <= 0:
            return
        self.linear_x = float(command.linear_x)
        self.linear_y = float(command.linear_y)
        self.angular_z = float(command.angular_z)
        self.yaw = _wrap_angle(self.yaw + self.angular_z * dt_sec)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.linear_x * cos_yaw - self.linear_y * sin_yaw) * dt_sec
        self.y += (self.linear_x * sin_yaw + self.linear_y * cos_yaw) * dt_sec

    def stop(self, reason: str) -> None:
        del reason
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0


__all__ = ["HolonomicKinematics"]
