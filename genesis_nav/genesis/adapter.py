"""Genesis-backed embodiment adapter.

The adapter is a thin wrapper around a Genesis entity that satisfies the
`EmbodimentAdapter` Protocol. `apply_command` records the latest velocity on
the entity; the actual physics integration is driven by the `GenesisBackend`
which calls `scene.step()` once per runtime tick.

Two ways the underlying entity is expected to behave are tolerated so that
multiple Genesis versions and stub entities can be wired identically:

1. `entity.set_velocity(linear_x, linear_y, angular_z)` if available.
2. Otherwise, attribute writes to `entity.linear_velocity` and
   `entity.angular_velocity`.

Pose reading mirrors the same idea: prefer `entity.get_pose() -> (x, y, yaw)`
and fall back to attribute access.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from genesis_nav.core.command_gate import RuntimeCommand


@dataclass
class GenesisDiffDriveAdapter:
    """EmbodimentAdapter implementation backed by a Genesis entity."""

    agent_id: str
    entity: Any
    last_linear_x: float = 0.0
    last_linear_y: float = 0.0
    last_angular_z: float = 0.0
    _stop_reasons: list[str] = field(default_factory=list)

    def read_pose(self) -> tuple[float, float, float]:
        getter = getattr(self.entity, "get_pose", None)
        if callable(getter):
            pose = getter()
            return (float(pose[0]), float(pose[1]), float(pose[2]))
        x = float(getattr(self.entity, "x", 0.0))
        y = float(getattr(self.entity, "y", 0.0))
        yaw = float(getattr(self.entity, "yaw", 0.0))
        return (x, y, yaw)

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None:
        del dt_sec  # the Genesis scene owns integration
        self.last_linear_x = float(command.linear_x)
        self.last_linear_y = float(command.linear_y)
        self.last_angular_z = float(command.angular_z)
        setter = getattr(self.entity, "set_velocity", None)
        if callable(setter):
            setter(self.last_linear_x, self.last_linear_y, self.last_angular_z)
            return
        if hasattr(self.entity, "linear_velocity"):
            self.entity.linear_velocity = (self.last_linear_x, self.last_linear_y)
        if hasattr(self.entity, "angular_velocity"):
            self.entity.angular_velocity = self.last_angular_z

    def stop(self, reason: str) -> None:
        self._stop_reasons.append(reason)
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0
        setter = getattr(self.entity, "set_velocity", None)
        if callable(setter):
            setter(0.0, 0.0, 0.0)
            return
        if hasattr(self.entity, "linear_velocity"):
            self.entity.linear_velocity = (0.0, 0.0)
        if hasattr(self.entity, "angular_velocity"):
            self.entity.angular_velocity = 0.0
