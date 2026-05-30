"""Genesis-backed embodiment adapter.

Wraps a real Genesis `RigidEntity` (a free rigid body) and satisfies the
`EmbodimentAdapter` Protocol. The body has no articulated drivetrain, so
genesis-nav drives it *kinematically*: each `apply_command` integrates the
commanded body-frame velocity over `dt_sec` and writes the new base pose with
the entity's `set_pos` / `set_quat`. The Genesis scene still owns physics
(ground plane, contacts, kernel-compiled stepping); the base pose is commanded,
which is the honest model for a v0.x diff-drive base.

This targets the *real* Genesis 1.0 API (verified on genesis-world 1.0.0):
`get_pos()`/`get_quat()` return tensors, `set_pos([x,y,z])`/`set_quat([w,x,y,z])`
move the body. The earlier `set_velocity`/`get_pose` surface did not exist on a
real entity; a duck-typed fast path is kept only for stub entities in tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from genesis_nav.core.command_gate import RuntimeCommand


@dataclass
class GenesisDiffDriveAdapter:
    """EmbodimentAdapter implementation backed by a Genesis rigid entity."""

    agent_id: str
    entity: Any
    z: float = 0.1
    last_linear_x: float = 0.0
    last_linear_y: float = 0.0
    last_angular_z: float = 0.0
    _x: float = 0.0
    _y: float = 0.0
    _yaw: float = 0.0
    _seeded: bool = False
    _stop_reasons: list[str] = field(default_factory=list)

    def read_pose(self) -> tuple[float, float, float]:
        # Stub-entity fast path (unit tests): explicit get_pose() wins.
        getter = getattr(self.entity, "get_pose", None)
        if callable(getter):
            pose = getter()
            return (float(pose[0]), float(pose[1]), float(pose[2]))
        # Real Genesis entity: get_pos() -> (x, y, z), get_quat() -> (w, x, y, z).
        pos = self._read_pos()
        if pos is not None:
            self._x, self._y = pos[0], pos[1]
        yaw = self._read_yaw()
        if yaw is not None:
            self._yaw = yaw
        return (self._x, self._y, self._yaw)

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None:
        self.last_linear_x = float(command.linear_x)
        self.last_linear_y = float(command.linear_y)
        self.last_angular_z = float(command.angular_z)

        # Stub-entity fast path: defer integration to the stub's own step().
        setter = getattr(self.entity, "set_velocity", None)
        if callable(setter):
            setter(self.last_linear_x, self.last_linear_y, self.last_angular_z)
            return

        # Real Genesis: integrate the unicycle model and command the new pose.
        self._sync_from_entity()
        dt = float(dt_sec)
        if dt > 0.0:
            self._yaw = _wrap(self._yaw + self.last_angular_z * dt)
            self._x += self.last_linear_x * math.cos(self._yaw) * dt
            self._y += self.last_linear_x * math.sin(self._yaw) * dt
        self._write_pose()

    def stop(self, reason: str) -> None:
        self._stop_reasons.append(reason)
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0
        setter = getattr(self.entity, "set_velocity", None)
        if callable(setter):
            setter(0.0, 0.0, 0.0)
            return
        # Real Genesis: hold position (zero velocity = leave pose as-is).
        self._sync_from_entity()
        self._write_pose()

    # -- real-entity helpers ------------------------------------------------

    def _sync_from_entity(self) -> None:
        """Seed the integrator from the entity's true pose on first use."""
        if self._seeded:
            return
        pos = self._read_pos()
        if pos is not None:
            self._x, self._y, self.z = pos[0], pos[1], pos[2]
        yaw = self._read_yaw()
        if yaw is not None:
            self._yaw = yaw
        self._seeded = True

    def _read_pos(self) -> tuple[float, float, float] | None:
        getter = getattr(self.entity, "get_pos", None)
        if not callable(getter):
            return None
        try:
            v = _to_list(getter())
            return (float(v[0]), float(v[1]), float(v[2]))
        except Exception:
            return None

    def _read_yaw(self) -> float | None:
        getter = getattr(self.entity, "get_quat", None)
        if not callable(getter):
            return None
        try:
            q = _to_list(getter())  # (w, x, y, z)
            w, x, y, zc = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            siny = 2.0 * (w * zc + x * y)
            cosy = 1.0 - 2.0 * (y * y + zc * zc)
            return math.atan2(siny, cosy)
        except Exception:
            return None

    def _write_pose(self) -> None:
        set_pos = getattr(self.entity, "set_pos", None)
        if callable(set_pos):
            try:
                set_pos([self._x, self._y, self.z])
            except Exception:
                pass
        set_quat = getattr(self.entity, "set_quat", None)
        if callable(set_quat):
            half = 0.5 * self._yaw
            try:
                set_quat([math.cos(half), 0.0, 0.0, math.sin(half)])
            except Exception:
                pass


def _wrap(theta: float) -> float:
    return ((theta + math.pi) % (2.0 * math.pi)) - math.pi


def _to_list(value: Any) -> list:
    """Coerce a Genesis tensor / numpy array / sequence into a flat list."""
    for attr in ("detach", "cpu", "numpy"):
        fn = getattr(value, attr, None)
        if callable(fn):
            try:
                value = fn()
            except Exception:
                pass
    if hasattr(value, "tolist"):
        value = value.tolist()
    # Some Genesis getters return shape (1, N) for a single env; flatten.
    if isinstance(value, (list, tuple)) and value and isinstance(value[0], (list, tuple)):
        value = value[0]
    return list(value)
