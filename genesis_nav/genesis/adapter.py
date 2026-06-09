"""Genesis-backed embodiment adapter.

Wraps a real Genesis `RigidEntity` (a free rigid body or URDF articulation)
and satisfies the `EmbodimentAdapter` Protocol. When the entity exposes diff-drive
wheel joints, commands are mapped to joint velocities via
``control_dofs_velocity``. Otherwise the body is driven *kinematically*: each
``apply_command`` integrates the commanded body-frame velocity over ``dt_sec`` and
writes the new base pose with ``set_pos`` / ``set_quat``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from genesis_nav.core.command_gate import RuntimeCommand

DEFAULT_WHEEL_JOINTS = ("left_wheel_joint", "right_wheel_joint")
DEFAULT_WHEEL_TRACK_M = 0.44
DIFF_DRIVE_URDF = (
    Path(__file__).resolve().parents[2] / "examples" / "robots" / "diff_drive.urdf"
)


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
    _wheel_dof_indices: tuple[int, ...] = ()
    wheel_track_m: float = DEFAULT_WHEEL_TRACK_M

    @classmethod
    def from_entity(cls, agent_id: str, entity: Any) -> "GenesisDiffDriveAdapter":
        dofs = resolve_wheel_dof_indices(entity, DEFAULT_WHEEL_JOINTS)
        return cls(agent_id=agent_id, entity=entity, _wheel_dof_indices=dofs)

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

        if self._wheel_dof_indices:
            self._apply_wheel_velocities(self.last_linear_x, self.last_angular_z)
            self._sync_from_entity()
            return

        # Real Genesis without wheel joints: integrate kinematically.
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
        if self._wheel_dof_indices:
            self._apply_wheel_velocities(0.0, 0.0)
            return
        # Real Genesis without wheel joints: hold pose.
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

    def _apply_wheel_velocities(self, linear_x: float, angular_z: float) -> None:
        half_track = self.wheel_track_m * 0.5
        left = linear_x - angular_z * half_track
        right = linear_x + angular_z * half_track
        control = getattr(self.entity, "control_dofs_velocity", None)
        if not callable(control):
            return
        try:
            control([left, right], list(self._wheel_dof_indices))
        except Exception:
            pass


def resolve_wheel_dof_indices(
    entity: Any, joint_names: tuple[str, ...]
) -> tuple[int, ...]:
    """Return local DOF indices for ``joint_names`` when the entity supports it."""

    indices: list[int] = []
    get_joint = getattr(entity, "get_joint", None)
    if not callable(get_joint):
        return ()
    for name in joint_names:
        try:
            joint = get_joint(name)
        except Exception:
            return ()
        dof = getattr(joint, "dof_idx_local", None)
        if dof is None:
            return ()
        indices.append(int(dof))
    return tuple(indices)


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
