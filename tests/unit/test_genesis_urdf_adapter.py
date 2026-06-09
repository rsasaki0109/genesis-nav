"""Tests for Genesis URDF spawn helpers (no Genesis required)."""

from __future__ import annotations

from genesis_nav.genesis.adapter import (
    DIFF_DRIVE_URDF,
    GenesisDiffDriveAdapter,
    resolve_wheel_dof_indices,
)


class _StubJoint:
    def __init__(self, dof: int) -> None:
        self.dof_idx_local = dof


class _UrdfStubEntity:
    def __init__(self) -> None:
        self.commands: list[tuple[list[float], list[int]]] = []
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def get_joint(self, name: str) -> _StubJoint:
        return {"left_wheel_joint": _StubJoint(0), "right_wheel_joint": _StubJoint(1)}[
            name
        ]

    def control_dofs_velocity(self, velocities, dofs) -> None:  # noqa: ANN001
        self.commands.append((list(velocities), list(dofs)))

    def get_pos(self):
        return [self._x, self._y, 0.1]

    def get_quat(self):
        import math

        half = 0.5 * self._yaw
        return [math.cos(half), 0.0, 0.0, math.sin(half)]


def test_diff_drive_urdf_asset_exists() -> None:
    assert DIFF_DRIVE_URDF.is_file()


def test_resolve_wheel_dof_indices() -> None:
    entity = _UrdfStubEntity()
    assert resolve_wheel_dof_indices(entity, ("left_wheel_joint", "right_wheel_joint")) == (
        0,
        1,
    )


def test_urdf_adapter_maps_unicycle_to_wheel_speeds() -> None:
    from genesis_nav.core.command_gate import RuntimeCommand
    from genesis_nav.core.authority import AuthorityMode

    entity = _UrdfStubEntity()
    adapter = GenesisDiffDriveAdapter.from_entity("r1", entity)
    cmd = RuntimeCommand(
        agent_id="r1",
        linear_x=1.0,
        linear_y=0.0,
        angular_z=0.5,
        authority=AuthorityMode.AUTONOMY,
        requester_id="test",
        issued_at_sec=0.0,
        ttl_ms=200,
        source="navigation",
    )
    adapter.apply_command(cmd, 0.05)
    assert entity.commands
    left, right = entity.commands[-1][0]
    assert right > left
