"""Genesis adapter unit tests.

These tests use a stub entity that implements the same surface (`set_velocity`,
`get_pose`) so they exercise the adapter without requiring Genesis itself. The
`build_genesis_backend` path is covered by an importorskip-style assertion that
the error message is actionable when Genesis is missing.
"""

from __future__ import annotations

import importlib

import pytest

from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.genesis.adapter import GenesisDiffDriveAdapter


class _StubEntity:
    def __init__(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0

    def set_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._linear_x = linear_x
        self._linear_y = linear_y
        self._angular_z = angular_z

    def step(self, dt_sec: float) -> None:
        import math

        self._yaw += self._angular_z * dt_sec
        self._x += self._linear_x * math.cos(self._yaw) * dt_sec
        self._y += self._linear_x * math.sin(self._yaw) * dt_sec

    def get_pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._yaw)


def test_apply_command_sets_entity_velocity() -> None:
    entity = _StubEntity()
    adapter = GenesisDiffDriveAdapter(agent_id="robot_001", entity=entity)
    adapter.apply_command(
        RuntimeCommand(
            agent_id="robot_001",
            linear_x=0.5,
            angular_z=0.1,
            authority=AuthorityMode.AUTONOMY,
            issued_at_sec=1.0,
        ),
        dt_sec=0.02,
    )
    assert entity._linear_x == 0.5
    assert entity._angular_z == 0.1


def test_stop_zeroes_entity_velocity() -> None:
    entity = _StubEntity()
    adapter = GenesisDiffDriveAdapter(agent_id="robot_001", entity=entity)
    adapter.last_linear_x = 0.5
    adapter.stop("safety")
    assert entity._linear_x == 0.0
    assert entity._angular_z == 0.0
    assert adapter._stop_reasons == ["safety"]


def test_read_pose_uses_entity_get_pose() -> None:
    entity = _StubEntity()
    entity._x, entity._y, entity._yaw = 1.0, 2.0, 0.3
    adapter = GenesisDiffDriveAdapter(agent_id="robot_001", entity=entity)
    assert adapter.read_pose() == (1.0, 2.0, 0.3)


def test_adapter_round_trip_with_stub_scene() -> None:
    entity = _StubEntity()
    adapter = GenesisDiffDriveAdapter(agent_id="robot_001", entity=entity)
    for _ in range(50):
        adapter.apply_command(
            RuntimeCommand(
                agent_id="robot_001",
                linear_x=0.2,
                authority=AuthorityMode.AUTONOMY,
                issued_at_sec=1.0,
            ),
            dt_sec=0.02,
        )
        entity.step(0.02)
    pose = adapter.read_pose()
    assert pose[0] > 0.1  # moved forward


def test_build_genesis_backend_reports_install_hint() -> None:
    if importlib.util.find_spec("genesis") is not None:
        pytest.skip("Genesis is installed; the missing-genesis branch is unreachable")
    from genesis_nav.benchmarks.scenario import load_scenario
    from genesis_nav.genesis.backend import (
        GenesisNotAvailableError,
        build_genesis_backend,
    )

    scenario = load_scenario("examples/scenarios/smoke.yaml")
    with pytest.raises(GenesisNotAvailableError) as exc:
        build_genesis_backend(scenario)
    assert "pip install genesis-world" in str(exc.value)
