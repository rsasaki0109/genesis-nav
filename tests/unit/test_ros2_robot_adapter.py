"""Unit tests for the real-robot adapter core (no rclpy required)."""

from __future__ import annotations

import pytest

from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.core.embodiment import EmbodimentAdapter
from genesis_nav.ros2_robot import (
    FakeRobotTransport,
    RobotTransport,
    Ros2RobotAdapter,
)


def _adapter(timeout: float = 0.5) -> tuple[Ros2RobotAdapter, FakeRobotTransport]:
    transport = FakeRobotTransport()
    adapter = Ros2RobotAdapter(
        agent_id="r0", transport=transport, command_timeout_sec=timeout
    )
    return adapter, transport


def test_adapter_satisfies_protocols() -> None:
    adapter, transport = _adapter()
    assert isinstance(adapter, EmbodimentAdapter)
    assert isinstance(transport, RobotTransport)


def test_apply_command_publishes_velocity() -> None:
    adapter, transport = _adapter()
    cmd = RuntimeCommand(
        agent_id="r0",
        linear_x=0.4,
        angular_z=-0.2,
        authority=AuthorityMode.AUTONOMY,
    )
    adapter.apply_command(cmd, dt_sec=0.1)
    assert transport.published == [(0.4, 0.0, -0.2)]
    assert adapter.last_linear_x == 0.4
    assert adapter.last_angular_z == -0.2


def test_read_pose_caches_last_sample() -> None:
    adapter, transport = _adapter()
    # No sample yet -> origin fallback, never crashes.
    assert adapter.read_pose() == (0.0, 0.0, 0.0)
    transport.set_pose(1.0, 2.0, 0.5)
    assert adapter.read_pose() == (1.0, 2.0, 0.5)
    # Transport goes quiet -> last known pose is retained.
    transport.pose = None
    assert adapter.read_pose() == (1.0, 2.0, 0.5)


def test_stop_publishes_zero_and_records_reason() -> None:
    adapter, transport = _adapter()
    adapter.apply_command(RuntimeCommand(agent_id="r0", linear_x=0.5), dt_sec=0.1)
    adapter.stop("estop")
    assert transport.published[-1] == (0.0, 0.0, 0.0)
    assert adapter.last_linear_x == 0.0
    assert adapter._stop_reasons == ["estop"]


def test_watchdog_does_not_trip_before_first_command() -> None:
    adapter, transport = _adapter(timeout=0.5)
    transport.advance(10.0)
    assert adapter.seconds_since_command() is None
    assert adapter.watchdog_expired() is False


def test_watchdog_trips_after_timeout() -> None:
    adapter, transport = _adapter(timeout=0.5)
    adapter.apply_command(RuntimeCommand(agent_id="r0", linear_x=0.3), dt_sec=0.1)
    transport.advance(0.4)
    assert adapter.watchdog_expired() is False
    transport.advance(0.2)  # now 0.6s since command, > 0.5 timeout
    assert adapter.watchdog_expired() is True
    assert adapter.seconds_since_command() == pytest.approx(0.6)


def test_fresh_command_resets_watchdog() -> None:
    adapter, transport = _adapter(timeout=0.5)
    adapter.apply_command(RuntimeCommand(agent_id="r0", linear_x=0.3), dt_sec=0.1)
    transport.advance(0.6)
    assert adapter.watchdog_expired() is True
    adapter.apply_command(RuntimeCommand(agent_id="r0", linear_x=0.3), dt_sec=0.1)
    assert adapter.watchdog_expired() is False
