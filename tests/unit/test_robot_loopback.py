"""Tests for the loop-closed real-robot transport (no rclpy required).

`LoopbackRobotTransport` models the part a real robot does itself — integrating
the commanded velocity into odom — so the `Ros2RobotAdapter` real-robot contract
can be driven end to end in core CI without `rclpy` or hardware. This closes the
"real-robot loop closure is future work" item from the v0.2 real-robot slice.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.ros2_robot import (
    LoopbackRobotBackend,
    LoopbackRobotTransport,
    RobotTransport,
    Ros2RobotAdapter,
    build_loopback_robot_backend,
    robot_transport_mode,
)
from genesis_nav.ros2_robot.backend import Ros2RobotBackend  # noqa: F401  (import smoke)
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.core.authority import AuthorityMode
from genesis_nav.observability.events import JsonlEventWriter

SMOKE = Path("examples/scenarios/smoke.yaml")


def _cmd(linear_x: float, angular_z: float = 0.0) -> RuntimeCommand:
    return RuntimeCommand(
        agent_id="robot_001",
        linear_x=linear_x,
        angular_z=angular_z,
        authority=AuthorityMode.AUTONOMY,
        source="navigation",
        issued_at_sec=0.0,
        ttl_ms=200,
    )


# ----------------------------------------------------- LoopbackRobotTransport


def test_loopback_transport_is_a_robot_transport() -> None:
    assert isinstance(LoopbackRobotTransport(), RobotTransport)


def test_loopback_integrates_forward_motion() -> None:
    t = LoopbackRobotTransport()
    t.publish_velocity(1.0, 0.0, 0.0)
    t.integrate(0.5)
    pose = t.latest_pose()
    assert pose is not None
    assert pose[0] == pytest.approx(0.5)
    assert pose[1] == pytest.approx(0.0)
    assert t.published == [(1.0, 0.0, 0.0)]


def test_loopback_integrates_rotation_then_drive() -> None:
    t = LoopbackRobotTransport()
    t.publish_velocity(0.0, 0.0, math.pi)  # turn
    t.integrate(0.5)  # +pi/2 yaw
    t.publish_velocity(1.0, 0.0, 0.0)  # drive along new heading (+y)
    t.integrate(1.0)
    x, y, yaw = t.latest_pose()
    assert yaw == pytest.approx(math.pi / 2, abs=1e-6)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(1.0, abs=1e-6)


def test_loopback_monotonic_advances_with_integration() -> None:
    t = LoopbackRobotTransport()
    assert t.monotonic_sec() == 0.0
    t.integrate(0.25)
    t.integrate(0.25)
    assert t.monotonic_sec() == pytest.approx(0.5)


def test_adapter_over_loopback_closes_the_loop() -> None:
    transport = LoopbackRobotTransport()
    adapter = Ros2RobotAdapter(agent_id="robot_001", transport=transport)
    assert adapter.read_pose() == (0.0, 0.0, 0.0)
    adapter.apply_command(_cmd(1.0), dt_sec=0.1)
    # The adapter only publishes a setpoint; pose updates after the robot
    # (the transport) integrates — the realistic one-tick odom lag.
    transport.integrate(0.1)
    assert adapter.read_pose()[0] == pytest.approx(0.1)


# --------------------------------------------------------------- transport mode


def test_transport_mode_defaults_to_ros2() -> None:
    assert robot_transport_mode(load_scenario(SMOKE)) == "ros2"


def test_transport_mode_reads_loopback(tmp_path: Path) -> None:
    import yaml

    raw = yaml.safe_load(SMOKE.read_text(encoding="utf-8"))
    raw["real_robot"] = {"transport": "loopback"}
    path = tmp_path / "lb.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    assert robot_transport_mode(load_scenario(path)) == "loopback"


def test_transport_mode_rejects_unknown(tmp_path: Path) -> None:
    import yaml

    raw = yaml.safe_load(SMOKE.read_text(encoding="utf-8"))
    raw["real_robot"] = {"transport": "carrier_pigeon"}
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError):
        robot_transport_mode(load_scenario(path))


# --------------------------------------------------------- backend + e2e loop


def test_loopback_backend_seeds_spawn_and_integrates() -> None:
    backend = build_loopback_robot_backend(load_scenario(SMOKE))
    scenario = load_scenario(SMOKE)
    adapter = backend.spawn(scenario.agents[0])
    assert isinstance(adapter, Ros2RobotAdapter)
    adapter.apply_command(_cmd(1.0), dt_sec=0.1)
    backend.step(0.1)
    assert adapter.read_pose()[0] == pytest.approx(0.1)


def test_real_robot_loop_reaches_goal_without_rclpy(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    backend = LoopbackRobotBackend()
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(
            scenario, events, adapter_factory=backend.spawn
        )
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")

        def on_step(_sim_time: float) -> None:
            backend.step(runtime.clock.step_sec)

        metrics = runtime.run_until_idle(
            episode_id="ep", max_sim_seconds=40.0, on_step=on_step
        )

    # The whole real-robot contract closed the loop to the goal: CommandGate ->
    # apply_command -> publish_velocity -> integrate (odom) -> read_pose.
    assert metrics.summary()["success_rate"] == 1.0
    transport = backend.transports["robot_001"]
    assert transport.published, "adapter must have published velocities"
    assert any(v[0] > 0.0 for v in transport.published), "robot should drive forward"

    lines = [json.loads(ln) for ln in log.read_text().splitlines() if ln]
    # No spurious comms-loss watchdog while commands flow every tick.
    safety = [
        ln for ln in lines
        if ln["event"] == "SAFETY_STOP"
        and ln.get("data", {}).get("reason") == "command_watchdog"
    ]
    assert safety == []
