"""Bridge integration tests.

Skipped automatically when rclpy or genesis_nav_msgs is unavailable so the unit
suite remains runnable in a venv without ROS 2.
"""

from __future__ import annotations

from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("genesis_nav_msgs.msg")

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.observability.events import JsonlEventWriter
from genesis_nav.ros.bridge import BridgeConfig, RosBridge


def _make_runtime(tmp_path: Path):
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"
    events = JsonlEventWriter(log_path).__enter__()
    runtime = Runtime.from_scenario(scenario, events)
    return scenario, runtime, events


def test_bridge_publishes_topics_for_each_agent(tmp_path: Path) -> None:
    scenario, runtime, events = _make_runtime(tmp_path)
    try:
        bridge = RosBridge(
            runtime.registry,
            runtime.command_gate,
            runtime.clock,
            events,
            config=BridgeConfig(qos_path="configs/qos/default.yaml"),
            external_command_handler=runtime.apply_external_command,
            episode_id="episode-bridge-test",
        )
        try:
            topic_names = {name for name, _ in bridge.node.get_topic_names_and_types()}
            assert "/clock" in topic_names
            assert "/genesis_nav/events" in topic_names
            for spec in scenario.agents:
                assert f"{spec.namespace}/state" in topic_names
                assert f"{spec.namespace}/odom" in topic_names
                assert f"{spec.namespace}/cmd_vel" in topic_names
        finally:
            bridge.shutdown()
    finally:
        events.__exit__(None, None, None)


def test_bridge_forwards_cmd_vel_through_command_gate(tmp_path: Path) -> None:
    import time

    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    scenario, runtime, events = _make_runtime(tmp_path)
    runtime.clock.sim_time_sec = 1.0
    try:
        bridge = RosBridge(
            runtime.registry,
            runtime.command_gate,
            runtime.clock,
            events,
            config=BridgeConfig(qos_path="configs/qos/default.yaml"),
            external_command_handler=runtime.apply_external_command,
            episode_id="episode-cmd-vel",
        )
        try:
            publisher_node = rclpy.create_node("test_cmd_vel_publisher")
            try:
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                pub = publisher_node.create_publisher(
                    Twist, f"{scenario.agents[0].namespace}/cmd_vel", qos
                )
                # rclpy discovery is async; give the subscription time to match.
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and pub.get_subscription_count() == 0:
                    rclpy.spin_once(bridge.node, timeout_sec=0.05)
                msg = Twist()
                msg.linear.x = 0.3
                pub.publish(msg)
                end = time.monotonic() + 2.0
                while time.monotonic() < end and bridge.external_command_count == 0:
                    rclpy.spin_once(bridge.node, timeout_sec=0.05)
            finally:
                publisher_node.destroy_node()

            assert bridge.external_command_count == 1
            state = runtime.registry.get_state(scenario.agents[0].agent_id)
            assert state.linear_velocity_x > 0.0
            assert state.pose != (0.0, 0.0, 0.0)
        finally:
            bridge.shutdown()
    finally:
        events.__exit__(None, None, None)
