"""Runtime auto-poll of the real-robot command-staleness watchdog.

The adapter-level watchdog (`Ros2RobotAdapter.watchdog_expired`) is unit-tested
in `test_ros2_robot_adapter.py`. Here we verify the *runtime* wiring added in
v0.2: `Runtime._poll_safety_signals` polls each adapter's watchdog on the
transport's monotonic clock and, on the rising edge, emits a `SAFETY_STOP`,
stops the actuator, and latches the emergency stop. Adapters without a
watchdog (the sim fallback) never trip, so pure-sim runs are unaffected.

All rclpy-free: the real robot is replaced by `FakeRobotTransport`, whose
monotonic clock the test advances by hand to simulate a comms stall.
"""

from __future__ import annotations

import json
from pathlib import Path

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.observability.events import JsonlEventWriter
from genesis_nav.ros2_robot import FakeRobotTransport, Ros2RobotAdapter

SMOKE = Path("examples/scenarios/smoke.yaml")


def _runtime_with_robot(scenario, events, timeout: float = 0.5):
    """Build a runtime whose agents are real-robot adapters over fakes."""

    transports: dict[str, FakeRobotTransport] = {}

    def factory(spec):
        transport = FakeRobotTransport()
        transports[spec.agent_id] = transport
        return Ros2RobotAdapter(
            agent_id=spec.agent_id,
            transport=transport,
            command_timeout_sec=timeout,
        )

    runtime = Runtime.from_scenario(scenario, events, adapter_factory=factory)
    return runtime, transports


def _events(log: Path) -> list[dict]:
    return [json.loads(ln) for ln in log.read_text().splitlines() if ln]


def test_watchdog_does_not_trip_in_pure_sim(tmp_path: Path) -> None:
    """The fallback adapter exposes no watchdog -> never trips, even over time."""

    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)  # default sim adapters
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        for _ in range(20):
            runtime.step(episode_id="ep")
    assert runtime.metrics.watchdog_stop_count == 0
    assert all(e["event"] != "SAFETY_STOP" for e in _events(log))


def test_stale_command_trips_safety_stop_once(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime, transports = _runtime_with_robot(scenario, events, timeout=0.5)
        agent_id = scenario.agents[0].agent_id
        transport = transports[agent_id]

        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        # Drive autonomy until the robot is executing and a command has been
        # published (last_command stamped at the transport clock, still 0).
        for _ in range(5):
            runtime.step(episode_id="ep")
        assert transport.published  # autonomy reached the actuator
        assert runtime.metrics.watchdog_stop_count == 0

        # Wall-clock advances with no fresh command: comms stalled.
        transport.advance(1.0)
        runtime.step(episode_id="ep")

        assert runtime.metrics.watchdog_stop_count == 1
        assert runtime.registry.get_state(agent_id).emergency_stopped is True
        # The base was zeroed.
        assert transport.published[-1] == (0.0, 0.0, 0.0)

        stops = [
            e
            for e in _events(log)
            if e["event"] == "SAFETY_STOP" and e["data"]["reason"] == "command_watchdog"
        ]
        assert len(stops) == 1
        assert stops[0]["data"]["source"] == "ros2_robot_adapter"
        assert stops[0]["data"]["command_age_sec"] >= 0.5

        # Latched + rising-edge: stepping again neither re-emits nor re-counts.
        transport.advance(1.0)
        runtime.step(episode_id="ep")
        assert runtime.metrics.watchdog_stop_count == 1
        again = [e for e in _events(log) if e["event"] == "SAFETY_STOP"]
        assert len(again) == 1


def test_watchdog_clears_when_commands_resume_before_trip(tmp_path: Path) -> None:
    """A command arriving within the timeout keeps the watchdog quiet."""

    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime, transports = _runtime_with_robot(scenario, events, timeout=0.5)
        agent_id = scenario.agents[0].agent_id
        transport = transports[agent_id]
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        # Each step re-stamps the command at the (unadvanced) transport clock, so
        # the age never exceeds the timeout.
        for _ in range(15):
            transport.advance(0.1)  # < 0.5 timeout between commands
            runtime.step(episode_id="ep")
    assert runtime.metrics.watchdog_stop_count == 0
    assert runtime.registry.get_state(agent_id).emergency_stopped is False
