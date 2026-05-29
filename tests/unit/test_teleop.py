"""Tests for the first-class operator teleop entry point.

`Runtime.submit_teleop_command` is the transport-agnostic equivalent of the
ROS bridge `/cmd_vel` path: it runs the operator command through `CommandGate`
and, on accept, holds off the autonomy loop so teleop overrides autonomy. All
of this is exercised here without rclpy.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.observability.events import JsonlEventWriter

SMOKE = Path("examples/scenarios/smoke.yaml")


def test_requires_requester_id(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        with pytest.raises(ValueError):
            runtime.submit_teleop_command("robot_001", requester_id="", linear_x=0.1)


def test_unknown_agent_is_rejected_without_events(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        decision = runtime.submit_teleop_command(
            "ghost", requester_id="op1", linear_x=0.1
        )
    assert decision.accepted is False
    assert runtime.metrics.command_rejection_count == 0
    assert [ln for ln in log.read_text().splitlines() if ln] == []


def test_accepted_teleop_moves_agent_and_logs_metadata(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.step(episode_id="ep")  # advance sim_time so the stamp is fresh
        before = runtime.registry.get_state("robot_001").pose
        decision = runtime.submit_teleop_command(
            "robot_001", requester_id="op1", linear_x=0.2, episode_id="ep"
        )
    assert decision.accepted is True
    state = runtime.registry.get_state("robot_001")
    assert state.pose != before
    assert runtime.metrics.command_accept_count == 1

    accepted = [
        json.loads(ln)
        for ln in log.read_text().splitlines()
        if ln and json.loads(ln)["event"] == "COMMAND_ACCEPTED"
    ]
    assert len(accepted) == 1
    data = accepted[0]["data"]
    assert data["authority"] == "teleop"
    assert data["requester_id"] == "op1"
    assert data["source"] == "teleop"


def test_non_finite_teleop_is_rejected(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.step(episode_id="ep")  # fresh stamp; isolate the non-finite reason
        before = runtime.registry.get_state("robot_001").pose
        decision = runtime.submit_teleop_command(
            "robot_001", requester_id="op1", linear_x=math.inf, episode_id="ep"
        )
    assert decision.accepted is False
    assert "non-finite" in decision.reason
    assert runtime.metrics.command_rejection_count == 1
    assert runtime.registry.get_state("robot_001").pose == before
    kinds = [json.loads(ln)["event"] for ln in log.read_text().splitlines() if ln]
    assert "COMMAND_REJECTED" in kinds


def test_teleop_overrides_autonomy_for_hold_window(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        # Let autonomy plan and start moving.
        for _ in range(5):
            runtime.step(episode_id="ep")
        c0 = runtime.metrics.command_accept_count
        assert c0 > 0

        # Operator takes over for a short hold.
        decision = runtime.submit_teleop_command(
            "robot_001", requester_id="op1", linear_x=0.1, hold_sec=0.1, episode_id="ep"
        )
        assert decision.accepted is True
        c1 = runtime.metrics.command_accept_count
        assert c1 == c0 + 1  # teleop applied once

        # Within the hold the autonomy loop yields: no new autonomy command.
        runtime.step(episode_id="ep")
        assert runtime.metrics.command_accept_count == c1

        # After the hold expires, autonomy resumes issuing commands.
        for _ in range(10):
            runtime.step(episode_id="ep")
        assert runtime.metrics.command_accept_count > c1
