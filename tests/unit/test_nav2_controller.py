"""Tests for the Nav2 controller backend (no rclpy/nav2 required).

Covers:
- `Nav2Controller` delegating velocity to a `Nav2ControllerService`, tagging
  the command `source="nav2_controller"` at ``AUTONOMY`` authority, and falling
  back to the in-tree controller when Nav2 has no command.
- the `runtime.navigation.controller` selector parsing + `_select_controller`.
- the safety contract: Nav2's velocity still traverses `CommandGate` on the
  runtime's autonomy path, so a non-finite Nav2 command is rejected and the
  agent is not driven.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.runtime import Runtime
from genesis_nav.nav2 import (
    FakeNav2ControllerService,
    Nav2Controller,
    Nav2ControllerService,
)
from genesis_nav.nav2.controller import NAV2_CONTROLLER_SOURCE
from genesis_nav.navigation.config import NavigationConfig
from genesis_nav.navigation.local_controller import SimpleLocalController
from genesis_nav.observability.events import JsonlEventWriter

SMOKE = Path("examples/scenarios/smoke.yaml")


# ------------------------------------------------------------- Nav2Controller


def test_controller_delegates_velocity_with_source_and_authority() -> None:
    service = FakeNav2ControllerService(velocities=[(0.3, 0.0, 0.1)])
    controller = Nav2Controller(service)
    assert isinstance(service, Nav2ControllerService)

    command = controller.compute(
        "robot_001", (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), issued_at_sec=1.0
    )
    assert command.linear_x == 0.3
    assert command.angular_z == 0.1
    assert command.source == NAV2_CONTROLLER_SOURCE
    assert command.authority == AuthorityMode.AUTONOMY
    assert service.requests == [("robot_001", (0.0, 0.0, 0.0), (2.0, 0.0, 0.0))]


def test_controller_falls_back_when_nav2_has_no_command() -> None:
    service = FakeNav2ControllerService(velocities=[None])
    controller = Nav2Controller(service)
    command = controller.compute(
        "robot_001", (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), issued_at_sec=1.0
    )
    # Fallback in-tree controller produced the command, not Nav2.
    assert command.source == "navigation"
    assert command.linear_x >= 0.0


def test_controller_at_goal_delegates_to_fallback() -> None:
    controller = Nav2Controller(FakeNav2ControllerService())
    assert controller.at_goal((2.0, 0.0, 0.0), (2.0, 0.0, 0.0)) is True
    assert controller.at_goal((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)) is False


# ----------------------------------------------------------------- selector


def test_navigation_config_parses_controller() -> None:
    cfg = NavigationConfig.from_scenario_raw(
        {"runtime": {"navigation": {"controller": "nav2"}}}
    )
    assert cfg.controller == "nav2"
    assert NavigationConfig.from_scenario_raw({}).controller == "local"


def test_navigation_config_rejects_unknown_controller() -> None:
    with pytest.raises(ValueError):
        NavigationConfig.from_scenario_raw(
            {"runtime": {"navigation": {"controller": "mpc"}}}
        )


def test_select_controller_defaults_to_local() -> None:
    scenario = load_scenario(SMOKE)
    with JsonlEventWriter(Path("/dev/null")) as events:
        runtime = Runtime.from_scenario(scenario, events)
    assert isinstance(runtime.controller, SimpleLocalController)


def test_nav2_controller_selector_uses_bridge(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sentinel = Nav2Controller(FakeNav2ControllerService())
    monkeypatch.setattr(
        "genesis_nav.nav2.bridge.build_nav2_controller", lambda scenario: sentinel
    )
    raw = yaml.safe_load(SMOKE.read_text(encoding="utf-8"))
    raw.setdefault("runtime", {}).setdefault("navigation", {})["controller"] = "nav2"
    path = tmp_path / "ctrl.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    scenario = load_scenario(path)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
    assert runtime.controller is sentinel


# ------------------------------------------------- runtime safety contract


class _DrivingService:
    """Nav2ControllerService that drives toward the target like a real one.

    Delegates direction to an in-tree controller so the task completes, but the
    velocity arrives through the Nav2 boundary — proving the Nav2 path drives
    the agent *and* traverses CommandGate (commands are tagged nav2_controller).
    """

    def __init__(self) -> None:
        self._local = SimpleLocalController()

    def compute_velocity(self, agent_id, pose, target):  # noqa: ANN001
        cmd = self._local.compute(agent_id, pose, target, issued_at_sec=0.0)
        return (cmd.linear_x, cmd.linear_y, cmd.angular_z)


class _NanService:
    """A misbehaving Nav2 controller emitting a non-finite velocity."""

    def compute_velocity(self, agent_id, pose, target):  # noqa: ANN001
        return (math.nan, 0.0, 0.0)


def _run_with_controller(tmp_path: Path, controller, max_sim_seconds=20.0):  # noqa: ANN001
    scenario = load_scenario(SMOKE)
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.controller = controller
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")
        metrics = runtime.run_until_idle(
            episode_id="ep", max_sim_seconds=max_sim_seconds
        )
    lines = [json.loads(ln) for ln in log.read_text().splitlines() if ln]
    return runtime, metrics, lines


def test_nav2_velocity_drives_agent_through_command_gate(tmp_path: Path) -> None:
    runtime, metrics, lines = _run_with_controller(tmp_path, Nav2Controller(_DrivingService()))
    # Task completed: the Nav2-sourced velocity actually drove the agent.
    assert metrics.summary()["success_rate"] == 1.0
    accepted = [
        ln for ln in lines
        if ln["event"] == "COMMAND_ACCEPTED"
        and ln["data"].get("source") == NAV2_CONTROLLER_SOURCE
    ]
    assert accepted, "expected at least one gate-accepted nav2_controller command"
    assert accepted[0]["data"]["authority"] == "autonomy"


def test_nonfinite_nav2_velocity_is_rejected_by_gate(tmp_path: Path) -> None:
    runtime, metrics, lines = _run_with_controller(
        tmp_path, Nav2Controller(_NanService()), max_sim_seconds=2.0
    )
    # The gate rejected every Nav2 command, so the agent never reached goal.
    assert metrics.summary()["success_rate"] == 0.0
    rejected = [ln for ln in lines if ln["event"] == "COMMAND_REJECTED"]
    assert rejected, "non-finite Nav2 velocity must be rejected by CommandGate"
    assert runtime.metrics.command_accept_count == 0
