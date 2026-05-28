"""Tests for the Nav2 planner backend (no rclpy/nav2 required).

Covers:
- `Nav2Planner` delegating to a `Nav2PathService` (the rclpy bridge is mocked
  out via `FakeNav2PathService`)
- the `runtime.navigation.planner` selector parsing
- `Runtime._select_planner` choosing grid / straight / nav2
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.nav2 import FakeNav2PathService, Nav2PathService, Nav2Planner
from genesis_nav.navigation.config import NavigationConfig
from genesis_nav.navigation.global_planner import StraightLinePlanner
from genesis_nav.navigation.grid_planner import GridAStarPlanner, PlannerError
from genesis_nav.observability.events import JsonlEventWriter


# --------------------------------------------------------------- Nav2Planner


def test_nav2_planner_delegates_and_pins_endpoints() -> None:
    service = FakeNav2PathService(path=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
    planner = Nav2Planner(service)
    assert isinstance(service, Nav2PathService)

    start = (0.0, 0.0, 0.0)
    goal = (2.0, 0.0, 1.57)
    waypoints = planner.plan(start, goal)

    assert service.requests == [(start, goal)]
    # Exact goal pose (incl. yaw) is pinned onto the last waypoint.
    assert waypoints[-1] == goal
    assert waypoints[0][:2] == start[:2]


def test_nav2_planner_inserts_start_when_missing() -> None:
    service = FakeNav2PathService(path=[(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
    planner = Nav2Planner(service)
    waypoints = planner.plan((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    assert waypoints[0][:2] == (0.0, 0.0)


def test_nav2_planner_raises_on_empty_path() -> None:
    planner = Nav2Planner(FakeNav2PathService(path=[]))
    with pytest.raises(PlannerError):
        planner.plan((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))


def test_nav2_replan_delegates_again() -> None:
    service = FakeNav2PathService(path=[(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
    planner = Nav2Planner(service)
    planner.plan((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    planner.replan((0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    assert len(service.requests) == 2


# ----------------------------------------------------------------- selector


def test_navigation_config_parses_planner() -> None:
    cfg = NavigationConfig.from_scenario_raw(
        {"runtime": {"navigation": {"planner": "nav2"}}}
    )
    assert cfg.planner == "nav2"
    assert NavigationConfig.from_scenario_raw({}).planner == "auto"


def test_navigation_config_rejects_unknown_planner() -> None:
    with pytest.raises(ValueError):
        NavigationConfig.from_scenario_raw(
            {"runtime": {"navigation": {"planner": "rrt"}}}
        )


def _scenario(tmp_path: Path, *, planner: str | None, grid: bool) -> Path:
    raw: dict = {
        "scenario_id": "sel",
        "seed": 0,
        "world": "examples.scenarios.flat_world",
        "agents": [
            {"id": "robot_001", "embodiment": "diff_drive", "spawn": [0.0, 0.0, 0.0]}
        ],
        "tasks": [
            {
                "id": "task_001",
                "type": "navigate_to_pose",
                "agent": "robot_001",
                "goal": [2.0, 0.0, 0.0],
            }
        ],
    }
    if grid:
        raw["occupancy_grid"] = {
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "cells": [[0, 0, 0], [0, 0, 0]],
        }
    if planner is not None:
        raw["runtime"] = {"navigation": {"planner": planner}}
    path = tmp_path / "sel.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_auto_picks_grid_when_present(tmp_path: Path) -> None:
    scenario = load_scenario(_scenario(tmp_path, planner=None, grid=True))
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
    assert isinstance(runtime.planner, GridAStarPlanner)


def test_straight_selector_overrides_grid(tmp_path: Path) -> None:
    scenario = load_scenario(_scenario(tmp_path, planner="straight", grid=True))
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
    assert isinstance(runtime.planner, StraightLinePlanner)


def test_grid_selector_without_grid_raises(tmp_path: Path) -> None:
    scenario = load_scenario(_scenario(tmp_path, planner="grid", grid=False))
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        with pytest.raises(ValueError):
            Runtime.from_scenario(scenario, events)


def test_nav2_selector_uses_bridge(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    sentinel = Nav2Planner(FakeNav2PathService(path=[(0.0, 0.0, 0.0)]))
    monkeypatch.setattr(
        "genesis_nav.nav2.bridge.build_nav2_planner", lambda scenario: sentinel
    )
    scenario = load_scenario(_scenario(tmp_path, planner="nav2", grid=False))
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
    assert runtime.planner is sentinel


def test_cli_reports_nav2_unavailable_cleanly(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from genesis_nav.cli.main import main
    from genesis_nav.nav2.bridge import Nav2NotAvailableError

    def _boom(scenario):  # noqa: ANN001
        raise Nav2NotAvailableError("rclpy / nav2_msgs are not importable.")

    monkeypatch.setattr("genesis_nav.nav2.bridge.build_nav2_planner", _boom)
    scenario_path = _scenario(tmp_path, planner="nav2", grid=False)
    code = main(["run", str(scenario_path), "--fast", "--output-dir", str(tmp_path / "runs")])
    assert code == 4  # clean exit, not an uncaught traceback
