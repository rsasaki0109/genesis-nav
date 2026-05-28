"""Tests for the Workstream H navigation MVP.

Covers:
- :class:`GridAStarPlanner` over an :class:`OccupancyGrid`
- :class:`BehaviorState` transitions emitted by the runtime
- Stuck detection + RECOVERING wait + retry exhaustion
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.core.task import TaskStatus
from genesis_nav.navigation.behavior import BehaviorState, can_transition
from genesis_nav.navigation.config import NavigationConfig
from genesis_nav.navigation.global_planner import StraightLinePlanner
from genesis_nav.navigation.grid_planner import (
    GridAStarPlanner,
    OccupancyGrid,
    PlannerError,
    build_planner,
)
from genesis_nav.observability.events import JsonlEventWriter


# --------------------------------------------------------------------- planner


def _grid(cells: list[list[int]]) -> OccupancyGrid:
    return OccupancyGrid(
        width=len(cells[0]),
        height=len(cells),
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(tuple(bool(c) for c in row) for row in cells),
    )


def test_grid_astar_routes_around_obstacle() -> None:
    grid = _grid(
        [
            [0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0],
        ]
    )
    planner = GridAStarPlanner(grid)
    path = planner.plan((0.5, 0.5, 0.0), (4.5, 1.5, 0.0))

    assert path[0] == (0.5, 0.5, 0.0)
    assert path[-1] == (4.5, 1.5, 0.0)
    # The planner must avoid the blocked row at y in [1.0, 2.0).
    for x, y, _ in path[1:-1]:
        col = int(x)
        row = int(y)
        if 0 <= col < grid.width and 0 <= row < grid.height:
            assert not grid.data[row][col]
    # Straight-line distance is ~4.1; A* around the wall should be < 8.
    total = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])
    )
    assert total < 8.0


def test_grid_astar_refuses_blocked_goal() -> None:
    # cells[0] is the bottom row; cell (col=1, row=0) is blocked.
    grid = _grid([[0, 1], [0, 0]])
    planner = GridAStarPlanner(grid)
    with pytest.raises(PlannerError):
        planner.plan((0.5, 1.5, 0.0), (1.5, 0.5, 0.0))


def test_grid_astar_refuses_unreachable_goal() -> None:
    grid = _grid(
        [
            [0, 1, 0],
            [0, 1, 0],
            [0, 1, 0],
        ]
    )
    planner = GridAStarPlanner(grid)
    with pytest.raises(PlannerError):
        planner.plan((0.5, 0.5, 0.0), (2.5, 1.5, 0.0))


def test_build_planner_falls_back_to_straight_line_when_no_grid() -> None:
    assert build_planner({}) is None
    assert build_planner({"world": "some.module"}) is None


def test_build_planner_reads_top_level_block() -> None:
    raw = {
        "occupancy_grid": {
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "cells": [[0, 0], [0, 0]],
        }
    }
    planner = build_planner(raw)
    assert isinstance(planner, GridAStarPlanner)


# ------------------------------------------------------------------ transitions


def test_behavior_transitions_match_documented_machine() -> None:
    # Happy path
    assert can_transition(BehaviorState.IDLE, BehaviorState.ASSIGNED)
    assert can_transition(BehaviorState.ASSIGNED, BehaviorState.PLANNING)
    assert can_transition(BehaviorState.PLANNING, BehaviorState.EXECUTING)
    assert can_transition(BehaviorState.EXECUTING, BehaviorState.RECOVERING)
    assert can_transition(BehaviorState.RECOVERING, BehaviorState.EXECUTING)
    assert can_transition(BehaviorState.EXECUTING, BehaviorState.SUCCEEDED)
    assert can_transition(BehaviorState.SUCCEEDED, BehaviorState.IDLE)
    assert can_transition(BehaviorState.FAILED, BehaviorState.IDLE)
    # Disallowed jumps
    assert not can_transition(BehaviorState.IDLE, BehaviorState.EXECUTING)
    assert not can_transition(BehaviorState.SUCCEEDED, BehaviorState.EXECUTING)
    assert not can_transition(BehaviorState.IDLE, BehaviorState.FAILED)


def _events(path: Path) -> list[dict]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line
    ]


def test_behavior_state_changed_events_in_order(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)

    transitions = [
        (rec["data"]["from"], rec["data"]["to"])
        for rec in _events(log_path)
        if rec["event"] == "BEHAVIOR_STATE_CHANGED"
        and rec["agent_id"] == "robot_001"
    ]
    expected = [
        ("idle", "assigned"),
        ("assigned", "planning"),
        ("planning", "executing"),
        ("executing", "succeeded"),
        ("succeeded", "idle"),
    ]
    assert transitions == expected


def test_plan_resolved_event_carries_waypoint_count(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)

    plans = [
        rec for rec in _events(log_path) if rec["event"] == "PLAN_RESOLVED"
    ]
    assert plans, "PLAN_RESOLVED event must be emitted"
    assert plans[0]["data"]["waypoint_count"] >= 2
    assert plans[0]["data"]["planner"] == "StraightLinePlanner"


# ------------------------------------------------------------------ stuck/recover


def test_stuck_detector_triggers_recovery_then_fails(tmp_path: Path) -> None:
    """A pinned agent (clamped pose) goes through RECOVERING then FAILED."""

    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        # Tight window so the detector fires quickly; small max retries.
        runtime.navigation_config = NavigationConfig(
            stuck_window_sec=0.5,
            stuck_min_progress_m=0.5,
            recovery_wait_sec=0.1,
            max_recovery_retries=1,
            waypoint_tolerance_m=0.15,
        )
        # Pin the adapter: reject every command by saturating the gate.
        adapter = runtime.adapters["robot_001"]
        original = adapter.apply_command

        def pinned(_command, _dt):  # type: ignore[no-untyped-def]
            # Never advance pose; controller will see zero progress.
            return original.__self__.read_pose()  # type: ignore[attr-defined]

        adapter.apply_command = lambda command, dt: None  # type: ignore[assignment]
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=10.0)

    records = _events(log_path)
    names = [rec["event"] for rec in records]
    assert "AGENT_STUCK" in names
    assert "TASK_FAILED" in names
    failed = next(rec for rec in records if rec["event"] == "TASK_FAILED")
    assert failed["data"]["reason"] == "stuck"
    transitions = [
        (rec["data"]["from"], rec["data"]["to"])
        for rec in records
        if rec["event"] == "BEHAVIOR_STATE_CHANGED"
    ]
    assert ("executing", "recovering") in transitions
    assert ("recovering", "executing") in transitions or (
        "executing",
        "failed",
    ) in transitions
    assert ("executing", "failed") in transitions or (
        "recovering",
        "failed",
    ) in transitions
    assert ("failed", "idle") in transitions


def test_stuck_metric_counters_present(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        metrics = runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)

    summary = metrics.summary()
    for key in ("stuck_event_count", "recovery_count", "plan_failure_count"):
        assert key in summary
        assert summary[key] == 0


# --------------------------------------------------------------------- planner choice


def test_runtime_picks_grid_planner_from_scenario(tmp_path: Path) -> None:
    raw = {
        "scenario_id": "grid_smoke",
        "seed": 0,
        "world": "examples.scenarios.flat_world",
        "occupancy_grid": {
            "resolution": 1.0,
            "origin": [-1.0, -1.0],
            "cells": [
                [0, 0, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 0, 0],
            ],
        },
        "agents": [
            {
                "id": "robot_001",
                "embodiment": "diff_drive",
                "spawn": [0.0, 0.0, 0.0],
            }
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
    scenario_path = tmp_path / "grid.yaml"
    scenario_path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    scenario = load_scenario(scenario_path)

    with JsonlEventWriter(tmp_path / "events.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        assert isinstance(runtime.planner, GridAStarPlanner)


def test_runtime_falls_back_to_straight_line_without_grid(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    with JsonlEventWriter(tmp_path / "events.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        assert isinstance(runtime.planner, StraightLinePlanner)


# ----------------------------------------------------------------- nav config


def test_navigation_config_reads_runtime_block() -> None:
    raw = {
        "runtime": {
            "navigation": {
                "waypoint_tolerance_m": 0.4,
                "stuck_window_sec": 2.0,
                "stuck_min_progress_m": 0.2,
                "recovery_wait_sec": 0.75,
                "max_recovery_retries": 5,
            }
        }
    }
    cfg = NavigationConfig.from_scenario_raw(raw)
    assert cfg.waypoint_tolerance_m == 0.4
    assert cfg.stuck_window_sec == 2.0
    assert cfg.max_recovery_retries == 5


def test_navigation_config_defaults_when_block_missing() -> None:
    cfg = NavigationConfig.from_scenario_raw({})
    assert cfg.waypoint_tolerance_m == 0.15
    assert cfg.max_recovery_retries == 3


# ----------------------------------------------------------- waypoint advancement


def test_runtime_advances_waypoints(tmp_path: Path) -> None:
    """Multi-waypoint plan: runtime must drop waypoints as they're passed."""

    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    with JsonlEventWriter(tmp_path / "events.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        # Force a planner that returns 3 colinear-but-distinct waypoints.

        class StubPlanner:
            def plan(self, start, goal):  # type: ignore[no-untyped-def]
                mid1 = ((start[0] + goal[0]) / 3.0, (start[1] + goal[1]) / 3.0, 0.0)
                mid2 = (2 * (start[0] + goal[0]) / 3.0, 2 * (start[1] + goal[1]) / 3.0, 0.0)
                return [start, mid1, mid2, goal]

        runtime.planner = StubPlanner()
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)

    summary = runtime.metrics.summary()
    assert summary["task_succeeded_count"] == 1
    state = runtime.registry.get_state("robot_001")
    goal = scenario.tasks[0].goal
    assert goal is not None
    assert math.hypot(state.pose[0] - goal[0], state.pose[1] - goal[1]) < 0.15
