"""Tests for the v0.2 dynamic-obstacle + replan slice.

Covers:
- :class:`OccupancyGrid.with_blocked` immutability + bounds handling
- :class:`ScriptedObstacleSource` deterministic, once-only emission
- `build_obstacle_source` scenario parsing
- the `executing -> planning` replan edge in the behavior machine
- end-to-end: an obstacle dropped on the path triggers a replan and the
  agent still reaches the goal
"""

from __future__ import annotations

import json
from pathlib import Path

import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.navigation.behavior import BehaviorState, can_transition
from genesis_nav.navigation.grid_planner import OccupancyGrid
from genesis_nav.navigation.obstacles import (
    ObstacleDelta,
    ObstacleSource,
    ScriptedObstacleSource,
    build_obstacle_source,
)
from genesis_nav.observability.events import JsonlEventWriter


def _grid(cells: list[list[int]]) -> OccupancyGrid:
    return OccupancyGrid(
        width=len(cells[0]),
        height=len(cells),
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        data=tuple(tuple(bool(c) for c in row) for row in cells),
    )


# ----------------------------------------------------------------- grid deltas


def test_with_blocked_is_immutable_and_blocks_cells() -> None:
    grid = _grid([[0, 0, 0], [0, 0, 0]])
    updated = grid.with_blocked([(1, 0)])
    assert updated is not grid
    assert updated.is_blocked(1, 0) is True
    # Original grid is untouched so replays can re-apply deltas in order.
    assert grid.is_blocked(1, 0) is False


def test_with_blocked_ignores_out_of_bounds() -> None:
    grid = _grid([[0, 0], [0, 0]])
    # Only OOB cells -> no change, same instance returned.
    assert grid.with_blocked([(9, 9)]) is grid


# --------------------------------------------------------------- scripted source


def test_scripted_source_emits_each_delta_once_in_order() -> None:
    src = ScriptedObstacleSource(
        deltas=(
            # intentionally out of order to prove sorting
            ObstacleDelta(at_sec=2.0, block=((1, 1),)),
            ObstacleDelta(at_sec=0.5, block=((0, 0),)),
        )
    )
    assert isinstance(src, ObstacleSource)
    assert src.due(0.1) == []
    first = src.due(0.5)
    assert len(first) == 1 and first[0].at_sec == 0.5
    # Same time again -> already emitted, nothing new.
    assert src.due(0.5) == []
    second = src.due(5.0)
    assert len(second) == 1 and second[0].at_sec == 2.0
    assert src.due(10.0) == []


def test_build_obstacle_source_parses_scenario() -> None:
    raw = {
        "dynamic_obstacles": {
            "events": [
                {"at_sec": 1.0, "block": [[3, 1], [3, 2]]},
            ]
        }
    }
    source = build_obstacle_source(raw)
    assert source is not None
    due = source.due(1.0)
    assert len(due) == 1
    assert due[0].block == ((3, 1), (3, 2))


def test_build_obstacle_source_none_without_block() -> None:
    assert build_obstacle_source({}) is None
    assert build_obstacle_source({"dynamic_obstacles": {}}) is None


# ------------------------------------------------------------- behavior machine


def test_executing_can_replan() -> None:
    assert can_transition(BehaviorState.EXECUTING, BehaviorState.PLANNING) is True
    assert can_transition(BehaviorState.PLANNING, BehaviorState.EXECUTING) is True


# ---------------------------------------------------------------- end-to-end


def _corridor_scenario(tmp_path: Path) -> Path:
    raw = {
        "scenario_id": "dyn_obstacle",
        "seed": 0,
        "world": "examples.scenarios.flat_world",
        "occupancy_grid": {
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "cells": [
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
            ],
        },
        "dynamic_obstacles": {
            # Dropped after the agent has planned a straight row-1 path but
            # well before it reaches column 3 (0.6 m/s, 0.02 s steps).
            "events": [{"at_sec": 0.1, "block": [[3, 1]]}],
        },
        "agents": [
            {"id": "robot_001", "embodiment": "diff_drive", "spawn": [0.5, 1.5, 0.0]}
        ],
        "tasks": [
            {
                "id": "task_001",
                "type": "navigate_to_pose",
                "agent": "robot_001",
                "goal": [5.5, 1.5, 0.0],
            }
        ],
    }
    path = tmp_path / "dyn.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_obstacle_triggers_replan_and_goal_is_reached(tmp_path: Path) -> None:
    scenario = load_scenario(_corridor_scenario(tmp_path))
    log_path = tmp_path / "events.jsonl"
    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        metrics = runtime.run_until_idle(episode_id="ep", max_sim_seconds=60.0)

    records = [json.loads(line) for line in log_path.read_text().splitlines()]
    kinds = [r["event"] for r in records]

    assert "OBSTACLE_CHANGED" in kinds
    assert "REPLAN_TRIGGERED" in kinds
    obstacle = next(r for r in records if r["event"] == "OBSTACLE_CHANGED")
    assert [3, 1] in obstacle["data"]["blocked_cells"]

    summary = metrics.summary() if hasattr(metrics, "summary") else metrics
    assert runtime.metrics.replan_count >= 1
    assert runtime.metrics.obstacle_event_count == 1
    assert summary["success_rate"] == 1.0

    # Behavior trajectory shows a second PLANNING after EXECUTING (the replan).
    behavior = [
        r["data"]["to"]
        for r in records
        if r["event"] == "BEHAVIOR_STATE_CHANGED"
    ]
    first_exec = behavior.index("executing")
    assert "planning" in behavior[first_exec + 1 :]
