"""Tests for costmap-aware path reservations on the grid planner."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from genesis_nav.fleet.costmap_reservation import CostmapReservationStore, collect_path_cells
from genesis_nav.navigation.config import NavigationConfig
from genesis_nav.navigation.grid_planner import GridAStarPlanner, OccupancyGrid, PlannerError


def _grid() -> OccupancyGrid:
    return OccupancyGrid.from_mapping(
        {
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "cells": [
                [1, 1, 1, 1, 1],
                [1, 0, 0, 0, 1],
                [1, 1, 1, 1, 1],
            ],
        }
    )


def test_collect_path_cells_samples_the_polyline() -> None:
    grid = _grid()
    cells = collect_path_cells(
        grid,
        [(0.5, 1.5, 0.0), (2.5, 1.5, 0.0), (3.5, 1.5, 0.0)],
    )
    assert (1, 1) in cells
    assert (2, 1) in cells
    assert (3, 1) in cells


def test_costmap_store_blocks_other_agents() -> None:
    store = CostmapReservationStore()
    store.set_cells("a", {(2, 1), (3, 1)})
    assert store.blocked_for("b") == frozenset({(2, 1), (3, 1)})
    assert store.blocked_for("a") == frozenset()


def test_grid_planner_respects_extra_blocked_cells() -> None:
    planner = GridAStarPlanner(_grid())
    start = (1.5, 1.5, 0.0)
    goal = (3.5, 1.5, 0.0)
    assert planner.plan(start, goal)
    with pytest.raises(PlannerError):
        planner.plan(start, goal, extra_blocked=frozenset({(2, 1), (3, 1)}))


def test_navigation_config_parses_costmap_reservation() -> None:
    cfg = NavigationConfig.from_scenario_raw(
        {"runtime": {"navigation": {"costmap_reservation": True}}}
    )
    assert cfg.costmap_reservation is True


def _run(tmp_path: Path, scenario_path: Path):  # noqa: ANN001
    out = tmp_path / "runs"
    from genesis_nav.cli.main import main

    assert main(["run", str(scenario_path), "--fast", "--output-dir", str(out)]) == 0
    run_dir = next(out.iterdir())
    metrics = json.loads((run_dir / "metrics.json").read_text())
    events = [
        json.loads(ln)
        for ln in (run_dir / "events.jsonl").read_text().splitlines()
        if ln
    ]
    return metrics, events


def test_costmap_reservation_serializes_corridor_agents(tmp_path: Path) -> None:
    metrics, events = _run(
        tmp_path, Path("benchmarks/multi_agent/costmap_corridor.yaml")
    )
    assert metrics["success_rate"] == 1.0
    assert metrics["costmap_wait_count"] >= 1
    assert metrics["collision_count"] == 0
    assert any(ev["event"] == "COSTMAP_RESERVATION_WAIT" for ev in events)
    assert any(ev["event"] == "COSTMAP_RESERVED" for ev in events)


def test_without_costmap_reservation_corridor_can_overlap(tmp_path: Path) -> None:
    raw = yaml.safe_load(
        Path("benchmarks/multi_agent/costmap_corridor.yaml").read_text()
    )
    raw["runtime"]["navigation"]["costmap_reservation"] = False
    raw["runtime"]["collision"] = {
        "collision_radius_m": 0.35,
        "near_miss_radius_m": 0.0,
        "yield_radius_m": 0.0,
        "headon_radius_m": 0.0,
    }
    path = tmp_path / "no_costmap.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    metrics, _ = _run(tmp_path, path)
    assert metrics["costmap_wait_count"] == 0
    # Both agents share the lane concurrently; they may still succeed but only
    # one should win the race without an explicit wait.
    assert metrics["success_rate"] >= 0.5
