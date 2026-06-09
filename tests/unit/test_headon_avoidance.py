"""Tests for head-on lateral reroute response."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.navigation.config import CollisionConfig
from genesis_nav.navigation.headon import is_headon_conflict, lateral_detour_waypoints
from genesis_nav.observability.events import JsonlEventWriter


def test_collision_config_parses_headon_fields() -> None:
    cfg = CollisionConfig.from_scenario_raw(
        {
            "runtime": {
                "collision": {
                    "headon_radius_m": 2.5,
                    "headon_lateral_offset_m": 0.9,
                }
            }
        }
    )
    assert cfg.headon_radius_m == 2.5
    assert cfg.headon_lateral_offset_m == 0.9
    assert cfg.headon_enabled is True


def test_is_headon_conflict_detects_opposing_corridor_agents() -> None:
    pose_a = (0.0, 0.0, 0.0)
    goal_a = (5.0, 0.0, 0.0)
    pose_b = (5.0, 0.0, 3.14159)
    goal_b = (0.0, 0.0, 3.14159)
    assert is_headon_conflict(pose_a, goal_a, pose_b, goal_b) is True


def test_is_headon_conflict_rejects_perpendicular_crossing() -> None:
    pose_a = (0.0, 0.0, 0.0)
    goal_a = (2.0, 0.0, 0.0)
    pose_b = (1.0, -1.0, 1.5708)
    goal_b = (1.0, 1.0, 1.5708)
    assert is_headon_conflict(pose_a, goal_a, pose_b, goal_b) is False


def test_lateral_detour_offsets_away_from_other_agent() -> None:
    pose = (0.0, 0.0, 0.0)
    goal = (5.0, 0.0, 0.0)
    other = (3.0, 0.0, 3.14159)
    waypoints = lateral_detour_waypoints(pose, goal, other, 0.8)
    assert len(waypoints) == 2
    detour = waypoints[0]
    assert abs(detour[1]) >= 0.5


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


def test_headon_reroute_resolves_corridor_conflict(tmp_path: Path) -> None:
    metrics, events = _run(
        tmp_path, Path("benchmarks/multi_agent/headon_avoidance.yaml")
    )
    assert metrics["success_rate"] == 1.0
    assert metrics["collision_count"] == 0
    assert metrics["headon_reroute_count"] >= 1
    reroute = next(ev for ev in events if ev["event"] == "HEADON_REROUTE")
    assert reroute["agent_id"] == "b"


def test_yield_only_still_collides_on_headon_corridor(tmp_path: Path) -> None:
    raw = yaml.safe_load(Path("benchmarks/multi_agent/headon_avoidance.yaml").read_text())
    raw["runtime"]["collision"] = {
        "collision_radius_m": 0.3,
        "near_miss_radius_m": 0.8,
        "yield_radius_m": 1.5,
        "headon_radius_m": 0.0,
    }
    path = tmp_path / "headon_yield_only.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    metrics, _ = _run(tmp_path, path)
    assert metrics["headon_reroute_count"] == 0
    assert metrics["collision_count"] >= 1


def test_headon_reroute_fires_once_per_approach(tmp_path: Path) -> None:
    scenario = load_scenario(Path("benchmarks/multi_agent/headon_avoidance.yaml"))
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=45.0)
    assert runtime.metrics.headon_reroute_count >= 1
    assert runtime.metrics.headon_reroute_count <= 2
