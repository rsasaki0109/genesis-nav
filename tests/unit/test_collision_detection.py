"""Tests for inter-agent collision / near-miss detection.

Detection is observation only (events + counters); it does not stop or reroute
agents. Disabled unless a radius is configured, so existing scenarios are
unaffected. All exercised without rclpy.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.navigation.config import CollisionConfig
from genesis_nav.observability.events import JsonlEventWriter


# ------------------------------------------------------------- CollisionConfig


def test_collision_config_defaults_disabled() -> None:
    cfg = CollisionConfig.from_scenario_raw({})
    assert cfg.collision_radius_m == 0.0
    assert cfg.near_miss_radius_m == 0.0
    assert cfg.enabled is False


def test_collision_config_parses_radii() -> None:
    cfg = CollisionConfig.from_scenario_raw(
        {"runtime": {"collision": {"collision_radius_m": 0.5, "near_miss_radius_m": 1.0}}}
    )
    assert cfg.collision_radius_m == 0.5
    assert cfg.near_miss_radius_m == 1.0
    assert cfg.enabled is True


def test_collision_config_rejects_negative() -> None:
    with pytest.raises(ValueError):
        CollisionConfig.from_scenario_raw(
            {"runtime": {"collision": {"collision_radius_m": -1.0}}}
        )


# ---------------------------------------------------------------- detection


def _two_agent_runtime(tmp_path: Path, *, collision_r: float, near_r: float):  # noqa: ANN001
    raw = {
        "scenario_id": "collide",
        "seed": 0,
        "world": "examples.scenarios.flat_world",
        "runtime": {
            "collision": {
                "collision_radius_m": collision_r,
                "near_miss_radius_m": near_r,
            }
        },
        "agents": [
            {"id": "a", "embodiment": "diff_drive", "spawn": [0.0, 0.0, 0.0]},
            {"id": "b", "embodiment": "diff_drive", "spawn": [10.0, 0.0, 0.0]},
        ],
        "tasks": [],
    }
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    scenario = load_scenario(path)
    events = JsonlEventWriter(tmp_path / "e.jsonl")
    runtime = Runtime.from_scenario(scenario, events.__enter__())
    return runtime, events


def _set_pose(runtime: Runtime, agent_id: str, x: float, y: float) -> None:
    adapter = runtime.adapters[agent_id]
    adapter.x = x  # DiffDriveKinematics fields
    adapter.y = y


def test_no_detection_when_disabled(tmp_path: Path) -> None:
    runtime, events = _two_agent_runtime(tmp_path, collision_r=0.0, near_r=0.0)
    _set_pose(runtime, "b", 0.0, 0.0)  # exactly overlapping
    runtime._poll_collisions(sim_time=0.1, episode_id="ep")
    events.__exit__(None, None, None)
    assert runtime.metrics.collision_count == 0
    assert runtime.metrics.near_miss_count == 0


def test_collision_counted_once_per_approach(tmp_path: Path) -> None:
    runtime, events = _two_agent_runtime(tmp_path, collision_r=0.5, near_r=1.0)
    # Bring them within the collision radius and poll twice: counted once.
    _set_pose(runtime, "b", 0.3, 0.0)
    runtime._poll_collisions(sim_time=0.1, episode_id="ep")
    runtime._poll_collisions(sim_time=0.2, episode_id="ep")
    assert runtime.metrics.collision_count == 1
    # Separate beyond both radii, then re-approach: counted again (rising edge).
    _set_pose(runtime, "b", 10.0, 0.0)
    runtime._poll_collisions(sim_time=0.3, episode_id="ep")
    _set_pose(runtime, "b", 0.3, 0.0)
    runtime._poll_collisions(sim_time=0.4, episode_id="ep")
    events.__exit__(None, None, None)
    assert runtime.metrics.collision_count == 2

    lines = [
        json.loads(ln)
        for ln in (tmp_path / "e.jsonl").read_text().splitlines()
        if ln
    ]
    collisions = [ln for ln in lines if ln["event"] == "COLLISION"]
    assert len(collisions) == 2
    assert collisions[0]["data"]["agents"] == ["a", "b"]
    assert collisions[0]["data"]["distance_m"] == pytest.approx(0.3)


def test_near_miss_counted_not_collision(tmp_path: Path) -> None:
    runtime, events = _two_agent_runtime(tmp_path, collision_r=0.5, near_r=1.5)
    _set_pose(runtime, "b", 1.0, 0.0)  # within near (1.5) but outside collision (0.5)
    runtime._poll_collisions(sim_time=0.1, episode_id="ep")
    events.__exit__(None, None, None)
    assert runtime.metrics.near_miss_count == 1
    assert runtime.metrics.collision_count == 0
    lines = [
        json.loads(ln)
        for ln in (tmp_path / "e.jsonl").read_text().splitlines()
        if ln
    ]
    assert any(ln["event"] == "NEAR_MISS" for ln in lines)


def test_receding_from_collision_does_not_recount_near_miss(tmp_path: Path) -> None:
    runtime, events = _two_agent_runtime(tmp_path, collision_r=0.5, near_r=1.5)
    # Enter straight into collision (no prior near-miss tick).
    _set_pose(runtime, "b", 0.3, 0.0)
    runtime._poll_collisions(sim_time=0.1, episode_id="ep")
    # Recede into the near zone (between 0.5 and 1.5): must NOT add a near-miss.
    _set_pose(runtime, "b", 1.0, 0.0)
    runtime._poll_collisions(sim_time=0.2, episode_id="ep")
    events.__exit__(None, None, None)
    assert runtime.metrics.collision_count == 1
    assert runtime.metrics.near_miss_count == 0


def test_collision_count_reaches_metrics_json(tmp_path: Path) -> None:
    from genesis_nav.cli.main import main

    raw = yaml.safe_load(Path("examples/scenarios/smoke.yaml").read_text())
    raw.setdefault("runtime", {})["collision"] = {"near_miss_radius_m": 0.0}
    # smoke is single-agent; just confirm the keys flow to metrics.json as 0.
    path = tmp_path / "s.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    out = tmp_path / "runs"
    assert main(["run", str(path), "--fast", "--output-dir", str(out)]) == 0
    run_dir = next(out.iterdir())
    metrics = json.loads((run_dir / "metrics.json").read_text())
    assert metrics["collision_count"] == 0
    assert metrics["near_miss_count"] == 0
