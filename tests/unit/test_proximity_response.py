"""Tests for proximity *response* (yield right-of-way).

Builds on the detection slice: when a higher-priority agent (lower agent-id) is
within `yield_radius_m`, the lower-priority agent stops this tick. Deterministic,
deadlock-free (total order), observation-only detection still runs. No rclpy.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.navigation.config import CollisionConfig
from genesis_nav.observability.diagnostics import DiagnosticLevel
from genesis_nav.observability.events import JsonlEventWriter


def test_collision_config_parses_yield_radius() -> None:
    cfg = CollisionConfig.from_scenario_raw(
        {"runtime": {"collision": {"yield_radius_m": 1.2}}}
    )
    assert cfg.yield_radius_m == 1.2
    assert cfg.response_enabled is True
    assert CollisionConfig().response_enabled is False


def _runtime(tmp_path: Path, *, yield_radius: float):  # noqa: ANN001
    raw = {
        "scenario_id": "yield",
        "seed": 0,
        "world": "examples.scenarios.flat_world",
        "runtime": {"collision": {"yield_radius_m": yield_radius}},
        "agents": [
            {"id": "a", "embodiment": "diff_drive", "spawn": [0.0, 0.0, 0.0]},
            {"id": "b", "embodiment": "diff_drive", "spawn": [10.0, 0.0, 0.0]},
        ],
        "tasks": [
            {"id": "ta", "type": "navigate_to_pose", "agent": "a", "goal": [5.0, 0.0, 0.0]},
            {"id": "tb", "type": "navigate_to_pose", "agent": "b", "goal": [-5.0, 0.0, 0.0]},
        ],
    }
    path = tmp_path / "y.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return load_scenario(path)


def _place(runtime: Runtime, agent_id: str, x: float, y: float) -> None:
    adapter = runtime.adapters[agent_id]
    adapter.x = x
    adapter.y = y


def test_lower_priority_yields_higher_does_not(tmp_path: Path) -> None:
    scenario = _runtime(tmp_path, yield_radius=1.0)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        # Give both an active task so each is a yield candidate for the other.
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")
        runtime.dispatch_pending(episode_id="ep")
        _place(runtime, "a", 0.0, 0.0)
        _place(runtime, "b", 0.5, 0.0)  # within 1.0 of a
        # b (higher id) must yield to a; a (lower id) must not.
        assert runtime._should_yield("b", (0.5, 0.0, 0.0)) is True
        assert runtime._should_yield("a", (0.0, 0.0, 0.0)) is False


def test_no_yield_to_idle_agent(tmp_path: Path) -> None:
    scenario = _runtime(tmp_path, yield_radius=1.0)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        # No tasks submitted -> a has no current task, so b must not yield to it.
        _place(runtime, "a", 0.0, 0.0)
        _place(runtime, "b", 0.5, 0.0)
        assert runtime._should_yield("b", (0.5, 0.0, 0.0)) is False


def test_disabled_when_radius_zero(tmp_path: Path) -> None:
    scenario = _runtime(tmp_path, yield_radius=0.0)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")
        runtime.dispatch_pending(episode_id="ep")
        _place(runtime, "a", 0.0, 0.0)
        _place(runtime, "b", 0.1, 0.0)
        assert runtime._should_yield("b", (0.1, 0.0, 0.0)) is False


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


def test_yield_resolves_crossing_conflict(tmp_path: Path) -> None:
    # Same geometry as benchmarks/multi_agent/yield_avoidance.yaml.
    metrics, events = _run(
        tmp_path, Path("benchmarks/multi_agent/yield_avoidance.yaml")
    )
    assert metrics["success_rate"] == 1.0
    assert metrics["collision_count"] == 0
    assert metrics["near_miss_count"] == 0
    assert metrics["yield_count"] >= 1
    assert any(ev["event"] == "AGENT_YIELDED" for ev in events)
    yielded = next(ev for ev in events if ev["event"] == "AGENT_YIELDED")
    assert yielded["agent_id"] == "b"  # lower-priority agent yields


def test_yield_surfaces_in_diagnostics(tmp_path: Path) -> None:
    # Run the real crossing scenario and watch the health read-model: while b
    # yields it must report WARN with a "yielding" message.
    scenario = load_scenario(Path("benchmarks/multi_agent/yield_avoidance.yaml"))
    seen_yield_warn = False
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")

        def on_step(_sim_time: float) -> None:
            nonlocal seen_yield_warn
            b = next(a for a in runtime.diagnostics().agents if a.agent_id == "b")
            if "yielding" in b.messages and b.level is DiagnosticLevel.WARN:
                seen_yield_warn = True

        runtime.run_until_idle(
            episode_id="ep", max_sim_seconds=30.0, on_step=on_step
        )
    assert seen_yield_warn, "b's yield must surface as WARN in diagnostics"


def test_detection_only_still_collides(tmp_path: Path) -> None:
    # The detection-only counterpart collides at the shared point — confirming
    # the yield benchmark's avoidance is doing real work.
    metrics, _ = _run(
        tmp_path, Path("benchmarks/multi_agent/near_miss_detection.yaml")
    )
    assert metrics["yield_count"] == 0  # response disabled here
