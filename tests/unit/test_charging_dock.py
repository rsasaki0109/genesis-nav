"""Tests for the charging-dock scenario and task dwell behavior."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.core.task import TaskSpec
from genesis_nav.observability.events import JsonlEventWriter


def _events(path: Path) -> list[dict]:
    records: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped:
                records.append(json.loads(stripped))
    return records


def test_task_spec_parses_dwell_sec() -> None:
    task = TaskSpec.from_mapping(
        {
            "id": "dock_001",
            "type": "navigate_to_pose",
            "agent": "robot_001",
            "goal": [1.0, 2.0, 0.0],
            "dwell_sec": 1.5,
        }
    )
    assert task.dwell_sec == 1.5


def test_task_spec_rejects_negative_dwell_sec() -> None:
    with pytest.raises(ValueError, match="dwell_sec"):
        TaskSpec.from_mapping(
            {
                "id": "dock_001",
                "goal": [0.0, 0.0, 0.0],
                "dwell_sec": -0.1,
            }
        )


def test_charging_dock_scenario_reaches_goal(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/charging_dock.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id="ep")
        runtime.dispatch_pending(episode_id="ep")
        metrics = runtime.run_until_idle(episode_id="ep", max_sim_seconds=90.0)

    summary = metrics.summary()
    assert summary["success_rate"] == 1.0
    assert summary["task_succeeded_count"] == 2
    assert summary["dwell_count"] == 1
    assert summary["dwell_time_sec"] == pytest.approx(2.0)

    names = [record["event"] for record in _events(log_path)]
    assert "DWELL_STARTED" in names
    assert "DWELL_FINISHED" in names
    events = _events(log_path)
    dwell_started = next(
        i
        for i, record in enumerate(events)
        if record["event"] == "DWELL_STARTED" and record["task_id"] == "dock_001"
    )
    dwell_finished = next(
        i for i, record in enumerate(events) if record["event"] == "DWELL_FINISHED"
    )
    dock_success = next(
        i
        for i, record in enumerate(events)
        if record["event"] == "TASK_SUCCEEDED" and record["task_id"] == "dock_001"
    )
    assert dwell_started < dwell_finished <= dock_success
