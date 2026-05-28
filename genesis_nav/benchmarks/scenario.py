"""Scenario schema loading and validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from genesis_nav.core.agent import AgentSpec
from genesis_nav.core.task import TaskSpec


@dataclass(frozen=True)
class ScenarioRecordConfig:
    rosbag: bool = False
    events: bool = True


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    seed: int
    world: str
    agents: tuple[AgentSpec, ...]
    tasks: tuple[TaskSpec, ...]
    metrics: tuple[str, ...]
    record: ScenarioRecordConfig
    raw: dict[str, Any]
    source_path: Path


def load_scenario(path: str | Path) -> Scenario:
    source_path = Path(path)
    with source_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    scenario_id = str(raw.get("scenario_id", "")).strip()
    if not scenario_id:
        raise ValueError("scenario requires 'scenario_id'")

    agents_raw = raw.get("agents")
    if not isinstance(agents_raw, list) or not agents_raw:
        raise ValueError("scenario requires at least one agent")

    tasks_raw = raw.get("tasks", [])
    if not isinstance(tasks_raw, list):
        raise ValueError("scenario 'tasks' must be a list")

    record_raw = raw.get("record") or {}
    return Scenario(
        scenario_id=scenario_id,
        seed=int(raw.get("seed", 0)),
        world=str(raw.get("world", "")),
        agents=tuple(AgentSpec.from_mapping(item) for item in agents_raw),
        tasks=tuple(TaskSpec.from_mapping(item) for item in tasks_raw),
        metrics=tuple(str(item) for item in raw.get("metrics", [])),
        record=ScenarioRecordConfig(
            rosbag=bool(record_raw.get("rosbag", False)),
            events=bool(record_raw.get("events", True)),
        ),
        raw=raw,
        source_path=source_path,
    )
