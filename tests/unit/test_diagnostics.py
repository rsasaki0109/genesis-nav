"""Tests for per-agent hardware/runtime diagnostics."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import yaml

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.core.runtime import Runtime
from genesis_nav.navigation.behavior import BehaviorState
from genesis_nav.observability.diagnostics import (
    DiagnosticLevel,
    collect_diagnostics,
)
from genesis_nav.observability.events import JsonlEventWriter
from genesis_nav.ros2_robot import FakeRobotTransport, Ros2RobotAdapter

SMOKE = Path("examples/scenarios/smoke.yaml")


def _state(agent_id: str, **kw) -> SimpleNamespace:
    defaults = dict(
        agent_id=agent_id,
        emergency_stopped=False,
        fall_detected=False,
        behavior_state=BehaviorState.EXECUTING,
    )
    defaults.update(kw)
    return SimpleNamespace(**defaults)


# ----------------------------------------------------------- pure collector


def test_healthy_agents_are_ok() -> None:
    report = collect_diagnostics([_state("a"), _state("b")], {})
    assert report.level is DiagnosticLevel.OK
    assert all(a.level is DiagnosticLevel.OK for a in report.agents)


def test_emergency_stop_is_error() -> None:
    report = collect_diagnostics([_state("a", emergency_stopped=True)], {})
    assert report.level is DiagnosticLevel.ERROR
    assert "emergency_stopped" in report.agents[0].messages


def test_fall_is_error() -> None:
    report = collect_diagnostics([_state("a", fall_detected=True)], {})
    assert report.agents[0].level is DiagnosticLevel.ERROR
    assert "fall_detected" in report.agents[0].messages


def test_failed_behavior_is_error() -> None:
    report = collect_diagnostics(
        [_state("a", behavior_state=BehaviorState.FAILED)], {}
    )
    assert report.agents[0].level is DiagnosticLevel.ERROR
    assert "task_failed" in report.agents[0].messages


def test_in_collision_is_error() -> None:
    report = collect_diagnostics([_state("a", in_collision=True)], {})
    assert report.agents[0].level is DiagnosticLevel.ERROR
    assert "in_collision" in report.agents[0].messages


def test_yielding_is_warn() -> None:
    report = collect_diagnostics([_state("a", yielding=True)], {})
    assert report.agents[0].level is DiagnosticLevel.WARN
    assert "yielding" in report.agents[0].messages


def test_collision_outranks_yield_on_same_agent() -> None:
    report = collect_diagnostics(
        [_state("a", yielding=True, in_collision=True)], {}
    )
    assert report.agents[0].level is DiagnosticLevel.ERROR


def test_overall_level_is_worst_agent() -> None:
    report = collect_diagnostics(
        [_state("ok"), _state("bad", emergency_stopped=True)], {}
    )
    assert report.level is DiagnosticLevel.ERROR


def test_adapter_watchdog_is_warn_and_reports_age() -> None:
    transport = FakeRobotTransport()
    adapter = Ros2RobotAdapter(agent_id="r0", transport=transport, command_timeout_sec=0.5)
    adapter.apply_command(RuntimeCommand(agent_id="r0", linear_x=0.2), dt_sec=0.1)
    transport.advance(0.6)  # exceed the 0.5s watchdog

    report = collect_diagnostics([_state("r0")], {"r0": adapter})
    diag = report.agents[0]
    assert report.level is DiagnosticLevel.WARN
    assert "command_watchdog_expired" in diag.messages
    assert diag.command_age_sec is not None and diag.command_age_sec > 0.5


# ----------------------------------------------------------- runtime + tool


def test_runtime_diagnostics_ok_on_smoke(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    with JsonlEventWriter(tmp_path / "e.jsonl") as events:
        runtime = Runtime.from_scenario(scenario, events)
        report = runtime.diagnostics()
        tool_report = runtime.tool_api(scenario_id="smoke").get_diagnostics()
    assert report.level is DiagnosticLevel.OK
    assert tool_report.level is DiagnosticLevel.OK
    assert {a.agent_id for a in report.agents} == {"robot_001"}


def _diag_scenario(tmp_path: Path, interval: float) -> Path:
    raw = yaml.safe_load(SMOKE.read_text())
    raw["runtime"] = {"navigation": {"diagnostics_interval_sec": interval}}
    path = tmp_path / "diag.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_periodic_diagnostics_event_emitted(tmp_path: Path) -> None:
    scenario = load_scenario(_diag_scenario(tmp_path, interval=0.1))
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)

    diags = [
        json.loads(ln)
        for ln in log.read_text().splitlines()
        if ln and json.loads(ln)["event"] == "DIAGNOSTICS"
    ]
    assert len(diags) >= 1
    assert diags[0]["data"]["level"] in {"OK", "WARN", "ERROR"}
    assert diags[0]["data"]["agents"][0]["agent_id"] == "robot_001"


def test_diagnostics_disabled_by_default(tmp_path: Path) -> None:
    scenario = load_scenario(SMOKE)
    log = tmp_path / "e.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)
    kinds = [json.loads(ln)["event"] for ln in log.read_text().splitlines() if ln]
    assert "DIAGNOSTICS" not in kinds
