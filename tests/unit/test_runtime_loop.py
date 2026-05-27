import json
from pathlib import Path

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.lifecycle import LifecycleState
from genesis_nav.core.runtime import Runtime
from genesis_nav.core.task import TaskStatus
from genesis_nav.observability.events import JsonlEventWriter


def _read_events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def test_single_agent_reaches_goal(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="episode-test")
        metrics = runtime.run_until_idle(episode_id="episode-test", max_sim_seconds=30.0)

    summary = metrics.summary()
    assert summary["task_succeeded_count"] == 1
    assert summary["task_failed_count"] == 0
    assert summary["success_rate"] == 1.0
    assert summary["command_rejection_count"] == 0

    record = metrics.tasks["task_001"]
    assert record.status is TaskStatus.SUCCEEDED
    assert record.finished_at_sec is not None
    assert record.finished_at_sec > 0.0

    state = runtime.registry.get_state("robot_001")
    assert state.current_task_id == ""
    assert state.current_task_status is TaskStatus.PENDING
    goal = scenario.tasks[0].goal
    assert goal is not None
    assert (state.pose[0] - goal[0]) ** 2 + (state.pose[1] - goal[1]) ** 2 <= 0.1 ** 2

    event_names = [event["event"] for event in _read_events(log_path)]
    assert "TASK_ASSIGNED" in event_names
    assert "TASK_STARTED" in event_names
    assert "COMMAND_ACCEPTED" in event_names
    assert "TASK_SUCCEEDED" in event_names


def test_run_until_idle_times_out_on_unreachable_goal(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="episode-timeout")
        metrics = runtime.run_until_idle(episode_id="episode-timeout", max_sim_seconds=0.1)

    summary = metrics.summary()
    assert summary["task_failed_count"] == 1
    assert summary["task_succeeded_count"] == 0
    assert summary["success_rate"] == 0.0

    record = metrics.tasks["task_001"]
    assert record.status is TaskStatus.FAILED
    assert record.failure_reason == "timeout"

    event_names = [event["event"] for event in _read_events(log_path)]
    assert "TASK_FAILED" in event_names


def test_emergency_stop_blocks_motion(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.registry.emergency_stop("robot_001", True)
        runtime.command_gate.set_emergency_stop("robot_001", True)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="episode-estop")
        for _ in range(5):
            runtime.step(episode_id="episode-estop")

    state = runtime.registry.get_state("robot_001")
    assert state.pose == (0.0, 0.0, 0.0)
    assert runtime.metrics.command_rejection_count == 5
    assert runtime.metrics.command_accept_count == 0
    event_names = [event["event"] for event in _read_events(log_path)]
    assert "COMMAND_REJECTED" in event_names


def test_pause_and_step_paused(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="episode-pause")

        runtime.pause()
        assert runtime.lifecycle_state is LifecycleState.PAUSED

        before_steps = runtime.metrics.sim_steps
        runtime.step(episode_id="episode-pause")
        assert runtime.metrics.sim_steps == before_steps

        runtime.step_paused(episode_id="episode-pause", count=2)
        assert runtime.metrics.sim_steps == before_steps + 2
        assert runtime.lifecycle_state is LifecycleState.PAUSED
