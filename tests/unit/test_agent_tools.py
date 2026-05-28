import json
from pathlib import Path

import pytest

from genesis_nav.agent.tools import AgentToolApi, AgentSnapshot, WorldSnapshot
from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.core.task import TaskSpec
from genesis_nav.observability.events import (
    FanoutEventSink,
    JsonlEventWriter,
    RingBufferEventSink,
)


def _read_events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _build(tmp_path: Path, scenario_path: str = "examples/scenarios/warehouse_10_agents.yaml"):
    scenario = load_scenario(Path(scenario_path))
    log = tmp_path / "events.jsonl"
    writer = JsonlEventWriter(log).__enter__()
    buffer = RingBufferEventSink(capacity=128)
    sink = FanoutEventSink([writer, buffer])
    runtime = Runtime.from_scenario(scenario, sink)
    runtime._current_episode_id = "ep_test"
    api = runtime.tool_api(scenario_id=scenario.scenario_id, event_buffer=buffer)
    return scenario, runtime, api, writer, log


def test_list_agents_returns_immutable_snapshots(tmp_path: Path) -> None:
    _, runtime, api, writer, _ = _build(tmp_path)
    try:
        agents = api.list_agents()
        assert len(agents) == len(runtime.registry.list_states())
        assert all(isinstance(a, AgentSnapshot) for a in agents)
        # Mutating the runtime state must not retroactively change the snapshot.
        first = agents[0]
        runtime.registry.emergency_stop(first.agent_id, True)
        assert first.emergency_stopped is False
        assert api.list_agents()[0].emergency_stopped is True
    finally:
        writer.__exit__(None, None, None)


def test_get_world_state_reports_pending_and_leases(tmp_path: Path) -> None:
    _, runtime, api, writer, _ = _build(tmp_path)
    try:
        # Queue a task via the AI surface so it shows up as pending.
        api.submit_task(
            TaskSpec(task_id="ai_t1", task_type="navigate_to_pose", goal=(3.0, 1.0, 0.0)),
            requester_id="planner_agent_v1",
        )
        # Pretend an agent acquired a lease.
        runtime._leases_by_agent.setdefault("robot_001", []).extend(["lease_a", "lease_b"])

        world = api.get_world_state()
        assert isinstance(world, WorldSnapshot)
        assert world.scenario_id == "warehouse_10_agents"
        assert "ai_t1" in world.pending_task_ids
        assert world.active_resource_leases == 2
        assert world.lifecycle_state == "active"
    finally:
        writer.__exit__(None, None, None)


def test_submit_task_requires_requester_and_stamps_trace_id(tmp_path: Path) -> None:
    _, runtime, api, writer, log = _build(tmp_path)
    try:
        with pytest.raises(ValueError):
            api.submit_task(
                TaskSpec(task_id="x", task_type="navigate_to_pose", goal=(1.0, 0.0, 0.0)),
                requester_id="",
            )

        snap = api.submit_task(
            TaskSpec(task_id="ai_t2", task_type="navigate_to_pose", goal=(1.0, 0.0, 0.0)),
            requester_id="planner_agent_v1",
        )
        assert snap.requester_id == "planner_agent_v1"
        assert snap.trace_id  # auto-stamped
        assert snap.status == "queued"

        # The PLAN_CREATED event carries the trace_id and requester_id.
        plan_events = [
            e for e in api.get_recent_events(event="PLAN_CREATED") if e.task_id == "ai_t2"
        ]
        assert len(plan_events) == 1
        assert plan_events[0].data["requester_id"] == "planner_agent_v1"
        assert plan_events[0].data["trace_id"] == snap.trace_id
    finally:
        writer.__exit__(None, None, None)


def test_submit_task_then_dispatch_completes(tmp_path: Path) -> None:
    _, runtime, api, writer, log = _build(tmp_path)
    try:
        api.submit_task(
            TaskSpec(
                task_id="ai_done",
                task_type="navigate_to_pose",
                goal=(2.0, 0.0, 0.0),
                constraints={"agent_selector": {"capabilities": ["navigate_2d"]}},
            ),
            requester_id="planner_agent_v1",
            trace_id="trace-fixed-1",
        )
        runtime.run_until_idle(episode_id="ep_test", max_sim_seconds=15.0)

        status = api.get_task_status("ai_done")
        assert status is not None
        assert status.status == "succeeded"
        assert status.trace_id == "trace-fixed-1"
        assert status.requester_id == "planner_agent_v1"

        names = [e["event"] for e in _read_events(log)]
        assert "PLAN_CREATED" in names
        assert "TASK_ASSIGNED" in names
        assert "TASK_SUCCEEDED" in names
    finally:
        writer.__exit__(None, None, None)


def test_pause_resume_emit_events(tmp_path: Path) -> None:
    _, runtime, api, writer, _ = _build(tmp_path)
    try:
        target = runtime.registry.list_states()[0].agent_id
        api.pause_agent(target, "operator hold", requester_id="planner_agent_v1")
        assert runtime.registry.get_state(target).emergency_stopped is True

        pause = api.get_recent_events(event="SAFETY_STOP", agent_id=target)
        assert len(pause) == 1
        assert pause[0].data["scope"] == "agent"
        assert pause[0].data["requester_id"] == "planner_agent_v1"

        api.resume_agent(target, requester_id="planner_agent_v1")
        assert runtime.registry.get_state(target).emergency_stopped is False
        resumed = api.get_recent_events(event="AGENT_RESUMED", agent_id=target)
        assert len(resumed) == 1
    finally:
        writer.__exit__(None, None, None)


def test_stop_all_stops_every_agent(tmp_path: Path) -> None:
    _, runtime, api, writer, _ = _build(tmp_path)
    try:
        api.stop_all("fleet-wide hold", requester_id="planner_agent_v1")
        assert all(s.emergency_stopped for s in runtime.registry.list_states())
        broadcast = api.get_recent_events(event="SAFETY_STOP")
        assert any(e.data.get("scope") == "all" for e in broadcast)
    finally:
        writer.__exit__(None, None, None)


def test_pause_unknown_agent_raises(tmp_path: Path) -> None:
    _, _, api, writer, _ = _build(tmp_path)
    try:
        with pytest.raises(KeyError):
            api.pause_agent("nope", "x", requester_id="planner_agent_v1")
    finally:
        writer.__exit__(None, None, None)


def test_pause_requires_requester(tmp_path: Path) -> None:
    _, runtime, api, writer, _ = _build(tmp_path)
    try:
        target = runtime.registry.list_states()[0].agent_id
        with pytest.raises(ValueError):
            api.pause_agent(target, "x", requester_id="")
        with pytest.raises(ValueError):
            api.resume_agent(target, requester_id="")
        with pytest.raises(ValueError):
            api.stop_all("x", requester_id="")
    finally:
        writer.__exit__(None, None, None)


def test_get_recent_events_filters_and_respects_limit(tmp_path: Path) -> None:
    _, runtime, api, writer, _ = _build(tmp_path)
    try:
        target = runtime.registry.list_states()[0].agent_id
        for i in range(5):
            api.pause_agent(target, f"hold-{i}", requester_id="planner")
            api.resume_agent(target, requester_id="planner")
        pause = api.get_recent_events(event="SAFETY_STOP", agent_id=target)
        assert len(pause) == 5
        only_two = api.get_recent_events(event="SAFETY_STOP", agent_id=target, limit=2)
        assert len(only_two) == 2
        # Most-recent-window semantics: limit returns the tail.
        assert only_two[-1].data["reason"] == "hold-4"
    finally:
        writer.__exit__(None, None, None)


def test_get_task_status_returns_none_for_unknown(tmp_path: Path) -> None:
    _, _, api, writer, _ = _build(tmp_path)
    try:
        assert api.get_task_status("ghost") is None
    finally:
        writer.__exit__(None, None, None)


def test_get_recent_events_returns_empty_without_buffer(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as writer:
        runtime = Runtime.from_scenario(scenario, writer)
        api = runtime.tool_api()
        assert api.get_recent_events() == []


def test_ai_tool_api_cannot_apply_cmd_vel(tmp_path: Path) -> None:
    """Smoke regression: the tool API exposes no actuator method."""

    _, _, api, writer, _ = _build(tmp_path)
    try:
        for forbidden in ("apply_command", "publish_cmd_vel", "send_velocity", "drive"):
            assert not hasattr(api, forbidden), f"AgentToolApi must not expose {forbidden}"
    finally:
        writer.__exit__(None, None, None)
