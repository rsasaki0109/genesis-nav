import json
import math
from pathlib import Path

import pytest

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.core.runtime import Runtime
from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.task import TaskStatus
from genesis_nav.humanoid import HumanoidIntentAdapter, HumanoidNavState
from genesis_nav.observability.events import JsonlEventWriter


def _read_events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _intent(linear_x: float, angular_z: float) -> RuntimeCommand:
    return RuntimeCommand(
        agent_id="humanoid_001",
        linear_x=linear_x,
        angular_z=angular_z,
        authority=AuthorityMode.AUTONOMY,
        issued_at_sec=0.0,
        ttl_ms=200,
        source="test",
    )


def test_intent_adapter_integrates_planar_pose() -> None:
    adapter = HumanoidIntentAdapter(agent_id="humanoid_001")
    adapter.apply_command(_intent(1.0, 0.0), dt_sec=0.5)
    pose = adapter.read_pose()
    assert pose == pytest.approx((0.5, 0.0, 0.0), abs=1e-6)
    adapter.apply_command(_intent(0.0, math.pi / 2), dt_sec=1.0)
    assert adapter.read_pose()[2] == pytest.approx(math.pi / 2, abs=1e-6)


def test_intent_adapter_zero_dt_is_noop() -> None:
    adapter = HumanoidIntentAdapter(agent_id="humanoid_001", x=1.0, y=2.0, yaw=0.3)
    adapter.apply_command(_intent(10.0, 5.0), dt_sec=0.0)
    assert adapter.read_pose() == (1.0, 2.0, 0.3)
    assert adapter.linear_x == 0.0  # never written
    assert adapter.angular_z == 0.0


def test_trigger_fall_zeros_intent_and_blocks_subsequent_commands() -> None:
    adapter = HumanoidIntentAdapter(agent_id="humanoid_001")
    adapter.apply_command(_intent(0.5, 0.1), dt_sec=0.1)
    assert adapter.linear_x == pytest.approx(0.5)

    adapter.trigger_fall("balance_lost")
    assert adapter.fall_detected is True
    assert adapter.fall_reason == "balance_lost"
    assert adapter.balance_margin == 0.0

    # After falling, applying further commands does not move the pose.
    pose_before = adapter.read_pose()
    adapter.apply_command(_intent(1.0, 1.0), dt_sec=0.5)
    assert adapter.read_pose() == pose_before
    assert adapter.linear_x == 0.0
    assert adapter.angular_z == 0.0


def test_humanoid_scenario_loads_with_pelvis_and_foot_frames() -> None:
    scenario = load_scenario(Path("examples/scenarios/humanoid_nav_intent.yaml"))
    spec = scenario.agents[0]
    assert spec.embodiment == "humanoid"
    assert spec.frames.pelvis == "humanoid_001/pelvis"
    assert spec.frames.left_foot == "humanoid_001/left_foot"
    assert spec.frames.right_foot == "humanoid_001/right_foot"
    assert "navigate_intent" in spec.capabilities


def test_runtime_builds_humanoid_adapter_for_humanoid_type(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/humanoid_nav_intent.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
    adapter = runtime.adapters["humanoid_001"]
    assert isinstance(adapter, HumanoidIntentAdapter)


def test_humanoid_scenario_reaches_goal(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/humanoid_nav_intent.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        metrics = runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)
    summary = metrics.summary()
    assert summary["task_succeeded_count"] == 1
    assert summary["success_rate"] == 1.0


def test_fall_during_task_triggers_safety_stop_and_stops_motion(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/humanoid_nav_intent.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        # Let the humanoid move a bit, then inject a fall.
        for _ in range(5):
            runtime.step(episode_id="ep")
        adapter = runtime.adapters["humanoid_001"]
        pose_before_fall = adapter.read_pose()
        assert isinstance(adapter, HumanoidIntentAdapter)
        adapter.trigger_fall("balance_lost")

        # Next step should detect the fall, emit events, and stop motion.
        runtime.step(episode_id="ep")
        for _ in range(10):
            runtime.step(episode_id="ep")

    state = runtime.registry.get_state("humanoid_001")
    assert state.emergency_stopped is True
    assert state.fall_detected is True
    # The pose must not advance once fallen.
    pose_after = runtime.adapters["humanoid_001"].read_pose()
    assert pose_after[0] == pytest.approx(pose_before_fall[0], abs=1e-6)
    assert pose_after[1] == pytest.approx(pose_before_fall[1], abs=1e-6)

    events = _read_events(log)
    fall_events = [e for e in events if e["event"] == "FALL_DETECTED"]
    safety_events = [
        e for e in events
        if e["event"] == "SAFETY_STOP" and e.get("data", {}).get("reason") == "fall_detected"
    ]
    assert len(fall_events) == 1
    assert fall_events[0]["data"]["source"] == "humanoid_adapter"
    assert len(safety_events) == 1
    assert safety_events[0]["data"]["scope"] == "agent"
    rejects = [e for e in events if e["event"] == "COMMAND_REJECTED"]
    assert any(r["data"]["reason"] == "emergency stop" for r in rejects)


def test_fall_event_emitted_once_per_rising_edge(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/humanoid_nav_intent.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        adapter = runtime.adapters["humanoid_001"]
        assert isinstance(adapter, HumanoidIntentAdapter)
        adapter.trigger_fall("balance_lost")
        # Many ticks while fall_detected stays True must emit only ONE FALL_DETECTED.
        for _ in range(20):
            runtime.step(episode_id="ep")

    fall_events = [e for e in _read_events(log) if e["event"] == "FALL_DETECTED"]
    assert len(fall_events) == 1


def test_existing_diff_drive_scenario_still_passes(tmp_path: Path) -> None:
    """Regression: extending FrameSpec must not break the smoke scenario."""

    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        for task in scenario.tasks:
            runtime.assign_task(task, ts=0.0, episode_id="ep")
        metrics = runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)
    assert metrics.summary()["success_rate"] == 1.0
    state = runtime.registry.get_state("robot_001")
    assert state.current_task_status is TaskStatus.PENDING
    # Pelvis/foot frames must default to empty for non-humanoid agents.
    spec = runtime.registry.get_spec("robot_001")
    assert spec.frames.pelvis == ""
    assert spec.frames.left_foot == ""
    assert spec.frames.right_foot == ""


def test_humanoid_nav_state_dataclass_round_trip() -> None:
    """Sanity check on the descriptive HumanoidNavState dataclass."""

    s = HumanoidNavState(
        pelvis_frame="h/pelvis",
        base_frame="h/base_link",
        left_foot_frame="h/left_foot",
        right_foot_frame="h/right_foot",
    )
    assert s.fall_detected is False
    assert s.balance_margin == 0.0
