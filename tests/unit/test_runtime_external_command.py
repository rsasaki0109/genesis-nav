import json
from pathlib import Path

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.core.runtime import Runtime
from genesis_nav.observability.events import JsonlEventWriter


def test_apply_external_command_moves_agent(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        before = runtime.registry.get_state("robot_001").pose
        command = RuntimeCommand(
            agent_id="robot_001",
            linear_x=0.2,
            angular_z=0.0,
            authority=AuthorityMode.TELEOP,
            source="ros_cmd_vel",
            issued_at_sec=0.0,
        )
        runtime.apply_external_command(command)

    state = runtime.registry.get_state("robot_001")
    assert state.pose != before
    assert state.linear_velocity_x == 0.2
    assert runtime.metrics.command_accept_count == 1


def test_apply_external_command_unknown_agent_is_noop(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log_path = tmp_path / "events.jsonl"

    with JsonlEventWriter(log_path) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.apply_external_command(
            RuntimeCommand(
                agent_id="ghost",
                linear_x=1.0,
                authority=AuthorityMode.TELEOP,
                issued_at_sec=0.0,
            )
        )

    contents = log_path.read_text(encoding="utf-8")
    parsed = [json.loads(line) for line in contents.splitlines() if line]
    assert parsed == []
    assert runtime.metrics.command_accept_count == 0
