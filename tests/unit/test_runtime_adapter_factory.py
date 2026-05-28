"""Verify Runtime.from_scenario honors the adapter_factory hook."""

from pathlib import Path

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.agent import AgentSpec
from genesis_nav.core.embodiment import DiffDriveKinematics
from genesis_nav.core.runtime import Runtime
from genesis_nav.observability.events import JsonlEventWriter


class _Recorder:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def write(self, **kwargs):  # type: ignore[no-untyped-def]
        self.calls.append(kwargs["event"])


def test_adapter_factory_is_used_for_every_agent(tmp_path: Path) -> None:
    scenario = load_scenario("examples/scenarios/warehouse_10_agents.yaml")
    seen: list[str] = []

    def factory(spec: AgentSpec) -> DiffDriveKinematics:
        seen.append(spec.agent_id)
        spawn = spec.spawn or (0.0, 0.0, 0.0)
        return DiffDriveKinematics(
            agent_id=spec.agent_id, x=spawn[0] + 100.0, y=spawn[1], yaw=spawn[2]
        )

    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events, adapter_factory=factory)

    assert seen == [a.agent_id for a in scenario.agents]
    for agent in scenario.agents:
        adapter = runtime.adapters[agent.agent_id]
        assert isinstance(adapter, DiffDriveKinematics)
        assert adapter.x >= 100.0
