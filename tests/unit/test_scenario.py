from pathlib import Path

from genesis_nav.benchmarks.scenario import load_scenario


def test_loads_smoke_scenario() -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))

    assert scenario.scenario_id == "smoke"
    assert scenario.seed == 42
    assert len(scenario.agents) == 1
    assert len(scenario.tasks) == 1
    assert scenario.agents[0].namespace == "/robot_001"
    assert scenario.tasks[0].goal == (2.0, 1.0, 0.0)
