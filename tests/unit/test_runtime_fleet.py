import json
from pathlib import Path

from genesis_nav.benchmarks.scenario import load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.core.task import TaskSpec
from genesis_nav.observability.events import JsonlEventWriter


def _read_events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def test_submit_task_dispatch_and_complete(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/warehouse_10_agents.yaml"))
    log = tmp_path / "events.jsonl"
    task = TaskSpec(
        task_id="dispatch_test",
        task_type="navigate_to_pose",
        goal=(5.0, 1.0, 0.0),
        constraints={"agent_selector": {"capabilities": ["navigate_2d"]}},
    )
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.submit_task(task, episode_id="ep")
        runtime.run_until_idle(episode_id="ep", max_sim_seconds=30.0)
    summary = runtime.metrics.summary()
    assert summary["task_succeeded_count"] == 1
    assert summary["task_dispatched_count"] == 1
    names = [e["event"] for e in _read_events(log)]
    assert "PLAN_CREATED" in names
    assert "TASK_ASSIGNED" in names


def test_reservation_conflict_emits_events(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/warehouse_10_agents.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        # Patch the resource catalog so the test resource is known.
        from genesis_nav.fleet.resources import Resource, ResourceCatalog
        runtime.resources = ResourceCatalog([Resource(resource_id="aisle_A")])

        first = runtime.reserve_resource("aisle_A", "robot_001", 5.0, episode_id="ep")
        blocked = runtime.reserve_resource("aisle_A", "robot_002", 5.0, episode_id="ep")
        runtime.release_resource(first.lease_id, requester_id="robot_001", episode_id="ep")
    assert first is not None
    assert blocked is None
    summary = runtime.metrics.summary()
    assert summary["reservation_granted_count"] == 1
    assert summary["reservation_conflict_count"] == 1
    assert summary["reservation_released_count"] == 1
    events_list = _read_events(log)
    names = [e["event"] for e in events_list]
    assert names.count("RESOURCE_RESERVED") == 1
    assert names.count("RESOURCE_RELEASED") >= 2  # conflict + explicit release


def test_unknown_resource_is_rejected_when_catalog_is_defined(tmp_path: Path) -> None:
    from genesis_nav.fleet.resources import Resource, ResourceCatalog

    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        runtime.resources = ResourceCatalog([Resource(resource_id="aisle_A")])
        lease = runtime.reserve_resource("ghost", "robot_001", 1.0, episode_id="ep")
    assert lease is None
    assert runtime.metrics.reservation_granted_count == 0


def test_empty_catalog_allows_free_reservation(tmp_path: Path) -> None:
    scenario = load_scenario(Path("examples/scenarios/smoke.yaml"))
    log = tmp_path / "events.jsonl"
    with JsonlEventWriter(log) as events:
        runtime = Runtime.from_scenario(scenario, events)
        lease = runtime.reserve_resource("any_resource", "robot_001", 1.0, episode_id="ep")
    assert lease is not None
    assert runtime.metrics.reservation_granted_count == 1
