from genesis_nav.core.agent import AgentRegistry, AgentSpec
from genesis_nav.core.task import TaskSpec
from genesis_nav.fleet.dispatcher import Dispatcher
from genesis_nav.fleet.queue import TaskQueue


def _make_registry(agents: list[tuple[str, tuple[float, float, float], tuple[str, ...]]]) -> AgentRegistry:
    reg = AgentRegistry()
    for agent_id, spawn, caps in agents:
        reg.register(
            AgentSpec(
                agent_id=agent_id,
                embodiment="mobile_base",
                namespace=f"/{agent_id}",
                capabilities=caps,
                spawn=spawn,
            )
        )
    return reg


def _task(task_id: str, *, agent: str | None = None, goal: tuple[float, float, float] = (5.0, 0.0, 0.0), capabilities: tuple[str, ...] = (), nearest: tuple[float, float] | None = None) -> TaskSpec:
    constraints: dict = {}
    if capabilities or nearest:
        selector: dict = {}
        if capabilities:
            selector["capabilities"] = list(capabilities)
        if nearest:
            selector["nearest_to"] = list(nearest)
        constraints["agent_selector"] = selector
    return TaskSpec(
        task_id=task_id,
        task_type="navigate_to_pose",
        agent_id=agent,
        goal=goal,
        constraints=constraints,
    )


def test_explicit_agent_id_assignment() -> None:
    reg = _make_registry([("r1", (0.0, 0.0, 0.0), ("navigate_2d",))])
    queue = TaskQueue()
    queue.submit(_task("t1", agent="r1"))
    dispatcher = Dispatcher(reg, queue)
    assigned: list[TaskSpec] = []
    results = dispatcher.tick(assigned.append)
    assert [r.agent_id for r in results] == ["r1"]
    assert assigned[0].task_id == "t1"


def test_dispatcher_skips_busy_agent() -> None:
    reg = _make_registry([("r1", (0.0, 0.0, 0.0), ("navigate_2d",))])
    reg.assign_task("r1", "existing")
    queue = TaskQueue()
    queue.submit(_task("t1", agent="r1"))
    dispatcher = Dispatcher(reg, queue)
    results = dispatcher.tick(lambda task: None)
    assert results == []
    # task remains queued for next tick
    assert "t1" in queue.task_ids()


def test_dispatcher_picks_nearest_free_agent() -> None:
    reg = _make_registry(
        [
            ("r1", (0.0, 0.0, 0.0), ("navigate_2d",)),
            ("r2", (10.0, 0.0, 0.0), ("navigate_2d",)),
            ("r3", (4.0, 0.0, 0.0), ("navigate_2d",)),
        ]
    )
    queue = TaskQueue()
    queue.submit(_task("t1", goal=(5.0, 0.0, 0.0)))
    dispatcher = Dispatcher(reg, queue)
    results = dispatcher.tick(lambda task: reg.assign_task(task.agent_id, task.task_id))
    assert [r.agent_id for r in results] == ["r3"]


def test_dispatcher_filters_by_capability() -> None:
    reg = _make_registry(
        [
            ("r_nav", (0.0, 0.0, 0.0), ("navigate_2d",)),
            ("r_arm", (0.0, 1.0, 0.0), ("navigate_2d", "manipulate")),
        ]
    )
    queue = TaskQueue()
    queue.submit(_task("t1", capabilities=("manipulate",)))
    dispatcher = Dispatcher(reg, queue)
    results = dispatcher.tick(lambda task: reg.assign_task(task.agent_id, task.task_id))
    assert [r.agent_id for r in results] == ["r_arm"]


def test_dispatcher_leaves_unmatchable_task_in_queue() -> None:
    reg = _make_registry([("r1", (0.0, 0.0, 0.0), ("navigate_2d",))])
    queue = TaskQueue()
    queue.submit(_task("t1", capabilities=("manipulate",)))
    dispatcher = Dispatcher(reg, queue)
    results = dispatcher.tick(lambda task: None)
    assert results == []
    assert "t1" in queue.task_ids()
