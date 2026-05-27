import pytest

from genesis_nav.core.task import TaskSpec
from genesis_nav.fleet.queue import TaskQueue


def _task(task_id: str, *, priority: int = 0, agent: str | None = None) -> TaskSpec:
    return TaskSpec(
        task_id=task_id,
        task_type="navigate_to_pose",
        agent_id=agent,
        priority=priority,
        goal=(1.0, 1.0, 0.0),
    )


def test_queue_pops_highest_priority_first() -> None:
    q = TaskQueue()
    q.submit(_task("low", priority=1))
    q.submit(_task("high", priority=10))
    q.submit(_task("mid", priority=5))

    order = [q.pop().task_id for _ in range(3)]
    assert order == ["high", "mid", "low"]
    assert q.pop() is None


def test_queue_fifo_within_same_priority() -> None:
    q = TaskQueue()
    q.submit(_task("a"))
    q.submit(_task("b"))
    q.submit(_task("c"))
    assert [q.pop().task_id for _ in range(3)] == ["a", "b", "c"]
    assert q.pop() is None


def test_queue_rejects_empty_id() -> None:
    q = TaskQueue()
    with pytest.raises(ValueError):
        q.submit(
            TaskSpec(task_id="", task_type="navigate_to_pose", goal=(1.0, 0.0, 0.0))
        )


def test_queue_rejects_duplicate_id() -> None:
    q = TaskQueue()
    q.submit(_task("dup"))
    with pytest.raises(ValueError):
        q.submit(_task("dup"))


def test_remove_marks_task_skipped() -> None:
    q = TaskQueue()
    q.submit(_task("a"))
    q.submit(_task("b"))
    assert q.remove("a") is True
    assert q.pop().task_id == "b"
    assert q.pop() is None


def test_snapshot_and_iteration() -> None:
    q = TaskQueue()
    q.submit(_task("a", priority=1))
    q.submit(_task("b", priority=5))
    ids = {task.task_id for task in q.snapshot()}
    assert ids == {"a", "b"}
    assert len(q) == 2
