"""Priority task queue used by the runtime dispatcher.

Tasks are popped highest-priority-first, ties broken by submission order. The
queue is intentionally minimal: it owns ordering and bookkeeping only -- task
validation, agent matching, and reservation logic live in their own modules.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Iterator

from genesis_nav.core.task import TaskSpec


@dataclass(order=True)
class _Entry:
    sort_key: tuple[int, int]
    task: TaskSpec = field(compare=False)
    removed: bool = field(default=False, compare=False)


class TaskQueue:
    def __init__(self) -> None:
        self._heap: list[_Entry] = []
        self._by_id: dict[str, _Entry] = {}
        self._counter: int = 0

    def __len__(self) -> int:
        return sum(1 for entry in self._heap if not entry.removed)

    def __iter__(self) -> Iterator[TaskSpec]:
        return (entry.task for entry in self._heap if not entry.removed)

    def submit(self, task: TaskSpec) -> None:
        if not task.task_id:
            raise ValueError("task requires task_id to be queued")
        if task.task_id in self._by_id and not self._by_id[task.task_id].removed:
            raise ValueError(f"task '{task.task_id}' is already queued")
        self._counter += 1
        entry = _Entry(sort_key=(-int(task.priority), self._counter), task=task)
        heapq.heappush(self._heap, entry)
        self._by_id[task.task_id] = entry

    def pop(self) -> TaskSpec | None:
        while self._heap:
            entry = heapq.heappop(self._heap)
            if entry.removed:
                continue
            self._by_id.pop(entry.task.task_id, None)
            return entry.task
        return None

    def peek(self) -> TaskSpec | None:
        while self._heap and self._heap[0].removed:
            heapq.heappop(self._heap)
        if not self._heap:
            return None
        return self._heap[0].task

    def remove(self, task_id: str) -> bool:
        entry = self._by_id.pop(task_id, None)
        if entry is None or entry.removed:
            return False
        entry.removed = True
        return True

    def task_ids(self) -> list[str]:
        return [entry.task.task_id for entry in self._heap if not entry.removed]

    def snapshot(self) -> list[TaskSpec]:
        return list(self)
