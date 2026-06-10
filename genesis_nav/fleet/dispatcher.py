"""Match queued tasks to free agents.

The dispatcher reads from a `TaskQueue`, inspects agent state via the
registry, and selects a target agent based on the task's `agent_id`,
`agent_selector.capabilities`, and `agent_selector.nearest_to`. It then hands
the chosen task to a `Runtime.assign_task`-shaped callable.

The dispatcher does *not* mutate the queue if no candidate is available -- the
task is left in place so it can be picked up on a later tick.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from genesis_nav.core.agent import AgentRegistry, AgentState
from genesis_nav.core.task import TaskSpec
from genesis_nav.fleet.queue import TaskQueue


AssignFn = Callable[[TaskSpec], None]


@dataclass(frozen=True)
class DispatchResult:
    task_id: str
    agent_id: str


class Dispatcher:
    def __init__(self, registry: AgentRegistry, queue: TaskQueue) -> None:
        self.registry = registry
        self.queue = queue

    def tick(self, assign: AssignFn) -> list[DispatchResult]:
        """Drain as many queue heads as can be assigned right now."""

        results: list[DispatchResult] = []
        deferred: list[TaskSpec] = []
        try:
            while True:
                task = self.queue.pop()
                if task is None:
                    break
                agent_id = self._select_agent(task)
                if agent_id is None:
                    deferred.append(task)
                    continue
                target = self._with_agent(task, agent_id)
                assign(target)
                results.append(DispatchResult(task_id=task.task_id, agent_id=agent_id))
        finally:
            for task in deferred:
                self.queue.submit(task)
        return results

    # ----------------------------------------------------------- selection

    def _select_agent(self, task: TaskSpec) -> str | None:
        if task.agent_id:
            state = self._free_state(task.agent_id)
            if state is None:
                return None
            return state.agent_id

        selector = task.constraints.get("agent_selector") if task.constraints else None
        capabilities = ()
        nearest_to: tuple[float, float] | None = None
        if isinstance(selector, dict):
            caps = selector.get("capabilities")
            if isinstance(caps, list | tuple):
                capabilities = tuple(str(item) for item in caps)
            nearest = selector.get("nearest_to")
            if isinstance(nearest, list | tuple) and len(nearest) >= 2:
                nearest_to = (float(nearest[0]), float(nearest[1]))

        if nearest_to is None and task.goal is not None:
            nearest_to = (task.goal[0], task.goal[1])

        candidates = [
            state
            for state in self.registry.list_states()
            if self._is_free(state)
            and self._matches_capabilities(state, capabilities)
        ]
        if not candidates:
            return None
        if nearest_to is None:
            return candidates[0].agent_id
        candidates.sort(
            key=lambda state: math.hypot(
                state.pose[0] - nearest_to[0], state.pose[1] - nearest_to[1]
            )
        )
        return candidates[0].agent_id

    def _matches_capabilities(self, state: AgentState, capabilities: tuple[str, ...]) -> bool:
        if not capabilities:
            return True
        owned = set(state.capabilities)
        return all(cap in owned for cap in capabilities)

    def _is_free(self, state: AgentState) -> bool:
        if state.emergency_stopped:
            return False
        return not state.current_task_id

    def _free_state(self, agent_id: str) -> AgentState | None:
        try:
            state = self.registry.get_state(agent_id)
        except KeyError:
            return None
        if not self._is_free(state):
            return None
        return state

    def _with_agent(self, task: TaskSpec, agent_id: str) -> TaskSpec:
        if task.agent_id == agent_id:
            return task
        return TaskSpec(
            task_id=task.task_id,
            task_type=task.task_type,
            agent_id=agent_id,
            priority=task.priority,
            goal=task.goal,
            dwell_sec=task.dwell_sec,
            constraints=task.constraints,
            requester_id=task.requester_id,
            trace_id=task.trace_id,
        )


__all__ = ["DispatchResult", "Dispatcher"]
