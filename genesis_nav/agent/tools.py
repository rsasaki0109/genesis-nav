"""AI-agent tool API.

This is the only Python surface AI agents are allowed to call against the
runtime. The API never publishes `cmd_vel`, never mutates QoS, and never
deletes logs. Every task it submits is stamped with a `requester_id` and a
`trace_id` so the resulting events are traceable in `events.jsonl`.

Safety contract (see `docs/ai_agents.md` and the ADR in `docs/decisions.md`):

- All write methods require a non-empty `requester_id`.
- `submit_task` routes through `Runtime.submit_task`, which in turn flows
  through the dispatcher and the command gate. AI agents therefore cannot
  bypass authority arbitration.
- `pause_agent` and `stop_all` set `emergency_stopped=True` on the affected
  agents. The runtime step loop already rejects commands for stopped agents.
- `resume_agent` clears the emergency stop but does not reassign tasks; the
  scheduled task (if any) keeps running once the operator confirms.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field, replace
from typing import Any

from genesis_nav.core.runtime import Runtime
from genesis_nav.core.task import TaskSpec, TaskStatus
from genesis_nav.observability.diagnostics import DiagnosticsReport
from genesis_nav.observability.events import RingBufferEventSink, RuntimeEvent


@dataclass(frozen=True)
class AgentSnapshot:
    agent_id: str
    embodiment_type: str
    namespace: str
    lifecycle_state: str
    authority_mode: str
    pose: tuple[float, float, float]
    linear_velocity_x: float
    angular_velocity_z: float
    current_task_id: str
    current_task_status: str
    current_goal: tuple[float, float, float] | None
    emergency_stopped: bool
    capabilities: tuple[str, ...]


@dataclass(frozen=True)
class TaskSnapshot:
    task_id: str
    agent_id: str
    status: str
    goal: tuple[float, float, float] | None
    assigned_at_sec: float
    started_at_sec: float | None
    finished_at_sec: float | None
    path_length_m: float
    failure_reason: str
    requester_id: str = ""
    trace_id: str = ""


@dataclass(frozen=True)
class WorldSnapshot:
    scenario_id: str
    episode_id: str
    sim_time_sec: float
    sim_steps: int
    lifecycle_state: str
    agents: tuple[AgentSnapshot, ...]
    pending_task_ids: tuple[str, ...]
    active_resource_leases: int


def _snapshot_agent(state: Any) -> AgentSnapshot:
    return AgentSnapshot(
        agent_id=state.agent_id,
        embodiment_type=state.embodiment_type,
        namespace=state.namespace,
        lifecycle_state=state.lifecycle_state.value,
        authority_mode=state.authority_mode.value,
        pose=tuple(state.pose),
        linear_velocity_x=state.linear_velocity_x,
        angular_velocity_z=state.angular_velocity_z,
        current_task_id=state.current_task_id,
        current_task_status=state.current_task_status.value,
        current_goal=tuple(state.current_goal) if state.current_goal else None,
        emergency_stopped=state.emergency_stopped,
        capabilities=tuple(state.capabilities),
    )


def _snapshot_task(record: Any, *, requester_id: str = "", trace_id: str = "") -> TaskSnapshot:
    return TaskSnapshot(
        task_id=record.task_id,
        agent_id=record.agent_id,
        status=record.status.value,
        goal=tuple(record.goal) if record.goal else None,
        assigned_at_sec=record.assigned_at_sec,
        started_at_sec=record.started_at_sec,
        finished_at_sec=record.finished_at_sec,
        path_length_m=record.path_length_m,
        failure_reason=record.failure_reason,
        requester_id=requester_id,
        trace_id=trace_id,
    )


@dataclass
class AgentToolApi:
    """Local Python tool surface for AI agents.

    Construct via :meth:`Runtime.tool_api` or directly. ``event_buffer`` is
    optional; if absent, :meth:`get_recent_events` returns ``[]``.
    """

    runtime: Runtime
    scenario_id: str = ""
    event_buffer: RingBufferEventSink | None = None
    _task_meta: dict[str, dict[str, str]] = field(default_factory=dict)

    # ------------------------------------------------------------------ reads

    def list_agents(self) -> list[AgentSnapshot]:
        return [_snapshot_agent(state) for state in self.runtime.registry.list_states()]

    def get_world_state(self) -> WorldSnapshot:
        pending = tuple(self.runtime.task_queue.task_ids())
        active_leases = sum(len(ids) for ids in self.runtime._leases_by_agent.values())
        return WorldSnapshot(
            scenario_id=self.scenario_id,
            episode_id=self.runtime._current_episode_id,
            sim_time_sec=self.runtime.clock.sim_time_sec,
            sim_steps=self.runtime.metrics.sim_steps,
            lifecycle_state=self.runtime.lifecycle_state.value,
            agents=tuple(self.list_agents()),
            pending_task_ids=pending,
            active_resource_leases=active_leases,
        )

    def get_diagnostics(self) -> "DiagnosticsReport":
        """Read-only per-agent health snapshot (OK / WARN / ERROR).

        Safe for AI agents: it exposes existing health state only and cannot
        mutate the runtime or actuators.
        """

        return self.runtime.diagnostics()

    def get_task_status(self, task_id: str) -> TaskSnapshot | None:
        record = self.runtime.metrics.tasks.get(task_id)
        if record is None:
            if task_id in self.runtime.task_queue.task_ids():
                meta = self._task_meta.get(task_id, {})
                queued = next(
                    (t for t in self.runtime.task_queue if t.task_id == task_id),
                    None,
                )
                return TaskSnapshot(
                    task_id=task_id,
                    agent_id=queued.agent_id or "" if queued else "",
                    status="queued",
                    goal=tuple(queued.goal) if queued and queued.goal else None,
                    assigned_at_sec=0.0,
                    started_at_sec=None,
                    finished_at_sec=None,
                    path_length_m=0.0,
                    failure_reason="",
                    requester_id=meta.get("requester_id", ""),
                    trace_id=meta.get("trace_id", ""),
                )
            return None
        meta = self._task_meta.get(task_id, {})
        return _snapshot_task(
            record,
            requester_id=meta.get("requester_id", ""),
            trace_id=meta.get("trace_id", ""),
        )

    def get_recent_events(
        self,
        *,
        event: str | None = None,
        agent_id: str | None = None,
        task_id: str | None = None,
        since_ts: float | None = None,
        limit: int = 100,
    ) -> list[RuntimeEvent]:
        if self.event_buffer is None:
            return []
        return self.event_buffer.filter(
            event=event,
            agent_id=agent_id,
            task_id=task_id,
            since_ts=since_ts,
            limit=limit,
        )

    # ----------------------------------------------------------------- writes

    def submit_task(
        self,
        task: TaskSpec,
        *,
        requester_id: str,
        trace_id: str | None = None,
    ) -> TaskSnapshot:
        if not requester_id:
            raise ValueError("AI-issued tasks require requester_id")
        if not task.task_id:
            raise ValueError("task requires task_id")
        final_trace_id = trace_id or task.trace_id or uuid.uuid4().hex
        stamped = replace(task, requester_id=requester_id, trace_id=final_trace_id)
        self._task_meta[stamped.task_id] = {
            "requester_id": requester_id,
            "trace_id": final_trace_id,
        }
        self.runtime.submit_task(stamped)
        return TaskSnapshot(
            task_id=stamped.task_id,
            agent_id=stamped.agent_id or "",
            status="queued",
            goal=stamped.goal,
            assigned_at_sec=self.runtime.clock.sim_time_sec,
            started_at_sec=None,
            finished_at_sec=None,
            path_length_m=0.0,
            failure_reason="",
            requester_id=requester_id,
            trace_id=final_trace_id,
        )

    def pause_agent(self, agent_id: str, reason: str, *, requester_id: str) -> None:
        if not requester_id:
            raise ValueError("requester_id required")
        # Surfaces KeyError early if the agent does not exist.
        self.runtime.registry.get_state(agent_id)
        self.runtime.registry.emergency_stop(agent_id, True)
        self.runtime.events.write(
            ts=self.runtime.clock.sim_time_sec,
            episode_id=self.runtime._current_episode_id,
            agent_id=agent_id,
            event="SAFETY_STOP",
            data={
                "reason": reason,
                "requester_id": requester_id,
                "source": "ai_tool_api",
                "scope": "agent",
            },
        )

    def resume_agent(self, agent_id: str, *, requester_id: str) -> None:
        if not requester_id:
            raise ValueError("requester_id required")
        self.runtime.registry.get_state(agent_id)
        self.runtime.registry.emergency_stop(agent_id, False)
        self.runtime.events.write(
            ts=self.runtime.clock.sim_time_sec,
            episode_id=self.runtime._current_episode_id,
            agent_id=agent_id,
            event="AGENT_RESUMED",
            data={"requester_id": requester_id, "source": "ai_tool_api"},
        )

    def stop_all(self, reason: str, *, requester_id: str) -> None:
        if not requester_id:
            raise ValueError("requester_id required")
        for state in self.runtime.registry.list_states():
            self.runtime.registry.emergency_stop(state.agent_id, True)
        self.runtime.events.write(
            ts=self.runtime.clock.sim_time_sec,
            episode_id=self.runtime._current_episode_id,
            agent_id="",
            event="SAFETY_STOP",
            data={
                "reason": reason,
                "requester_id": requester_id,
                "source": "ai_tool_api",
                "scope": "all",
            },
        )


__all__ = [
    "AgentSnapshot",
    "AgentToolApi",
    "TaskSnapshot",
    "WorldSnapshot",
]
