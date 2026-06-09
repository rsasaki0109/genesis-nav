"""Task model used by the runtime and future AI-agent API."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class TaskStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    task_type: str
    agent_id: str | None = None
    priority: int = 0
    goal: tuple[float, float, float] | None = None
    dwell_sec: float = 0.0
    constraints: dict[str, Any] = field(default_factory=dict)
    requester_id: str = "scenario"
    trace_id: str = ""

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "TaskSpec":
        task_id = str(data.get("id") or data.get("task_id") or "").strip()
        if not task_id:
            raise ValueError("task entry requires 'id'")
        goal = data.get("goal")
        parsed_goal: tuple[float, float, float] | None = None
        if goal is not None:
            if not isinstance(goal, list | tuple) or len(goal) != 3:
                raise ValueError(f"task '{task_id}' goal must be [x, y, yaw]")
            parsed_goal = (float(goal[0]), float(goal[1]), float(goal[2]))
        dwell_sec = float(data.get("dwell_sec", 0.0))
        if dwell_sec < 0.0:
            raise ValueError(f"task '{task_id}' dwell_sec must be >= 0")
        return cls(
            task_id=task_id,
            task_type=str(data.get("type", "navigate_to_pose")),
            agent_id=data.get("agent"),
            priority=int(data.get("priority", 0)),
            goal=parsed_goal,
            dwell_sec=dwell_sec,
            constraints=dict(data.get("constraints", {})),
            requester_id=str(data.get("requester_id", "scenario")),
            trace_id=str(data.get("trace_id", "")),
        )
