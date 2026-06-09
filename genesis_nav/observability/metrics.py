"""Metrics report model."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class MetricsSnapshot:
    scenario_id: str
    seed: int
    agent_count: int
    task_count: int
    success_rate: float
    task_succeeded_count: int = 0
    task_failed_count: int = 0
    collision_count: int = 0
    near_miss_count: int = 0
    command_accept_count: int = 0
    command_rejection_count: int = 0
    emergency_stop_count: int = 0
    time_to_goal_mean_sec: float = 0.0
    path_length_mean_m: float = 0.0
    sim_steps: int = 0
    real_time_factor: float = 0.0
    sim_steps_per_sec: float = 0.0
    task_dispatched_count: int = 0
    task_pending_peak: int = 0
    reservation_granted_count: int = 0
    reservation_conflict_count: int = 0
    reservation_released_count: int = 0
    replan_count: int = 0
    obstacle_event_count: int = 0
    watchdog_stop_count: int = 0
    yield_count: int = 0
    headon_reroute_count: int = 0
    costmap_wait_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
