"""Metrics report model."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

_WILSON_Z_95 = 1.959963984540054


def wilson_success_rate_ci(
    succeeded: int,
    total: int,
    *,
    confidence: float = 0.95,
) -> dict[str, float] | None:
    """Return a Wilson score interval for a binomial success rate.

    ``None`` when ``total <= 1`` (no meaningful interval for a single trial).
    """

    if total <= 1:
        return None
    if succeeded < 0 or succeeded > total:
        raise ValueError("succeeded must be between 0 and total")
    if confidence != 0.95:
        raise ValueError("only confidence=0.95 is supported")

    p_hat = succeeded / total
    z = _WILSON_Z_95
    z2 = z * z
    denom = 1.0 + z2 / total
    center = (p_hat + z2 / (2.0 * total)) / denom
    margin = (z / denom) * math.sqrt(
        (p_hat * (1.0 - p_hat) / total) + (z2 / (4.0 * total * total))
    )
    return {
        "low": max(0.0, center - margin),
        "high": min(1.0, center + margin),
        "confidence": confidence,
    }


@dataclass(frozen=True)
class MetricsSnapshot:
    scenario_id: str
    seed: int
    agent_count: int
    task_count: int
    success_rate: float
    task_succeeded_count: int = 0
    task_failed_count: int = 0
    success_rate_ci: dict[str, float] | None = None
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
    dwell_count: int = 0
    dwell_time_sec: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        if data["success_rate_ci"] is None:
            del data["success_rate_ci"]
        return data
