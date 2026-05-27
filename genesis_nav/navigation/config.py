"""Configuration knobs for the navigation behavior loop."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class NavigationConfig:
    """Tunable parameters for stuck detection and recovery.

    Defaults match the v0.1 smoke and warehouse scenarios. Scenarios can
    override via a top-level ``runtime.navigation`` mapping.
    """

    waypoint_tolerance_m: float = 0.15
    stuck_window_sec: float = 1.5
    stuck_min_progress_m: float = 0.05
    recovery_wait_sec: float = 0.5
    max_recovery_retries: int = 3

    @classmethod
    def from_scenario_raw(cls, raw: dict[str, Any] | None) -> "NavigationConfig":
        if not raw:
            return cls()
        block = (raw.get("runtime") or {}).get("navigation") or {}
        if not isinstance(block, dict):
            raise ValueError("runtime.navigation must be a mapping")
        return cls(
            waypoint_tolerance_m=float(
                block.get("waypoint_tolerance_m", cls.waypoint_tolerance_m)
            ),
            stuck_window_sec=float(
                block.get("stuck_window_sec", cls.stuck_window_sec)
            ),
            stuck_min_progress_m=float(
                block.get("stuck_min_progress_m", cls.stuck_min_progress_m)
            ),
            recovery_wait_sec=float(
                block.get("recovery_wait_sec", cls.recovery_wait_sec)
            ),
            max_recovery_retries=int(
                block.get("max_recovery_retries", cls.max_recovery_retries)
            ),
        )


__all__ = ["NavigationConfig"]
