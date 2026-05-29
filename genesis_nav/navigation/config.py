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
    # How long an accepted teleop command holds off the autonomy loop for the
    # same agent, so the operator keeps control between command bursts.
    teleop_hold_sec: float = 0.5
    # Sim-seconds between periodic DIAGNOSTICS events. 0 disables emission
    # (the snapshot is still always available via Runtime.diagnostics()).
    diagnostics_interval_sec: float = 0.0
    # Planner backend selector. ``auto`` keeps the v0.1 behaviour (grid if the
    # scenario declares an ``occupancy_grid``, else straight line). ``nav2``
    # delegates global planning to a running Nav2 stack (see the 2026-05-29
    # Nav2 ADR). Generalizes the grid|straight selector requested in issue #9.
    planner: str = "auto"
    # Local controller backend selector. ``local`` (default) keeps the v0.1
    # `SimpleLocalController` that chases waypoints in-process. ``nav2``
    # delegates velocity generation to a running Nav2 controller server; that
    # `cmd_vel` still traverses `CommandGate` as an ``AUTONOMY`` command before
    # it reaches the actuator (see the 2026-05-29 Nav2 ADR).
    controller: str = "local"

    PLANNER_CHOICES = ("auto", "grid", "straight", "nav2")
    CONTROLLER_CHOICES = ("local", "nav2")

    @classmethod
    def from_scenario_raw(cls, raw: dict[str, Any] | None) -> "NavigationConfig":
        if not raw:
            return cls()
        block = (raw.get("runtime") or {}).get("navigation") or {}
        if not isinstance(block, dict):
            raise ValueError("runtime.navigation must be a mapping")
        planner = str(block.get("planner", cls.planner))
        if planner not in cls.PLANNER_CHOICES:
            choices = ", ".join(cls.PLANNER_CHOICES)
            raise ValueError(
                f"runtime.navigation.planner must be one of: {choices} (got '{planner}')"
            )
        controller = str(block.get("controller", cls.controller))
        if controller not in cls.CONTROLLER_CHOICES:
            choices = ", ".join(cls.CONTROLLER_CHOICES)
            raise ValueError(
                f"runtime.navigation.controller must be one of: {choices} "
                f"(got '{controller}')"
            )
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
            teleop_hold_sec=float(
                block.get("teleop_hold_sec", cls.teleop_hold_sec)
            ),
            diagnostics_interval_sec=float(
                block.get("diagnostics_interval_sec", cls.diagnostics_interval_sec)
            ),
            planner=planner,
            controller=controller,
        )


__all__ = ["NavigationConfig"]
