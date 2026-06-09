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
    # When true (and the planner is a grid), agents reserve cells along their
    # remaining path; other agents treat those cells as blocked while planning.
    costmap_reservation: bool = False

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
            costmap_reservation=bool(block.get("costmap_reservation", cls.costmap_reservation)),
        )


@dataclass(frozen=True)
class CollisionConfig:
    """Inter-agent proximity detection thresholds (planar, metres).

    Both radii default to 0.0, which disables detection — existing scenarios
    keep ``collision_count``/``near_miss_count`` at 0 and pay no overhead.
    Detection is observation only: it emits `COLLISION` / `NEAR_MISS` events and
    counts them; it does not (yet) stop or reroute agents. Read from a top-level
    ``runtime.collision`` mapping.
    """

    collision_radius_m: float = 0.0
    near_miss_radius_m: float = 0.0
    # Proximity *response*: while another agent with right-of-way is within this
    # radius, the lower-priority agent yields (stops this tick). 0 disables, so
    # detection (above) and response are independently switchable.
    yield_radius_m: float = 0.0
    # Head-on *response*: when a higher-priority agent is ahead on a shared
    # corridor, the lower-priority agent replans with a lateral detour. 0
    # disables. Independent from yield (crossing vs head-on).
    headon_radius_m: float = 0.0
    headon_lateral_offset_m: float = 0.8

    @property
    def enabled(self) -> bool:
        return self.collision_radius_m > 0.0 or self.near_miss_radius_m > 0.0

    @property
    def response_enabled(self) -> bool:
        return self.yield_radius_m > 0.0

    @property
    def headon_enabled(self) -> bool:
        return self.headon_radius_m > 0.0

    @classmethod
    def from_scenario_raw(cls, raw: dict[str, Any] | None) -> "CollisionConfig":
        if not raw:
            return cls()
        block = (raw.get("runtime") or {}).get("collision") or {}
        if not isinstance(block, dict):
            raise ValueError("runtime.collision must be a mapping")
        collision_radius_m = float(
            block.get("collision_radius_m", cls.collision_radius_m)
        )
        near_miss_radius_m = float(
            block.get("near_miss_radius_m", cls.near_miss_radius_m)
        )
        yield_radius_m = float(block.get("yield_radius_m", cls.yield_radius_m))
        headon_radius_m = float(block.get("headon_radius_m", cls.headon_radius_m))
        headon_lateral_offset_m = float(
            block.get("headon_lateral_offset_m", cls.headon_lateral_offset_m)
        )
        if (
            min(
                collision_radius_m,
                near_miss_radius_m,
                yield_radius_m,
                headon_radius_m,
                headon_lateral_offset_m,
            )
            < 0.0
        ):
            raise ValueError("runtime.collision radii and offsets must be non-negative")
        return cls(
            collision_radius_m=collision_radius_m,
            near_miss_radius_m=near_miss_radius_m,
            yield_radius_m=yield_radius_m,
            headon_radius_m=headon_radius_m,
            headon_lateral_offset_m=headon_lateral_offset_m,
        )


__all__ = ["CollisionConfig", "NavigationConfig"]
