"""Nav2-backed global planner and its ROS-free service boundary.

`Nav2Planner` implements the planner contract (`plan` / `replan`) used by the
runtime, so selecting Nav2 is a backend swap, not a runtime change. It does not
reimplement any Nav2 planner: it *delegates* path computation to a running Nav2
stack through the `Nav2PathService` boundary (a `ComputePathToPose` action in
the real bridge). genesis-nav's own local controller still chases the returned
waypoints, and that controller's commands still traverse `CommandGate` — so the
AI safety boundary is unchanged.

NOTE (v0.2 scope): this slice delegates *global planning* only. Full
delegation of Nav2's controller (its `cmd_vel` traversing `CommandGate` as an
`AUTONOMY` external command) is a documented follow-up in the Nav2 ADR.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from genesis_nav.navigation.grid_planner import PlannerError


@runtime_checkable
class Nav2PathService(Protocol):
    """Boundary to a Nav2 global planner.

    Returns an ordered list of ``(x, y, yaw)`` world poses from ``start`` to
    ``goal``, or an empty list if Nav2 reports no path.
    """

    def compute_path(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]: ...


@dataclass
class FakeNav2PathService:
    """In-memory `Nav2PathService` for unit tests.

    Returns ``path`` verbatim and records every request so a test can assert
    the planner delegated with the expected start/goal.
    """

    path: list[tuple[float, float, float]] = field(default_factory=list)
    requests: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = field(
        default_factory=list
    )

    def compute_path(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        self.requests.append((start, goal))
        return list(self.path)


class Nav2Planner:
    """Planner that delegates global planning to Nav2 via a `Nav2PathService`."""

    def __init__(self, service: Nav2PathService) -> None:
        self.service = service

    def plan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        path = self.service.compute_path(start, goal)
        if not path:
            raise PlannerError(f"Nav2 returned no path from {start[:2]} to {goal[:2]}")
        waypoints = [(float(x), float(y), float(yaw)) for x, y, yaw in path]
        # Pin the exact start and goal poses so the controller drives to the
        # precise request endpoints regardless of Nav2's pose discretization.
        if waypoints[0][:2] != start[:2]:
            waypoints.insert(0, (start[0], start[1], start[2]))
        waypoints[-1] = (goal[0], goal[1], goal[2])
        return waypoints

    def replan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        return self.plan(start, goal)


__all__ = ["FakeNav2PathService", "Nav2PathService", "Nav2Planner"]
