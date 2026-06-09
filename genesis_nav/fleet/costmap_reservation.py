"""Costmap cell reservations for multi-agent grid planning.

When enabled, an executing agent holds the grid cells along its remaining path.
Other agents treat those cells as blocked during global planning and enter
``RESERVING`` until the corridor clears.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from genesis_nav.navigation.grid_planner import OccupancyGrid


@dataclass
class CostmapReservationStore:
    """Tracks which agent owns which grid cells."""

    _by_agent: dict[str, set[tuple[int, int]]] = field(default_factory=dict)

    def blocked_for(self, agent_id: str) -> frozenset[tuple[int, int]]:
        blocked: set[tuple[int, int]] = set()
        for other_id, cells in self._by_agent.items():
            if other_id != agent_id:
                blocked.update(cells)
        return frozenset(blocked)

    def cells_for(self, agent_id: str) -> frozenset[tuple[int, int]]:
        return frozenset(self._by_agent.get(agent_id, set()))

    def set_cells(self, agent_id: str, cells: set[tuple[int, int]]) -> None:
        self._by_agent[agent_id] = set(cells)

    def release(self, agent_id: str) -> None:
        self._by_agent.pop(agent_id, None)


def collect_path_cells(
    grid: OccupancyGrid,
    points: list[tuple[float, float, float]],
) -> set[tuple[int, int]]:
    """Return every in-bounds grid cell touched by the polyline ``points``."""

    if len(points) < 2:
        if points:
            return {grid.world_to_cell(points[0][0], points[0][1])}
        return set()

    cells: set[tuple[int, int]] = set()
    step = grid.resolution * 0.5
    for a, b in zip(points, points[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        samples = max(1, int(dist / step))
        for i in range(samples + 1):
            t = i / samples
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            col, row = grid.world_to_cell(x, y)
            if grid.in_bounds(col, row):
                cells.add((col, row))
    return cells


__all__ = ["CostmapReservationStore", "collect_path_cells"]
