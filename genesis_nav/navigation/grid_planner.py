"""Grid-based A* planner over a 2D occupancy grid.

The runtime feeds this with the scenario's optional ``world.occupancy_grid``
block. If a grid is not supplied the planner is not constructed at all; the
runtime falls back to :class:`StraightLinePlanner`.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Any, Sequence


@dataclass(frozen=True)
class OccupancyGrid:
    """Static 2D occupancy grid in world coordinates.

    ``data[row][col]`` is truthy when the cell is blocked. ``origin`` is the
    world coordinate of the lower-left corner (cell ``[0, 0]``). ``resolution``
    is metres per cell. The grid is row-major with row ``0`` at the bottom.
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[tuple[bool, ...], ...]
    inflate_cells: int = 0

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> "OccupancyGrid":
        if not isinstance(raw, dict):
            raise ValueError("occupancy_grid must be a mapping")
        cells = raw.get("cells")
        if not isinstance(cells, list) or not cells:
            raise ValueError("occupancy_grid.cells must be a non-empty list of rows")
        rows = tuple(tuple(bool(c) for c in row) for row in cells)
        height = len(rows)
        width = len(rows[0])
        if any(len(row) != width for row in rows):
            raise ValueError("occupancy_grid.cells rows must share the same width")
        resolution = float(raw.get("resolution", 1.0))
        if resolution <= 0.0:
            raise ValueError("occupancy_grid.resolution must be > 0")
        origin = raw.get("origin", [0.0, 0.0])
        if not isinstance(origin, list | tuple) or len(origin) != 2:
            raise ValueError("occupancy_grid.origin must be [x, y]")
        inflate_cells = int(raw.get("inflate_cells", 0))
        if inflate_cells < 0:
            raise ValueError("occupancy_grid.inflate_cells must be >= 0")
        if inflate_cells > 0:
            rows = _inflate_data(rows, inflate_cells)
        return cls(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            data=rows,
            inflate_cells=inflate_cells,
        )

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def is_blocked(self, col: int, row: int) -> bool:
        if not self.in_bounds(col, row):
            return True
        return self.data[row][col]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return col, row

    def cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def with_blocked(
        self, cells: Sequence[tuple[int, int]]
    ) -> "OccupancyGrid":
        """Return a new grid with ``cells`` (``(col, row)``) marked blocked.

        Out-of-bounds cells are ignored. The original grid is unchanged so a
        replay can reconstruct the exact obstacle timeline by re-applying the
        recorded deltas in order.
        """

        block = {(c, r) for c, r in cells if self.in_bounds(c, r)}
        if not block:
            return self
        if self.inflate_cells > 0:
            block = _dilate_blocked(
                block, self.inflate_cells, self.width, self.height
            )
        rows = [list(row) for row in self.data]
        for col, row in block:
            rows[row][col] = True
        return OccupancyGrid(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            data=tuple(tuple(r) for r in rows),
            inflate_cells=self.inflate_cells,
        )


_NEIGHBOURS: tuple[tuple[int, int, float], ...] = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)


class GridAStarPlanner:
    """8-connected A* planner over a static :class:`OccupancyGrid`."""

    def __init__(self, grid: OccupancyGrid) -> None:
        self.grid = grid

    def plan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
        *,
        extra_blocked: frozenset[tuple[int, int]] = frozenset(),
    ) -> list[tuple[float, float, float]]:
        grid = self.grid
        if extra_blocked:
            grid = self.grid.with_blocked(extra_blocked)
        return self._plan_on_grid(grid, start, goal)

    def _plan_on_grid(
        self,
        grid: OccupancyGrid,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        sc = self.grid.world_to_cell(start[0], start[1])
        gc = self.grid.world_to_cell(goal[0], goal[1])
        if grid.is_blocked(*gc):
            raise PlannerError(f"goal cell {gc} is blocked")
        if grid.is_blocked(*sc):
            raise PlannerError(f"start cell {sc} is blocked")

        cells = _astar(grid, sc, gc)
        if not cells:
            raise PlannerError(f"no path from {sc} to {gc}")

        waypoints: list[tuple[float, float, float]] = [start]
        # Convert intermediate cells to world coords; skip the start cell
        # itself (already covered by `start`) and the goal cell (replaced by
        # the precise world goal so the controller drives to the exact pose).
        for col, row in cells[1:-1]:
            wx, wy = self.grid.cell_to_world(col, row)
            waypoints.append((wx, wy, 0.0))
        waypoints.append(goal)
        return _simplify(waypoints)

    def replan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        """Re-plan against the current grid.

        Identical to :meth:`plan` for this planner because A* is stateless; the
        runtime swaps in an updated grid before calling. The separate verb
        keeps the planner contract honest for backends that can replan
        incrementally (see the 2026-05-29 dynamic-obstacles ADR).
        """

        return self.plan(start, goal)


class PlannerError(RuntimeError):
    """Raised when no path exists or start/goal are blocked."""


def _astar(
    grid: OccupancyGrid,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    if start == goal:
        return [start]

    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start: 0.0}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(came_from, current)
        for dx, dy, step_cost in _NEIGHBOURS:
            nxt = (current[0] + dx, current[1] + dy)
            if grid.is_blocked(*nxt):
                continue
            # forbid corner cuts through blocked diagonals
            if dx != 0 and dy != 0:
                if grid.is_blocked(current[0] + dx, current[1]):
                    continue
                if grid.is_blocked(current[0], current[1] + dy):
                    continue
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(nxt, math.inf):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f_score = tentative + _heuristic(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (f_score, counter, nxt))
    return []


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)


def _reconstruct(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    return cells


def _simplify(
    waypoints: Sequence[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    """Drop colinear intermediate waypoints to shorten the queue."""

    if len(waypoints) <= 2:
        return list(waypoints)
    out: list[tuple[float, float, float]] = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        prev = out[-1]
        cur = waypoints[i]
        nxt = waypoints[i + 1]
        if _collinear(prev, cur, nxt):
            continue
        out.append(cur)
    out.append(waypoints[-1])
    return out


def _collinear(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
    tol: float = 1e-6,
) -> bool:
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return abs(cross) <= tol


def _dilate_blocked(
    blocked: set[tuple[int, int]],
    radius: int,
    width: int,
    height: int,
) -> set[tuple[int, int]]:
    """Expand blocked cells by ``radius`` using Chebyshev (square) dilation."""

    if radius <= 0:
        return blocked
    out = set(blocked)
    for col, row in blocked:
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                c, r = col + dc, row + dr
                if 0 <= c < width and 0 <= r < height:
                    out.add((c, r))
    return out


def _inflate_data(
    rows: tuple[tuple[bool, ...], ...],
    inflate_cells: int,
) -> tuple[tuple[bool, ...], ...]:
    """Return ``rows`` with obstacles dilated by ``inflate_cells``."""

    if inflate_cells <= 0:
        return rows
    height = len(rows)
    width = len(rows[0])
    blocked = {
        (col, row)
        for row in range(height)
        for col in range(width)
        if rows[row][col]
    }
    inflated = _dilate_blocked(blocked, inflate_cells, width, height)
    new_rows = [[False] * width for _ in range(height)]
    for col, row in inflated:
        new_rows[row][col] = True
    return tuple(tuple(r) for r in new_rows)


def build_planner(scenario_raw: dict[str, Any] | None):
    """Return a :class:`GridAStarPlanner` if the scenario declares a grid.

    The scenario may put ``occupancy_grid`` at the top level or nested
    under ``world`` (when ``world`` is a mapping, not a Python import
    string). Returns ``None`` so callers can fall back to the
    straight-line planner.
    """

    if not scenario_raw:
        return None
    block = scenario_raw.get("occupancy_grid")
    if block is None:
        world = scenario_raw.get("world")
        if isinstance(world, dict):
            block = world.get("occupancy_grid")
    if not block:
        return None
    return GridAStarPlanner(OccupancyGrid.from_mapping(block))


__all__ = [
    "GridAStarPlanner",
    "OccupancyGrid",
    "PlannerError",
    "build_planner",
]
