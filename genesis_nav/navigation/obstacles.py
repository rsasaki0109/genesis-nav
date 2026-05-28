"""Dynamic obstacle sources for the navigation runtime.

Per the 2026-05-29 ADR "Dynamic obstacles and replanning extend the planner
contract", obstacles arrive as timestamped grid deltas. Each delta is recorded
as an ``OBSTACLE_CHANGED`` runtime event so a replay reconstructs the exact
obstacle timeline from the run directory alone.

v0.2 ships one source: ``ScriptedObstacleSource``, driven by the scenario so
runs stay deterministic. A live perception-backed source would implement the
same ``ObstacleSource`` protocol.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, Sequence, runtime_checkable


@dataclass(frozen=True)
class ObstacleDelta:
    """Cells to block at a given sim time. ``block`` holds ``(col, row)``."""

    at_sec: float
    block: tuple[tuple[int, int], ...]


@runtime_checkable
class ObstacleSource(Protocol):
    """Yields obstacle deltas that have become due by ``now_sec``."""

    def due(self, now_sec: float) -> list[ObstacleDelta]: ...


@dataclass
class ScriptedObstacleSource:
    """Deterministic source: replays scenario-defined deltas by sim time.

    ``due`` returns every not-yet-emitted delta whose ``at_sec <= now_sec`` in
    chronological order, so calling it once per tick with a monotonically
    increasing clock surfaces each delta exactly once.
    """

    deltas: tuple[ObstacleDelta, ...]
    _emitted: int = field(default=0)

    def __post_init__(self) -> None:
        ordered = tuple(sorted(self.deltas, key=lambda d: d.at_sec))
        object.__setattr__(self, "deltas", ordered)

    def due(self, now_sec: float) -> list[ObstacleDelta]:
        out: list[ObstacleDelta] = []
        while self._emitted < len(self.deltas) and self.deltas[self._emitted].at_sec <= now_sec:
            out.append(self.deltas[self._emitted])
            self._emitted += 1
        return out


def _coerce_cells(raw: Any) -> tuple[tuple[int, int], ...]:
    if not isinstance(raw, (list, tuple)):
        raise ValueError("dynamic_obstacles delta 'block' must be a list of [col, row]")
    cells: list[tuple[int, int]] = []
    for item in raw:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError("each blocked cell must be [col, row]")
        cells.append((int(item[0]), int(item[1])))
    return tuple(cells)


def build_obstacle_source(scenario_raw: dict[str, Any] | None) -> ScriptedObstacleSource | None:
    """Build a :class:`ScriptedObstacleSource` from a scenario, or ``None``.

    Scenario shape::

        dynamic_obstacles:
          events:
            - at_sec: 2.0
              block: [[3, 1], [3, 2]]
    """

    if not scenario_raw:
        return None
    block = scenario_raw.get("dynamic_obstacles")
    if not isinstance(block, dict):
        return None
    events = block.get("events")
    if not isinstance(events, list) or not events:
        return None
    deltas: list[ObstacleDelta] = []
    for ev in events:
        if not isinstance(ev, dict):
            raise ValueError("dynamic_obstacles.events entries must be mappings")
        deltas.append(
            ObstacleDelta(
                at_sec=float(ev.get("at_sec", 0.0)),
                block=_coerce_cells(ev.get("block", [])),
            )
        )
    return ScriptedObstacleSource(deltas=tuple(deltas))


__all__ = [
    "ObstacleDelta",
    "ObstacleSource",
    "ScriptedObstacleSource",
    "build_obstacle_source",
]
