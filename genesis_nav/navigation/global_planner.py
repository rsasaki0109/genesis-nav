"""Minimal planner placeholder for v0.1 smoke scenarios."""

from __future__ import annotations


class StraightLinePlanner:
    def plan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        return [start, goal]
