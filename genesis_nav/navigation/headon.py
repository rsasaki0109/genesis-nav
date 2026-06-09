"""Head-on conflict detection and lateral detour planning.

Crossing conflicts are handled by yield (stop-and-wait). Head-on conflicts on a
shared corridor need a lateral reroute because the lower-priority agent stopping
in place blocks the higher-priority agent's path.
"""

from __future__ import annotations

import math


def wrap_angle(theta: float) -> float:
    return ((theta + math.pi) % (2.0 * math.pi)) - math.pi


def heading_toward(
    pose: tuple[float, float, float], target_xy: tuple[float, float]
) -> float:
    return math.atan2(target_xy[1] - pose[1], target_xy[0] - pose[0])


def perpendicular_distance(
    pose: tuple[float, float, float],
    goal: tuple[float, float, float],
    point: tuple[float, float, float],
) -> float:
    """Planar distance from ``point`` to the infinite line pose → goal."""

    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return math.hypot(point[0] - pose[0], point[1] - pose[1])
    px = point[0] - pose[0]
    py = point[1] - pose[1]
    return abs(px * dy - py * dx) / length


def is_headon_conflict(
    pose: tuple[float, float, float],
    goal: tuple[float, float, float],
    other_pose: tuple[float, float, float],
    other_goal: tuple[float, float, float],
    *,
    max_lateral_m: float = 0.5,
    heading_tolerance_rad: float = math.pi / 4,
) -> bool:
    """True when two agents are approaching each other on the same corridor."""

    my_heading = heading_toward(pose, (goal[0], goal[1]))
    other_heading = heading_toward(other_pose, (other_goal[0], other_goal[1]))

    # Headings must be roughly opposite (≥ ~120° apart).
    if math.cos(my_heading - other_heading) > -0.5:
        return False

    to_other = heading_toward(pose, (other_pose[0], other_pose[1]))
    if abs(wrap_angle(to_other - my_heading)) > heading_tolerance_rad:
        return False

    to_me = heading_toward(other_pose, (pose[0], pose[1]))
    if abs(wrap_angle(to_me - other_heading)) > heading_tolerance_rad:
        return False

    if perpendicular_distance(pose, goal, other_pose) > max_lateral_m:
        return False

    return True


def lateral_detour_waypoints(
    pose: tuple[float, float, float],
    goal: tuple[float, float, float],
    other_pose: tuple[float, float, float],
    offset_m: float,
) -> list[tuple[float, float, float]]:
    """Return waypoints that sidestep ``other_pose`` before continuing to ``goal``."""

    heading = heading_toward(pose, (goal[0], goal[1]))
    perp_x = -math.sin(heading)
    perp_y = math.cos(heading)
    rel_x = other_pose[0] - pose[0]
    rel_y = other_pose[1] - pose[1]
    side = -1.0 if rel_x * perp_x + rel_y * perp_y >= 0.0 else 1.0

    mid_x = (pose[0] + other_pose[0]) * 0.5
    mid_y = (pose[1] + other_pose[1]) * 0.5
    detour_x = mid_x + side * offset_m * perp_x
    detour_y = mid_y + side * offset_m * perp_y
    return [(detour_x, detour_y, heading), goal]


__all__ = [
    "is_headon_conflict",
    "lateral_detour_waypoints",
    "perpendicular_distance",
]
