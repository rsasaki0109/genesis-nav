"""Minimal local controller used by the v0.1 navigation MVP."""

from __future__ import annotations

import math
from dataclasses import dataclass

from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import RuntimeCommand


@dataclass(frozen=True)
class LocalControllerConfig:
    max_linear_x: float = 0.6
    max_linear_y: float = 0.6
    max_angular_z: float = 1.2
    linear_gain: float = 1.0
    angular_gain: float = 2.0
    goal_tolerance_m: float = 0.1
    heading_tolerance_rad: float = 0.1


def _wrap_angle(theta: float) -> float:
    return ((theta + math.pi) % (2.0 * math.pi)) - math.pi


def _clamp(value: float, absolute_limit: float) -> float:
    limit = abs(absolute_limit)
    return max(-limit, min(limit, value))


class SimpleLocalController:
    """Heading-then-drive controller for differential-drive bases."""

    def __init__(self, config: LocalControllerConfig | None = None) -> None:
        self.config = config or LocalControllerConfig()

    def at_goal(self, pose: tuple[float, float, float], goal: tuple[float, float, float]) -> bool:
        return math.hypot(goal[0] - pose[0], goal[1] - pose[1]) <= self.config.goal_tolerance_m

    def compute(
        self,
        agent_id: str,
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
        *,
        issued_at_sec: float,
        ttl_ms: int = 200,
        authority: AuthorityMode = AuthorityMode.AUTONOMY,
        source: str = "navigation",
    ) -> RuntimeCommand:
        cfg = self.config
        dx = goal[0] - pose[0]
        dy = goal[1] - pose[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = _wrap_angle(target_heading - pose[2])

        if distance <= cfg.goal_tolerance_m:
            linear, angular = 0.0, 0.0
        elif abs(heading_error) > cfg.heading_tolerance_rad:
            linear = 0.0
            angular = _clamp(cfg.angular_gain * heading_error, cfg.max_angular_z)
        else:
            linear = min(cfg.linear_gain * distance, cfg.max_linear_x)
            angular = _clamp(cfg.angular_gain * heading_error, cfg.max_angular_z)

        return RuntimeCommand(
            agent_id=agent_id,
            linear_x=linear,
            angular_z=angular,
            authority=authority,
            source=source,
            issued_at_sec=issued_at_sec,
            ttl_ms=ttl_ms,
        )


class HolonomicLocalController:
    """Direct holonomic controller that strafes toward the goal in body frame."""

    def __init__(self, config: LocalControllerConfig | None = None) -> None:
        self.config = config or LocalControllerConfig()

    def at_goal(self, pose: tuple[float, float, float], goal: tuple[float, float, float]) -> bool:
        return math.hypot(goal[0] - pose[0], goal[1] - pose[1]) <= self.config.goal_tolerance_m

    def compute(
        self,
        agent_id: str,
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
        *,
        issued_at_sec: float,
        ttl_ms: int = 200,
        authority: AuthorityMode = AuthorityMode.AUTONOMY,
        source: str = "navigation",
    ) -> RuntimeCommand:
        cfg = self.config
        dx = goal[0] - pose[0]
        dy = goal[1] - pose[1]
        distance = math.hypot(dx, dy)
        if distance <= cfg.goal_tolerance_m:
            heading_error = _wrap_angle(goal[2] - pose[2])
            if abs(heading_error) > cfg.heading_tolerance_rad:
                angular = _clamp(cfg.angular_gain * heading_error, cfg.max_angular_z)
                return RuntimeCommand(
                    agent_id=agent_id,
                    angular_z=angular,
                    authority=authority,
                    source=source,
                    issued_at_sec=issued_at_sec,
                    ttl_ms=ttl_ms,
                )
            return RuntimeCommand(
                agent_id=agent_id,
                authority=authority,
                source=source,
                issued_at_sec=issued_at_sec,
                ttl_ms=ttl_ms,
            )

        speed = min(cfg.linear_gain * distance, cfg.max_linear_x)
        vx_world = speed * dx / distance
        vy_world = speed * dy / distance
        cos_yaw = math.cos(pose[2])
        sin_yaw = math.sin(pose[2])
        linear_x = vx_world * cos_yaw + vy_world * sin_yaw
        linear_y = -vx_world * sin_yaw + vy_world * cos_yaw
        max_speed = min(cfg.max_linear_x, cfg.max_linear_y)
        magnitude = math.hypot(linear_x, linear_y)
        if magnitude > max_speed > 0:
            scale = max_speed / magnitude
            linear_x *= scale
            linear_y *= scale

        return RuntimeCommand(
            agent_id=agent_id,
            linear_x=linear_x,
            linear_y=linear_y,
            authority=authority,
            source=source,
            issued_at_sec=issued_at_sec,
            ttl_ms=ttl_ms,
        )


__all__ = ["HolonomicLocalController", "LocalControllerConfig", "SimpleLocalController"]
