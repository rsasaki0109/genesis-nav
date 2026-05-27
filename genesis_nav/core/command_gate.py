"""Safety and authority gate for actuator commands."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from time import monotonic

from genesis_nav.core.authority import (
    DEFAULT_AUTHORITY_PRIORITY,
    AuthorityMode,
    parse_authority,
)


@dataclass(frozen=True)
class RuntimeCommand:
    agent_id: str
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    authority: AuthorityMode = AuthorityMode.AUTONOMY
    source: str = "runtime"
    issued_at_sec: float = 0.0
    ttl_ms: int = 200
    requester_id: str = ""
    trace_id: str = ""

    @classmethod
    def stop(
        cls,
        agent_id: str,
        *,
        authority: AuthorityMode = AuthorityMode.SAFETY,
        issued_at_sec: float = 0.0,
        source: str = "safety",
    ) -> "RuntimeCommand":
        return cls(
            agent_id=agent_id,
            authority=authority,
            source=source,
            issued_at_sec=issued_at_sec,
        )


@dataclass(frozen=True)
class CommandGateConfig:
    max_linear_x: float = 1.0
    max_linear_y: float = 1.0
    max_angular_z: float = 1.5
    default_ttl_ms: int = 200
    allow_ai_velocity_commands: bool = False


@dataclass(frozen=True)
class CommandDecision:
    accepted: bool
    reason: str
    command: RuntimeCommand | None = None


@dataclass(frozen=True)
class _AuthorityLock:
    authority: AuthorityMode
    expires_at_sec: float
    reason: str


class CommandGate:
    """Applies the minimum v0.1 safety rules before actuator commands execute."""

    def __init__(
        self,
        config: CommandGateConfig | None = None,
        priorities: dict[AuthorityMode, int] | None = None,
    ) -> None:
        self.config = config or CommandGateConfig()
        self._priorities = priorities or DEFAULT_AUTHORITY_PRIORITY
        self._emergency_stopped: set[str] = set()
        self._locks: dict[str, _AuthorityLock] = {}

    def set_emergency_stop(self, agent_id: str, stopped: bool = True) -> None:
        if stopped:
            self._emergency_stopped.add(agent_id)
        else:
            self._emergency_stopped.discard(agent_id)

    def lock_authority(
        self,
        agent_id: str,
        authority: str | AuthorityMode,
        *,
        ttl_sec: float,
        reason: str,
        now_sec: float | None = None,
    ) -> None:
        if ttl_sec <= 0:
            raise ValueError("ttl_sec must be positive")
        now = monotonic() if now_sec is None else now_sec
        self._locks[agent_id] = _AuthorityLock(
            authority=parse_authority(authority),
            expires_at_sec=now + ttl_sec,
            reason=reason,
        )

    def clear_authority_lock(self, agent_id: str) -> None:
        self._locks.pop(agent_id, None)

    def evaluate(self, command: RuntimeCommand, *, now_sec: float | None = None) -> CommandDecision:
        now = monotonic() if now_sec is None else now_sec
        authority = parse_authority(command.authority)
        command = RuntimeCommand(
            agent_id=command.agent_id,
            linear_x=command.linear_x,
            linear_y=command.linear_y,
            angular_z=command.angular_z,
            authority=authority,
            source=command.source,
            issued_at_sec=command.issued_at_sec,
            ttl_ms=command.ttl_ms or self.config.default_ttl_ms,
            requester_id=command.requester_id,
            trace_id=command.trace_id,
        )

        if authority is AuthorityMode.AI and not self.config.allow_ai_velocity_commands:
            return CommandDecision(False, "AI agents may submit tasks but cannot publish velocity commands")

        if command.agent_id in self._emergency_stopped:
            return CommandDecision(False, "agent is emergency stopped")

        if not self._is_fresh(command, now):
            return CommandDecision(False, "command is stale")

        if not all(isfinite(value) for value in (command.linear_x, command.linear_y, command.angular_z)):
            return CommandDecision(False, "command contains non-finite velocity")

        lock = self._active_lock(command.agent_id, now)
        if lock and self._priority(authority) < self._priority(lock.authority):
            return CommandDecision(False, f"authority locked by {lock.authority.value}: {lock.reason}")

        limited = self._limit(command)
        reason = "accepted"
        if limited != command:
            reason = "accepted with velocity limits"
        return CommandDecision(True, reason, limited)

    def _is_fresh(self, command: RuntimeCommand, now_sec: float) -> bool:
        issued_at = command.issued_at_sec
        if issued_at <= 0:
            return False
        ttl_sec = (command.ttl_ms or self.config.default_ttl_ms) / 1000.0
        return now_sec - issued_at <= ttl_sec

    def _active_lock(self, agent_id: str, now_sec: float) -> _AuthorityLock | None:
        lock = self._locks.get(agent_id)
        if lock is None:
            return None
        if lock.expires_at_sec <= now_sec:
            self._locks.pop(agent_id, None)
            return None
        return lock

    def _priority(self, authority: AuthorityMode) -> int:
        return self._priorities[authority]

    def _limit(self, command: RuntimeCommand) -> RuntimeCommand:
        return RuntimeCommand(
            agent_id=command.agent_id,
            linear_x=_clamp(command.linear_x, self.config.max_linear_x),
            linear_y=_clamp(command.linear_y, self.config.max_linear_y),
            angular_z=_clamp(command.angular_z, self.config.max_angular_z),
            authority=command.authority,
            source=command.source,
            issued_at_sec=command.issued_at_sec,
            ttl_ms=command.ttl_ms,
            requester_id=command.requester_id,
            trace_id=command.trace_id,
        )


def _clamp(value: float, absolute_limit: float) -> float:
    limit = abs(absolute_limit)
    return max(-limit, min(limit, value))
