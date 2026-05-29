"""Per-agent hardware/runtime diagnostics.

A read-only health surface over the runtime: it folds emergency-stop, fall,
task-failure, and adapter command-staleness (the real-robot watchdog's
consumer) into per-agent diagnostic levels, ordered OK < WARN < ERROR like
ROS 2 `diagnostic_msgs`. Adapters that do not expose a signal simply do not
contribute to it (duck-typed, mirroring `_poll_safety_signals`).

The collector is a pure function so it is unit-testable without a runtime;
`Runtime.diagnostics()` wraps it, and the runtime can emit a periodic
`DIAGNOSTICS` event so a replay reconstructs the health timeline.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Iterable, Mapping


class DiagnosticLevel(IntEnum):
    OK = 0
    WARN = 1
    ERROR = 2


@dataclass(frozen=True)
class AgentDiagnostic:
    agent_id: str
    level: DiagnosticLevel
    behavior_state: str
    messages: tuple[str, ...]
    command_age_sec: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "agent_id": self.agent_id,
            "level": self.level.name,
            "behavior_state": self.behavior_state,
            "messages": list(self.messages),
            "command_age_sec": self.command_age_sec,
        }


@dataclass(frozen=True)
class DiagnosticsReport:
    level: DiagnosticLevel
    agents: tuple[AgentDiagnostic, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "level": self.level.name,
            "agents": [a.to_dict() for a in self.agents],
        }


def _agent_diagnostic(state: Any, adapter: Any) -> AgentDiagnostic:
    level = DiagnosticLevel.OK
    messages: list[str] = []

    def raise_to(target: DiagnosticLevel, message: str) -> None:
        nonlocal level
        level = max(level, target, key=int)
        messages.append(message)

    if getattr(state, "emergency_stopped", False):
        raise_to(DiagnosticLevel.ERROR, "emergency_stopped")
    if getattr(state, "fall_detected", False):
        raise_to(DiagnosticLevel.ERROR, "fall_detected")
    behavior = getattr(state, "behavior_state", None)
    behavior_name = getattr(behavior, "value", str(behavior))
    if behavior_name == "failed":
        raise_to(DiagnosticLevel.ERROR, "task_failed")

    command_age: float | None = None
    if adapter is not None:
        seconds_since = getattr(adapter, "seconds_since_command", None)
        if callable(seconds_since):
            command_age = seconds_since()
        watchdog = getattr(adapter, "watchdog_expired", None)
        if callable(watchdog) and watchdog():
            raise_to(DiagnosticLevel.WARN, "command_watchdog_expired")

    return AgentDiagnostic(
        agent_id=getattr(state, "agent_id", ""),
        level=level,
        behavior_state=behavior_name,
        messages=tuple(messages),
        command_age_sec=command_age,
    )


def collect_diagnostics(
    states: Iterable[Any],
    adapters: Mapping[str, Any],
) -> DiagnosticsReport:
    """Build a `DiagnosticsReport` from agent states + their adapters."""

    agents = tuple(
        _agent_diagnostic(state, adapters.get(getattr(state, "agent_id", "")))
        for state in states
    )
    overall = max((a.level for a in agents), default=DiagnosticLevel.OK)
    return DiagnosticsReport(level=overall, agents=agents)


__all__ = [
    "AgentDiagnostic",
    "DiagnosticLevel",
    "DiagnosticsReport",
    "collect_diagnostics",
]
