"""Behavior state machine for the navigation MVP.

Behavior states are runtime-internal and orthogonal to :class:`TaskStatus`.
``TaskStatus`` answers "what is the status of this task?"; ``BehaviorState``
answers "what is the agent doing right now in service of that task?".

Transitions emitted by the runtime appear in ``events.jsonl`` as
``BEHAVIOR_STATE_CHANGED`` records so replay tooling can reconstruct the
exact behavior trajectory.
"""

from __future__ import annotations

from enum import Enum


class BehaviorState(str, Enum):
    IDLE = "idle"
    ASSIGNED = "assigned"
    PLANNING = "planning"
    RESERVING = "reserving"
    EXECUTING = "executing"
    RECOVERING = "recovering"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


# Allowed transitions. The runtime should never publish an event that is not
# in this map; doing so would mean the state machine drifted from the docs.
_ALLOWED: dict[BehaviorState, frozenset[BehaviorState]] = {
    BehaviorState.IDLE: frozenset({BehaviorState.ASSIGNED}),
    BehaviorState.ASSIGNED: frozenset(
        {BehaviorState.PLANNING, BehaviorState.FAILED}
    ),
    BehaviorState.PLANNING: frozenset(
        {BehaviorState.RESERVING, BehaviorState.EXECUTING, BehaviorState.FAILED}
    ),
    BehaviorState.RESERVING: frozenset(
        {BehaviorState.EXECUTING, BehaviorState.FAILED}
    ),
    BehaviorState.EXECUTING: frozenset(
        {
            BehaviorState.PLANNING,
            BehaviorState.RECOVERING,
            BehaviorState.SUCCEEDED,
            BehaviorState.FAILED,
        }
    ),
    BehaviorState.RECOVERING: frozenset(
        {BehaviorState.EXECUTING, BehaviorState.FAILED}
    ),
    BehaviorState.SUCCEEDED: frozenset({BehaviorState.IDLE}),
    BehaviorState.FAILED: frozenset({BehaviorState.IDLE}),
}


def can_transition(src: BehaviorState, dst: BehaviorState) -> bool:
    return dst in _ALLOWED.get(src, frozenset())


__all__ = ["BehaviorState", "can_transition"]
