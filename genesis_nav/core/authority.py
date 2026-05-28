"""Authority model for command arbitration."""

from __future__ import annotations

from enum import Enum


class AuthorityMode(str, Enum):
    AI = "ai"
    AUTONOMY = "autonomy"
    SAFETY = "safety"
    TELEOP = "teleop"


DEFAULT_AUTHORITY_PRIORITY: dict[AuthorityMode, int] = {
    AuthorityMode.AI: 10,
    AuthorityMode.AUTONOMY: 20,
    AuthorityMode.SAFETY: 30,
    AuthorityMode.TELEOP: 40,
}


def parse_authority(value: str | AuthorityMode) -> AuthorityMode:
    if isinstance(value, AuthorityMode):
        return value
    aliases = {"autonomous": AuthorityMode.AUTONOMY}
    if value in aliases:
        return aliases[value]
    try:
        return AuthorityMode(value)
    except ValueError as exc:
        known = ", ".join(mode.value for mode in AuthorityMode)
        raise ValueError(f"unknown authority '{value}', expected one of: {known}") from exc
