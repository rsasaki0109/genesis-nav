"""QoS profile loading helpers.

The YAML loader is intentionally rclpy-free so that scenario tooling can read
the profile catalog without installing ROS 2. `build_qos_profile` lifts a
profile dict into an `rclpy.qos.QoSProfile` on demand.
"""

from __future__ import annotations

import fnmatch
from pathlib import Path
from typing import Any

import yaml


def load_qos_profiles(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if "topics" not in data:
        raise ValueError("QoS profile file requires a 'topics' mapping")
    return data


def resolve_qos_for(topic: str, profiles: dict[str, Any]) -> dict[str, Any]:
    """Return the most specific profile dict for the given topic.

    Patterns are evaluated using glob semantics (`/robot_*/cmd_vel`). Exact
    matches always win over wildcards.
    """

    topics = profiles.get("topics", {})
    if topic in topics:
        return dict(topics[topic])
    for pattern, spec in topics.items():
        if pattern == topic:
            return dict(spec)
        if any(symbol in pattern for symbol in "*?[") and fnmatch.fnmatchcase(topic, pattern):
            return dict(spec)
    return {"reliability": "reliable", "durability": "volatile"}


def build_qos_profile(spec: dict[str, Any], *, depth: int = 10):  # type: ignore[no-untyped-def]
    """Convert a profile spec dict into an `rclpy.qos.QoSProfile`.

    Imports rclpy lazily so the rest of the codebase keeps importing on systems
    without ROS 2 installed.
    """

    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    reliability = str(spec.get("reliability", "reliable")).lower()
    durability = str(spec.get("durability", "volatile")).lower()
    history_depth = int(spec.get("depth", depth))

    reliability_map = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }
    durability_map = {
        "volatile": DurabilityPolicy.VOLATILE,
        "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
    }
    if reliability not in reliability_map:
        raise ValueError(f"unknown reliability '{reliability}'")
    if durability not in durability_map:
        raise ValueError(f"unknown durability '{durability}'")

    return QoSProfile(
        depth=history_depth,
        reliability=reliability_map[reliability],
        durability=durability_map[durability],
    )


def qos_profile_for(topic: str, profiles: dict[str, Any], *, depth: int = 10):  # type: ignore[no-untyped-def]
    """Resolve and build the rclpy QoSProfile for a topic in one call."""

    return build_qos_profile(resolve_qos_for(topic, profiles), depth=depth)
