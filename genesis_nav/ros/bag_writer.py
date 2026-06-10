"""rosbag2 recording helpers for the in-process ROS bridge.

All rosbag2 / rclpy imports stay lazy so the core package remains installable
without a sourced ROS 2 environment.
"""

from __future__ import annotations

import fnmatch
from pathlib import Path
from typing import Any, Sequence

import yaml

TOPIC_TYPE_BY_NAME: dict[str, str] = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/genesis_nav/events": "genesis_nav_msgs/msg/RuntimeEvent",
    "/genesis_nav/scenario_state": "genesis_nav_msgs/msg/ScenarioState",
    "/genesis_nav/fleet_state": "genesis_nav_msgs/msg/FleetState",
    "/genesis_nav/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
}


class RosbagNotAvailableError(RuntimeError):
    """Raised when rosbag recording is requested but rosbag2_py is missing."""


def load_rosbag_profile(path: str | Path) -> list[str]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    topics = data.get("topics")
    if not isinstance(topics, list) or not topics:
        raise ValueError("rosbag profile requires a non-empty 'topics' list")
    return [str(item) for item in topics]


def publishable_topics(agent_namespaces: Sequence[str]) -> set[str]:
    topics = {
        "/clock",
        "/tf",
        "/tf_static",
        "/genesis_nav/events",
        "/genesis_nav/scenario_state",
        "/genesis_nav/fleet_state",
        "/genesis_nav/diagnostics",
    }
    for namespace in agent_namespaces:
        ns = namespace.rstrip("/")
        topics.add(f"{ns}/state")
        topics.add(f"{ns}/odom")
    return topics


def resolve_record_topics(
    patterns: Sequence[str],
    agent_namespaces: Sequence[str],
) -> set[str]:
    """Expand profile patterns against topics the bridge actually publishes."""

    candidates = publishable_topics(agent_namespaces)
    selected: set[str] = set()
    for pattern in patterns:
        if any(symbol in pattern for symbol in "*?["):
            for topic in candidates:
                if fnmatch.fnmatchcase(topic, pattern):
                    selected.add(topic)
            continue
        if pattern in candidates:
            selected.add(pattern)
    return selected


def topic_type_for(topic: str) -> str:
    if topic in TOPIC_TYPE_BY_NAME:
        return TOPIC_TYPE_BY_NAME[topic]
    if topic.endswith("/odom"):
        return "nav_msgs/msg/Odometry"
    if topic.endswith("/state"):
        return "genesis_nav_msgs/msg/AgentState"
    raise ValueError(f"no message type mapping for rosbag topic '{topic}'")


class RosbagRecorder:
    """Write selected bridge topics to a rosbag2 sqlite3 bag at ``uri``."""

    def __init__(
        self,
        uri: Path,
        patterns: Sequence[str],
        agent_namespaces: Sequence[str],
    ) -> None:
        try:
            from rosbag2_py import (
                ConverterOptions,
                SequentialWriter,
                StorageOptions,
                TopicMetadata,
            )
            from rclpy.serialization import serialize_message
        except ImportError as exc:  # pragma: no cover - exercised without ROS
            raise RosbagNotAvailableError(
                "rosbag recording requires rosbag2_py and rclpy; "
                "source your ROS 2 installation and ensure rosbag2 is installed"
            ) from exc

        self._SequentialWriter = SequentialWriter
        self._StorageOptions = StorageOptions
        self._ConverterOptions = ConverterOptions
        self._TopicMetadata = TopicMetadata
        self._serialize_message = serialize_message
        self._topics = resolve_record_topics(patterns, agent_namespaces)
        self._registered: set[str] = set()
        self._next_topic_id = 0
        self._writer = SequentialWriter()
        self._writer.open(
            StorageOptions(uri=str(uri), storage_id="sqlite3"),
            ConverterOptions("", ""),
        )

    def close(self) -> None:
        self._writer = None  # type: ignore[assignment]

    def write(self, topic: str, msg: Any, sim_time_sec: float) -> None:
        if topic not in self._topics or self._writer is None:
            return
        if topic not in self._registered:
            self._writer.create_topic(
                self._TopicMetadata(
                    self._next_topic_id,
                    topic,
                    topic_type_for(topic),
                    "cdr",
                )
            )
            self._next_topic_id += 1
            self._registered.add(topic)
        timestamp_ns = max(0, int(round(float(sim_time_sec) * 1e9)))
        self._writer.write(
            topic,
            self._serialize_message(msg),
            timestamp_ns,
        )


def write_recording_skipped_marker(run_dir: Path, *, reason: str) -> None:
    bag_dir = run_dir / "rosbag"
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / "RECORDING_SKIPPED").write_text(reason.strip() + "\n", encoding="utf-8")


__all__ = [
    "RosbagNotAvailableError",
    "RosbagRecorder",
    "load_rosbag_profile",
    "publishable_topics",
    "resolve_record_topics",
    "topic_type_for",
    "write_recording_skipped_marker",
]
