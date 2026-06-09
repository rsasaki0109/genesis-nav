"""Tests for rosbag profile resolution (rclpy-free)."""

from __future__ import annotations

from pathlib import Path

import pytest

from genesis_nav.ros.bag_writer import (
    load_rosbag_profile,
    publishable_topics,
    resolve_record_topics,
    topic_type_for,
    write_recording_skipped_marker,
)


def test_load_rosbag_profile_reads_topics() -> None:
    patterns = load_rosbag_profile("configs/rosbag/minimal.yaml")
    assert "/clock" in patterns
    assert "/robot_001/state" in patterns


def test_resolve_record_topics_expands_wildcards() -> None:
    patterns = [
        "/clock",
        "/genesis_nav/events",
        "/robot_*/state",
        "/robot_*/odom",
    ]
    topics = resolve_record_topics(patterns, ["/robot_001", "/robot_002"])
    assert "/clock" in topics
    assert "/robot_001/state" in topics
    assert "/robot_002/odom" in topics
    assert "/robot_001/cmd_vel" not in topics


def test_publishable_topics_includes_bridge_outputs() -> None:
    topics = publishable_topics(["/robot_001"])
    assert "/robot_001/state" in topics
    assert "/genesis_nav/diagnostics" in topics


def test_topic_type_for_known_topics() -> None:
    assert topic_type_for("/clock") == "rosgraph_msgs/msg/Clock"
    assert topic_type_for("/robot_001/odom") == "nav_msgs/msg/Odometry"
    assert topic_type_for("/robot_001/state") == "genesis_nav_msgs/msg/AgentState"


def test_topic_type_for_unknown_topic_raises() -> None:
    with pytest.raises(ValueError, match="no message type mapping"):
        topic_type_for("/unknown/topic")


def test_write_recording_skipped_marker(tmp_path: Path) -> None:
    write_recording_skipped_marker(tmp_path, reason="needs --ros")
    marker = tmp_path / "rosbag" / "RECORDING_SKIPPED"
    assert marker.is_file()
    assert "needs --ros" in marker.read_text(encoding="utf-8")
