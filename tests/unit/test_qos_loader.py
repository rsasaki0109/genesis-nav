from pathlib import Path

import pytest

from genesis_nav.ros.qos import load_qos_profiles, resolve_qos_for


def test_load_qos_profiles_requires_topics(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("other: 1\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_qos_profiles(bad)


def test_resolve_qos_exact_match_wins_over_wildcard() -> None:
    profiles = {
        "topics": {
            "/robot_*/cmd_vel": {"reliability": "best_effort"},
            "/robot_001/cmd_vel": {"reliability": "reliable"},
        }
    }
    assert resolve_qos_for("/robot_001/cmd_vel", profiles)["reliability"] == "reliable"


def test_resolve_qos_wildcard_match() -> None:
    profiles = {"topics": {"/robot_*/scan": {"reliability": "best_effort"}}}
    assert resolve_qos_for("/robot_002/scan", profiles)["reliability"] == "best_effort"


def test_resolve_qos_unknown_topic_uses_default() -> None:
    profiles = {"topics": {}}
    resolved = resolve_qos_for("/genesis_nav/scenario_state", profiles)
    assert resolved["reliability"] == "reliable"
    assert resolved["durability"] == "volatile"


def test_load_default_qos_profile_resolves_known_topics() -> None:
    profiles = load_qos_profiles("configs/qos/default.yaml")
    clock = resolve_qos_for("/clock", profiles)
    cmd_vel = resolve_qos_for("/robot_001/cmd_vel", profiles)
    assert clock["durability"] == "transient_local"
    assert cmd_vel["deadline_ms"] == 100
