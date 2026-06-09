"""Tests for replay rosbag export helpers."""

from __future__ import annotations

from pathlib import Path

import pytest

from genesis_nav.ros.bag_writer import RosbagNotAvailableError
from genesis_nav.ros.replay_export import (
    default_qos_profile,
    default_rosbag_profile,
    prepare_bag_directory,
)


def test_default_rosbag_profile_prefers_run_dir(tmp_path: Path) -> None:
    stored = tmp_path / "rosbag_profile.yaml"
    stored.write_text("topics:\n  - /clock\n", encoding="utf-8")
    assert default_rosbag_profile(tmp_path) == stored


def test_default_rosbag_profile_honors_override(tmp_path: Path) -> None:
    override = tmp_path / "custom.yaml"
    override.write_text("topics:\n  - /clock\n", encoding="utf-8")
    assert default_rosbag_profile(tmp_path, override) == override


def test_default_qos_profile_prefers_run_dir(tmp_path: Path) -> None:
    stored = tmp_path / "qos_profile.yaml"
    stored.write_text("topics: {}\n", encoding="utf-8")
    assert default_qos_profile(tmp_path) == stored


def test_prepare_bag_directory_removes_existing_tree(tmp_path: Path) -> None:
    bag_dir = tmp_path / "rosbag"
    bag_dir.mkdir()
    (bag_dir / "RECORDING_SKIPPED").write_text("skipped\n", encoding="utf-8")
    prepare_bag_directory(bag_dir)
    assert not bag_dir.exists()


def test_replay_to_rosbag_exports_bag(tmp_path: Path) -> None:
    pytest.importorskip("rosbag2_py")
    pytest.importorskip("genesis_nav_msgs.msg")

    from genesis_nav.cli.main import main

    code = main(
        [
            "run",
            "examples/scenarios/smoke.yaml",
            "--fast",
            "--record",
            "--output-dir",
            str(tmp_path),
        ]
    )
    assert code == 0
    run_dir = next(tmp_path.iterdir())
    assert (run_dir / "rosbag" / "RECORDING_SKIPPED").is_file()

    code = main(["replay", str(run_dir), "--to-rosbag"])
    assert code == 0
    bag_dir = run_dir / "rosbag"
    assert (bag_dir / "metadata.yaml").is_file()
    assert any(path.suffix == ".db3" for path in bag_dir.iterdir())
    assert not (bag_dir / "RECORDING_SKIPPED").exists()


def test_replay_to_rosbag_without_ros_exits_nonzero(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from genesis_nav.cli.main import main

    main(
        [
            "run",
            "examples/scenarios/smoke.yaml",
            "--fast",
            "--output-dir",
            str(tmp_path),
        ]
    )
    run_dir = next(tmp_path.iterdir())

    class _BrokenRecorder:
        def __init__(self, *args, **kwargs) -> None:
            del args, kwargs
            raise RosbagNotAvailableError("missing rosbag2_py")

    monkeypatch.setattr("genesis_nav.ros.replay_export.RosbagRecorder", _BrokenRecorder)
    code = main(["replay", str(run_dir), "--to-rosbag"])

    assert code == 3
