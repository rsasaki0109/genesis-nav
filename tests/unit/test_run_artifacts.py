"""Tests for env capture, QoS profile copy, and gnav replay validation."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from genesis_nav.cli.main import main
from genesis_nav.observability.env import collect_env_metadata, write_env_metadata


def _run_smoke(tmp_path: Path, extra: list[str] | None = None) -> Path:
    code = main(
        [
            "run",
            "examples/scenarios/smoke.yaml",
            "--fast",
            "--output-dir",
            str(tmp_path),
            *(extra or []),
        ]
    )
    assert code == 0
    return next(tmp_path.iterdir())


def test_env_metadata_collector_has_expected_keys() -> None:
    metadata = collect_env_metadata(
        scenario_id="smoke",
        seed=7,
        backend="fallback",
        mode="fast",
        ros_enabled=False,
        record_rosbag=False,
    )
    for key in (
        "scenario_id",
        "seed",
        "backend",
        "mode",
        "ros_enabled",
        "record_rosbag",
        "python_version",
        "platform",
        "hostname",
        "git",
        "ros_distro",
        "genesis_version",
    ):
        assert key in metadata
    assert metadata["scenario_id"] == "smoke"
    assert metadata["seed"] == 7
    assert metadata["backend"] == "fallback"
    assert isinstance(metadata["git"], dict)
    assert {"sha", "branch", "dirty"} <= metadata["git"].keys()


def test_write_env_metadata_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "env.json"
    payload = {"scenario_id": "x", "seed": 1}
    write_env_metadata(path, payload)
    assert json.loads(path.read_text(encoding="utf-8")) == payload


def test_run_writes_env_json(tmp_path: Path) -> None:
    run_dir = _run_smoke(tmp_path)
    env_path = run_dir / "env.json"
    assert env_path.exists()
    env = json.loads(env_path.read_text(encoding="utf-8"))
    assert env["scenario_id"] == "smoke"
    assert env["seed"] == 42
    assert env["mode"] == "fast"
    assert env["ros_enabled"] is False
    assert env["backend"] == "fallback"


def test_run_without_ros_skips_qos_copy(tmp_path: Path) -> None:
    run_dir = _run_smoke(tmp_path)
    assert not (run_dir / "qos_profile.yaml").exists()


def test_replay_strict_fails_without_env_json(tmp_path: Path) -> None:
    run_dir = _run_smoke(tmp_path)
    (run_dir / "env.json").unlink()
    assert main(["replay", str(run_dir)]) == 2


def test_replay_strict_fails_on_corrupt_events(tmp_path: Path) -> None:
    run_dir = _run_smoke(tmp_path)
    events_path = run_dir / "events.jsonl"
    events_path.write_text("this is not json\n", encoding="utf-8")
    assert main(["replay", str(run_dir)]) == 2


def test_replay_strict_fails_on_missing_scenario_started(tmp_path: Path) -> None:
    run_dir = _run_smoke(tmp_path)
    events_path = run_dir / "events.jsonl"
    lines = events_path.read_text(encoding="utf-8").splitlines(keepends=True)
    events_path.write_text("".join(lines[1:]), encoding="utf-8")
    assert main(["replay", str(run_dir)]) == 2


def test_replay_strict_fails_on_metrics_missing_keys(tmp_path: Path) -> None:
    run_dir = _run_smoke(tmp_path)
    (run_dir / "metrics.json").write_text("{}", encoding="utf-8")
    assert main(["replay", str(run_dir)]) == 2


def test_replay_print_events_streams_task_lifecycle(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    run_dir = _run_smoke(tmp_path)
    assert main(["replay", str(run_dir), "--print-events"]) == 0
    captured = capsys.readouterr().out
    for marker in (
        "SCENARIO_STARTED",
        "TASK_ASSIGNED",
        "TASK_STARTED",
        "TASK_SUCCEEDED",
        "SCENARIO_FINISHED",
    ):
        assert marker in captured
