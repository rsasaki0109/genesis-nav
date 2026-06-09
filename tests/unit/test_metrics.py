"""Tests for metrics snapshot helpers."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from genesis_nav.cli.main import main
from genesis_nav.observability.metrics import (
    MetricsSnapshot,
    wilson_success_rate_ci,
)


def test_wilson_success_rate_ci_none_for_single_task() -> None:
    assert wilson_success_rate_ci(1, 1) is None
    assert wilson_success_rate_ci(0, 1) is None


def test_wilson_success_rate_ci_none_for_zero_tasks() -> None:
    assert wilson_success_rate_ci(0, 0) is None


def test_wilson_success_rate_ci_rejects_invalid_counts() -> None:
    with pytest.raises(ValueError, match="succeeded must be"):
        wilson_success_rate_ci(3, 2)


def test_wilson_success_rate_ci_known_two_of_two() -> None:
    ci = wilson_success_rate_ci(2, 2)
    assert ci is not None
    assert ci["confidence"] == 0.95
    assert ci["low"] == pytest.approx(0.342, rel=1e-2)
    assert ci["high"] == 1.0


def test_wilson_success_rate_ci_known_one_of_two() -> None:
    ci = wilson_success_rate_ci(1, 2)
    assert ci is not None
    assert ci["low"] == pytest.approx(0.094, rel=1e-2)
    assert ci["high"] == pytest.approx(0.906, rel=1e-2)


def test_metrics_snapshot_omits_null_success_rate_ci() -> None:
    snapshot = MetricsSnapshot(
        scenario_id="x",
        seed=1,
        agent_count=1,
        task_count=1,
        success_rate=1.0,
        task_succeeded_count=1,
        task_failed_count=0,
    )
    assert "success_rate_ci" not in snapshot.to_dict()


def test_run_single_task_metrics_omit_success_rate_ci(tmp_path: Path) -> None:
    rc = main(
        [
            "run",
            "examples/scenarios/smoke.yaml",
            "--fast",
            "--output-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    run_dir = next(tmp_path.iterdir())
    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["task_count"] == 1
    assert "success_rate_ci" not in metrics


def test_run_multi_task_metrics_include_success_rate_ci(tmp_path: Path) -> None:
    rc = main(
        [
            "run",
            "examples/scenarios/charging_dock.yaml",
            "--fast",
            "--output-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    run_dir = next(tmp_path.iterdir())
    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["task_count"] == 2
    assert metrics["task_succeeded_count"] == 2
    ci = metrics["success_rate_ci"]
    assert ci["confidence"] == 0.95
    assert 0.0 <= ci["low"] <= ci["high"] <= 1.0
    report = (run_dir / "report.md").read_text(encoding="utf-8")
    assert "success_rate_ci_95" in report
