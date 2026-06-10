"""Tests for the benchmark expectation, report, and gnav bench --run harness."""

from __future__ import annotations

import json
from pathlib import Path

from genesis_nav.benchmarks.report import (
    BenchmarkExpectation,
    BenchmarkScenarioResult,
    BenchmarkSuiteReport,
    discover_scenarios,
    is_integration_scenario,
)
from genesis_nav.benchmarks.runner import append_benchmark_report, load_jsonl_events
from genesis_nav.cli.main import main
from genesis_nav.observability.events import BENCHMARK_REPORT


def test_expectation_min_passes_when_metric_meets_threshold() -> None:
    expectation = BenchmarkExpectation(raw={"success_rate_min": 1.0})
    assert expectation.evaluate({"success_rate": 1.0}) == []


def test_expectation_min_fails_below_threshold() -> None:
    expectation = BenchmarkExpectation(raw={"success_rate_min": 1.0})
    failures = expectation.evaluate({"success_rate": 0.5})
    assert len(failures) == 1
    assert "success_rate" in failures[0]
    assert "0.5" in failures[0]


def test_expectation_replan_and_obstacle_min_keys() -> None:
    expectation = BenchmarkExpectation(
        raw={"replan_count_min": 1, "obstacle_event_count_min": 1}
    )
    assert expectation.evaluate({"replan_count": 1, "obstacle_event_count": 2}) == []
    failures = expectation.evaluate({"replan_count": 0, "obstacle_event_count": 0})
    assert len(failures) == 2


def test_expectation_watchdog_stop_min_and_max_keys() -> None:
    min_expectation = BenchmarkExpectation(raw={"watchdog_stop_count_min": 1})
    assert min_expectation.evaluate({"watchdog_stop_count": 1}) == []
    assert len(min_expectation.evaluate({"watchdog_stop_count": 0})) == 1

    max_expectation = BenchmarkExpectation(raw={"watchdog_stop_count_max": 0})
    assert max_expectation.evaluate({"watchdog_stop_count": 0}) == []
    assert len(max_expectation.evaluate({"watchdog_stop_count": 2})) == 1


def test_expectation_collision_and_near_miss_min_keys() -> None:
    expectation = BenchmarkExpectation(
        raw={"near_miss_count_min": 1, "collision_count_max": 0}
    )
    assert expectation.evaluate({"near_miss_count": 1, "collision_count": 0}) == []
    failures = expectation.evaluate({"near_miss_count": 0, "collision_count": 2})
    assert len(failures) == 2


def test_expectation_max_fails_above_threshold() -> None:
    expectation = BenchmarkExpectation(raw={"command_rejection_count_max": 0})
    failures = expectation.evaluate({"command_rejection_count": 3})
    assert len(failures) == 1
    assert "command_rejection_count" in failures[0]


def test_expectation_reports_unknown_keys() -> None:
    expectation = BenchmarkExpectation(raw={"made_up_metric_min": 1})
    failures = expectation.evaluate({})
    assert len(failures) == 1
    assert "made_up_metric_min" in failures[0]


def test_expectation_reports_missing_metric() -> None:
    expectation = BenchmarkExpectation(raw={"sim_steps_min": 100})
    failures = expectation.evaluate({})
    assert len(failures) == 1
    assert "sim_steps" in failures[0]
    assert "missing" in failures[0]


def test_expectation_from_scenario_raw_handles_missing_block() -> None:
    expectation = BenchmarkExpectation.from_scenario_raw({})
    assert expectation.raw == {}
    assert expectation.evaluate({"success_rate": 1.0}) == []


def test_is_integration_scenario_flag() -> None:
    assert is_integration_scenario({"benchmark": {"integration": True}}) is True
    assert is_integration_scenario({"benchmark": {"integration": False}}) is False
    assert is_integration_scenario({"benchmark": {}}) is False
    assert is_integration_scenario({}) is False


def test_bench_run_skips_integration_scenario_by_default(tmp_path: Path) -> None:
    report_path = tmp_path / "report.json"
    rc = main(
        [
            "bench",
            "--run",
            "benchmarks/nav2_integration",
            "--output-dir",
            str(tmp_path / "_runs"),
            "--report",
            str(report_path),
        ]
    )
    # All scenarios are integration-only, so nothing runs, nothing fails.
    assert rc == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["total"] == 0
    assert report["passed"] == 0
    assert report["skipped_count"] == 1
    assert report["skipped"][0]["scenario_id"] == "bench_nav2_integration_single_agent"
    assert "integration" in report["skipped"][0]["reason"]


def test_discover_scenarios_returns_sorted_yaml_files() -> None:
    found = discover_scenarios(Path("benchmarks/nav_basic"))
    assert found
    assert all(p.suffix == ".yaml" for p in found)
    assert found == sorted(found)


def test_bench_run_nav_basic_passes(tmp_path: Path) -> None:
    output_dir = tmp_path / "_runs"
    report_path = tmp_path / "report.json"
    rc = main(
        [
            "bench",
            "--run",
            "benchmarks/nav_basic",
            "--output-dir",
            str(output_dir),
            "--report",
            str(report_path),
        ]
    )
    assert rc == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["benchmark_suite"] == "nav_basic"
    assert report["passed"] == report["total"] >= 1
    for scenario in report["scenarios"]:
        assert scenario["passed"] is True
        assert scenario["failures"] == []
        assert Path(scenario["run_dir"]).is_dir()
        assert scenario["metrics"]["success_rate"] >= 1.0
        events = load_jsonl_events(Path(scenario["run_dir"]) / "events.jsonl")
        assert events[-1]["event"] == BENCHMARK_REPORT
        assert events[-2]["event"] == "SCENARIO_FINISHED"
        assert events[-1]["data"]["passed"] is True
        assert events[-1]["data"]["report_path"] == str(report_path)
        assert events[-1]["data"]["benchmark_suite"] == "nav_basic"


def test_bench_run_multi_agent_passes(tmp_path: Path) -> None:
    output_dir = tmp_path / "_runs"
    report_path = tmp_path / "report.json"
    rc = main(
        [
            "bench",
            "--run",
            "benchmarks/multi_agent",
            "--output-dir",
            str(output_dir),
            "--report",
            str(report_path),
        ]
    )
    assert rc == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["passed"] == 5
    assert report["failed"] == 0
    total_succeeded = sum(
        s["metrics"]["task_succeeded_count"] for s in report["scenarios"]
    )
    assert total_succeeded >= 10


def test_bench_run_humanoid_passes(tmp_path: Path) -> None:
    output_dir = tmp_path / "_runs"
    report_path = tmp_path / "report.json"
    rc = main(
        [
            "bench",
            "--run",
            "benchmarks/humanoid",
            "--output-dir",
            str(output_dir),
            "--report",
            str(report_path),
        ]
    )
    assert rc == 0


def test_bench_run_fails_when_expectation_violated(tmp_path: Path) -> None:
    suite_dir = tmp_path / "failing_suite"
    suite_dir.mkdir()
    failing = suite_dir / "must_fail.yaml"
    failing.write_text(
        Path("benchmarks/nav_basic/single_agent_empty.yaml").read_text(
            encoding="utf-8"
        ),
        encoding="utf-8",
    )
    raw = failing.read_text(encoding="utf-8")
    raw = raw.replace("success_rate_min: 1.0", "success_rate_min: 2.0")
    failing.write_text(raw, encoding="utf-8")

    output_dir = tmp_path / "_runs"
    report_path = tmp_path / "report.json"
    rc = main(
        [
            "bench",
            "--run",
            str(suite_dir),
            "--output-dir",
            str(output_dir),
            "--report",
            str(report_path),
        ]
    )
    assert rc == 1
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["failed"] == 1
    failed = report["scenarios"][0]
    assert failed["passed"] is False
    assert failed["failures"]
    assert Path(failed["run_dir"]).is_dir()
    events = load_jsonl_events(Path(failed["run_dir"]) / "events.jsonl")
    assert events[-1]["event"] == BENCHMARK_REPORT
    assert events[-1]["data"]["passed"] is False
    assert events[-1]["data"]["failures"]
    assert main(["replay", failed["run_dir"]]) == 0


def test_bench_validate_single_scenario_still_works(
    capsys, tmp_path: Path
) -> None:
    rc = main(["bench", "benchmarks/nav_basic/single_agent_empty.yaml"])
    assert rc == 0
    captured = capsys.readouterr()
    payload = json.loads(captured.out)
    assert payload["scenario_id"] == "bench_nav_basic_single_agent_empty"
    assert payload["agents"] == 1
    assert payload["tasks"] == 1


def test_benchmark_suite_report_aggregates_passed_failed() -> None:
    results = [
        BenchmarkScenarioResult(
            scenario_id="ok",
            scenario_path="x.yaml",
            seed=1,
            run_dir="r1",
            passed=True,
        ),
        BenchmarkScenarioResult(
            scenario_id="bad",
            scenario_path="y.yaml",
            seed=1,
            run_dir="r2",
            passed=False,
            failures=["nope"],
        ),
    ]
    report = BenchmarkSuiteReport(
        benchmark_suite="mixed",
        ran_at="2026-05-28T00-00-00Z",
        scenarios=results,
    )
    assert report.total == 2
    assert report.passed == 1
    assert report.failed == 1
    d = report.to_dict()
    assert d["total"] == 2
    assert d["passed"] == 1
    assert d["failed"] == 1
    assert d["scenarios"][1]["failures"] == ["nope"]


def test_append_benchmark_report_requires_scenario_finished(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    events_path = run_dir / "events.jsonl"
    events_path.write_text(
        json.dumps(
            {
                "ts": 1.0,
                "episode_id": "ep1",
                "event": "SCENARIO_STARTED",
                "agent_id": "",
                "task_id": "",
                "data": {},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    report_path = tmp_path / "suite_report.json"
    append_benchmark_report(
        run_dir,
        benchmark_suite="test_suite",
        passed=True,
        failures=[],
        report_path=report_path,
    )
    events = load_jsonl_events(events_path)
    assert len(events) == 1
    assert events[0]["event"] == "SCENARIO_STARTED"
