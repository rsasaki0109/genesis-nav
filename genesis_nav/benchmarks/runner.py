"""Benchmark suite runner and post-run event emission."""

from __future__ import annotations

import json
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

from genesis_nav.benchmarks.report import (
    BenchmarkExpectation,
    BenchmarkScenarioResult,
    BenchmarkSuiteReport,
    discover_scenarios,
    is_integration_scenario,
    now_iso,
)
from genesis_nav.benchmarks.scenario import Scenario, load_scenario
from genesis_nav.observability.events import (
    BENCHMARK_REPORT,
    POST_SCENARIO_EVENTS,
    RuntimeEvent,
    append_runtime_event,
)


def pick_latest_run_dir(suite_runs_dir: Path, scenario: Scenario) -> Path:
    candidates = sorted(
        p
        for p in suite_runs_dir.iterdir()
        if p.is_dir() and p.name.endswith(f"_{scenario.scenario_id}_seed{scenario.seed}")
    )
    if not candidates:
        raise FileNotFoundError(
            f"could not locate run directory for {scenario.scenario_id}"
        )
    return candidates[-1]


def read_metrics(run_dir: Path) -> dict[str, Any]:
    path = run_dir / "metrics.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def terminal_scenario_record(records: Sequence[dict[str, Any]]) -> dict[str, Any] | None:
    for record in reversed(records):
        if record["event"] not in POST_SCENARIO_EVENTS:
            return record
    return None


def load_jsonl_events(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            stripped = raw.strip()
            if stripped:
                records.append(json.loads(stripped))
    return records


def append_benchmark_report(
    run_dir: Path,
    *,
    benchmark_suite: str,
    passed: bool,
    failures: list[str],
    report_path: Path,
) -> None:
    """Append ``BENCHMARK_REPORT`` after ``SCENARIO_FINISHED`` in ``events.jsonl``."""

    events_path = run_dir / "events.jsonl"
    if not events_path.is_file():
        return

    records = load_jsonl_events(events_path)
    terminal = terminal_scenario_record(records)
    if terminal is None or terminal["event"] != "SCENARIO_FINISHED":
        return

    append_runtime_event(
        events_path,
        RuntimeEvent(
            ts=float(terminal["ts"]),
            episode_id=str(terminal["episode_id"]),
            event=BENCHMARK_REPORT,
            data={
                "benchmark_suite": benchmark_suite,
                "passed": passed,
                "failures": list(failures),
                "report_path": str(report_path),
            },
        ),
    )


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
        handle.write("\n")


def run_benchmark_suite(
    suite_dir: Path,
    output_dir: Path,
    report_path: Path | None,
    *,
    include_integration: bool,
    run_command: Callable[[list[str]], int],
) -> int:
    try:
        scenarios = discover_scenarios(suite_dir)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if not scenarios:
        print(f"no benchmark scenarios found under {suite_dir}", file=sys.stderr)
        return 2

    suite_name = suite_dir.name
    output_dir.mkdir(parents=True, exist_ok=True)
    suite_runs_dir = output_dir / suite_name
    suite_runs_dir.mkdir(parents=True, exist_ok=True)

    runnable: list[tuple[Path, Scenario]] = []
    skipped: list[dict[str, Any]] = []
    for scenario_path in scenarios:
        scenario = load_scenario(scenario_path)
        if not include_integration and is_integration_scenario(scenario.raw):
            skipped.append(
                {
                    "scenario_id": scenario.scenario_id,
                    "scenario_path": str(scenario_path),
                    "reason": "integration-only (needs external stack); "
                    "pass --include-integration to run",
                }
            )
            continue
        runnable.append((scenario_path, scenario))

    for entry in skipped:
        print(
            f"benchmark suite '{suite_name}': skipping {entry['scenario_id']} "
            f"({entry['reason']})",
            file=sys.stderr,
        )

    results: list[BenchmarkScenarioResult] = []
    for scenario_path, scenario in runnable:
        try:
            expectation = BenchmarkExpectation.from_scenario_raw(scenario.raw)
        except ValueError as exc:
            print(f"{scenario_path}: {exc}", file=sys.stderr)
            return 2

        rc = run_command(
            [
                "run",
                str(scenario_path),
                "--fast",
                "--output-dir",
                str(suite_runs_dir),
            ]
        )
        if rc != 0:
            results.append(
                BenchmarkScenarioResult(
                    scenario_id=scenario.scenario_id,
                    scenario_path=str(scenario_path),
                    seed=scenario.seed,
                    run_dir="",
                    passed=False,
                    failures=[f"gnav run exited with code {rc}"],
                    expected=dict(expectation.raw),
                )
            )
            continue

        actual_run_dir = pick_latest_run_dir(suite_runs_dir, scenario)
        metrics = read_metrics(actual_run_dir)
        failures = expectation.evaluate(metrics)
        results.append(
            BenchmarkScenarioResult(
                scenario_id=scenario.scenario_id,
                scenario_path=str(scenario_path),
                seed=scenario.seed,
                run_dir=str(actual_run_dir),
                passed=not failures,
                failures=failures,
                metrics=metrics,
                expected=dict(expectation.raw),
            )
        )

    report = BenchmarkSuiteReport(
        benchmark_suite=suite_name,
        ran_at=now_iso(),
        scenarios=results,
        skipped=skipped,
    )
    resolved_report_path = report_path or (output_dir / f"{suite_name}_report.json")
    write_json(resolved_report_path, report.to_dict())

    for result in results:
        if not result.run_dir:
            continue
        append_benchmark_report(
            Path(result.run_dir),
            benchmark_suite=suite_name,
            passed=result.passed,
            failures=list(result.failures),
            report_path=resolved_report_path,
        )

    skipped_note = f", {len(skipped)} skipped" if skipped else ""
    print(
        f"benchmark suite '{suite_name}': "
        f"{report.passed}/{report.total} passed{skipped_note} -> {resolved_report_path}"
    )
    for result in results:
        if not result.passed:
            print(
                f"  FAIL {result.scenario_id}: {'; '.join(result.failures)} "
                f"(run_dir={result.run_dir or 'n/a'})",
                file=sys.stderr,
            )
    return 0 if report.failed == 0 else 1


__all__ = [
    "append_benchmark_report",
    "load_jsonl_events",
    "pick_latest_run_dir",
    "read_metrics",
    "run_benchmark_suite",
    "terminal_scenario_record",
    "write_json",
]
