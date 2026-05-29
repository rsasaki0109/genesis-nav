"""Benchmark expectation and report model.

A benchmark is just a scenario YAML with an optional top-level
``benchmark.expected`` block:

```yaml
benchmark:
  expected:
    success_rate_min: 1.0
    task_succeeded_count_min: 1
    time_to_goal_mean_max_sec: 10.0
    command_rejection_count_max: 0
```

Each key is checked against the matching field in ``metrics.json``.
A benchmark passes when every checked predicate holds.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


_MIN_CHECKS: Mapping[str, str] = {
    "success_rate_min": "success_rate",
    "task_succeeded_count_min": "task_succeeded_count",
    "task_dispatched_count_min": "task_dispatched_count",
    "sim_steps_min": "sim_steps",
    "replan_count_min": "replan_count",
    "obstacle_event_count_min": "obstacle_event_count",
    "watchdog_stop_count_min": "watchdog_stop_count",
    "collision_count_min": "collision_count",
    "near_miss_count_min": "near_miss_count",
}

_MAX_CHECKS: Mapping[str, str] = {
    "success_rate_max": "success_rate",
    "task_failed_count_max": "task_failed_count",
    "command_rejection_count_max": "command_rejection_count",
    "collision_count_max": "collision_count",
    "near_miss_count_max": "near_miss_count",
    "emergency_stop_count_max": "emergency_stop_count",
    "watchdog_stop_count_max": "watchdog_stop_count",
    "reservation_conflict_count_max": "reservation_conflict_count",
    "time_to_goal_mean_max_sec": "time_to_goal_mean_sec",
    "path_length_mean_max_m": "path_length_mean_m",
}


@dataclass(frozen=True)
class BenchmarkExpectation:
    """Parsed ``benchmark.expected`` block, plus its evaluation logic."""

    raw: Mapping[str, Any]

    @classmethod
    def from_scenario_raw(cls, scenario_raw: Mapping[str, Any]) -> "BenchmarkExpectation":
        bench_block = scenario_raw.get("benchmark") or {}
        expected = bench_block.get("expected") or {}
        if not isinstance(expected, Mapping):
            raise ValueError(
                f"benchmark.expected must be a mapping, got {type(expected).__name__}"
            )
        return cls(raw=dict(expected))

    def evaluate(self, metrics: Mapping[str, Any]) -> list[str]:
        """Return a list of human-readable failure strings (empty if pass)."""

        failures: list[str] = []
        for key, value in self.raw.items():
            if key in _MIN_CHECKS:
                metric_key = _MIN_CHECKS[key]
                actual = _coerce_number(metrics.get(metric_key))
                threshold = _coerce_number(value)
                if actual is None:
                    failures.append(f"{metric_key} missing from metrics.json")
                elif actual < threshold:
                    failures.append(
                        f"{metric_key}={actual} < {key}={threshold}"
                    )
            elif key in _MAX_CHECKS:
                metric_key = _MAX_CHECKS[key]
                actual = _coerce_number(metrics.get(metric_key))
                threshold = _coerce_number(value)
                if actual is None:
                    failures.append(f"{metric_key} missing from metrics.json")
                elif actual > threshold:
                    failures.append(
                        f"{metric_key}={actual} > {key}={threshold}"
                    )
            else:
                failures.append(f"unknown expectation key '{key}'")
        return failures


def is_integration_scenario(scenario_raw: Mapping[str, Any]) -> bool:
    """True when a scenario declares ``benchmark.integration: true``.

    Integration scenarios depend on an external stack (e.g. a running Nav2
    server) and so are not part of the deterministic regression set. `gnav
    bench --run` skips them unless ``--include-integration`` is passed.
    """

    bench_block = scenario_raw.get("benchmark") or {}
    return bool(bench_block.get("integration", False))


def _coerce_number(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return float(int(value))
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


@dataclass
class BenchmarkScenarioResult:
    scenario_id: str
    scenario_path: str
    seed: int
    run_dir: str
    passed: bool
    failures: list[str] = field(default_factory=list)
    metrics: Mapping[str, Any] = field(default_factory=dict)
    expected: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class BenchmarkSuiteReport:
    benchmark_suite: str
    ran_at: str
    scenarios: list[BenchmarkScenarioResult]
    skipped: list[dict[str, Any]] = field(default_factory=list)

    @property
    def total(self) -> int:
        return len(self.scenarios)

    @property
    def passed(self) -> int:
        return sum(1 for s in self.scenarios if s.passed)

    @property
    def failed(self) -> int:
        return self.total - self.passed

    def to_dict(self) -> dict[str, Any]:
        return {
            "benchmark_suite": self.benchmark_suite,
            "ran_at": self.ran_at,
            "total": self.total,
            "passed": self.passed,
            "failed": self.failed,
            "skipped_count": len(self.skipped),
            "scenarios": [s.to_dict() for s in self.scenarios],
            "skipped": list(self.skipped),
        }


def now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")


def discover_scenarios(suite_dir: Path) -> list[Path]:
    """Return *.yaml scenario files directly under ``suite_dir``, sorted."""

    if not suite_dir.is_dir():
        raise FileNotFoundError(f"benchmark suite directory not found: {suite_dir}")
    return sorted(p for p in suite_dir.glob("*.yaml") if p.is_file())


__all__ = [
    "BenchmarkExpectation",
    "BenchmarkScenarioResult",
    "BenchmarkSuiteReport",
    "discover_scenarios",
    "is_integration_scenario",
    "now_iso",
]
