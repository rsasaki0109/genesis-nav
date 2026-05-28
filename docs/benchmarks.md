# Benchmarks

`genesis-nav` treats benchmarks as **regression harnesses**, not leaderboards.
Each benchmark is a normal scenario YAML with an extra `benchmark.expected`
block that declares pass/fail thresholds against `metrics.json`.

## Suite Layout

```
benchmarks/
  nav_basic/      single-agent point-to-goal scenarios
  multi_agent/    multi-agent dispatch/coordination scenarios
  runtime/        observability/replay/dispatcher regressions
  humanoid/       humanoid navigation-intent + fall-stop coverage
  _runs/          generated run artifacts and suite reports (gitignored)
```

Every scenario file is a self-contained `gnav run` input. Suites do not
share state; one scenario's failure does not abort the suite.

## Adding a Benchmark

1. Drop `<name>.yaml` into the appropriate category directory.
2. Use a `scenario_id` prefixed with `bench_<category>_` to keep run-dir
   names unambiguous.
3. Add a `benchmark.expected` block. Supported keys:

   | Key | Compared against `metrics.json` field | Predicate |
   |---|---|---|
   | `success_rate_min` | `success_rate` | `actual >= value` |
   | `success_rate_max` | `success_rate` | `actual <= value` |
   | `task_succeeded_count_min` | `task_succeeded_count` | `actual >= value` |
   | `task_failed_count_max` | `task_failed_count` | `actual <= value` |
   | `task_dispatched_count_min` | `task_dispatched_count` | `actual >= value` |
   | `sim_steps_min` | `sim_steps` | `actual >= value` |
   | `replan_count_min` | `replan_count` | `actual >= value` |
   | `obstacle_event_count_min` | `obstacle_event_count` | `actual >= value` |
   | `command_rejection_count_max` | `command_rejection_count` | `actual <= value` |
   | `collision_count_max` | `collision_count` | `actual <= value` |
   | `near_miss_count_max` | `near_miss_count` | `actual <= value` |
   | `emergency_stop_count_max` | `emergency_stop_count` | `actual <= value` |
   | `reservation_conflict_count_max` | `reservation_conflict_count` | `actual <= value` |
   | `time_to_goal_mean_max_sec` | `time_to_goal_mean_sec` | `actual <= value` |
   | `path_length_mean_max_m` | `path_length_mean_m` | `actual <= value` |

   Missing predicates are treated as "no expectation". Unknown keys are
   reported as failures so typos surface immediately.

4. Keep the scenario **deterministic**: fix `seed`, keep `max_sim_seconds`
   tight, avoid wall-clock dependence.

## Running a Suite

```bash
gnav bench --run benchmarks/nav_basic
gnav bench --run benchmarks/multi_agent
gnav bench --run benchmarks/runtime
gnav bench --run benchmarks/humanoid
```

Each invocation runs every `*.yaml` under the directory in `--fast`
mode, writes a normal run directory (with `events.jsonl`, `metrics.json`,
`env.json`, `report.md`, …) under `benchmarks/_runs/<suite>/`, and
emits an aggregated report at `benchmarks/_runs/<suite>_report.json`.

Use `--output-dir` to redirect artifacts (CI typically points this at a
clean per-job directory) and `--report` to override the report path.

Exit codes:

- `0` — every scenario passed every expectation.
- `1` — at least one scenario failed at least one expectation.
- `2` — the suite directory is missing or a scenario is malformed.

## Report Shape

```json
{
  "benchmark_suite": "nav_basic",
  "ran_at": "2026-05-28T07-37-25Z",
  "total": 1,
  "passed": 1,
  "failed": 0,
  "scenarios": [
    {
      "scenario_id": "bench_nav_basic_single_agent_empty",
      "scenario_path": "benchmarks/nav_basic/single_agent_empty.yaml",
      "seed": 42,
      "run_dir": "benchmarks/_runs/nav_basic/...",
      "passed": true,
      "failures": [],
      "metrics": { "success_rate": 1.0, "...": "..." },
      "expected": { "success_rate_min": 1.0, "...": "..." }
    }
  ]
}
```

When a scenario fails, `failures` lists the human-readable predicate
strings (e.g. `"success_rate=0.5 < success_rate_min=1.0"`) and `run_dir`
points at the full replay artifact so
`gnav replay <run_dir> --print-events` can dig in.

## CI Smoke

`tests/unit/test_benchmark_harness.py::test_bench_run_nav_basic_passes`
exercises the harness end-to-end against `benchmarks/nav_basic/`. It is
part of the default `pytest tests/unit` run, so the smoke benchmark
ships with the unit-test gate.
