# Add Benchmark Scenario

Use this skill when adding a scenario or benchmark case.

## Workflow

1. Add a deterministic YAML scenario under `examples/scenarios/` or `benchmarks/`.
2. Include `scenario_id`, `seed`, `world`, `agents`, `tasks`, `metrics`, and
   `record`.
3. Add or update a smoke test under `tests/scenarios/` when useful.
4. Run:

```bash
gnav run path/to/scenario.yaml --fast
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit
```

5. Append the result to `docs/experiments.md`.
