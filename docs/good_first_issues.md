# Good First Issues

Ten concrete tasks for new contributors. Each has a clear file footprint and
acceptance criteria, so you can finish it in a single PR. Pick one, open an
issue using the matching template, and assign yourself.

Size key:
- **XS** — half-day, mostly doc or single-function
- **S** — one to two days, one module + tests
- **M** — about a week, multiple modules or a new boundary

---

## 1. Add `holonomic` robot adapter

**Size:** S — **Template:** robot_adapter

`genesis_nav/robots/` currently has `diff_drive` and the humanoid intent shell.
Add a holonomic / omni adapter that accepts `(vx, vy, wz)` and produces a
matching kinematics integrator for the in-memory backend.

- Files: `genesis_nav/robots/holonomic.py`, `genesis_nav/scenario/agents.py`
  registration, unit test under `tests/unit/`.
- Update `docs/interfaces.md` if a new agent `type` is added.

---

## 2. Add an `inflate` flag to `OccupancyGrid`

**Size:** S — **Template:** task

A real grid planner inflates obstacles by the robot footprint. Add an
optional `inflate_cells: int` to `OccupancyGrid.from_mapping` and apply it
before the A* search.

- Files: `genesis_nav/navigation/grid_planner.py`,
  `tests/unit/test_navigation_mvp.py`.
- Update `docs/interfaces.md` § Scenario Schema.

---

## 3. Add a `BENCHMARK_REPORT` runtime event

**Size:** XS — **Template:** task

When `gnav bench` finishes a scenario, emit one event with the report path
and pass/fail. Right now this lives only in the JSON file.

- Files: `genesis_nav/benchmarks/runner.py`,
  `genesis_nav/observability/events.py` event enum,
  `docs/interfaces.md` runtime events list.

---

## 4. `gnav doctor --json`

**Size:** XS — **Template:** task

Add a `--json` flag to `gnav doctor` that prints the same content as
machine-readable JSON. Useful for CI gates and bug reports.

- Files: `genesis_nav/cli/doctor.py`, `tests/unit/test_cli_doctor.py`.

---

## 5. Add `replay --to-rosbag` exporter

**Size:** M — **Template:** task

`gnav replay` can already stream events; let it also write a rosbag2 record
of the run's mirrored topics. Gate behind ROS 2 availability with a clear
error otherwise.

- Files: `genesis_nav/cli/replay.py`, `genesis_nav/ros/bag_writer.py`,
  unit + integration tests.
- Update `docs/replay.md` and `docs/interfaces.md`.

---

## 6. Add `examples/scenarios/charging_dock.yaml`

**Size:** S — **Template:** scenario

A single agent navigates to a docking pose, waits, then returns to start.
Exercises pose tolerance + dwell time.

- Files: scenario YAML, optional dwell-time metric, row in
  `docs/experiments.md`.

---

## 7. Add `success_rate_ci` to the metrics collector

**Size:** S — **Template:** task

Currently `success_rate` is a point estimate. Add a Wilson 95% CI when more
than one task ran.

- Files: `genesis_nav/observability/metrics.py`, tests.
- Update `docs/benchmarks.md`.

---

## 8. Architecture diagrams in `docs/architecture.md`

**Size:** XS — **Template:** task

Add Mermaid diagrams for: (a) runtime → bridge → ROS 2 fanout, (b) task
lifecycle, (c) command authority chain. Keep them readable in plain text.

- Files: `docs/architecture.md`.

---

## 9. Add `runtime.navigation.planner: grid|straight` selector

**Size:** S — **Template:** task

Right now the planner is picked implicitly by the presence of an
`occupancy_grid` block. Add an explicit selector so contributors can force
the straight-line planner even when a grid is present (useful for ablations).

- Files: `genesis_nav/navigation/config.py`,
  `genesis_nav/core/runtime.py` `from_scenario`,
  tests.
- Update `docs/interfaces.md` § Scenario Schema and add an ADR if the default
  changes.

---

## 10. Document the v0.1 → v0.2 boundary

**Size:** XS — **Template:** architecture

`PLAN.md` covers v0.1 in detail. Add a short `docs/roadmap_v02.md` (or a
section in `docs/roadmap.md`) that lists the three biggest things v0.1 left
unsolved (e.g., dynamic obstacles, multi-robot reservation, Nav2 adapter).
Output should be one page max.

- Files: `docs/roadmap.md` or new doc.
- No code changes.

---

## Picking the right one

- New to ROS 2? Start with 4, 6, or 8.
- Familiar with Python but new to robotics? 2, 7, or 9.
- Strong on robotics? 1, 5, or 10.
