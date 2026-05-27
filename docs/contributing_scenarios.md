# Scenario Contribution Guide

A scenario is the contract that turns a robotics idea into something the
runtime can run, replay, and benchmark. This guide is for contributors adding
or modifying scenarios under `examples/scenarios/` or as part of a benchmark
suite under `benchmarks/<suite>/`.

## What a scenario is

A scenario is a YAML file that declares:

- a deterministic seed
- a world (Genesis script path or in-memory fallback)
- one or more agents with capabilities and authority modes
- zero or more tasks
- metric names to collect
- recording switches for `events.jsonl` and rosbag

The runtime loads a scenario via `genesis_nav.scenario.load`, builds agents,
selects a planner, and executes tasks through the dispatcher. The output is a
`runs/<timestamp>_<scenario_id>/` directory with the scenario copy,
`events.jsonl`, `metrics.json`, `env.json`, and `report.md`.

## Minimal example

```yaml
scenario_id: hallway_pickup
seed: 7
world: examples/worlds/warehouse_small.py
agents:
  - id: robot_001
    type: diff_drive
    namespace: /robot_001
    spawn: [0.0, 0.0, 0.0]
    frames:
      map: map
      odom: robot_001/odom
      base: robot_001/base_link
    capabilities: [navigate_2d, stop, report_pose]
    authority:
      mode: autonomous
      command_ttl_ms: 200
tasks:
  - id: task_001
    type: navigate_to_pose
    agent: robot_001
    goal: [3.0, 1.5, 0.0]
metrics: [success_rate, collision_count, time_to_goal]
record:
  rosbag: false
  events: true
```

## Optional blocks

- `occupancy_grid:` — declares a static grid that swaps the runtime in to
  `GridAStarPlanner`. See `docs/interfaces.md` § Scenario Schema.
- `runtime.navigation:` — overrides `waypoint_tolerance_m`,
  `stuck_window_sec`, `stuck_min_progress_m`, `recovery_wait_sec`,
  `max_recovery_retries`.
- `dispatcher:` — selector policy and reservation hints.
- `resources:` — named shared resources for the reservation system.

If you depend on an optional block, add a comment in the YAML pointing at the
schema doc so future readers know it is not boilerplate.

## Rules of thumb

1. **Determinism first.** A scenario must reproduce. If you set a seed, fix
   spawn poses, fix task IDs. Reviewers will run your scenario twice; the
   resulting `metrics.json` must be byte-equal on the keys you declared.
2. **Real names.** Use `task_001`, `robot_001`, not random UUIDs. The IDs end
   up in logs and replay output — give them to readers, not to entropy.
3. **Metrics you mean.** Only list metrics you will actually look at. If you
   add `collision_count`, the scenario should be capable of producing
   collisions (otherwise the value is uninformative).
4. **No backdoors.** Scenarios cannot bypass `CommandGate`. If your scenario
   needs an unusual authority mode, document why in the PR.
5. **Locality.** Worlds, meshes, and other binary blobs live alongside the
   scenario YAML they belong to. Cross-scenario shared assets go under
   `examples/worlds/`.

## Submitting a scenario PR

A scenario PR should include:

- The YAML under `examples/scenarios/` (or under a benchmark suite).
- A row in `docs/experiments.md` with the date, scenario path, commit
  placeholder (`pending`), and the headline result.
- Verification output in the PR body. Minimum:

  ```bash
  gnav run examples/scenarios/<your_scenario>.yaml --fast --record
  ```

  Paste the `report.md` block and confirm `success_rate` matches what you
  claim in `experiments.md`.

- If the scenario exercises a new public interface (event type, schema field,
  metric name), update `docs/interfaces.md` in the same PR.

## Submitting a benchmark scenario

Benchmarks live under `benchmarks/<suite>/<scenario>.yaml` plus an
`expectations.yaml` next to them. The predicate vocabulary is documented in
`docs/benchmarks.md`. Run:

```bash
gnav bench --run benchmarks/<suite>
```

and paste the JSON report summary in the PR body.

## When to extend an existing scenario vs add a new one

- Extend when the existing scenario is the obvious home for the change
  (new agent in `warehouse_10_agents.yaml`, new metric in `smoke.yaml`).
- Add new when the topology, agent count, or task shape changes meaningfully.
  Reviewers will push back if you "stuff a new robot" into a scenario that
  is already someone's reference.

## Anti-patterns

- Adding a scenario that exists only to make a test pass — write a unit test
  instead.
- Using a real-time clock dependency (`time.sleep`, wall-clock seeds). The
  runtime is sim-time-driven; scenarios must be too.
- Hiding configuration in a Python world script when the same value could
  live in YAML. Worlds are for geometry; YAML is for parameters.
