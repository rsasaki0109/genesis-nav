# Changelog

All notable changes to `genesis-nav` are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) loosely and the
project follows [Semantic Versioning](https://semver.org/) for the Python
package; the ROS 2 packages under `ros2_ws/src/` follow ROS 2 release
practice independently.

## [0.2.0a0] — 2026-05-29

v0.2 groundwork (alpha). The design ADRs dated 2026-05-29 in
`docs/decisions.md` are now backed by tested, sim-first slices. Each keeps
all `rclpy` use behind a boundary so the core stays unit-testable without
ROS 2, and every actuator-bound command still passes `CommandGate`. Tagged
as an alpha pre-release because several paths are intentionally partial — the
real-robot loop, Nav2 controller delegation, and watchdog auto-poll are
documented follow-ups, not yet closed.

### Added

- **Real-robot backend** — `gnav run --backend ros2_robot` drives a real
  robot as an ordinary `EmbodimentAdapter` over `/<agent>/cmd_vel` +
  `/<agent>/odom`. All ROS use is behind `genesis_nav.ros2_robot.RobotTransport`
  (`FakeRobotTransport` for tests); a command-staleness watchdog helper is
  included.
- **Dynamic obstacles + replan** — scenarios may declare timestamped grid
  deltas (`dynamic_obstacles.events`). Executing agents whose remaining path
  is newly blocked re-enter `planning` (new `executing → planning` behavior
  edge), replan around the obstacle (`REPLAN_TRIGGERED`), and continue. Each
  delta is recorded as an `OBSTACLE_CHANGED` event so replays reconstruct the
  obstacle timeline. New counters `replan_count`, `obstacle_event_count`;
  `examples/scenarios/dynamic_obstacle.yaml` demonstrates it.
- **Nav2 planner backend** — `runtime.navigation.planner: auto | grid |
  straight | nav2`. `nav2` delegates global planning to a running Nav2 stack
  via a `ComputePathToPose` action behind `genesis_nav.nav2.Nav2PathService`,
  while genesis-nav stays the runtime/arbiter. Generalizes the selector from
  issue #9.
- **Teleop (operator override)** — `Runtime.submit_teleop_command(...)`, a
  transport-agnostic operator entry point that runs a `TELEOP` command through
  `CommandGate` and, on accept, holds off the autonomy loop for
  `navigation.teleop_hold_sec` so the operator keeps control. Exercised in core
  CI without `rclpy`.
- **Hardware diagnostics** — `Runtime.diagnostics()` and
  `AgentToolApi.get_diagnostics()` return a per-agent health report
  (`OK`/`WARN`/`ERROR`) folding emergency-stop, fall, task-failure, and the
  real-robot command-staleness watchdog. Optional periodic `DIAGNOSTICS`
  events via `navigation.diagnostics_interval_sec`.
- **Observability** — `env.json` records the selected `planner`.

### Known limitations (carried into v0.2 work)

- Real-robot watchdog auto-poll, Nav2 controller `cmd_vel` through
  `CommandGate`, and `env.json` Nav2 version capture are documented
  follow-ups (see the 2026-05-29 ADRs and `docs/experiments.md`).

## [0.1.0] — 2026-05-29

The first tagged release. v0.1 closes PLAN.md §9 workstreams F / G / H / I /
J / K / L / M / N. The runtime is sim-first and ROS 2-bridged; real-robot
adapters are deferred to v0.2 (see issue #10).

### Added

- **Runtime core** — agent registry, task model with status / behavior state
  separation, ring-buffer event sink, `CommandGate` with authority +
  requester + timestamp metadata, fleet dispatcher with nearest-fit
  selection, lease-based reservations.
- **Navigation MVP** — `GridAStarPlanner` (8-connected, Octile heuristic,
  corner-cut prevention, colinear waypoint simplification),
  `StraightLinePlanner` fallback, behavior state machine (`idle` /
  `assigned` / `planning` / `reserving` / `executing` / `recovering` /
  `succeeded` / `failed`), stuck detection + `RECOVERING` wait,
  `NavigationConfig` scenario block.
- **ROS 2 bridge** — in-process `EventSink` fanout, per-agent `/state` and
  `/odom`, tf / tf_static, `/clock`, `/genesis_nav/events`, external
  `/cmd_vel` traversing `CommandGate`. Marker package `genesis_nav_ros`,
  message package `genesis_nav_msgs`, bringup package
  `genesis_nav_bringup`.
- **Observability** — `events.jsonl`, `metrics.json`, `env.json`,
  `report.md` under deterministic `runs/<timestamp>_<scenario>/`. New
  events `PLAN_RESOLVED`, `PLAN_FAILED`, `BEHAVIOR_STATE_CHANGED`,
  `STUCK_RECOVERED`. Counters: `stuck_event_count`, `recovery_count`,
  `plan_failure_count`.
- **Replay** — `gnav replay <run_dir>` validates artifacts strictly and
  streams the event timeline via `--print-events`.
- **Benchmarks** — `gnav bench --run benchmarks/<suite>` with expectation
  predicates across `nav_basic`, `multi_agent`, `runtime`, `humanoid`
  suites. Reports under `benchmarks/_runs/<suite>_report.json` carry
  `run_dir` for failure debug.
- **AI safety boundary** — `genesis_nav.agent.AgentToolApi` as the only
  Python surface AI agents may call. Audit trail via `data.source =
  "ai_tool_api"` and `requester_id` on every AI-originated event.
- **Humanoid shell** — navigation-intent type only; gait / balance /
  footstep planning intentionally out of scope (see `docs/humanoid.md`).
- **Tests** — 109 unit tests, 1 skipped; smoke + `warehouse_10_agents` +
  `humanoid_nav_intent` scenarios green end-to-end.
- **Community surface** — `CONTRIBUTING.md`,
  `docs/contributing_scenarios.md`, `docs/good_first_issues.md` (10
  curated entries), 6 issue templates (`task` / `architecture` /
  `benchmark` / `robot_adapter` / `scenario` / `bug`), comparison table
  in README, demo script and launch-post drafts.
- **Demo automation** — `make demo-gif` records the smoke tour with
  `asciinema` + `agg` into `docs/media/smoke_demo.gif` and
  `docs/media/smoke_demo.cast`.

### Known limitations

- No dynamic obstacles in the grid planner.
- No spatial conflict resolution between agents (reservations are
  lease-based, not costmap-aware).
- No Nav2 plugin bridge (intentional — see ADR in `docs/decisions.md`).
- No real-robot adapters yet; the path is via the ROS 2 contract.

[0.2.0a0]: https://github.com/rsasaki0109/genesis-nav/releases/tag/v0.2.0a0
[0.1.0]: https://github.com/rsasaki0109/genesis-nav/releases/tag/v0.1.0
