# Changelog

All notable changes to `genesis-nav` are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) loosely and the
project follows [Semantic Versioning](https://semver.org/) for the Python
package; the ROS 2 packages under `ros2_ws/src/` follow ROS 2 release
practice independently.

## [Unreleased]

## [0.2.0] — 2026-06-10

v0.2 formal release. Builds on `0.2.0a1` with real Genesis 1.0 integration,
head-on lateral reroute, Genesis URDF wheel-joint spawn, furnished warehouse
world, v0.2 boundary docs, architecture diagrams, and a Genesis CI lane.

### Added
- **Head-on lateral reroute** — `runtime.collision.headon_radius_m` /
  `headon_lateral_offset_m` (0 = disabled). Lower-priority agents replan with a
  lateral detour when facing a higher-priority agent on a shared corridor.
  Emits `HEADON_REROUTE`, bumps `headon_reroute_count`. Evaluated before yield
  (stop-and-wait fails on narrow corridors). New bench
  `benchmarks/multi_agent/headon_avoidance.yaml`; predicates
  `headon_reroute_count_min`.
- **Genesis URDF diff-drive** — `examples/robots/diff_drive.urdf` with
  `left_wheel_joint` / `right_wheel_joint`. Set `GENESIS_NAV_SPAWN_URDF=1` to
  spawn via `gs.morphs.URDF`; `GenesisDiffDriveAdapter` maps unicycle commands
  to `control_dofs_velocity` when wheel DOFs are present (kinematic box remains
  the default).
- **Furnished warehouse world** — perimeter walls and shelf blocks in
  `examples/worlds/warehouse_small.py`.
- **Genesis CI lane** — `.github/workflows/genesis.yaml` runs integration tests
  when `genesis-world` is installable (skips otherwise).
- **Docs** — `docs/roadmap_v02.md` (v0.1 → v0.2 boundary), Mermaid diagrams in
  `docs/architecture.md`.

### Changed
- **Genesis backend now runs on real Genesis 1.0.** Rewrote the Genesis world,
  adapter, and backend against the actual Genesis 1.0 API (`gs.init(backend=…)`,
  `gs.Scene(show_viewer=False)`, `add_entity(gs.morphs.*)`, `scene.build()` after
  spawns, `get_pos`/`get_quat`/`set_pos`/`set_quat`) — replacing an imagined
  `set_velocity`/`get_pose` surface that never existed on a real entity and was
  never run against Genesis. A diff-drive base is driven kinematically
  (unicycle integration written back to the base pose). New
  `GenesisBackend.finalize()` builds the scene once after all agents spawn.
  `gnav run --backend genesis` now runs a real scene stepped on `gs.cuda`
  (RTX 4070 Ti SUPER; smoke: success_rate=1.0, 265 steps, path≈2.139 m,
  matching the fallback; `genesis_version=1.0.0` captured in env.json; replay
  rc=0). Guarded by `tests/unit/test_genesis_integration.py` (2 passed on real
  Genesis), which skips without Genesis so core CI is unchanged. All Genesis
  imports stay lazy behind the backend boundary.

## [0.2.0a1] — 2026-05-31

Second v0.2 alpha. Builds on `0.2.0a0` with the real-robot loop closed in
process, full Nav2 delegation (planner + controller, both through
`CommandGate`), Nav2 reproducibility capture, a multi-agent spatial-safety
loop (detect → near-miss → yield → diagnostics), and that health surfaced over
the ROS bridge. Every slice keeps `rclpy` behind a boundary so the core stays
unit-testable without ROS 2, and every actuator-bound command still passes
`CommandGate`.

### Added
- **Diagnostics over the ROS bridge** — `gnav run --ros` now publishes the
  per-agent health report each tick on `/genesis_nav/diagnostics` as a standard
  `diagnostic_msgs/DiagnosticArray` (one `DiagnosticStatus` per agent; level
  `OK`/`WARN`/`ERROR` maps 1:1; `values` carry `behavior_state` and
  `command_age_sec`), so RViz and the ROS diagnostic aggregator see the same
  health the runtime computes — including the inter-agent proximity axes. The
  bridge stays decoupled via `RosBridge.set_diagnostics_provider(...)`. No
  `genesis_nav_msgs` change (standard `diagnostic_msgs`).

### Changed
- **Diagnostics fold inter-agent proximity** — `Runtime.diagnostics()` /
  `AgentToolApi.get_diagnostics()` now report `in_collision` (→ ERROR) and
  `yielding` (→ WARN), maintained by the runtime each tick, so the
  collision-detection and yield slices surface in the per-agent health
  read-model and the periodic `DIAGNOSTICS` event (and thus in replays). No
  change when proximity detection is disabled.

### Added
- **Proximity response (yield right-of-way)** — `runtime.collision.yield_radius_m`
  (0 = disabled). An executing agent yields (stops for that tick) while a
  higher-priority agent — lexicographic agent-id order, a deadlock-free total
  order — with an active task is within the yield radius. Emits `AGENT_YIELDED`
  and bumps `yield_count`; resets the stuck window so a brief wait is not
  mistaken for being stuck; never yields to an idle agent parked at its goal.
  Resolves crossing conflicts (stop-and-wait); head-on reroute, smarter
  priority, and costmap-aware reservation remain follow-ups. New bench
  predicates `yield_count_min` / `yield_count_max`; new guard scenario
  `benchmarks/multi_agent/yield_avoidance.yaml` (the crossing that collides
  under detection-only reaches collision = near_miss = 0 with response on).

### Added
- **Inter-agent proximity detection** — optional `runtime.collision` block
  (`collision_radius_m` / `near_miss_radius_m`, both default 0 = disabled). Each
  tick the runtime measures the planar distance between every agent pair and, on
  the rising edge of entering a radius, emits a `COLLISION` / `NEAR_MISS` event
  and bumps `collision_count` / `near_miss_count` (previously dead counters).
  Observation only — no agent is stopped or rerouted; proximity *response* is a
  follow-up. New bench predicates `collision_count_min` / `near_miss_count_min`;
  new guard scenario `benchmarks/multi_agent/near_miss_detection.yaml`.

### Added
- **Real-robot loop closure (loopback transport)** — `real_robot.transport:
  loopback` selects an rclpy-free `LoopbackRobotTransport` that integrates the
  commanded velocity into odom (same diff-drive model as the fallback), closing
  the real-robot loop in process. The *same* `Ros2RobotAdapter` is used — only
  the transport differs — so the full contract (`CommandGate` → `apply_command`
  → `publish_velocity` → odom feedback → `read_pose` → controller) runs end to
  end without `rclpy` or hardware. `gnav run --backend ros2_robot` with
  `transport: loopback` now reaches the goal deterministically in core CI,
  closing the "real-robot loop closure is future work" item. `transport: ros2`
  (default) is the live `rclpy` path as before.

### Added
- **Nav2 controller delegation** — `runtime.navigation.controller: local |
  nav2`. `nav2` delegates velocity generation to a running Nav2 controller
  server via the `genesis_nav.nav2.Nav2ControllerService` boundary.
  `Nav2Controller` is a drop-in for `SimpleLocalController`, so Nav2's `cmd_vel`
  flows through the **existing `CommandGate` autonomy path** before reaching the
  actuator — a non-finite/over-limit Nav2 velocity is rejected and the agent
  stopped, and genesis-nav never lets Nav2 drive the actuator directly. Falls
  back to the in-tree controller when Nav2 has no command yet.
  `COMMAND_ACCEPTED` events now carry `source` (`navigation` | `nav2_controller`
  | …). `FakeNav2ControllerService` keeps it unit-testable without ROS 2.

### Added
- **Dynamic-obstacle benchmark** — `benchmarks/nav_basic/dynamic_obstacle_replan.yaml`
  guards the replan path via `gnav bench` (`obstacle_event_count_min`,
  `replan_count_min` predicates). `metrics.json` now exposes `replan_count` and
  `obstacle_event_count`.

### Added
- **Integration-only benchmark scenarios** — a scenario may declare
  `benchmark.integration: true` to mark it as depending on an external stack
  (e.g. a live Nav2 server). `gnav bench --run` skips such scenarios by
  default — recording them under the report's new `skipped` array (with a
  `reason`) and logging the skip, so coverage is never silently truncated —
  and runs them only with the new `--include-integration` flag. The report
  also gains `skipped_count`. New reference suite
  `benchmarks/nav2_integration/` (`planner: nav2`).

### Added
- **`env.json` Nav2 version capture** — run metadata now records
  `nav2_version`, resolved from the Nav2 `package.xml` `<version>` via the
  `ament_index_python` index (`nav2_bringup` / `nav2_msgs` / `nav2_core`).
  Best-effort and never raises: core CI / pure-sim runs (no ament index)
  collect `""`. Together with the existing `ros_distro` this lets a replay of
  a `nav2` run state which Nav2 it ran against.

### Added
- **Real-robot command-watchdog auto-poll** — the runtime now polls each
  adapter's `watchdog_expired` every tick (`_poll_safety_signals`). When a real
  robot's command pipeline stalls, the rising edge emits a `SAFETY_STOP`
  (`reason="command_watchdog"`), zeroes the actuator, and latches the emergency
  stop. New `watchdog_stop_count` metric and `watchdog_stop_count_min` /
  `watchdog_stop_count_max` bench predicates. Duck-typed, so pure-sim runs never
  trip; latched, so it does not auto-clear when commands resume.

### Changed
- **ROS bridge `/cmd_vel` unified onto teleop API** — `RosBridge._on_cmd_vel`
  now forwards each Twist to `Runtime.submit_teleop_command(...,
  requester_id="ros_cmd_vel")` instead of building its own `RuntimeCommand` and
  evaluating `CommandGate` inline. The bridge is now pure transport; gating,
  command events, the actuator apply, and the autonomy hold all live in the
  runtime. This fixes two gaps in the old path: external `/cmd_vel` now carries
  the required `requester_id` metadata and now yields autonomy for
  `teleop_hold_sec` (an operator over ROS truly overrides autonomy).
  `RosBridge.__init__` no longer takes a `CommandGate`; its
  `external_command_handler` parameter is replaced by `teleop_command_handler`
  (signature `(agent_id, linear_x, linear_y, angular_z) -> CommandDecision`).

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

[Unreleased]: https://github.com/rsasaki0109/genesis-nav/compare/v0.2.0a1...HEAD
[0.2.0a1]: https://github.com/rsasaki0109/genesis-nav/compare/v0.2.0a0...v0.2.0a1
[0.2.0a0]: https://github.com/rsasaki0109/genesis-nav/releases/tag/v0.2.0a0
[0.1.0]: https://github.com/rsasaki0109/genesis-nav/releases/tag/v0.1.0
