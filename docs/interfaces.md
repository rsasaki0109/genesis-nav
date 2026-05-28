# Interfaces

This document is the public contract for humans and AI coding agents. Update it
when changing ROS topics, messages, actions, services, QoS, runtime events,
scenario schema, task schema, or agent state schema.

## Stability Policy

`v0.1` interfaces are allowed to evolve, but breaking changes must be documented
in this file and called out in `docs/decisions.md` when architectural.

## ROS Topics

Runtime topics:

| Topic | Type | Purpose |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Simulation time source |
| `/genesis_nav/events` | `genesis_nav_msgs/RuntimeEvent` | Structured runtime event stream |
| `/genesis_nav/fleet_state` | `genesis_nav_msgs/FleetState` | Fleet-level task and agent state |
| `/genesis_nav/scenario_state` | `genesis_nav_msgs/ScenarioState` | Active scenario and run state |

Per-agent topics:

| Topic | Type | Purpose |
|---|---|---|
| `/<agent>/state` | `genesis_nav_msgs/AgentState` | Runtime state snapshot |
| `/<agent>/odom` | `nav_msgs/Odometry` | Agent odometry |
| `/<agent>/cmd_vel` | `geometry_msgs/Twist` | Command input to `CommandGate` |
| `/<agent>/scan` | `sensor_msgs/LaserScan` | Optional range data |
| `/<agent>/task_status` | `genesis_nav_msgs/TaskStatus` | Active task status |
| `/<agent>/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Agent diagnostics |

Global tf topics:

- `/tf`
- `/tf_static`

## ROS Actions

### `NavigateEmbodied.action`

Goal:

```text
string task_id
string agent_id
geometry_msgs/PoseStamped goal
string map_id
string behavior_profile
string[] constraints
```

Result:

```text
bool success
string error_code
string message
float32 duration_sec
float32 path_length
```

Feedback:

```text
string state
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
string active_behavior
```

### `ExecuteTaskGraph.action`

Goal:

```text
string task_id
string agent_id
string task_graph_json
```

Result:

```text
bool success
string error_code
string message
```

Feedback:

```text
string active_node_id
string status
```

## ROS Services

### `ReserveResource.srv`

```text
string requester_id
string resource_id
builtin_interfaces/Duration duration
---
bool accepted
string lease_id
string reason
```

## QoS Profiles

`configs/qos/default.yaml` is the source configuration. Treat QoS as part of the
interface contract. `genesis_nav.ros.qos.resolve_qos_for` matches topic names
against the profile catalog (glob wildcards supported; exact match wins).

| Topic | Reliability | Durability | Notes |
|---|---|---|---|
| `/clock` | reliable | transient_local | Simulation-time backbone |
| `/robot_*/scan` | best_effort | volatile | Sensor data may drop |
| `/robot_*/cmd_vel` | reliable | volatile | Deadline target: 100 ms |
| `/genesis_nav/events` | reliable | volatile | Runtime audit stream |

## EmbodimentAdapter Contract

Every embodiment backend implements `genesis_nav.core.embodiment.EmbodimentAdapter`:

```python
class EmbodimentAdapter(Protocol):
    agent_id: str
    def read_pose(self) -> tuple[float, float, float]: ...
    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None: ...
    def stop(self, reason: str) -> None: ...
```

`Runtime.from_scenario(scenario, events, adapter_factory=...)` accepts a
factory `Callable[[AgentSpec], EmbodimentAdapter]` so backends can be plugged
without subclassing Runtime. The default factory returns
`DiffDriveKinematics`. The Genesis backend supplies `GenesisBackend.spawn`.

Selected by `gnav run --backend {fallback | genesis | ros2_robot}`.

## Real-Robot Backend (`--backend ros2_robot`)

The real-robot path is an `EmbodimentAdapter`, not a separate runtime mode (see
the 2026-05-29 ADR in `docs/decisions.md`). `gnav run --backend ros2_robot`
builds a `Ros2RobotBackend` whose `spawn` returns a `Ros2RobotAdapter` per
agent. Per-agent topic convention (REP-103 frames, identity units):

- publishes `geometry_msgs/Twist` on `/<agent_id>/cmd_vel`
- subscribes `nav_msgs/Odometry` on `/<agent_id>/odom`

Commands published to the robot have already passed `CommandGate`, so the AI
safety boundary is identical to simulation. All `rclpy` use lives behind the
`genesis_nav.ros2_robot.RobotTransport` protocol
(`publish_velocity` / `latest_pose` / `monotonic_sec`); `FakeRobotTransport`
makes the adapter unit-testable without ROS 2. Optional scenario block:

```yaml
real_robot:
  command_timeout_sec: 0.5   # adapter watchdog horizon
```

The adapter exposes a command-staleness watchdog (`seconds_since_command`,
`watchdog_expired`) evaluated on the transport's monotonic (wall) clock. The
runtime auto-polls it every tick from `_poll_safety_signals`: on the rising
edge of expiry it emits `SAFETY_STOP` (`data.reason="command_watchdog"`,
`source="ros2_robot_adapter"`, `command_age_sec`), zeroes the actuator, and
latches the agent's emergency stop (which does not auto-clear when commands
resume — an operator must clear it, as with comms loss on real hardware). The
`watchdog_stop_count` metric counts these trips. Adapters without a
`watchdog_expired` method (the sim fallback, Genesis, humanoid) never
participate, so pure-sim runs are unaffected.

## Planner Backends

The planner is selected by `runtime.navigation.planner`:

```yaml
runtime:
  navigation:
    planner: auto   # auto | grid | straight | nav2
```

- `auto` (default) — grid A* if the scenario declares an `occupancy_grid`,
  else straight line. Matches v0.1 behaviour.
- `grid` — force `GridAStarPlanner` (errors if no `occupancy_grid`).
- `straight` — force `StraightLinePlanner`.
- `nav2` — delegate global planning to a running Nav2 stack (see the
  2026-05-29 Nav2 ADR). genesis-nav stays the runtime/arbiter; `Nav2Planner`
  implements the same `plan()` contract and bridges to a `ComputePathToPose`
  action behind `genesis_nav.nav2.Nav2PathService`. `FakeNav2PathService`
  makes it unit-testable without ROS 2. Optional scenario block:

  ```yaml
  nav2:
    compute_path_action: compute_path_to_pose
    frame_id: map
    timeout_sec: 5.0
  ```

  A `nav2` run is only as reproducible as the external stack; `env.json` now
  records `nav2_version` (resolved from the Nav2 `package.xml` via the ament
  index, `""` when Nav2 is not on the path) alongside `ros_distro` so a replay
  captures which Nav2 it ran against. `nav2` benchmark scenarios are marked
  `benchmark.integration: true` so the deterministic regression set skips them
  unless `--include-integration` is passed (see the Benchmark Report Schema
  section and `benchmarks/nav2_integration/`). This slice delegates global
  planning only; routing Nav2's controller `cmd_vel` through `CommandGate` as
  an `AUTONOMY` command is the remaining follow-up.

## Teleop (Operator Override)

`Runtime.submit_teleop_command(agent_id, *, requester_id, linear_x=0.0,
linear_y=0.0, angular_z=0.0, source="teleop", ttl_ms=None, hold_sec=None)` is
the single operator entry point: both the in-process API and the ROS bridge
`/cmd_vel` path flow through it (the bridge calls it with
`requester_id="ros_cmd_vel"`). It stamps a `TELEOP` command with the operator's
`requester_id` and current sim time, runs it through `CommandGate`, emits
`COMMAND_ACCEPTED` / `COMMAND_REJECTED`, and on accept applies it and holds off
the autonomy loop for `hold_sec` (default `runtime.navigation.teleop_hold_sec`,
0.5 s). During the hold the agent retains the operator's last command; autonomy
resumes when it expires. `requester_id` is mandatory. AI callers cannot drive
actuators through this path — AI-authority velocities are still gate-rejected,
and teleop authority is the operator's responsibility. A command stamped at sim
time 0 is rejected as stale.

```yaml
runtime:
  navigation:
    teleop_hold_sec: 0.5
```

## Diagnostics

`Runtime.diagnostics()` returns a read-only `DiagnosticsReport`: per-agent
`DiagnosticLevel` (`OK` < `WARN` < `ERROR`, ordered like ROS 2
`diagnostic_msgs`) plus an overall = worst-agent level. It folds
`emergency_stopped` / `fall_detected` (→ ERROR), behavior `failed` (→ ERROR),
and adapter command-staleness (the real-robot watchdog → WARN, with
`command_age_sec`). Adapters without those signals do not contribute.
`AgentToolApi.get_diagnostics()` exposes the same snapshot read-only to AI
agents. When `runtime.navigation.diagnostics_interval_sec > 0`, a periodic
`DIAGNOSTICS` event carrying `report.to_dict()` is emitted so replays
reconstruct the health timeline (default 0 = disabled).

```yaml
runtime:
  navigation:
    diagnostics_interval_sec: 0.0   # >0 to emit periodic DIAGNOSTICS events
```

## World File Contract

Scenario `world` fields point to a Python module that defines:

```python
def build_scene(seed: int) -> Scene: ...
def spawn_diff_drive(scene, spec) -> Any: ...
```

Loaded by `genesis_nav.genesis.load_world_entry`. The reference world is
`examples/worlds/warehouse_small.py`.

## ROS 2 Bridge

`gnav run --ros` activates `genesis_nav.ros.bridge.RosBridge`. The bridge:

- creates a single `genesis_nav_bridge` node
- publishes `/clock`, `/genesis_nav/events`, `/genesis_nav/scenario_state`,
  `/genesis_nav/fleet_state` and per-agent `<ns>/state` and `<ns>/odom`
- broadcasts `<map>` → `<ns>/odom` on `/tf_static` and `<ns>/odom` →
  `<ns>/base_link` on `/tf`
- subscribes to `<ns>/cmd_vel` and forwards each Twist to
  `Runtime.submit_teleop_command(..., requester_id="ros_cmd_vel")` via the
  injected `teleop_command_handler`. The bridge is **pure transport**: gating,
  `COMMAND_ACCEPTED` / `COMMAND_REJECTED` events, the actuator apply, and the
  autonomy hold all live in the runtime. AI-authority velocities are still
  rejected by the gate, and an external `/cmd_vel` now yields autonomy for
  `teleop_hold_sec` exactly like the in-process teleop API.

`RosBridge(registry, runtime_clock, events, *, config=None,
teleop_command_handler=None, episode_id="")` no longer takes a `CommandGate`
(the gate lives in the runtime). `teleop_command_handler` has signature
`(agent_id, linear_x, linear_y, angular_z) -> CommandDecision`.

## Runtime Events

Known event names:

- `TASK_ASSIGNED`
- `TASK_STARTED`
- `TASK_SUCCEEDED`
- `TASK_FAILED`
- `PLAN_CREATED`
- `PLAN_RESOLVED`
- `PLAN_FAILED`
- `PLAN_REJECTED`
- `BEHAVIOR_STATE_CHANGED`
- `RESOURCE_RESERVED`
- `RESOURCE_RELEASED`
- `COMMAND_ACCEPTED`
- `COMMAND_REJECTED`
- `SAFETY_STOP`
- `AGENT_RESUMED`
- `FALL_DETECTED`
- `COLLISION`
- `NEAR_MISS`
- `AGENT_STUCK`
- `STUCK_RECOVERED`
- `OBSTACLE_CHANGED`
- `REPLAN_TRIGGERED`
- `DIAGNOSTICS`
- `SIM_RESET`
- `SCENARIO_STARTED`
- `SCENARIO_FINISHED`

### Behavior State Machine

Agents carry an orthogonal `behavior_state` field that describes what the
agent is doing right now in service of the current task. `current_task_status`
is the task-side view (assigned / executing / succeeded / failed);
`behavior_state` is the runtime-side view.

| State | Meaning |
|---|---|
| `idle` | No active task. |
| `assigned` | Task assigned, not yet planned. |
| `planning` | Planner is producing a path. |
| `reserving` | Reserved for resource lease holds (not exercised in v0.1). |
| `executing` | Following the planned waypoints. May re-enter `planning` to replan when a dynamic obstacle blocks the remaining path. |
| `recovering` | Stuck; holding still for `recovery_wait_sec` before retrying. |
| `succeeded` | Final pose reached; transition to `idle` immediately. |
| `failed` | Planner could not produce a path, stuck retries exhausted, or task timed out; transition to `idle` immediately. |

`BEHAVIOR_STATE_CHANGED` event payload:

```json
{
  "from": "executing",
  "to": "recovering",
  "reason": "stuck"
}
```

`PLAN_RESOLVED` carries `waypoint_count` and `planner` (class name).
`PLAN_FAILED` carries the goal and a `reason` string. `AGENT_STUCK` carries
`progress_m`, `window_sec`, and the 1-indexed `retry` count.
`OBSTACLE_CHANGED` carries `blocked_cells` (a list of `[col, row]`) applied to
the grid at that timestamp. `REPLAN_TRIGGERED` carries `waypoint_count` and a
`reason` (e.g. `obstacle`); it is emitted between an `executing → planning`
and a `planning → executing` `BEHAVIOR_STATE_CHANGED` pair.

### Dynamic Obstacles

Scenarios with an `occupancy_grid` may declare timestamped grid deltas. Each
delta blocks cells at its `at_sec` sim time; executing agents whose remaining
path crosses a newly blocked cell replan around it (or fail with
`reason="blocked"` if no path remains). Deltas are recorded as
`OBSTACLE_CHANGED` events so a replay reconstructs the obstacle timeline from
the run directory alone (see the 2026-05-29 dynamic-obstacles ADR).

```yaml
dynamic_obstacles:
  events:
    - at_sec: 2.0
      block: [[3, 1], [3, 2]]   # [col, row] cells
```

## Scenario Schema

Required fields:

```yaml
scenario_id: string
seed: integer
world: path-or-id
max_sim_seconds: number  # optional, default 60
agents:
  - id: string
    type: string
    spawn: [x, y, yaw]
tasks:
  - id: string
    type: navigate_to_pose
    agent: string             # optional; dispatcher matches if omitted
    priority: integer         # optional; higher dispatches first
    goal: [x, y, yaw]
    constraints:              # optional dispatcher hints
      agent_selector:
        capabilities: [navigate_2d]
        nearest_to: [x, y]
resources:                    # optional; lease-managed shared zones
  - id: string
    kind: zone
    capacity: 1
occupancy_grid:               # optional; enables GridAStarPlanner
  resolution: 0.5             # metres per cell
  origin: [-5.0, -5.0]        # world coord of the lower-left corner
  cells:                      # row 0 is the bottom row; 1 = blocked
    - [0, 0, 1, 0]
    - [0, 0, 1, 0]
    - [0, 0, 0, 0]
runtime:                      # optional; tunes the navigation behavior loop
  navigation:
    waypoint_tolerance_m: 0.15
    stuck_window_sec: 1.5
    stuck_min_progress_m: 0.05
    recovery_wait_sec: 0.5
    max_recovery_retries: 3
metrics:
  - success_rate
record:
  rosbag: boolean
  events: boolean
```

When `occupancy_grid` is omitted, the runtime falls back to
`StraightLinePlanner`. The grid is static for v0.1; dynamic obstacles and
replanning land later. `runtime.navigation.max_recovery_retries=0` makes
stuck conditions fail immediately without entering `recovering`.

## Task Schema

```json
{
  "task_id": "task_001",
  "type": "navigate_to_pose",
  "priority": 10,
  "agent_selector": {
    "capabilities": ["navigate_2d"],
    "nearest_to": [1.0, 2.0, 0.0]
  },
  "goal": {
    "pose": [5.0, 3.0, 1.57],
    "frame_id": "map"
  },
  "constraints": {
    "deadline_sec": 120,
    "avoid_zones": ["human_zone_1"]
  }
}
```

## AI Tool API

`genesis_nav.agent.AgentToolApi` is the only Python entry point AI agents may
use to operate the runtime. Construct via `runtime.tool_api(...)`. The full
method list, safety contract, and event semantics are documented in
[`ai_agents.md`](ai_agents.md). Summary:

- Reads: `list_agents`, `get_world_state`, `get_task_status`,
  `get_recent_events`.
- Writes: `submit_task`, `pause_agent`, `resume_agent`, `stop_all`. All
  writes require a non-empty `requester_id`. `submit_task` auto-stamps a
  `trace_id` if not provided.
- AI-originated events carry `data.source = "ai_tool_api"` and the
  `requester_id`.
- The API exposes no actuator method; `cmd_vel` publishing remains forbidden.

## Agent State Schema

Minimum runtime fields:

- `agent_id`
- `embodiment_type` (`diff_drive` | `humanoid` | backend-defined)
- `namespace`
- `frames`
- `capabilities`
- `authority_mode`
- `lifecycle_state`
- `current_task_id`
- `current_task_status` (`pending` | `assigned` | `executing` | `succeeded` | `failed`)
- `behavior_state` (`idle` | `assigned` | `planning` | `reserving` | `executing` | `recovering` | `succeeded` | `failed`)
- `current_goal` (`[x, y, yaw]` or null)
- `pose` (`[x, y, yaw]`)
- `linear_velocity_x`
- `linear_velocity_y`
- `angular_velocity_z`
- `emergency_stopped`
- `fall_detected` (humanoid agents only; always `false` on wheeled bases)

## Humanoid Shell

The humanoid agent type is a navigation-intent shell. `EmbodimentAdapter`
implementations for humanoids must expose:

- `fall_detected: bool` — observed each tick by the runtime
- `fall_reason: str`, `balance_margin: float` — optional metadata copied
  into the `FALL_DETECTED` event payload

`FrameSpec` carries `pelvis`, `left_foot`, and `right_foot` for humanoid
agents. Wheeled agents leave these as empty strings.

On the rising edge of `fall_detected`, the runtime emits `FALL_DETECTED`
followed by `SAFETY_STOP` with `data.reason="fall_detected"` and
`data.source="humanoid_adapter"`, then sets `emergency_stopped=True` so the
existing emergency-stop branch handles `adapter.stop` and
`COMMAND_REJECTED`. See [`humanoid.md`](humanoid.md) for the full contract
and the non-goals.

## Run Directory Layout

Every `gnav run` writes a self-contained directory under `--output-dir`
(default `runs/`). Required files:

- `scenario.yaml` — verbatim copy of the input scenario.
- `resolved_config.yaml` — the parsed scenario after schema normalization.
- `env.json` — host metadata: git sha/branch/dirty, ROS distro, Nav2 version
  (`nav2_version`, resolved from the Nav2 `package.xml` via the ament index,
  `""` when Nav2 is absent), Genesis version (if importable), Python version,
  hostname, platform, scenario id/seed, backend (`fallback` | `genesis` |
  `ros2_robot`), planner (`auto` | `grid` | `straight` | `nav2`), mode
  (`fast` | `realtime`), `ros_enabled`, `record_rosbag`.
- `events.jsonl` — one JSON record per line, each carrying `ts`,
  `episode_id`, `event`, optional `agent_id`/`task_id`/`data`. The first
  record is always `SCENARIO_STARTED` and the last `SCENARIO_FINISHED`.
- `metrics.json` — machine-readable summary; see the Metrics Schema
  section below.
- `report.md` — human-readable rendering of the metrics.
- `traces/` — reserved for backend-specific traces.
- `rosbag/` — present only when `--record` or the scenario opts in.
- `qos_profile.yaml` — copied from `--qos-profile` when `--ros` is set.

`gnav replay <run_dir>` validates this layout. It requires every file in
the list above, parses `events.jsonl` line by line, refuses runs that
start without `SCENARIO_STARTED` or end without `SCENARIO_FINISHED`, and
checks that `metrics.json` contains at least `scenario_id`, `seed`,
`success_rate`, and `sim_steps`. With `--print-events` it streams the
task lifecycle events (`TASK_ASSIGNED` → `TASK_STARTED` →
`TASK_SUCCEEDED`/`TASK_FAILED`) plus safety events (`SAFETY_STOP`,
`FALL_DETECTED`, `COLLISION`, `AGENT_STUCK`) in order. Exit codes:
`0` on success, `2` on validation failure.

## Benchmark Report Schema

`gnav bench --run <suite_dir>` aggregates per-scenario results into a
single JSON document. Top-level keys:

- `benchmark_suite` — directory name of the suite.
- `ran_at` — UTC timestamp string.
- `total`, `passed`, `failed` — counters across scenarios that actually ran.
- `skipped_count` — number of scenarios skipped (integration-only).
- `scenarios` — array of per-scenario entries, each with:
  - `scenario_id`, `scenario_path`, `seed`
  - `run_dir` — absolute or repo-relative path to the run artifact
  - `passed` — boolean
  - `failures` — list of human-readable predicate violations
  - `metrics` — the full `metrics.json` for the run
  - `expected` — the `benchmark.expected` block as declared
- `skipped` — array of `{scenario_id, scenario_path, reason}` for scenarios
  not run this invocation.

The predicate vocabulary is documented in
[`benchmarks.md`](benchmarks.md). Unknown predicate keys produce a
failure entry rather than being silently ignored.

A scenario declaring `benchmark.integration: true` depends on an external
stack (e.g. a live Nav2 `ComputePathToPose` server) and is therefore **not**
part of the deterministic regression set: `gnav bench --run` skips it by
default — recording it under `skipped` and logging the skip to stderr rather
than truncating coverage silently — and runs it only when
`--include-integration` is passed. `benchmarks/nav2_integration/` is the
reference integration suite.

## Metrics Schema

`metrics.json` is the machine-readable run summary. v0.1 fields:

- `scenario_id`, `seed`, `agent_count`, `task_count`
- `success_rate`, `task_succeeded_count`, `task_failed_count`
- `time_to_goal_mean_sec`, `path_length_mean_m`
- `command_accept_count`, `command_rejection_count`
- `task_dispatched_count`, `task_pending_peak`
- `reservation_granted_count`, `reservation_conflict_count`,
  `reservation_released_count`
- `collision_count`, `near_miss_count`, `emergency_stop_count`
- `sim_steps`, `sim_steps_per_sec`, `real_time_factor`
