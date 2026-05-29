# Decisions

## 2026-05-28: Position genesis-nav as runtime infrastructure

Context:
Genesis World provides the simulation substrate. ROS 2, Nav2, Autoware, and
Open-RMF already cover adjacent layers.

Decision:
Build `genesis-nav` as ROS 2-native embodied runtime infrastructure for Genesis
World, not as a Nav2 clone, RL framework, VLA framework, or demo wrapper.

Consequences:
Runtime, replay, observability, multi-agent support, and real-robot deployment
interfaces are core from the first version.

## 2026-05-28: Use ROS 2 namespaces for agent isolation in v0.1

Context:
Multi-agent support is required early, but DDS-domain partitioning adds
deployment complexity.

Decision:
Use one `ROS_DOMAIN_ID` with per-agent namespaces for v0.1.

Consequences:
RViz, rosbag, and local development stay simple. Large-fleet networking and
multi-host topologies will be revisited later.

## 2026-05-28: Run the ROS 2 bridge in-process inside `gnav run`

Context:
The v0.1 bridge could be either a standalone `rclpy` node or an in-process
component embedded in the runtime loop. A standalone node would mean writing
its own clock/event consumer and either polling the runtime state via IPC or
duplicating the loop.

Decision:
Embed `RosBridge` in `gnav run --ros` so it shares the runtime clock, registry,
command gate, and event sink directly. The bridge implements the `EventSink`
protocol and is fanned out alongside the JSONL writer. The marker package
`genesis_nav_ros` exists only so other ROS 2 packages can express a build/exec
dependency on the bridge boundary; the implementation lives in the Python
package `genesis_nav.ros`.

Consequences:
No IPC layer is needed for v0.1. The trade-off is that `--ros` requires the
runtime to be launched from a process where `rclpy` and `genesis_nav_msgs` are
importable. A standalone bridge node is left as a v0.2 option if remote
inspection becomes a requirement.

## 2026-05-28: External `/cmd_vel` must traverse `CommandGate`

Context:
ROS 2 teleop tools, joysticks, and other operator paths publish to
`/<agent>/cmd_vel`. If the bridge applied these directly to the embodiment
adapter, the runtime would lose the AI/AUTONOMY/SAFETY/TELEOP arbitration
contract and the freshness / E-stop checks.

Decision:
The bridge converts each incoming `Twist` into a `RuntimeCommand` stamped with
`AuthorityMode.TELEOP`, `source="ros_cmd_vel"`, and the current sim time, then
calls `CommandGate.evaluate` before invoking
`Runtime.apply_external_command`. Rejected commands emit `COMMAND_REJECTED`
runtime events; accepted commands emit `COMMAND_ACCEPTED`.

Consequences:
Teleop commands can override autonomy when they arrive faster than the
controller can re-issue, which matches operator expectations. AI agents still
cannot drive actuators because the gate rejects `AuthorityMode.AI` velocity
commands by default.

## 2026-05-28: World files own Genesis imports; Runtime stays Genesis-free

Context:
`Runtime` and the scenario loader must remain importable in environments where
Genesis is not installed (CI for the core Python package, contributors using
the fallback diff-drive integrator). At the same time we need a single,
documented place for Genesis-specific imports so the rest of the codebase does
not grow conditional `import genesis` blocks.

Decision:
The scenario `world` field points to a Python module that defines
`build_scene(seed)` and `spawn_diff_drive(scene, spec)`. World modules import
`genesis` lazily inside these functions. The Runtime never imports Genesis
directly; it receives an `EmbodimentAdapter` per agent through the
`adapter_factory` hook on `Runtime.from_scenario`. The CLI selects between
backends via `--backend fallback|genesis`.

Consequences:
Adding new worlds is a single-file change. Adding new backends is a new
factory + adapter pair under `genesis_nav/<backend>/`. Genesis API churn is
contained to `genesis_nav/genesis/` and the world files.

## 2026-05-28: Nearest-fit dispatch + leave-in-queue retry for unmatched tasks

Context:
The v0.1 fleet needs to match tasks to agents without committing to a full
MAPF solver. Tasks may arrive with explicit `agent_id`, with a capability
filter, or with a `nearest_to` hint.

Decision:
The dispatcher uses a three-step strategy: (1) honor explicit `agent_id` if
the agent is free, (2) filter by `capabilities`, (3) pick the candidate with
the smallest Euclidean distance to `nearest_to` (defaulting to the task goal
when `nearest_to` is omitted). Tasks that cannot be matched right now are
re-queued at the head so they get another chance on the next tick instead of
failing immediately.

Consequences:
Behavior is deterministic and easy to reason about. Future MAPF/Open-RMF
adapters can replace the dispatcher without touching the queue or the runtime
state machine.

## 2026-05-28: Resources are lease-managed; runtime owns release on task end

Context:
Shared resources (narrow aisles, charging docks) must be reservable but must
not leak leases if an agent's task ends or fails mid-flight.

Decision:
The Runtime tracks active leases per agent in `_leases_by_agent`. On
`TASK_SUCCEEDED` and `TASK_FAILED` (including timeout failures), every lease
held by that agent is released through `release_resource`, which emits a
`RESOURCE_RELEASED` event. Unknown resource IDs are rejected before they
reach the manager so scenario typos surface immediately.

Consequences:
Resource ownership matches task lifetime without operator intervention. The
metrics expose `reservation_granted_count`, `reservation_conflict_count`, and
`reservation_released_count` so deadlock-style scenarios produce a
machine-readable signal.

## 2026-05-28: AI tool API is the only Python surface AI agents may use

Context:
AI agents need a way to operate the runtime (submit tasks, pause/resume
agents, query state, read recent events) without bypassing the
authority arbitration that protects actuators. Letting them touch
`Runtime.apply_external_command`, the registry, or the event sink directly
would defeat the safety contract.

Decision:
`genesis_nav.agent.AgentToolApi`, constructed via `Runtime.tool_api`, is the
sole supported entry point for AI agents. It exposes read-only snapshots
(`AgentSnapshot`, `WorldSnapshot`, `TaskSnapshot`) and four write methods
(`submit_task`, `pause_agent`, `resume_agent`, `stop_all`). Every write
requires a non-empty `requester_id`; `submit_task` auto-stamps a UUID4
`trace_id` if the caller omits one. `submit_task` routes through
`Runtime.submit_task`, so it still passes the dispatcher and the command
gate. Pause/resume/stop operate via `AgentRegistry.emergency_stop`, which
the step loop already honors. All AI-originated events carry
`data.source = "ai_tool_api"` plus the `requester_id`.

Consequences:
There is one place to audit AI-agent safety. New AI capabilities must be
added here or rejected. Direct registry mutation by AI tools is a contract
violation. The runtime can grow new entry points without renegotiating the
safety boundary.

## 2026-05-28: Use an in-memory ring buffer for `get_recent_events`

Context:
`get_recent_events` needs to be cheap (AI agents may call it frequently
during deliberation) and must not require re-reading or tailing
`events.jsonl`. A full event database is overkill for v0.1.

Decision:
`RingBufferEventSink` is a capacity-bounded `deque` registered alongside
`JsonlEventWriter` in the runtime's `FanoutEventSink`. Default capacity is
2048 events in the CLI, configurable per construction. `AgentToolApi`
filters the buffer by `event` / `agent_id` / `task_id` / `since_ts` and
returns the most recent matches up to `limit`. The JSONL file remains the
durable record.

Consequences:
The tail query is O(capacity) and runs in-process, so it is fast even for
fleets that generate many events per tick. Older events fall off; agents
that need historical replay should read `events.jsonl` or query a future
trace store instead.

## 2026-05-28: Humanoid v0.1 is a navigation-intent shell, not a locomotion stack

Context:
Humanoid agents need to appear in scenarios so that frame conventions, fleet
plumbing, ROS interfaces, and the safety-stop pipeline can be exercised. A
real whole-body controller (gait, balance, footstep planning) is out of scope
for v0.1 and would dwarf the rest of the runtime.

Decision:
The humanoid agent type uses `HumanoidIntentAdapter`, which treats incoming
velocity commands as *base-frame navigation intent* and integrates the
pelvis-projected base pose with the same planar kinematics as
`DiffDriveKinematics`. It carries `fall_detected` / `balance_margin` /
`fall_reason` so the runtime can poll for safety conditions. `FrameSpec`
gains optional `pelvis`, `left_foot`, and `right_foot` fields used by
humanoid scenarios only.

The runtime polls every adapter once per tick. On the rising edge of
`fall_detected` it emits `FALL_DETECTED` (the observation) and
`SAFETY_STOP` with `reason="fall_detected"` (the action), then sets
`emergency_stopped=True` to reuse the existing per-task stop branch. No new
emergency-stop code path is introduced.

Consequences:
The shell is honest: the README and `docs/humanoid.md` explicitly say it
does not simulate gait, balance, or whole-body control. When a real
locomotion adapter is added (Genesis-backed or otherwise), it only needs to
expose the same `fall_detected` signal — no runtime changes are required.

## 2026-05-28: Run-directory is the single source of truth for replay

Context:
Reproducibility for v0.1 needs scenario, host environment, event log, and
metrics in one place. A separate trace database, remote store, or DB would
add deployment burden without buying anything until v0.2.

Decision:
Every `gnav run` writes a self-describing directory containing
`scenario.yaml`, `resolved_config.yaml`, `env.json` (git sha/branch/dirty,
ROS distro, Genesis version, Python version, hostname, platform, backend,
mode), `events.jsonl`, `metrics.json`, `report.md`, plus optional
`qos_profile.yaml` (under `--ros`) and `rosbag/` (under `--record`).
`gnav replay` parses these directly: it requires every artifact, refuses
malformed `events.jsonl`, requires `SCENARIO_STARTED`/`SCENARIO_FINISHED`
at both ends, and verifies `metrics.json` carries the documented keys.
`--print-events` reads the same file to stream the task and safety
lifecycle.

Consequences:
The runtime never depends on a remote service to be "replayable"; a run
directory is sufficient evidence. Adding a future event-replay loop or
trace store is additive — the on-disk format is already the contract.

## 2026-05-28: Benchmarks are scenario + expectation, run via `gnav bench --run`

Context:
Benchmarks must be regression harnesses first. A separate benchmark
runner with its own scheduler, fixtures, and metrics pipeline would
fragment the runtime contract — every benchmark would need to
reimplement the things `gnav run` already provides (env capture, event
log, replay artifact, metrics.json).

Decision:
A benchmark is a normal scenario YAML with an optional top-level
`benchmark.expected` block. `gnav bench --run <suite_dir>` discovers
`*.yaml` files, executes each through the same code path as `gnav run`
under `--fast`, and evaluates the predicates against `metrics.json`.
The aggregated report (`benchmarks/_runs/<suite>_report.json`) lists
failures next to the `run_dir` so failures point straight at the replay
artifact.

Consequences:
There is exactly one runtime code path; benchmarks cannot drift from
production behaviour. Adding a benchmark is a one-file change and a
predicate. Failing benchmarks are debuggable with the same
`gnav replay` machinery as ordinary runs. A future "real" benchmark
runner can wrap this command without forking the runtime.

## 2026-05-28: Behavior state machine is orthogonal to TaskStatus

Context:
The runtime needs to expose what each agent is doing right now (planning,
chasing waypoints, waiting after a stuck event, recovering) so replays and
RViz panels can show navigation behavior. Overloading `TaskStatus` with these
nuances would tie task-side reporting to runtime-side scheduling and make
the AI tool API harder to reason about.

Decision:
Introduce a `BehaviorState` enum tracked on `AgentState.behavior_state`
(`idle / assigned / planning / reserving / executing / recovering /
succeeded / failed`). `TaskStatus` keeps its existing meaning ("what is
the status of this task?") while `BehaviorState` answers "what is the agent
doing right now in service of that task?". Every transition emits a
`BEHAVIOR_STATE_CHANGED` runtime event carrying `from`, `to`, and `reason`.
The legal transitions are encoded in
`genesis_nav.navigation.behavior.can_transition`; the runtime refuses to
publish events that violate the machine.

Consequences:
RViz, ROS bridge consumers, and replay tooling get a deterministic stream
of behavior transitions. The AI tool API can read `behavior_state` without
having to derive it from event traces. Future locomotion or planning
extensions slot in by emitting new transitions inside the same vocabulary
instead of inventing parallel status fields.

## 2026-05-28: Static occupancy grid + GridAStarPlanner for v0.1

Context:
The v0.1 navigation MVP needs to demonstrate planning around obstacles
without depending on Nav2 or a costmap layer. A separate planner package
or plugin system would burn the budget the workstream actually buys.

Decision:
Scenarios may declare a top-level `occupancy_grid` block. When present, the
runtime constructs a `GridAStarPlanner` (8-connected A*, Octile heuristic,
no diagonal corner-cutting through blocked cells). When absent, the
runtime falls back to `StraightLinePlanner`. The planner runs once per
task on the `ASSIGNED -> PLANNING -> EXECUTING` transition; the controller
chases the resulting waypoint queue with a fixed `waypoint_tolerance_m`.

Consequences:
Workstream H ships a single, debuggable planning path. Dynamic obstacles,
costmaps, and incremental replanning are deferred. The on-disk grid format
is small enough that benchmark scenarios can describe non-trivial maps in
YAML, and replays remain reproducible because the grid is checked into the
scenario alongside `seed` and `agents`.

## 2026-05-28: Keep Genesis-specific code behind a thin adapter

Context:
Genesis APIs are expected to evolve quickly.

Decision:
Keep Genesis-specific imports and API calls under `genesis_nav/genesis/`. Core
runtime modules must be unit-testable without Genesis installed.

Consequences:
The runtime can stabilize its own contracts while tracking Genesis changes in a
smaller adapter surface.

## 2026-05-28: Curated good-first-issues list lives in docs, not just GitHub

Context:
GitHub issues are the operational backlog, but the list churns and historic
"good first" labels rot quickly. New contributors landing on the repo need a
stable answer to "where do I start?" that the maintainers update intentionally.

Decision:
Maintain a curated catalogue of ten concrete starter tasks in
`docs/good_first_issues.md`. Each entry names files, size estimate, and the
matching issue template. The corresponding GitHub issues are filed from this
list rather than the other way around; if a list entry no longer applies, we
edit the doc in the same PR that fixes the underlying state.

Consequences:
New contributors get a predictable starting surface that survives label drift,
and we have a single place to audit whether the project still has friendly
on-ramps. The cost is that the list must be reviewed each minor release —
stale entries become a credibility problem faster than missing entries.

## 2026-05-28: Public comparison table lives in README, not a separate page

Context:
"How does this compare to Nav2 / Isaac Lab / Gazebo / raw Genesis?" is the
first question newcomers ask. A separate `docs/comparison.md` would let it
grow longer, but a comparison hidden behind a click loses its job — it has
to anchor expectations before someone reads any code.

Decision:
Keep a single short comparison table in `README.md` covering ROS 2-native
runtime, multi-agent scenarios, replayable artifacts, benchmark predicates,
AI safety boundary, and real-robot path. Expand individual rows only when
the deeper discussion fits naturally in an existing doc (e.g., the Nav2
boundary lives in this ADR file; the AI surface lives in `docs/ai_agents.md`).

Consequences:
The table stays opinionated and short. We accept that it cannot answer every
nuance — that is what the linked docs are for. If the table grows past about
eight rows or starts hedging, that is a signal to split it out and re-open
this ADR.

## 2026-05-29: Real-robot adapter is an `EmbodimentAdapter`, not a new runtime path

Status: Proposed (v0.2). No code exists yet; this ADR fixes the contract so
the v0.1 → v0.2 boundary is honest and the real-robot path stays visible in
runtime interfaces (per `AGENTS.md`).

Context:
v0.1 ships two embodiments — the deterministic `DiffDriveKinematics` fallback
and the Genesis-backed adapter — both behind the `EmbodimentAdapter` Protocol
(`read_pose`, `apply_command`, `stop`). v0.2's headline is "ROS 2 robot
deployment" (Roadmap Phase 2). The temptation is to add a parallel
"real-robot mode" with its own command flow. That would fork the safety
contract: the whole point of `CommandGate` is that *nothing* reaches an
actuator without arbitration, freshness, and E-stop checks.

Decision:
A real robot is just another `EmbodimentAdapter` selected via a new backend
factory (`--backend ros2_robot`, implementation under `genesis_nav/ros2_robot/`).
It satisfies the exact same three-method contract:
- `read_pose()` reads the latest `/odom` (or `tf` `map→base_link`) sample held
  by a spun `rclpy` node; it never blocks the runtime tick.
- `apply_command(command, dt_sec)` receives a `RuntimeCommand` **that has
  already passed `CommandGate`** and publishes the corresponding `Twist`
  (or `FollowJointTrajectory` goal for non-diff-drive bases) to the robot.
- `stop(reason)` latches a zero-velocity command and trips the hardware E-stop
  surface.
The adapter is the *outbound* hardware edge and is deliberately distinct from
the v0.1 *inbound* `/cmd_vel → CommandGate → apply_external_command` teleop
path (see the 2026-05-28 `/cmd_vel` ADR). Sim-real parity means the *same*
scenario YAML runs against `--backend genesis` or `--backend ros2_robot`; only
the factory changes. A hardware watchdog (max command age) is the adapter's
responsibility and emits `SAFETY_STOP` through the existing event sink on
staleness.

Update (2026-05-29): the command-staleness watchdog auto-poll is now wired.
`Runtime._poll_safety_signals` polls each adapter's `watchdog_expired` (on the
transport's monotonic clock, not sim time) and, on the rising edge, emits
`SAFETY_STOP` (`reason="command_watchdog"`), zeroes the actuator via
`adapter.stop`, latches the emergency stop, and bumps `watchdog_stop_count`.
The check is duck-typed, so sim/Genesis/humanoid adapters never participate.
It is latched (no auto-clear on command resume) and stops the actuator directly
rather than only via the per-task branch, so an idle/teleop robot whose command
pipeline stalls is still zeroed. This closes the follow-up flagged in the
2026-05-29 diagnostics ADR.

Consequences:
There is still exactly one place actuators can be driven and one arbiter in
front of it. AI agents cannot reach hardware for the same reason they cannot
reach the sim — `apply_command` only ever sees gate-approved commands. The new
work is confined to a node lifecycle, QoS choices, and a frame/units mapping;
the runtime loop, dispatcher, behavior machine, and replay format are
untouched. The open risk is real-time latency: if `read_pose`/`apply_command`
cannot meet the tick budget over DDS, v0.2 may need an async command buffer —
that is a follow-up ADR, not a contract change.

Update (2026-05-29, same day): the real-robot loop is now closeable in
process. The earlier `--backend ros2_robot` slice proved the *outbound* edge
(commands reach `/cmd_vel`) but, with no robot to integrate them, poses stayed
at origin — loop closure was future work. `real_robot.transport: loopback`
selects a `LoopbackRobotTransport` (rclpy-free) that does the part a real robot
does itself: integrate the commanded velocity into its own pose and report it
back as odom, using the same diff-drive model as `DiffDriveKinematics`. The
adapter is unchanged — the *same* `Ros2RobotAdapter`, only the transport
differs — so the full contract (`CommandGate` → `apply_command` →
`publish_velocity` → odom feedback → `read_pose` → controller) runs end to end
and deterministically, without `rclpy` or hardware. Verified e2e: `--backend
ros2_robot` with `transport: loopback` reaches the smoke goal in 265 sim steps
(matching the fallback baseline, confirming the integration model agrees) and
replays strictly. The transport keeps the realistic one-tick odom lag (the
backend integrates from its per-tick `step(dt)`, the loopback equivalent of
draining odom callbacks). This makes the real-robot path a first-class,
testable citizen of core CI rather than a dry connection. The DDS-latency
follow-up above is unchanged.

## 2026-05-29: Nav2 is a planner backend behind `plan()`, not a runtime replacement

Status: Proposed (v0.2). Supersedes the forward-looking half of the
2026-05-28 "static occupancy grid + GridAStarPlanner" ADR, which deferred the
Nav2 question; v0.1's "no Nav2 plugin layer" stance is unchanged for v0.1.

Context:
The v0.1 comparison table and identity statement promise genesis-nav is *not*
a Nav2 clone. Yet real-robot deployment (Phase 2) will want Nav2's mature
costmaps, recovery behaviors, and controllers. The risk is two-sided: either
we reimplement Nav2 (violating the identity), or we let Nav2 own the runtime
loop (losing the arbitration, observability, and replay that are our core).

Decision:
Nav2 enters as a `Nav2Planner` that implements the same `plan()` signature as
`GridAStarPlanner` / `StraightLinePlanner`, bridging to an already-running
Nav2 stack via an `rclpy` `NavigateToPose` action client. genesis-nav remains
the runtime, arbiter, and observability owner:
- Planning is *delegated*; the resulting path still flows through the existing
  `PLAN_RESOLVED` event and behavior state machine, so replays look identical
  regardless of planner backend.
- The `cmd_vel` Nav2's controller produces is treated as an external command
  stamped `AuthorityMode.AUTONOMY` and **must traverse `CommandGate`** — the
  same mechanism as teleop, so safety arbitration and E-stop still win.
- genesis-nav never registers a Nav2 plugin and never reimplements a Nav2
  planner; the boundary is the action/topic contract, nothing deeper.
Backend selection is `runtime.navigation.planner: grid | straight | nav2`
(this generalizes the selector requested in issue #9).

Consequences:
Sim scenarios keep using the in-tree deterministic planners (replayable, no
ROS dependency); real deployments can borrow Nav2 without genesis-nav
absorbing Nav2's surface area or losing its own contracts. The cost is that a
`nav2` run is only as reproducible as the external stack — so `env.json` must
record the Nav2 distro/version, and benchmark suites must mark `nav2`
scenarios as integration-only, not part of the deterministic regression set.

Update (2026-05-29, same day): the `env.json` half of that reproducibility
requirement is now closed. `collect_env_metadata` records `nav2_version`,
resolved from the Nav2 `package.xml` (`<version>`) via `ament_index_python`
across `nav2_bringup` / `nav2_msgs` / `nav2_core`. The lookup is best-effort
and never raises — core CI and pure-sim runs have no ament index, so the field
collects as `""`. `ros_distro` already captured the distro half. A replay of a
`nav2` run now states which Nav2 it ran against.

Update (2026-05-29, same day): the benchmark half is also closed. A scenario
may declare `benchmark.integration: true`; `gnav bench --run` skips such
scenarios by default (recording them under the report's `skipped` array and
logging the skip — no silent truncation) and runs them only with
`--include-integration`. `benchmarks/nav2_integration/single_agent_nav2.yaml`
(a `planner: nav2` scenario) is the reference: the deterministic suites stay
green without a live Nav2 server, while the integration path is one flag away.

Update (2026-05-29, same day): the last Nav2 follow-up — routing Nav2's
controller `cmd_vel` through `CommandGate` — is now landed sim-first.
`runtime.navigation.controller: nav2` selects a `Nav2Controller`, a drop-in for
`SimpleLocalController` backed by the `Nav2ControllerService` boundary (real
bridge subscribes to each agent's `cmd_vel`; `FakeNav2ControllerService` for
tests). The key decision: `Nav2Controller.compute()` returns an ordinary
`RuntimeCommand`, so Nav2's velocity flows through the *existing* `CommandGate`
evaluation on the autonomy path — no new apply path, and the AI safety boundary
holds by construction. A non-finite/over-limit Nav2 velocity is rejected and
the agent stopped (proved by a unit test); when Nav2 has no command yet the
controller falls back to the in-tree controller so motion degrades gracefully.
`COMMAND_ACCEPTED` now carries `source` so a replay shows whether `navigation`
or `nav2_controller` drove each command. This closes the Nav2 ADR's deferred
items; only deeper Nav2 surface (costmap layers, recovery behaviors) remains
explicitly out of scope per the project identity.

## 2026-05-29: Dynamic obstacles and replanning extend the planner contract, not a new subsystem

Status: Proposed (v0.2). Names the third deferred item from issue #10 so the
boundary is explicit.

Context:
v0.1's `GridAStarPlanner` plans once on the `ASSIGNED → PLANNING → EXECUTING`
transition against a static grid checked into the scenario. Real environments
have moving obstacles, and Phase 2/3 fleets need agents to wait or replan
rather than drive through a freshly occupied cell. The wrong move is a new
"dynamic world" subsystem that bypasses the deterministic, replayable grid.

Decision:
Dynamic obstacles arrive through an `ObstacleSource` protocol that publishes
timestamped grid deltas into the runtime; each delta is recorded as a runtime
event so replays reproduce the exact obstacle timeline from the run directory
alone. Replanning is a new legal edge in the existing behavior machine
(`executing → planning`) triggered when a chased waypoint becomes blocked;
planners gain an optional `replan(from_pose, grid_snapshot)` that defaults to a
full `plan()` for backends that cannot do incremental updates. No parallel
state field and no out-of-band world mutation are introduced.

Consequences:
Determinism and the run-directory-as-truth contract survive: an obstacle
stream is just more events, and a replay reconstructs the same plan/replan
sequence. Planners opt into incremental replanning when they can and degrade
to full replans otherwise. The cost is a new event type and a behavior-machine
edge that every replay consumer must tolerate; that is additive, matching the
"new transitions inside the same vocabulary" principle from the behavior-state
ADR.

## 2026-05-29: Teleop is a first-class runtime API; autonomy yields during a hold

Status: Implemented (v0.2). Realizes the teleop-adapter item of Roadmap
Phase 2 and makes good on the `apply_external_command` docstring's promise that
"the autonomy loop is suspended for any agent under teleop control".

Context:
v0.1 only exposed the operator path through the ROS bridge's `/cmd_vel`
subscription, which inlined the gate-evaluate / event-emit / apply sequence and
needed `rclpy` to exercise at all. Two problems followed: the
operator-overrides-autonomy guarantee was untestable without ROS, and nothing
actually stopped the autonomy loop from re-issuing a command on the very next
tick and fighting the operator.

Decision:
Add `Runtime.submit_teleop_command(agent_id, *, requester_id, ...)` as the
transport-agnostic operator entry point. It stamps a `TELEOP` `RuntimeCommand`
with the operator's `requester_id` and the current sim time, evaluates it
through the same `CommandGate`, emits `COMMAND_ACCEPTED` / `COMMAND_REJECTED`,
and on accept calls `apply_external_command` and records a per-agent teleop
*hold* (`sim_time + navigation.teleop_hold_sec`). The step loop checks this
hold and yields the autonomy loop for that agent until it expires, so the
agent retains the operator's last command instead of being overwritten. The
hold is tracked in sim time (not the gate's monotonic authority lock) so it
stays deterministic and replayable.

Update (2026-05-29, same day): the ROS bridge `/cmd_vel` path was unified onto
this API. `RosBridge._on_cmd_vel` is now pure transport — it forwards the Twist
to an injected `teleop_command_handler` wired to
`submit_teleop_command(..., requester_id="ros_cmd_vel")`. The bridge no longer
builds `RuntimeCommand`s, evaluates the gate, or emits command events itself,
and it no longer takes a `CommandGate` (the gate lives in the runtime). This
closed two real gaps in the old bridge path: external `/cmd_vel` carried no
`requester_id` (violating the authority/requester/timestamp metadata rule) and
did not set the autonomy hold (so an operator over ROS did not actually suppress
autonomy). Both are now fixed by construction.

Consequences:
The operator-override contract is now exercised in core CI without `rclpy`,
and teleop genuinely suppresses autonomy for the hold window. Because the hold
is sim-time based, a teleop session is reproducible from the run directory.
`requester_id` is mandatory, satisfying the authority/requester/timestamp
metadata rule for command sources. One sharp edge: a teleop command stamped at
sim time 0 is rejected as stale (freshness requires `issued_at > 0`), which is
correct for live runs but means the very first tick cannot be teleoperated.

## 2026-05-29: Diagnostics are a folded read-model + optional periodic event

Status: Implemented (v0.2). Realizes the hardware-diagnostics item of Roadmap
Phase 2 and gives the real-robot command-staleness watchdog a consumer.

Context:
The real-robot adapter exposes a `watchdog_expired` helper, but nothing
consumed it, and there was no single place to ask "is this fleet healthy?".
Health signals were scattered across `AgentState` (`emergency_stopped`,
`fall_detected`), the behavior machine (`failed`), and per-adapter watchdogs.
A heavyweight `diagnostic_updater`-style subsystem would dwarf the need.

Decision:
Add `collect_diagnostics(states, adapters)`, a pure function that folds those
signals into per-agent `DiagnosticLevel` (`OK < WARN < ERROR`, ordered like
ROS 2 `diagnostic_msgs`) with an overall = worst-agent level. Adapter signals
are duck-typed (`watchdog_expired` / `seconds_since_command`), so sim adapters
that lack them simply do not contribute — the same pattern as
`_poll_safety_signals`. `Runtime.diagnostics()` wraps it as an always-available
read-model; `AgentToolApi.get_diagnostics()` exposes it read-only to AI agents
(safe — no mutation). When `navigation.diagnostics_interval_sec > 0` the step
loop emits a periodic `DIAGNOSTICS` event so a replay reconstructs the health
timeline; the default (0) keeps quiet runs quiet.

Consequences:
The watchdog now has a consumer, and health is one call away for RViz panels,
AI deliberation, and operators. Because the collector is pure and duck-typed,
new health axes (battery, temperature, comms loss) are added by raising a level
inside one function, not by wiring a new subsystem. Emission is off by default
to avoid bloating event logs; turning it on is a per-scenario knob. The level
mapping is intentionally coarse (three levels, watchdog = WARN); a richer
taxonomy (e.g. ROS `STALE`) can be layered later without breaking the contract.
