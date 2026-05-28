# genesis-nav Plan

Last updated: 2026-05-28

## 0. Executive Summary

`genesis-nav` should not become "Nav2 for Genesis." The useful open-source
position is narrower, more durable, and more operational:

> ROS 2-native runtime infrastructure for embodied navigation, fleet simulation,
> replay, observability, and future real-robot deployment on Genesis World.

The project should win by being the thin, practical runtime layer between:

```text
Policy / VLA / RL / world model
        |
Embodied runtime infrastructure   <- genesis-nav
        |
ROS 2 / real robot / fleet / replay / telemetry
```

The first public version must show a working runtime, not a pile of abstractions.
The v0.1 target is a reproducible Genesis scenario that spawns multiple agents,
bridges stable ROS 2 interfaces, records runtime evidence, replays artifacts,
and reports metrics.

## 1. Product Positioning

### One-Sentence Definition

`genesis-nav` is ROS 2-native embodied runtime infrastructure for Genesis World.

### OSS-Friendly Definition

`genesis-nav` lets developers run, observe, record, replay, benchmark, and
eventually deploy multi-agent mobile robot and humanoid navigation workloads from
Genesis World into ROS 2-oriented robotics systems.

### Avoided Positioning

Do not describe the project as:

- Genesis navigation library
- Genesis RL environment
- Genesis Nav2 clone
- Demo collection
- Generic simulator abstraction
- VLA framework
- Photorealistic dataset generator
- Cloud fleet product

### Strategic Message

Embodied AI has policies. Robots need runtime.

The project should repeatedly demonstrate that the important value is not
"a planner moved a robot," but:

- the scenario is reproducible
- the runtime state is inspectable
- the ROS graph is stable
- the run is recorded
- the result can be replayed
- metrics can be compared
- safety gates are explicit
- real-robot adapters have a visible path

## 2. Design Principles

These principles are binding for v0.1:

1. `genesis-nav` is runtime infrastructure, not a policy framework.
2. Genesis is the primary simulator, ROS 2 is the public contract.
3. Replay, observability, and metrics are core features.
4. Multi-agent is the default, not an extension.
5. A real-robot deployment path must remain visible.
6. AI agents submit tasks; they do not drive actuators directly.
7. Humanoids are supported through navigation intent before full-body control.
8. Prefer simple working modules over abstract plugin systems.
9. Every public interface must be documented in `docs/interfaces.md`.
10. Every experiment must be reproducible from scenario, seed, config, and commit.

## 3. Current Repository State

The repository currently contains a v0.1 skeleton:

- Python package under `genesis_nav/`
- ROS 2 packages under `ros2_ws/src/`
- `gnav` CLI skeleton
- scenario loader
- smoke and warehouse scenario YAML
- `CommandGate`
- `AgentRegistry`
- JSONL event writer
- metrics report model
- ROS 2 msg/srv/action interface files
- QoS and rosbag profile configs
- docs and AI-agent instructions
- unit tests for the current runtime skeleton

This is enough to establish the intended shape of the repository, but not yet
enough to claim a working Genesis runtime or ROS 2 bridge demo.

## 4. v0.1 MVP

### MVP Statement

A user can install `genesis-nav`, run one command, watch multiple robots navigate
in Genesis, inspect ROS 2 topics, record a rosbag, replay the run, and see
metrics.

### Required Demo Command

```bash
gnav run examples/scenarios/warehouse_10_agents.yaml --fast --record
```

### Acceptance Criteria

v0.1 is done when:

- At least 3 mobile agents navigate to assigned goals in Genesis.
- `/clock` is published from the runtime.
- `/tf` and `/tf_static` are populated for agent frames.
- Each agent exposes namespace-scoped topics:
  - `/robot_i/state`
  - `/robot_i/odom`
  - `/robot_i/cmd_vel`
  - `/robot_i/task_status`
- Commands pass through `CommandGate`.
- Stale commands are rejected.
- Emergency stop is available.
- `events.jsonl` is generated.
- `metrics.json` is generated.
- `report.md` is generated.
- A rosbag directory is generated when recording is enabled.
- `gnav replay <run_dir>` validates the run and restores visible runtime state.
- `docs/interfaces.md` documents every public ROS 2 and runtime interface.
- `docs/experiments.md` contains baseline results.
- README quickstart stays under 10 commands.
- One humanoid shell scenario runs navigation intent and safety-stop logic.

## 5. Non-Goals for v0.1

The following should not be implemented in v0.1:

- Full Nav2 replacement
- Full Autoware port
- Full humanoid whole-body control
- RL training framework
- VLA framework
- Cloud fleet product
- Photorealistic dataset generator
- Generic simulator abstraction layer
- Plugin architecture for everything
- Multi-host distributed runtime
- Open-RMF adapter implementation
- Isaac/Gazebo/Webots/CoppeliaSim adapters
- Full behavior tree framework

## 6. Target Users

Primary early users:

- ROS 2 developers who want to use Genesis World.
- Embodied AI and robotics researchers who need evaluation/runtime, not only
  policy code.
- Robotics engineers exploring multi-agent, warehouse, fleet, or humanoid
  navigation in simulation-first workflows.
- Nav2, Autoware, and Open-RMF contributors interested in embodied runtime
  infrastructure.
- Developers using AI coding agents to grow robotics OSS.

The first implementation should not claim universal robot support. It should
focus on:

- differential and holonomic mobile bases
- simple mobile manipulator shells without manipulation
- humanoid navigation shells without full locomotion control

## 7. Architecture Boundaries

### Core Runtime

Location:

- `genesis_nav/core/`

Responsibilities:

- runtime clock
- agent registry
- task model
- lifecycle model
- authority model
- command gate
- runtime state transitions

Core runtime must not import Genesis or ROS 2 packages directly.

### Genesis Adapter

Location:

- `genesis_nav/genesis/`

Responsibilities:

- world loading
- entity spawning
- pose and twist extraction
- sensor extraction
- command application
- collision query
- simulation step
- reset
- seed control

Must not own:

- fleet scheduling
- task planning
- AI-agent decisions
- ROS 2 QoS policy
- benchmark scoring
- human-readable experiment logs

### ROS 2 Bridge

Locations:

- `genesis_nav/ros/`
- `ros2_ws/src/genesis_nav_ros/`
- `ros2_ws/src/genesis_nav_msgs/`

Responsibilities:

- `/clock` publishing
- per-agent state publishing
- odometry publishing
- tf publishing
- command subscription
- action/service interfaces
- QoS profile loading
- launch/bringup assets

ROS 2 is the stable public contract. Genesis API changes should be absorbed by
the Genesis adapter whenever possible.

### Observability

Location:

- `genesis_nav/observability/`

Responsibilities:

- runtime events
- metrics snapshots
- run directory layout
- report generation
- future tracing integration

Observability is not optional. If a runtime decision affects behavior, it should
be explainable from artifacts.

### Fleet

Location:

- `genesis_nav/fleet/`

Responsibilities:

- task queue
- dispatcher
- reservation manager
- traffic monitor
- resource locks
- future Open-RMF adapter boundary

v0.1 should use reservation-based coordination, not full MAPF.

### Navigation

Location:

- `genesis_nav/navigation/`

Responsibilities:

- minimal global planner
- minimal local controller
- state-machine behavior executor
- recovery hooks
- future Nav2 adapter boundary

v0.1 should avoid Nav2 plugin architecture. A direct, understandable path is
more useful for the first release.

### Humanoid

Location:

- `genesis_nav/humanoid/`

Responsibilities:

- humanoid agent schema
- pelvis/base/foot frame conventions
- navigation intent conversion
- fall-detected safety state
- balance-related state fields

Must not claim:

- whole-body MPC
- footstep planning
- full-body collision avoidance
- locomotion-policy training

### AI-Agent API

Location:

- `genesis_nav/agent/`

Responsibilities:

- list agents
- get world state
- submit tasks
- pause/resume agents
- stop all
- get task status
- get recent events

AI agents must not publish raw actuator commands.

## 8. Repository Shape

The intended monorepo structure is:

```text
genesis-nav/
├── README.md
├── PLAN.md
├── LICENSE
├── AGENTS.md
├── CLAUDE.md
├── pyproject.toml
├── package.xml
├── CMakeLists.txt
├── docs/
├── genesis_nav/
├── ros2_ws/
│   └── src/
│       ├── genesis_nav_msgs/
│       ├── genesis_nav_ros/
│       ├── genesis_nav_bringup/
│       └── genesis_nav_examples/
├── examples/
├── configs/
├── scripts/
├── tests/
└── tools/
```

Rules:

- Python runtime code belongs in `genesis_nav/`.
- ROS 2 package artifacts belong in `ros2_ws/src/`.
- Public contracts are documented in `docs/interfaces.md`.
- Experiments and benchmark results are documented in `docs/experiments.md`.
- Architectural tradeoffs are documented in `docs/decisions.md`.

## 9. Workstreams

### Workstream A: Repository and Developer Experience

Goal:

Make the repository easy to install, test, and understand.

Tasks:

- Keep README quickstart working.
- Keep `gnav doctor` useful.
- Add `CONTRIBUTING.md`.
- Add scenario contribution guide.
- Add release checklist.
- Add good-first-issue templates.
- Add CI for Python unit tests.
- Add CI for ROS 2 interface build.
- Add docs contract CI.
- Keep AI-agent guidance in `AGENTS.md`, `CLAUDE.md`, `.cursor/rules/`, and
  `.agents/skills/`.

Acceptance:

- New contributor can run Python tests in a venv.
- ROS 2 developer can build message packages with colcon.
- Interface-change rules are visible before editing code.

### Workstream B: Scenario System

Goal:

Make scenarios the unit of reproducibility.

Tasks:

- Formalize scenario YAML schema.
- Add schema validation errors with useful messages.
- Support deterministic seed.
- Support world entry point.
- Support agents list.
- Support tasks list.
- Support metrics list.
- Support record config.
- Support resource zones.
- Support map/resource references.
- Add `gnav bench`.
- Add scenario lint tool.

Acceptance:

- `gnav run examples/scenarios/smoke.yaml --fast` works.
- Invalid scenarios fail before runtime startup.
- Run artifact includes original scenario and resolved config.

### Workstream C: Runtime Clock and Simulation Loop

Goal:

Build deterministic runtime stepping before adding advanced planning.

Tasks:

- Add fixed-step runtime loop.
- Add simulation modes:
  - `fast`
  - `realtime`
  - `lockstep`
  - `replay`
- Add pause/resume/step controls.
- Add `/clock` publisher.
- Add episode ID.
- Add seed propagation.
- Add real-time-factor measurement.
- Add clean shutdown path.

Acceptance:

- Runtime can run without wall-time sync.
- Runtime can run near real time for teleop/RViz.
- Pause, resume, and step events are logged.
- `/clock` timestamps align with event log timestamps.

### Workstream D: Agent Registry

Goal:

Make every agent visible to runtime, ROS 2, fleet, observability, and AI tools.

Tasks:

- Keep `AgentSpec` and `AgentState` minimal.
- Add namespace and frame validation.
- Add capability filtering.
- Add current task tracking.
- Add health state.
- Add emergency stop state.
- Add humanoid-specific extension path.
- Add registry snapshot export.

Acceptance:

- Runtime can list all agents.
- Agent state can be published to ROS 2.
- AI-agent API can list agents without touching actuators.
- Event log can reference `agent_id` consistently.

### Workstream E: Command Gate and Safety

Goal:

Ensure no planner, AI agent, or teleop path bypasses command safety.

Tasks:

- Enforce velocity limits.
- Enforce stale-command rejection.
- Enforce command TTL.
- Add emergency stop service boundary.
- Add per-agent locks.
- Add authority arbitration:
  - teleop
  - safety
  - autonomy
  - AI
- Add command decision events.
- Add unit tests for every rejection path.

Acceptance:

- AI velocity commands are rejected by default.
- Stale commands are rejected.
- Emergency stopped agents reject commands.
- Lower authority commands cannot override active higher authority locks.
- Accepted/rejected commands produce structured events.

### Workstream F: Genesis Adapter

Goal:

Connect runtime to Genesis without letting Genesis-specific API details leak into
the core runtime.

Tasks:

- Add world loader.
- Add entity registry/mapping.
- Add mobile-base entity adapter.
- Add sensor adapter skeleton.
- Add actuator adapter skeleton.
- Add collision query wrapper.
- Add reset support.
- Add seed support.
- Add fast smoke world.
- Add basic warehouse world.

Acceptance:

- Smoke scenario spawns at least one robot in Genesis.
- Warehouse scenario spawns multiple robots in Genesis.
- Runtime core unit tests still pass without Genesis installed.
- Genesis-specific failures produce actionable `gnav doctor` output.

### Workstream G: ROS 2 Bridge

Goal:

Expose stable ROS 2 contracts for runtime inspection and future real-robot
deployment.

Tasks:

- Publish `/clock`.
- Publish `/genesis_nav/events`.
- Publish `/genesis_nav/fleet_state`.
- Publish `/genesis_nav/scenario_state`.
- Publish per-agent `/state`.
- Publish per-agent `/odom`.
- Publish tf and tf_static.
- Subscribe per-agent `/cmd_vel`.
- Wire `/cmd_vel` through `CommandGate`.
- Load QoS from `configs/qos/default.yaml`.
- Add launch files.
- Add lifecycle-node plan or skeleton.

Acceptance:

- `ros2 topic list` shows runtime and per-agent topics.
- `ros2 topic info --verbose` matches documented QoS where possible.
- Command subscriber does not bypass command gate.
- `docs/interfaces.md` matches generated interfaces.

### Workstream H: Navigation MVP

Goal:

Provide enough navigation behavior to demonstrate runtime value without becoming
a full navigation stack.

Tasks:

- Add straight-line planner baseline.
- Add grid A* planner for occupancy maps.
- Add simple local controller.
- Add velocity limiter integration.
- Add behavior state machine:
  - `IDLE`
  - `ASSIGNED`
  - `PLANNING`
  - `RESERVING`
  - `EXECUTING`
  - `RECOVERING`
  - `SUCCEEDED`
  - `FAILED`
- Add stuck detection.
- Add wait behavior.
- Add stop behavior.

Acceptance:

- Single-agent navigate-to-pose works.
- At least 3 agents can complete assigned tasks in the warehouse scenario.
- Behavior transitions are visible in event logs.

### Workstream I: Fleet and Reservation Runtime

Goal:

Demonstrate multi-agent runtime coordination without overbuilding MAPF.

Tasks:

- Add task queue.
- Add dispatcher.
- Add resource model.
- Add reservation manager.
- Add resource lease events.
- Add narrow-aisle demo.
- Add wait/retry behavior.
- Add simple congestion/deadlock detection.

Acceptance:

- Agents reserve constrained resources before entering.
- Conflicting reservations cause wait or retry.
- Reservation decisions are logged.
- Fleet metrics include wait time and resource conflicts.

### Workstream J: Observability and Replay

Goal:

Make every run explainable and reproducible.

Tasks:

- Finalize run directory layout.
- Add structured event taxonomy.
- Add metrics collector.
- Add report generator.
- Add ROS graph snapshot.
- Add git SHA capture.
- Add Genesis version capture when installed.
- Add ROS distro capture.
- Add QoS profile capture.
- Add `gnav replay` artifact validation.
- Add initial event-log replay.
- Add rosbag profile support.

Acceptance:

- Every run has scenario, config, events, metrics, and report.
- Runtime decisions can be inspected from `events.jsonl`.
- Metrics are machine-readable.
- Replay validates required artifacts and restores visible state for the demo.

### Workstream K: Benchmarks

Goal:

Use benchmarks as regression harnesses first, not as leaderboard marketing.

Tasks:

- Add `benchmarks/nav_basic/`.
- Add `benchmarks/multi_agent/`.
- Add `benchmarks/runtime/`.
- Add `benchmarks/humanoid/`.
- Add benchmark report JSON format.
- Add CI-friendly smoke benchmark.
- Add benchmark history to `docs/experiments.md`.

Initial scenarios:

- `single_agent_empty.yaml`
- `single_agent_obstacles.yaml`
- `narrow_passage.yaml`
- `4_agents_crossing.yaml`
- `10_agents_warehouse.yaml`
- `deadlock_corridor.yaml`
- `pause_resume.yaml`
- `replay_consistency.yaml`
- `rosbag_roundtrip.yaml`
- `humanoid_nav_intent.yaml`
- `fall_stop.yaml`

Acceptance:

- Benchmark scenarios are deterministic.
- Metrics have stable names and JSON shape.
- Failing benchmark output points to replay artifacts.

### Workstream L: Humanoid Shell

Goal:

Show humanoid-ready runtime interfaces without overclaiming locomotion.

Tasks:

- Add humanoid agent type.
- Add pelvis/base/foot frame conventions.
- Add humanoid state schema.
- Add navigation intent adapter.
- Add fall-detected safety stop.
- Add humanoid navigation intent scenario.
- Document non-goals clearly.

Acceptance:

- Humanoid shell scenario runs.
- Navigate-to-pose becomes base intent or locomotion intent.
- `fall_detected` triggers emergency stop.
- README and docs avoid claiming full humanoid locomotion.

### Workstream M: AI-Agent Tool API

Goal:

Let AI agents operate the runtime through task and supervision APIs, never raw
actuator APIs.

Tasks:

- Add `list_agents()`.
- Add `get_world_state()`.
- Add `submit_task()`.
- Add `pause_agent()`.
- Add `resume_agent()`.
- Add `stop_all()`.
- Add `get_task_status()`.
- Add `get_recent_events()`.
- Add requester ID and trace ID enforcement.
- Add docs in `docs/ai_agents.md`.

Acceptance:

- AI tool API cannot publish `cmd_vel`.
- Submitted tasks pass through runtime validation.
- AI-issued tasks are traceable in events.

### Workstream N: Community and Launch

Goal:

Make the project legible and useful to early contributors.

Tasks:

- Add contributor guide.
- Add scenario contribution guide.
- Add 10 good-first issues.
- Add architecture issue template.
- Add benchmark contribution template.
- Add robot adapter issue template.
- Add first demo GIF.
- Add 2-minute demo video script.
- Add comparison table in README or docs.
- Prepare ROS Discourse launch post.
- Prepare Genesis community launch post.

Acceptance:

- A new contributor can identify where to contribute in under 5 minutes.
- The README shows what works in 30 seconds.
- The first demo shows runtime, ROS 2 topics, replay, and metrics.

## 10. Three-Month Schedule

### Month 1: Runtime Skeleton and ROS 2 Bridge

#### Week 1: Repository Bootstrap

Status target:

- Done or nearly done.

Tasks:

- Initialize monorepo.
- Add Python package.
- Add ROS 2 workspace skeleton.
- Add README positioning.
- Add docs skeleton.
- Add AGENTS/CLAUDE/Cursor/skills.
- Add smoke scenario.
- Add `gnav` CLI skeleton.
- Add Python CI.
- Add ROS 2 CI.

Exit criteria:

- `gnav doctor` runs.
- `gnav run examples/scenarios/smoke.yaml --fast` writes artifacts.
- Python unit tests pass.
- ROS 2 interface packages build.

#### Week 2: Fixed-Step Runtime and Agent State

Tasks:

- Implement runtime loop.
- Add runtime clock.
- Add sim modes.
- Add pause/resume/step state.
- Add agent registry validation.
- Add per-agent state update path.
- Add event types for runtime lifecycle.

Exit criteria:

- Runtime can step deterministically.
- Agent state snapshots are emitted.
- Scenario seed and episode ID are recorded.

#### Week 3: Command Path and Safety

Tasks:

- Wire command gate into runtime.
- Add command accepted/rejected events.
- Add emergency stop path.
- Add command timeout tests.
- Add ROS 2 command subscriber skeleton.
- Add rosbag profile selection.

Exit criteria:

- No command path bypasses `CommandGate`.
- Stale commands are rejected.
- Emergency stop is visible in state and events.

#### Week 4: Single-Agent Navigation

Tasks:

- Add simple planner.
- Add simple local controller.
- Add behavior state machine.
- Add NavigateEmbodied action server skeleton.
- Add single-agent demo.
- Capture first rough demo clip.

Exit criteria:

- Single agent can reach a simple goal.
- Runtime events show task assignment, plan, execution, and success/failure.

### Month 2: Multi-Agent, Fleet, and Observability

#### Week 5: Multi-Agent Spawn and Namespaces

Tasks:

- Spawn multiple agents from scenario.
- Publish namespace-scoped state.
- Publish namespace-scoped odom.
- Publish tf frames.
- Add event log coverage for each agent.

Exit criteria:

- Multiple agents appear in ROS 2 graph.
- Runtime state is namespaced and unambiguous.

#### Week 6: Task Queue, Dispatcher, and Reservation Manager

Tasks:

- Add task queue.
- Add dispatcher.
- Add resource model YAML.
- Add lease-based reservation.
- Add reservation events.

Exit criteria:

- Runtime assigns tasks to available agents.
- Resource conflicts produce wait/retry behavior.

#### Week 7: Warehouse Scenario

Tasks:

- Build warehouse scenario.
- Add constrained aisle resources.
- Add blocked aisle event.
- Add wait/recover behavior.
- Add collision and near-miss metrics.

Exit criteria:

- At least 3 agents complete tasks in the warehouse scenario.
- Metrics include safety and fleet values.

#### Week 8: Replay and Benchmark Runner

Tasks:

- Expand run artifact layout.
- Add replay validation.
- Add first event-log replay behavior.
- Add benchmark runner.
- Add `report.md` generation.
- Record baseline in `docs/experiments.md`.

Exit criteria:

- A run can be validated and replayed enough for the demo.
- Benchmark output is deterministic and machine-readable.

### Month 3: Humanoid Shell, AI Tools, and v0.1 Release

#### Week 9: Humanoid Navigation Shell

Tasks:

- Add humanoid agent type.
- Add humanoid frame schema.
- Add humanoid state message or extension.
- Add fall detection state.
- Add humanoid navigation intent scenario.

Exit criteria:

- Humanoid shell scenario runs.
- Fall detection triggers safety stop.

#### Week 10: AI Tool API

Tasks:

- Implement local Python API.
- Add CLI operator commands if time allows.
- Add list agents.
- Add submit task.
- Add stop all.
- Add event trace IDs.
- Update `docs/ai_agents.md`.

Exit criteria:

- AI-agent API can submit a task but cannot drive velocity.
- Task requester and trace ID are recorded.

#### Week 11: Adapter RFCs and Interface Cleanup

Tasks:

- Write Nav2 adapter RFC.
- Write Open-RMF adapter RFC.
- Write real robot adapter design doc.
- Clean ROS 2 launch files.
- Review QoS profiles.
- Review docs/interfaces.md.

Exit criteria:

- v0.2 adapter direction is clear.
- v0.1 public interfaces are documented.

#### Week 12: v0.1 Release

Tasks:

- Freeze v0.1 scope.
- Record demo GIF.
- Record 2-minute demo video.
- Complete docs.
- Complete benchmark report.
- Prepare release notes.
- Publish launch posts.

Exit criteria:

- v0.1 tag is ready.
- README has demo, quickstart, architecture, comparison, roadmap, contributing.
- Launch material explains why this is runtime infrastructure.

## 11. P0 Issue Backlog

### Bootstrap

- `[repo]` Initialize monorepo with Python and ROS 2 workspace.
- `[docs]` Add README positioning and non-goals.
- `[docs]` Add `docs/interfaces.md` initial contract.
- `[docs]` Add `docs/decisions.md` with initial ADRs.
- `[ai]` Add AGENTS.md, CLAUDE.md, Cursor rules, and skills.
- `[ci]` Add pytest and colcon GitHub Actions.

### Genesis Runtime

- `[genesis]` Add minimal Genesis world loader.
- `[runtime]` Implement fixed-step simulation loop.
- `[runtime]` Add RuntimeClock and sim-time modes.
- `[cli]` Implement `gnav run`.
- `[scenario]` Define scenario YAML schema.
- `[test]` Add smoke scenario test.

### ROS 2 Bridge

- `[msgs]` Add AgentState.msg.
- `[msgs]` Add NavigateEmbodied.action.
- `[ros]` Publish `/clock` from runtime.
- `[ros]` Publish per-agent state topic.
- `[ros]` Subscribe per-agent `cmd_vel`.
- `[ros]` Add QoS profile loader.
- `[ros]` Add launch file for smoke demo.

### Command and Safety

- `[runtime]` Implement CommandGate.
- `[safety]` Add command timeout.
- `[safety]` Add emergency stop service.
- `[safety]` Add authority modes.
- `[test]` CommandGate rejects stale command.

### Navigation

- `[nav]` Add minimal global planner.
- `[nav]` Add simple local controller.
- `[nav]` Implement NavigateEmbodied server.
- `[behavior]` Add navigate/wait/recover state machine.
- `[demo]` Add single-agent navigate-to-pose demo.

### Observability

- `[obs]` Add events.jsonl writer.
- `[obs]` Add metrics collector.
- `[obs]` Add run output directory structure.
- `[bag]` Add rosbag recording profiles.
- `[cli]` Add `gnav replay` skeleton.

## 12. P1 Issue Backlog

### Multi-Agent and Fleet

- `[multi-agent]` Spawn multiple agents from scenario.
- `[fleet]` Add TaskQueue.
- `[fleet]` Add Dispatcher.
- `[fleet]` Add ResourceManager with leases.
- `[fleet]` Add narrow aisle reservation demo.
- `[bench]` Add warehouse_10_agents benchmark.

### Humanoid

- `[humanoid]` Add HumanoidAgent schema.
- `[humanoid]` Add pelvis/base/foot frame conventions.
- `[humanoid]` Add fall_detected safety state.
- `[humanoid]` Add humanoid_nav_intent demo.

### AI Agent

- `[agent]` Add Python tool API.
- `[agent]` Add list_agents.
- `[agent]` Add submit_task.
- `[agent]` Add stop_all.
- `[docs]` Add docs/ai_agents.md safety model.

### Community

- `[docs]` Add contributing guide.
- `[docs]` Add scenario contribution guide.
- `[demo]` Record first demo GIF.
- `[release]` Prepare v0.1.0 release notes.

## 13. Public Interface Plan

### ROS 2 Topics

Runtime:

- `/clock`
- `/genesis_nav/events`
- `/genesis_nav/fleet_state`
- `/genesis_nav/scenario_state`

Per agent:

- `/<agent>/state`
- `/<agent>/odom`
- `/<agent>/cmd_vel`
- `/<agent>/scan`
- `/<agent>/task_status`
- `/<agent>/diagnostics`

Global:

- `/tf`
- `/tf_static`

### ROS 2 Actions

- `NavigateEmbodied.action`
- `ExecuteTaskGraph.action`

### ROS 2 Services

- `ReserveResource.srv`
- emergency stop service, exact name still to be decided
- pause/resume/step services, exact names still to be decided

### QoS Contract

QoS belongs in `configs/qos/default.yaml` and must be mirrored in
`docs/interfaces.md`.

Initial policy:

- `/clock`: reliable, transient local
- `/robot_*/scan`: best effort, volatile
- `/robot_*/cmd_vel`: reliable, volatile, deadline target 100 ms
- `/genesis_nav/events`: reliable, volatile

## 14. Scenario and Run Artifact Plan

### Scenario Is the Reproducibility Unit

Direct Python demos should not be the primary entry point. The entry point should
be a scenario file:

```text
scenario.yaml
  -> world loader
  -> runtime
  -> ROS 2 bridge
  -> event log
  -> metrics report
```

### Run Directory Shape

```text
runs/
└── 2026-05-28_warehouse_10_agents_seed42/
    ├── scenario.yaml
    ├── resolved_config.yaml
    ├── events.jsonl
    ├── metrics.json
    ├── report.md
    ├── rosbag/
    └── traces/
```

### Required Metadata

Every run should eventually include:

- scenario ID
- episode ID
- seed
- git SHA
- Genesis version
- ROS distro
- QoS profile hash or copy
- runtime config
- robot config
- metric names and values

## 15. Metrics Plan

### Navigation Metrics

- success rate
- time to goal
- path length
- final pose error
- number of replans
- stuck time

### Safety Metrics

- collision count
- near miss count
- minimum distance to agents
- command rejection count
- emergency stop count

### Fleet Metrics

- task throughput
- average wait time
- resource conflict count
- deadlock count
- task completion latency

### Runtime Metrics

- real-time factor
- sim steps per second
- ROS publish latency
- dropped messages
- replay determinism score

### Humanoid Metrics

- fall count
- minimum balance margin
- support phase errors

## 16. Testing Plan

### Unit Tests

Required areas:

- scenario validation
- agent registry
- runtime clock
- command gate
- reservation manager
- metrics formatting
- event writer
- AI-agent safety constraints

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/unit
```

### Integration Tests

Required areas:

- `gnav run` artifact output
- `gnav replay` validation
- smoke scenario lifecycle
- multi-agent scenario parsing
- command-gate event logging

### ROS 2 Tests

Required areas:

- message package build
- bridge node startup
- QoS profile loading
- `/clock` publication
- command topic subscription
- action/service availability

Command:

```bash
colcon build --base-paths ros2_ws/src --packages-select genesis_nav_msgs genesis_nav_ros
colcon test --base-paths ros2_ws/src --packages-select genesis_nav_msgs genesis_nav_ros
```

### Scenario Tests

Required areas:

- smoke
- single-agent empty map
- single-agent obstacle map
- multi-agent crossing
- warehouse 10 agents
- humanoid nav intent
- fall stop

## 17. Risk Register

### Risk: Genesis API churn

Impact:

- Runtime code becomes fragile if Genesis-specific types leak everywhere.

Mitigation:

- Keep Genesis imports under `genesis_nav/genesis/`.
- Keep core runtime unit-testable without Genesis.
- Add adapter tests with fake entities before relying on full Genesis.

### Risk: Recreating Nav2 badly

Impact:

- Project becomes a weak navigation stack instead of a strong runtime substrate.

Mitigation:

- Keep v0.1 planner/controller minimal.
- Document non-goals.
- Add Nav2 adapter later instead of cloning architecture early.

### Risk: Plugin architecture too early

Impact:

- Repo turns into interfaces with no working demo.

Mitigation:

- Add only `EmbodimentAdapter`-level boundaries in v0.1.
- Require an issue before adding plugin systems.

### Risk: ROS 2 QoS mismatch

Impact:

- Topics exist but messages do not flow.

Mitigation:

- Treat QoS as interface contract.
- Keep `configs/qos/default.yaml`.
- Document QoS in `docs/interfaces.md`.
- Add bridge debug skill and tests.

### Risk: AI-agent safety overreach

Impact:

- AI tools may accidentally bypass runtime safety.

Mitigation:

- AI agents submit tasks only.
- Command publishing remains blocked.
- All AI tasks include requester and trace IDs.

### Risk: Demo looks like "just simulation"

Impact:

- Project is misunderstood as another simulator demo.

Mitigation:

- Demo must show ROS 2 topics, event log, metrics, replay, and fleet tasks.
- README must emphasize runtime infrastructure.

## 18. Release Criteria

### v0.1.0-alpha

Can be released when:

- Python package installs in a venv.
- Smoke scenario runs without Genesis.
- ROS 2 message packages build.
- CommandGate tests pass.
- Scenario artifacts are generated.
- README and docs are coherent.

Purpose:

- Invite early design feedback.
- Establish project direction.

### v0.1.0

Can be released when:

- Genesis warehouse demo runs.
- ROS 2 graph is visible.
- Multi-agent tasks execute.
- rosbag profile recording path works.
- replay command demonstrates visible state recovery or documented artifact
  replay.
- metrics report is generated.
- humanoid shell demo runs.
- docs/interfaces.md is complete for v0.1.
- docs/experiments.md contains baseline results.
- demo GIF is in README.

Purpose:

- Public launch.
- Contributor recruitment.
- Foundation for v0.2 adapters.

## 19. Launch Assets

Required:

- 20-second demo GIF.
- 2-minute demo video.
- architecture diagram.
- comparison table.
- quickstart.
- "why runtime infrastructure?" essay.
- benchmark report.
- release notes.
- ROS Discourse post.
- Genesis community post.

The demo should show:

- 10 robots in a warehouse.
- ROS 2 topics visible.
- fleet tasks assigned.
- collision avoidance or waiting.
- event log streaming.
- rosbag/replay path.
- metrics report.

The launch should avoid claiming:

- full autonomy stack
- full humanoid locomotion
- full Nav2 replacement
- generic simulator support

## 20. v0.2 Direction

After v0.1:

- Nav2 adapter.
- real robot adapter.
- teleop adapter.
- hardware diagnostics.
- better rosbag replay.
- Open-RMF adapter RFC implementation.
- distributed runtime design.
- ros2_tracing integration.
- richer benchmark suite.

v0.2 should still preserve the same core identity: Genesis-first runtime,
ROS 2-native public contract, reproducible scenarios, observability-first
debuggability, and no raw actuator access from AI agents.
