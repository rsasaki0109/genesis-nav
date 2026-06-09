# Architecture

## Positioning

`genesis-nav` is not a navigation algorithm library. It is the embodied runtime
control plane between policies, planners, simulators, ROS 2, replay artifacts,
fleet tasks, and future real robots.

## Runtime → ROS 2 fanout

```mermaid
flowchart LR
  subgraph runtime [Runtime core]
    R[Runtime tick]
    CG[CommandGate]
    ER[AgentRegistry]
    EV[EventSink]
  end
  subgraph bridge [ROS boundary]
    BR[RosBridge]
  end
  subgraph ros2 [ROS 2 graph]
    CLK["/clock"]
    EVT["/genesis_nav/events"]
    ST["/{agent}/state"]
    ODOM["/{agent}/odom"]
    DIAG["/genesis_nav/diagnostics"]
  end
  R --> CG
  R --> ER
  R --> EV
  EV --> BR
  BR --> CLK
  BR --> EVT
  BR --> ST
  BR --> ODOM
  BR --> DIAG
  CG -.->|external /cmd_vel| BR
```

## Task lifecycle

```mermaid
stateDiagram-v2
  [*] --> idle
  idle --> assigned: TASK_ASSIGNED
  assigned --> planning: planner_start
  planning --> executing: plan_ready
  executing --> planning: replan / headon / obstacle
  executing --> recovering: stuck
  recovering --> executing: recovery_complete
  executing --> succeeded: goal_reached
  planning --> failed: plan_failed
  executing --> failed: timeout / blocked
  succeeded --> idle: task_complete
  failed --> idle: task_complete
```

## Command authority chain

```mermaid
flowchart TD
  OP[Operator teleop / ROS /cmd_vel] --> STC[submit_teleop_command]
  NAV[Autonomy loop / Nav2 controller] --> CG[CommandGate]
  AI[AgentToolApi] -->|tasks pause stop only| RT[Runtime]
  STC --> CG
  CG -->|accepted| ADP[EmbodimentAdapter.apply_command]
  CG -->|rejected| EV[COMMAND_REJECTED event]
  ADP --> SIM[Fallback / Genesis / ros2_robot]
```

## Core Split

Control plane:

- task assignment
- fleet state
- resource leases
- runtime mode
- safety authority
- scenario control

Data plane:

- odometry
- tf
- sensor data
- velocity commands
- images
- point clouds

## Runtime Components

- `AgentRegistry`: source of truth for agent identity, namespace, frames, and
  capabilities.
- `CommandGate`: authority arbitration, velocity limits, stale-command
  rejection, emergency stop handling.
- `TaskDispatcher`: assigns task specs to agents.
- `ReservationManager`: lease-based resource locking.
- `EventBus`: structured runtime events for replay and debugging.
- `SafetySupervisor`: stops agents, pauses simulation, and rejects unsafe
  commands.

## Adapter Rule

Genesis-specific API usage belongs under `genesis_nav/genesis/`. Runtime core
must be unit-testable without Genesis installed.

ROS 2-specific code belongs under `genesis_nav/ros/` and `ros2_ws/src/`.
Public ROS 2 contracts are documented in `docs/interfaces.md`.
