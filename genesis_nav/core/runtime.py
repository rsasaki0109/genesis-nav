"""Fixed-step runtime loop used by the v0.1 CLI and tests."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from genesis_nav.benchmarks.scenario import Scenario
from genesis_nav.core.agent import AgentRegistry
from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.clock import RuntimeClock, SimulationMode
from genesis_nav.core.command_gate import CommandDecision, CommandGate, RuntimeCommand
from genesis_nav.core.embodiment import DiffDriveKinematics, EmbodimentAdapter
from genesis_nav.core.lifecycle import LifecycleState
from genesis_nav.core.task import TaskSpec, TaskStatus
from genesis_nav.fleet.dispatcher import Dispatcher
from genesis_nav.fleet.queue import TaskQueue
from genesis_nav.fleet.reservation import ReservationManager
from genesis_nav.fleet.resources import ResourceCatalog
from genesis_nav.humanoid.adapter import HumanoidIntentAdapter
from genesis_nav.navigation.behavior import BehaviorState, can_transition
from genesis_nav.navigation.config import CollisionConfig, NavigationConfig
from genesis_nav.navigation.global_planner import StraightLinePlanner
from genesis_nav.navigation.grid_planner import (
    GridAStarPlanner,
    PlannerError,
    build_planner,
)
from genesis_nav.navigation.obstacles import ObstacleSource, build_obstacle_source
from genesis_nav.navigation.local_controller import SimpleLocalController
from genesis_nav.observability.diagnostics import (
    DiagnosticsReport,
    collect_diagnostics,
)
from genesis_nav.observability.events import EventSink

StepCallback = Callable[[float], None]
AdapterFactory = Callable[["object"], "EmbodimentAdapter"]  # noqa: F821


@dataclass
class _TaskRecord:
    task_id: str
    agent_id: str
    goal: tuple[float, float, float] | None
    assigned_at_sec: float
    started_at_sec: float | None = None
    finished_at_sec: float | None = None
    status: TaskStatus = TaskStatus.ASSIGNED
    path_length_m: float = 0.0
    failure_reason: str = ""


@dataclass
class RuntimeMetrics:
    tasks: dict[str, _TaskRecord] = field(default_factory=dict)
    command_accept_count: int = 0
    command_rejection_count: int = 0
    sim_steps: int = 0
    task_pending_peak: int = 0
    task_dispatched_count: int = 0
    reservation_granted_count: int = 0
    reservation_conflict_count: int = 0
    reservation_released_count: int = 0
    stuck_event_count: int = 0
    recovery_count: int = 0
    plan_failure_count: int = 0
    replan_count: int = 0
    obstacle_event_count: int = 0
    watchdog_stop_count: int = 0
    collision_count: int = 0
    near_miss_count: int = 0
    yield_count: int = 0

    def summary(self) -> dict[str, float | int]:
        total = len(self.tasks)
        succeeded = [t for t in self.tasks.values() if t.status is TaskStatus.SUCCEEDED]
        failed = [t for t in self.tasks.values() if t.status is TaskStatus.FAILED]
        success_rate = (len(succeeded) / total) if total else 0.0
        durations = [
            t.finished_at_sec - t.assigned_at_sec
            for t in succeeded
            if t.finished_at_sec is not None
        ]
        time_to_goal_mean = sum(durations) / len(durations) if durations else 0.0
        path_lengths = [t.path_length_m for t in succeeded]
        path_length_mean = sum(path_lengths) / len(path_lengths) if path_lengths else 0.0
        return {
            "success_rate": success_rate,
            "time_to_goal_mean_sec": time_to_goal_mean,
            "path_length_mean_m": path_length_mean,
            "task_succeeded_count": len(succeeded),
            "task_failed_count": len(failed),
            "command_accept_count": self.command_accept_count,
            "command_rejection_count": self.command_rejection_count,
            "sim_steps": self.sim_steps,
            "task_dispatched_count": self.task_dispatched_count,
            "task_pending_peak": self.task_pending_peak,
            "reservation_granted_count": self.reservation_granted_count,
            "reservation_conflict_count": self.reservation_conflict_count,
            "reservation_released_count": self.reservation_released_count,
            "stuck_event_count": self.stuck_event_count,
            "recovery_count": self.recovery_count,
            "plan_failure_count": self.plan_failure_count,
            "replan_count": self.replan_count,
            "obstacle_event_count": self.obstacle_event_count,
            "watchdog_stop_count": self.watchdog_stop_count,
            "collision_count": self.collision_count,
            "near_miss_count": self.near_miss_count,
            "yield_count": self.yield_count,
        }


class Runtime:
    """Fixed-step runtime that owns the agent registry, command gate, and adapters."""

    def __init__(
        self,
        registry: AgentRegistry,
        command_gate: CommandGate,
        events: EventSink,
        *,
        clock: RuntimeClock | None = None,
        adapters: dict[str, EmbodimentAdapter] | None = None,
        controller: SimpleLocalController | None = None,
        task_queue: TaskQueue | None = None,
        dispatcher: Dispatcher | None = None,
        reservations: ReservationManager | None = None,
        resources: ResourceCatalog | None = None,
        planner: object | None = None,
        navigation_config: NavigationConfig | None = None,
        collision_config: CollisionConfig | None = None,
        obstacle_source: ObstacleSource | None = None,
    ) -> None:
        self.registry = registry
        self.command_gate = command_gate
        self.events = events
        self.clock = clock or RuntimeClock()
        self.adapters: dict[str, EmbodimentAdapter] = adapters or {}
        self.controller = controller or SimpleLocalController()
        self.lifecycle_state = LifecycleState.ACTIVE
        self.metrics = RuntimeMetrics()
        self._motion_started: set[str] = set()
        self.task_queue = task_queue or TaskQueue()
        self.dispatcher = dispatcher or Dispatcher(self.registry, self.task_queue)
        self.reservations = reservations or ReservationManager()
        self.resources = resources or ResourceCatalog()
        self._leases_by_agent: dict[str, list[str]] = {}
        self._current_episode_id: str = ""
        self.planner = planner or StraightLinePlanner()
        self.navigation_config = navigation_config or NavigationConfig()
        self.collision_config = collision_config or CollisionConfig()
        self._obstacle_source = obstacle_source
        self._waypoints: dict[str, list[tuple[float, float, float]]] = {}
        self._pose_history: dict[
            str, deque[tuple[float, float, float]]
        ] = {}
        self._recovery_resume_at_sec: dict[str, float] = {}
        self._recovery_retries: dict[str, int] = {}
        self._teleop_hold_until: dict[str, float] = {}
        self._last_diagnostics_emit_sec: float = 0.0
        # Agents whose real-robot command-staleness watchdog has already tripped,
        # so the rising edge fires the safety stop exactly once.
        self._watchdog_tripped: set[str] = set()
        # Agent pairs currently inside the collision / near-miss radius, so each
        # approach is counted once on the rising edge (cleared on separation).
        self._collision_pairs: set[frozenset[str]] = set()
        self._near_miss_pairs: set[frozenset[str]] = set()
        # Agents currently yielding right-of-way, so AGENT_YIELDED fires once per
        # yield episode (on the rising edge).
        self._yielding: set[str] = set()

    @classmethod
    def from_scenario(
        cls,
        scenario: Scenario,
        events: EventSink,
        *,
        clock: RuntimeClock | None = None,
        adapter_factory: Callable[["AgentSpec"], EmbodimentAdapter] | None = None,  # noqa: F821
    ) -> "Runtime":
        registry = AgentRegistry()
        registry.register_many(scenario.agents)
        adapters: dict[str, EmbodimentAdapter] = {}
        for spec in scenario.agents:
            if adapter_factory is not None:
                adapters[spec.agent_id] = adapter_factory(spec)
            else:
                spawn = spec.spawn or (0.0, 0.0, 0.0)
                if spec.embodiment == "humanoid":
                    adapters[spec.agent_id] = HumanoidIntentAdapter(
                        agent_id=spec.agent_id, x=spawn[0], y=spawn[1], yaw=spawn[2]
                    )
                else:
                    adapters[spec.agent_id] = DiffDriveKinematics(
                        agent_id=spec.agent_id, x=spawn[0], y=spawn[1], yaw=spawn[2]
                    )
        resources = ResourceCatalog.from_scenario(scenario.raw)
        navigation_config = NavigationConfig.from_scenario_raw(scenario.raw)
        collision_config = CollisionConfig.from_scenario_raw(scenario.raw)
        planner = cls._select_planner(scenario, navigation_config)
        controller = cls._select_controller(scenario, navigation_config)
        obstacle_source = build_obstacle_source(scenario.raw)
        return cls(
            registry=registry,
            command_gate=CommandGate(),
            events=events,
            clock=clock,
            adapters=adapters,
            resources=resources,
            planner=planner,
            controller=controller,
            navigation_config=navigation_config,
            collision_config=collision_config,
            obstacle_source=obstacle_source,
        )

    @staticmethod
    def _select_planner(scenario: Scenario, navigation_config: NavigationConfig):  # type: ignore[no-untyped-def]
        """Choose the planner backend from ``runtime.navigation.planner``.

        ``auto`` (default) keeps v0.1 behaviour: grid if the scenario declares an
        ``occupancy_grid``, else straight line. ``nav2`` delegates global
        planning to a running Nav2 stack via a lazily-imported `rclpy` bridge
        (see the 2026-05-29 Nav2 ADR).
        """

        choice = navigation_config.planner
        grid = build_planner(scenario.raw)
        if choice == "straight":
            return StraightLinePlanner()
        if choice == "grid":
            if grid is None:
                raise ValueError(
                    "runtime.navigation.planner: grid requires an 'occupancy_grid' block"
                )
            return grid
        if choice == "nav2":
            from genesis_nav.nav2.bridge import build_nav2_planner

            return build_nav2_planner(scenario)
        return grid or StraightLinePlanner()

    @staticmethod
    def _select_controller(scenario: Scenario, navigation_config: NavigationConfig):  # type: ignore[no-untyped-def]
        """Choose the local controller from ``runtime.navigation.controller``.

        ``local`` (default) keeps the in-tree `SimpleLocalController`. ``nav2``
        delegates velocity generation to a running Nav2 controller server via a
        lazily-imported `rclpy` bridge; that `cmd_vel` still traverses
        `CommandGate` as an ``AUTONOMY`` command on the autonomy path (see the
        2026-05-29 Nav2 ADR).
        """

        if navigation_config.controller == "nav2":
            from genesis_nav.nav2.bridge import build_nav2_controller

            return build_nav2_controller(scenario)
        return SimpleLocalController()

    # ------------------------------------------------------------------ tasks

    def assign_task(self, task: TaskSpec, *, ts: float, episode_id: str) -> None:
        if not task.agent_id:
            raise ValueError(f"task '{task.task_id}' requires an agent for direct assign")
        self.registry.assign_task(task.agent_id, task.task_id)
        state = self.registry.get_state(task.agent_id)
        state.current_task_status = TaskStatus.ASSIGNED
        state.current_goal = task.goal
        self.metrics.tasks[task.task_id] = _TaskRecord(
            task_id=task.task_id,
            agent_id=task.agent_id,
            goal=task.goal,
            assigned_at_sec=ts,
        )
        self.events.write(
            ts=ts,
            episode_id=episode_id,
            agent_id=task.agent_id,
            event="TASK_ASSIGNED",
            task_id=task.task_id,
            data={
                "goal": task.goal,
                "task_type": task.task_type,
                "requester_id": task.requester_id,
                "trace_id": task.trace_id,
            },
        )
        self._transition_behavior(
            state,
            BehaviorState.ASSIGNED,
            ts=ts,
            episode_id=episode_id,
            reason="task_assigned",
            task_id=task.task_id,
        )

    def submit_task(self, task: TaskSpec, *, episode_id: str | None = None) -> None:
        """Queue a task for the dispatcher to assign on the next tick.

        Tasks without a fixed `agent_id` are matched by the dispatcher using
        the optional `agent_selector.capabilities` and `nearest_to` hints.
        """

        self.task_queue.submit(task)
        self.metrics.task_pending_peak = max(
            self.metrics.task_pending_peak, len(self.task_queue)
        )
        self.events.write(
            ts=self.clock.sim_time_sec,
            episode_id=episode_id or self._current_episode_id,
            agent_id=task.agent_id or "",
            event="PLAN_CREATED",
            task_id=task.task_id,
            data={
                "goal": task.goal,
                "task_type": task.task_type,
                "priority": task.priority,
                "requester_id": task.requester_id,
                "trace_id": task.trace_id,
            },
        )

    def dispatch_pending(self, *, episode_id: str | None = None) -> int:
        ep = episode_id or self._current_episode_id
        ts = self.clock.sim_time_sec

        def _assign(task: TaskSpec) -> None:
            self.assign_task(task, ts=ts, episode_id=ep)

        results = self.dispatcher.tick(_assign)
        self.metrics.task_dispatched_count += len(results)
        return len(results)

    # ----------------------------------------------------------- reservations

    def reserve_resource(
        self,
        resource_id: str,
        requester_id: str,
        duration_sec: float,
        *,
        episode_id: str | None = None,
    ):  # type: ignore[no-untyped-def]
        if self.resources and resource_id not in self.resources:
            self.events.write(
                ts=self.clock.sim_time_sec,
                episode_id=episode_id or self._current_episode_id,
                agent_id=requester_id,
                event="RESOURCE_RELEASED",
                data={"resource_id": resource_id, "reason": "unknown_resource"},
            )
            return None
        lease = self.reservations.reserve(
            resource_id, requester_id, duration_sec, now_sec=self.clock.sim_time_sec
        )
        ep = episode_id or self._current_episode_id
        if lease is None:
            self.metrics.reservation_conflict_count += 1
            self.events.write(
                ts=self.clock.sim_time_sec,
                episode_id=ep,
                agent_id=requester_id,
                event="RESOURCE_RELEASED",
                data={"resource_id": resource_id, "reason": "conflict"},
            )
            return None
        self.metrics.reservation_granted_count += 1
        self._leases_by_agent.setdefault(requester_id, []).append(lease.lease_id)
        self.events.write(
            ts=self.clock.sim_time_sec,
            episode_id=ep,
            agent_id=requester_id,
            event="RESOURCE_RESERVED",
            data={
                "resource_id": resource_id,
                "lease_id": lease.lease_id,
                "expires_at_sec": lease.expires_at_sec,
            },
        )
        return lease

    def release_resource(
        self,
        lease_id: str,
        *,
        requester_id: str = "",
        episode_id: str | None = None,
    ) -> bool:
        released = self.reservations.release(lease_id)
        if released:
            self.metrics.reservation_released_count += 1
            ep = episode_id or self._current_episode_id
            self.events.write(
                ts=self.clock.sim_time_sec,
                episode_id=ep,
                agent_id=requester_id,
                event="RESOURCE_RELEASED",
                data={"lease_id": lease_id},
            )
        return released

    # ------------------------------------------------------------ life cycle

    def pause(self) -> None:
        self.lifecycle_state = LifecycleState.PAUSED

    def resume(self) -> None:
        self.lifecycle_state = LifecycleState.ACTIVE

    def step_paused(self, *, episode_id: str, count: int = 1) -> None:
        previous = self.lifecycle_state
        self.lifecycle_state = LifecycleState.ACTIVE
        try:
            for _ in range(count):
                self.step(episode_id=episode_id)
        finally:
            self.lifecycle_state = previous

    def has_pending_work(self) -> bool:
        if any(state.current_task_id for state in self.registry.list_states()):
            return True
        return len(self.task_queue) > 0

    def apply_external_command(self, command: RuntimeCommand) -> None:
        """Apply an externally-sourced command that already passed CommandGate.

        Used by the ROS bridge to forward `/cmd_vel` traffic after the gate has
        accepted it. The autonomy loop is suspended for any agent under teleop
        control until the task is reassigned by the operator.
        """

        adapter = self.adapters.get(command.agent_id)
        if adapter is None:
            return
        try:
            state = self.registry.get_state(command.agent_id)
        except KeyError:
            return
        dt = self.clock.step_sec
        adapter.apply_command(command, dt)
        new_pose = adapter.read_pose()
        state.pose = new_pose
        state.linear_velocity_x = command.linear_x
        state.linear_velocity_y = command.linear_y
        state.angular_velocity_z = command.angular_z
        self.metrics.command_accept_count += 1

    def submit_teleop_command(
        self,
        agent_id: str,
        *,
        requester_id: str,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
        source: str = "teleop",
        ttl_ms: int | None = None,
        hold_sec: float | None = None,
        episode_id: str | None = None,
    ) -> CommandDecision:
        """First-class, transport-agnostic operator teleop entry point.

        Stamps a `TELEOP` `RuntimeCommand` with the operator's `requester_id`
        and the current sim time, evaluates it through `CommandGate`, emits
        `COMMAND_ACCEPTED` / `COMMAND_REJECTED`, and on accept applies it and
        holds off the autonomy loop for `hold_sec` (default
        `navigation.teleop_hold_sec`) so the operator keeps control. This is
        the in-process equivalent of the ROS bridge's `/cmd_vel` path; both
        share `CommandGate` and `apply_external_command`.

        AI callers cannot use this to drive actuators: a `TELEOP` authority
        from a non-operator source is still the operator's responsibility, and
        AI-authority velocity commands remain rejected by the gate.
        """

        if not requester_id:
            raise ValueError("submit_teleop_command requires a non-empty requester_id")
        ep = episode_id or self._current_episode_id
        sim_time = self.clock.sim_time_sec
        try:
            spec = self.registry.get_spec(agent_id)
        except KeyError:
            return CommandDecision(False, "unknown agent")

        ttl = ttl_ms if ttl_ms is not None else spec.command_ttl_ms
        command = RuntimeCommand(
            agent_id=agent_id,
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            authority=AuthorityMode.TELEOP,
            source=source,
            issued_at_sec=sim_time,
            ttl_ms=ttl,
            requester_id=requester_id,
        )
        decision = self.command_gate.evaluate(command, now_sec=sim_time)
        if decision.accepted and decision.command is not None:
            self.events.write(
                ts=sim_time,
                episode_id=ep,
                agent_id=agent_id,
                event="COMMAND_ACCEPTED",
                data={
                    "linear_x": decision.command.linear_x,
                    "angular_z": decision.command.angular_z,
                    "authority": decision.command.authority.value,
                    "source": decision.command.source,
                    "requester_id": requester_id,
                },
            )
            self.apply_external_command(decision.command)
            hold = self.navigation_config.teleop_hold_sec if hold_sec is None else hold_sec
            self._teleop_hold_until[agent_id] = sim_time + max(0.0, hold)
        else:
            self._emit_command_rejected(
                sim_time,
                ep,
                agent_id,
                self.registry.get_state(agent_id).current_task_id,
                decision.reason,
            )
        return decision

    def diagnostics(self) -> DiagnosticsReport:
        """Read-only per-agent health snapshot (OK / WARN / ERROR).

        Folds emergency-stop, fall, task-failure, and adapter command-staleness
        (the real-robot watchdog) into per-agent levels. Always available; the
        runtime also emits a periodic `DIAGNOSTICS` event when
        ``navigation.diagnostics_interval_sec > 0``.
        """

        return collect_diagnostics(self.registry.list_states(), self.adapters)

    def _maybe_emit_diagnostics(self, *, sim_time: float, episode_id: str) -> None:
        interval = self.navigation_config.diagnostics_interval_sec
        if interval <= 0.0:
            return
        if sim_time - self._last_diagnostics_emit_sec < interval:
            return
        self._last_diagnostics_emit_sec = sim_time
        report = self.diagnostics()
        self.events.write(
            ts=sim_time,
            episode_id=episode_id,
            event="DIAGNOSTICS",
            data=report.to_dict(),
        )

    # ------------------------------------------------------------------ loop

    def step(self, *, episode_id: str) -> float:
        if self.lifecycle_state is not LifecycleState.ACTIVE:
            return self.clock.sim_time_sec
        dt = self.clock.step_sec
        sim_time = self.clock.step()
        self.metrics.sim_steps += 1
        self._current_episode_id = episode_id
        self._poll_safety_signals(sim_time=sim_time, episode_id=episode_id)
        self._apply_obstacle_updates(sim_time=sim_time, episode_id=episode_id)
        self._maybe_emit_diagnostics(sim_time=sim_time, episode_id=episode_id)
        if len(self.task_queue) > 0:
            self.dispatch_pending(episode_id=episode_id)
            self.metrics.task_pending_peak = max(
                self.metrics.task_pending_peak, len(self.task_queue)
            )

        for state in self.registry.list_states():
            if not state.current_task_id:
                continue
            adapter = self.adapters.get(state.agent_id)
            if adapter is None:
                continue

            previous_pose = adapter.read_pose()
            state.pose = previous_pose
            goal = state.current_goal
            task_id = state.current_task_id
            record = self.metrics.tasks.get(task_id)

            if state.emergency_stopped:
                self._emit_command_rejected(
                    sim_time, episode_id, state.agent_id, task_id, "emergency stop"
                )
                adapter.stop("emergency stop")
                state.linear_velocity_x = 0.0
                state.angular_velocity_z = 0.0
                continue

            # Operator override: while a teleop command holds this agent, the
            # autonomy loop yields so it cannot fight the operator. The agent
            # retains the last teleop velocity until the hold expires.
            if sim_time < self._teleop_hold_until.get(state.agent_id, 0.0):
                continue

            if goal is None:
                continue

            # Proximity response: yield right-of-way to a higher-priority agent
            # inside the yield radius. The yielding agent stops this tick; the
            # autonomy loop resumes once the other clears. Observation-only
            # detection (COLLISION/NEAR_MISS) still runs regardless.
            if self._should_yield(state.agent_id, previous_pose):
                if state.agent_id not in self._yielding:
                    self._yielding.add(state.agent_id)
                    self.metrics.yield_count += 1
                    self.events.write(
                        ts=sim_time,
                        episode_id=episode_id,
                        agent_id=state.agent_id,
                        event="AGENT_YIELDED",
                        task_id=task_id,
                        data={
                            "reason": "proximity_yield",
                            "radius_m": self.collision_config.yield_radius_m,
                        },
                    )
                adapter.stop("yield")
                state.linear_velocity_x = 0.0
                state.linear_velocity_y = 0.0
                state.angular_velocity_z = 0.0
                # Restart the stuck window after the wait so a brief yield is not
                # mistaken for being stuck.
                self._pose_history[state.agent_id] = deque()
                continue
            if state.agent_id in self._yielding:
                self._yielding.discard(state.agent_id)

            if state.behavior_state is BehaviorState.ASSIGNED:
                self._transition_behavior(
                    state,
                    BehaviorState.PLANNING,
                    ts=sim_time,
                    episode_id=episode_id,
                    reason="planner_start",
                    task_id=task_id,
                )
                planned = self._run_planner(
                    state.agent_id, previous_pose, goal, sim_time, episode_id, task_id
                )
                if planned is None:
                    self._fail_task(
                        state, task_id, sim_time, episode_id, reason="plan_failed"
                    )
                    continue
                self._waypoints[state.agent_id] = planned
                self._pose_history[state.agent_id] = deque()
                self._recovery_retries[state.agent_id] = 0
                state.current_task_status = TaskStatus.EXECUTING
                if record is not None:
                    record.started_at_sec = sim_time
                    record.status = TaskStatus.EXECUTING
                self.events.write(
                    ts=sim_time,
                    episode_id=episode_id,
                    agent_id=state.agent_id,
                    event="TASK_STARTED",
                    task_id=task_id,
                    data={"goal": goal, "waypoint_count": len(planned)},
                )
                self._transition_behavior(
                    state,
                    BehaviorState.EXECUTING,
                    ts=sim_time,
                    episode_id=episode_id,
                    reason="plan_ready",
                    task_id=task_id,
                )

            if state.behavior_state is BehaviorState.RECOVERING:
                resume_at = self._recovery_resume_at_sec.get(state.agent_id, 0.0)
                adapter.stop("recovering")
                state.linear_velocity_x = 0.0
                state.angular_velocity_z = 0.0
                if sim_time < resume_at:
                    continue
                self._transition_behavior(
                    state,
                    BehaviorState.EXECUTING,
                    ts=sim_time,
                    episode_id=episode_id,
                    reason="recovery_complete",
                    task_id=task_id,
                )
                self.metrics.recovery_count += 1
                self.events.write(
                    ts=sim_time,
                    episode_id=episode_id,
                    agent_id=state.agent_id,
                    event="STUCK_RECOVERED",
                    task_id=task_id,
                    data={"retries": self._recovery_retries.get(state.agent_id, 0)},
                )
                self._pose_history[state.agent_id] = deque()

            target = self._next_waypoint(state.agent_id, previous_pose)
            if target is None:
                target = goal

            if self._reached_goal(previous_pose, goal):
                if record is not None:
                    record.finished_at_sec = sim_time
                    record.status = TaskStatus.SUCCEEDED
                state.current_task_status = TaskStatus.SUCCEEDED
                self.events.write(
                    ts=sim_time,
                    episode_id=episode_id,
                    agent_id=state.agent_id,
                    event="TASK_SUCCEEDED",
                    task_id=task_id,
                    data={"final_pose": previous_pose},
                )
                self._transition_behavior(
                    state,
                    BehaviorState.SUCCEEDED,
                    ts=sim_time,
                    episode_id=episode_id,
                    reason="goal_reached",
                    task_id=task_id,
                )
                self._release_agent_leases(state.agent_id, episode_id=episode_id)
                self.registry.clear_task(state.agent_id)
                state.current_goal = None
                state.current_task_status = TaskStatus.PENDING
                state.linear_velocity_x = 0.0
                state.angular_velocity_z = 0.0
                adapter.stop("goal reached")
                self._transition_behavior(
                    state,
                    BehaviorState.IDLE,
                    ts=sim_time,
                    episode_id=episode_id,
                    reason="task_complete",
                    task_id="",
                )
                self._clear_navigation_state(state.agent_id)
                continue

            spec = self.registry.get_spec(state.agent_id)
            command = self.controller.compute(
                state.agent_id,
                previous_pose,
                target,
                issued_at_sec=sim_time,
                ttl_ms=spec.command_ttl_ms,
                authority=AuthorityMode.AUTONOMY,
            )
            decision = self.command_gate.evaluate(command, now_sec=sim_time)
            if decision.accepted and decision.command is not None:
                adapter.apply_command(decision.command, dt)
                new_pose = adapter.read_pose()
                state.pose = new_pose
                state.linear_velocity_x = decision.command.linear_x
                state.angular_velocity_z = decision.command.angular_z
                self.metrics.command_accept_count += 1
                if record is not None:
                    record.path_length_m += math.hypot(
                        new_pose[0] - previous_pose[0],
                        new_pose[1] - previous_pose[1],
                    )
                if task_id not in self._motion_started:
                    self._motion_started.add(task_id)
                    self.events.write(
                        ts=sim_time,
                        episode_id=episode_id,
                        agent_id=state.agent_id,
                        event="COMMAND_ACCEPTED",
                        task_id=task_id,
                        data={
                            "linear_x": decision.command.linear_x,
                            "angular_z": decision.command.angular_z,
                            "authority": decision.command.authority.value,
                            "source": decision.command.source,
                        },
                    )
                self._update_stuck_detection(
                    state, sim_time, episode_id, task_id
                )
            else:
                self._emit_command_rejected(
                    sim_time, episode_id, state.agent_id, task_id, decision.reason
                )
                adapter.stop(decision.reason)
                state.linear_velocity_x = 0.0
                state.angular_velocity_z = 0.0

        return sim_time

    def run_until_idle(
        self,
        *,
        episode_id: str,
        max_sim_seconds: float = 60.0,
        on_step: StepCallback | None = None,
    ) -> RuntimeMetrics:
        max_steps = max(1, int(max_sim_seconds / max(self.clock.step_sec, 1e-9)))
        self._current_episode_id = episode_id
        # Drain queued work before the first physics tick.
        if len(self.task_queue) > 0:
            self.dispatch_pending(episode_id=episode_id)
        while self.has_pending_work() and self.metrics.sim_steps < max_steps:
            sim_time = self.step(episode_id=episode_id)
            if on_step is not None:
                on_step(sim_time)
        if self.has_pending_work():
            self._fail_remaining(episode_id=episode_id, reason="timeout")
        return self.metrics

    # ---------------------------------------------------------------- helpers

    def _transition_behavior(
        self,
        state,  # type: ignore[no-untyped-def]
        target: BehaviorState,
        *,
        ts: float,
        episode_id: str,
        reason: str,
        task_id: str = "",
    ) -> None:
        if state.behavior_state is target:
            return
        if not can_transition(state.behavior_state, target):
            return
        previous = state.behavior_state
        state.behavior_state = target
        self.events.write(
            ts=ts,
            episode_id=episode_id,
            agent_id=state.agent_id,
            event="BEHAVIOR_STATE_CHANGED",
            task_id=task_id,
            data={
                "from": previous.value,
                "to": target.value,
                "reason": reason,
            },
        )

    def _run_planner(
        self,
        agent_id: str,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
        ts: float,
        episode_id: str,
        task_id: str,
    ) -> list[tuple[float, float, float]] | None:
        try:
            waypoints = self.planner.plan(start, goal)
        except PlannerError as exc:
            self.metrics.plan_failure_count += 1
            self.events.write(
                ts=ts,
                episode_id=episode_id,
                agent_id=agent_id,
                event="PLAN_FAILED",
                task_id=task_id,
                data={"reason": str(exc), "goal": goal},
            )
            return None
        if not waypoints:
            self.metrics.plan_failure_count += 1
            self.events.write(
                ts=ts,
                episode_id=episode_id,
                agent_id=agent_id,
                event="PLAN_FAILED",
                task_id=task_id,
                data={"reason": "empty_plan", "goal": goal},
            )
            return None
        self.events.write(
            ts=ts,
            episode_id=episode_id,
            agent_id=agent_id,
            event="PLAN_RESOLVED",
            task_id=task_id,
            data={
                "waypoint_count": len(waypoints),
                "planner": type(self.planner).__name__,
            },
        )
        return list(waypoints)

    def _next_waypoint(
        self, agent_id: str, pose: tuple[float, float, float]
    ) -> tuple[float, float, float] | None:
        queue = self._waypoints.get(agent_id)
        if not queue:
            return None
        tol = self.navigation_config.waypoint_tolerance_m
        while len(queue) > 1:
            wp = queue[0]
            if math.hypot(pose[0] - wp[0], pose[1] - wp[1]) <= tol:
                queue.pop(0)
                continue
            break
        return queue[0] if queue else None

    def _reached_goal(
        self,
        pose: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> bool:
        return self.controller.at_goal(pose, goal)

    def _update_stuck_detection(
        self,
        state,  # type: ignore[no-untyped-def]
        sim_time: float,
        episode_id: str,
        task_id: str,
    ) -> None:
        cfg = self.navigation_config
        history = self._pose_history.setdefault(state.agent_id, deque())
        history.append((sim_time, state.pose[0], state.pose[1]))
        cutoff = sim_time - cfg.stuck_window_sec
        while history and history[0][0] < cutoff:
            history.popleft()
        if len(history) < 2:
            return
        window_span = history[-1][0] - history[0][0]
        if window_span < cfg.stuck_window_sec - 1e-6:
            return
        first = history[0]
        last = history[-1]
        progress = math.hypot(last[1] - first[1], last[2] - first[2])
        if progress >= cfg.stuck_min_progress_m:
            return

        retries = self._recovery_retries.get(state.agent_id, 0)
        self.metrics.stuck_event_count += 1
        self.events.write(
            ts=sim_time,
            episode_id=episode_id,
            agent_id=state.agent_id,
            event="AGENT_STUCK",
            task_id=task_id,
            data={
                "progress_m": progress,
                "window_sec": cfg.stuck_window_sec,
                "retry": retries + 1,
            },
        )
        if retries >= cfg.max_recovery_retries:
            self._fail_task(
                state, task_id, sim_time, episode_id, reason="stuck"
            )
            return
        self._recovery_retries[state.agent_id] = retries + 1
        self._recovery_resume_at_sec[state.agent_id] = (
            sim_time + cfg.recovery_wait_sec
        )
        self._transition_behavior(
            state,
            BehaviorState.RECOVERING,
            ts=sim_time,
            episode_id=episode_id,
            reason="stuck",
            task_id=task_id,
        )

    def _fail_task(
        self,
        state,  # type: ignore[no-untyped-def]
        task_id: str,
        sim_time: float,
        episode_id: str,
        *,
        reason: str,
    ) -> None:
        record = self.metrics.tasks.get(task_id)
        if record is not None:
            record.status = TaskStatus.FAILED
            record.finished_at_sec = sim_time
            record.failure_reason = reason
        state.current_task_status = TaskStatus.FAILED
        self.events.write(
            ts=sim_time,
            episode_id=episode_id,
            agent_id=state.agent_id,
            event="TASK_FAILED",
            task_id=task_id,
            data={"reason": reason},
        )
        self._transition_behavior(
            state,
            BehaviorState.FAILED,
            ts=sim_time,
            episode_id=episode_id,
            reason=reason,
            task_id=task_id,
        )
        adapter = self.adapters.get(state.agent_id)
        if adapter is not None:
            adapter.stop(reason)
        state.linear_velocity_x = 0.0
        state.angular_velocity_z = 0.0
        self._release_agent_leases(state.agent_id, episode_id=episode_id)
        self.registry.clear_task(state.agent_id)
        state.current_goal = None
        self._transition_behavior(
            state,
            BehaviorState.IDLE,
            ts=sim_time,
            episode_id=episode_id,
            reason="task_complete",
            task_id="",
        )
        self._clear_navigation_state(state.agent_id)

    def _clear_navigation_state(self, agent_id: str) -> None:
        self._waypoints.pop(agent_id, None)
        self._pose_history.pop(agent_id, None)
        self._recovery_resume_at_sec.pop(agent_id, None)
        self._recovery_retries.pop(agent_id, None)

    def _poll_safety_signals(self, *, sim_time: float, episode_id: str) -> None:
        """Detect rising-edge safety conditions from adapters.

        Watches humanoid ``fall_detected`` and the real-robot command-staleness
        watchdog. Adapters without those signals are ignored. On the rising edge
        we emit the observation followed by ``SAFETY_STOP`` (the action), then
        set the registry's emergency-stop flag so the existing per-task branch
        handles ``adapter.stop`` and ``COMMAND_REJECTED`` uniformly.
        """

        for state in self.registry.list_states():
            adapter = self.adapters.get(state.agent_id)
            if adapter is None:
                continue
            self._poll_command_watchdog(adapter, state, sim_time, episode_id)
            fallen = bool(getattr(adapter, "fall_detected", False))
            if fallen and not state.fall_detected:
                state.fall_detected = True
                reason = str(getattr(adapter, "fall_reason", "") or "fall_detected")
                margin = float(getattr(adapter, "balance_margin", 0.0))
                self.events.write(
                    ts=sim_time,
                    episode_id=episode_id,
                    agent_id=state.agent_id,
                    event="FALL_DETECTED",
                    task_id=state.current_task_id,
                    data={
                        "reason": reason,
                        "balance_margin": margin,
                        "source": "humanoid_adapter",
                    },
                )
                self.events.write(
                    ts=sim_time,
                    episode_id=episode_id,
                    agent_id=state.agent_id,
                    event="SAFETY_STOP",
                    task_id=state.current_task_id,
                    data={
                        "reason": "fall_detected",
                        "scope": "agent",
                        "source": "humanoid_adapter",
                    },
                )
                self.registry.emergency_stop(state.agent_id, True)
            elif not fallen and state.fall_detected:
                state.fall_detected = False

        self._poll_collisions(sim_time=sim_time, episode_id=episode_id)

    def _poll_collisions(self, *, sim_time: float, episode_id: str) -> None:
        """Detect inter-agent proximity (collision / near-miss), rising edge.

        Observation only: emits `COLLISION` / `NEAR_MISS` and bumps the matching
        counter the first tick a pair enters each radius, clearing the pair on
        separation so a later re-approach is counted again. It does not stop or
        reroute agents — proximity *response* (yield / replan) is a documented
        follow-up. Disabled (zero overhead) unless a radius is configured, so
        existing scenarios are unaffected.
        """

        cfg = self.collision_config
        if not cfg.enabled:
            return
        poses: list[tuple[str, tuple[float, float, float]]] = []
        for state in self.registry.list_states():
            adapter = self.adapters.get(state.agent_id)
            if adapter is None:
                continue
            poses.append((state.agent_id, adapter.read_pose()))

        for i in range(len(poses)):
            id_a, pose_a = poses[i]
            for j in range(i + 1, len(poses)):
                id_b, pose_b = poses[j]
                pair = frozenset((id_a, id_b))
                distance = math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])

                colliding = (
                    cfg.collision_radius_m > 0.0
                    and distance <= cfg.collision_radius_m
                )
                near = (
                    cfg.near_miss_radius_m > 0.0
                    and distance <= cfg.near_miss_radius_m
                )

                if colliding:
                    if pair not in self._collision_pairs:
                        self._collision_pairs.add(pair)
                        self.metrics.collision_count += 1
                        self.events.write(
                            ts=sim_time,
                            episode_id=episode_id,
                            agent_id=id_a,
                            event="COLLISION",
                            data={
                                "agents": sorted((id_a, id_b)),
                                "distance_m": distance,
                                "radius_m": cfg.collision_radius_m,
                                "scope": "pair",
                            },
                        )
                else:
                    self._collision_pairs.discard(pair)

                # Mark the pair on first entry into the near zone (even if it
                # entered straight into collision), so receding from a collision
                # back through the near zone is not counted as a fresh near-miss.
                if near:
                    if pair not in self._near_miss_pairs:
                        self._near_miss_pairs.add(pair)
                        if not colliding:
                            self.metrics.near_miss_count += 1
                            self.events.write(
                                ts=sim_time,
                                episode_id=episode_id,
                                agent_id=id_a,
                                event="NEAR_MISS",
                                data={
                                    "agents": sorted((id_a, id_b)),
                                    "distance_m": distance,
                                    "radius_m": cfg.near_miss_radius_m,
                                    "scope": "pair",
                                },
                            )
                else:
                    self._near_miss_pairs.discard(pair)

    def _should_yield(self, agent_id: str, pose: tuple[float, float, float]) -> bool:
        """True if a higher-priority agent is within the yield radius.

        Right-of-way is the lexicographic agent-id order: the agent with the
        smaller id has priority, so only the larger-id agent of a conflicting
        pair yields. A total order means no two agents can each be waiting on the
        other, so a pair never deadlocks (chains across 3+ agents resolve in
        priority order too). A smarter priority (goal distance, reciprocal
        velocity obstacles) is a documented follow-up.
        """

        radius = self.collision_config.yield_radius_m
        if radius <= 0.0:
            return False
        for state in self.registry.list_states():
            other_id = state.agent_id
            if other_id >= agent_id:
                continue  # only yield to strictly higher-priority agents
            # Only yield to an agent that is itself navigating a task; an idle
            # agent parked at its goal must not hold others up forever.
            if not state.current_task_id:
                continue
            other = self.adapters.get(other_id)
            if other is None:
                continue
            other_pose = other.read_pose()
            if math.hypot(pose[0] - other_pose[0], pose[1] - other_pose[1]) <= radius:
                return True
        return False

    def _poll_command_watchdog(
        self, adapter: EmbodimentAdapter, state, sim_time: float, episode_id: str
    ) -> None:
        """Trip a safety stop when a real-robot command watchdog expires.

        The watchdog runs on the transport's *monotonic* clock (wall time), not
        sim time, because it guards against real comms loss / a stalled command
        pipeline — a condition sim time cannot represent. Adapters without a
        ``watchdog_expired`` method (the fallback, Genesis, and humanoid
        adapters) never participate, so this is a no-op in pure-sim runs.

        Rising-edge only. Unlike the per-task emergency-stop branch, this stops
        the actuator directly so a real base is zeroed even when no task is
        active (e.g. an operator teleops then stops streaming). It is latched:
        the stop does not auto-clear when commands resume — an operator must
        clear the emergency stop, matching how comms-loss is handled on real
        hardware.
        """

        check = getattr(adapter, "watchdog_expired", None)
        if not callable(check):
            return
        agent_id = state.agent_id
        expired = bool(check())
        if expired and agent_id not in self._watchdog_tripped:
            self._watchdog_tripped.add(agent_id)
            self.metrics.watchdog_stop_count += 1
            seconds_since = getattr(adapter, "seconds_since_command", None)
            age = seconds_since() if callable(seconds_since) else None
            self.events.write(
                ts=sim_time,
                episode_id=episode_id,
                agent_id=agent_id,
                event="SAFETY_STOP",
                task_id=state.current_task_id,
                data={
                    "reason": "command_watchdog",
                    "scope": "agent",
                    "source": "ros2_robot_adapter",
                    "command_age_sec": age,
                },
            )
            adapter.stop("command_watchdog")
            state.linear_velocity_x = 0.0
            state.linear_velocity_y = 0.0
            state.angular_velocity_z = 0.0
            self.registry.emergency_stop(agent_id, True)
        elif not expired and agent_id in self._watchdog_tripped:
            self._watchdog_tripped.discard(agent_id)

    def _apply_obstacle_updates(self, *, sim_time: float, episode_id: str) -> None:
        """Apply due obstacle deltas and replan affected executing agents.

        Deltas mutate the grid the planner runs against and are recorded as
        ``OBSTACLE_CHANGED`` events so a replay reconstructs the obstacle
        timeline. Only agents whose remaining path crosses a newly blocked
        cell are replanned; everyone else keeps their plan.
        """

        source = self._obstacle_source
        if source is None or not isinstance(self.planner, GridAStarPlanner):
            return
        due = source.due(sim_time)
        if not due:
            return

        newly_blocked: set[tuple[int, int]] = set()
        for delta in due:
            grid = self.planner.grid
            in_bounds = [(c, r) for c, r in delta.block if grid.in_bounds(c, r)]
            self.planner.grid = grid.with_blocked(delta.block)
            self.metrics.obstacle_event_count += 1
            newly_blocked.update(in_bounds)
            self.events.write(
                ts=sim_time,
                episode_id=episode_id,
                event="OBSTACLE_CHANGED",
                data={"blocked_cells": [list(c) for c in in_bounds]},
            )
        if not newly_blocked:
            return

        for state in self.registry.list_states():
            if state.behavior_state is not BehaviorState.EXECUTING:
                continue
            if not state.current_task_id:
                continue
            adapter = self.adapters.get(state.agent_id)
            if adapter is None:
                continue
            pose = adapter.read_pose()
            if not self._path_hits_cells(state.agent_id, pose, newly_blocked):
                continue
            self._replan_agent(state, pose, sim_time=sim_time, episode_id=episode_id)

    def _path_hits_cells(
        self,
        agent_id: str,
        pose: tuple[float, float, float],
        blocked: set[tuple[int, int]],
    ) -> bool:
        """True if the agent's remaining path crosses any ``blocked`` cell.

        Waypoints are sparse (colinear points are dropped), so each segment is
        sampled at half-cell steps before mapping to grid cells.
        """

        queue = self._waypoints.get(agent_id)
        if not queue or not isinstance(self.planner, GridAStarPlanner):
            return False
        grid = self.planner.grid
        step = grid.resolution * 0.5
        points = [pose, *queue]
        for a, b in zip(points, points[1:]):
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            samples = max(1, int(dist / step))
            for i in range(samples + 1):
                t = i / samples
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                if grid.world_to_cell(x, y) in blocked:
                    return True
        return False

    def _replan_agent(
        self,
        state,  # type: ignore[no-untyped-def]
        pose: tuple[float, float, float],
        *,
        sim_time: float,
        episode_id: str,
    ) -> None:
        task_id = state.current_task_id
        goal = state.current_goal
        if goal is None:
            return
        self._transition_behavior(
            state,
            BehaviorState.PLANNING,
            ts=sim_time,
            episode_id=episode_id,
            reason="obstacle_replan",
            task_id=task_id,
        )
        planned = self._run_planner(
            state.agent_id, pose, goal, sim_time, episode_id, task_id
        )
        if planned is None:
            self._fail_task(
                state, task_id, sim_time, episode_id, reason="blocked"
            )
            return
        self._waypoints[state.agent_id] = planned
        self._pose_history[state.agent_id] = deque()
        self._recovery_retries[state.agent_id] = 0
        self.metrics.replan_count += 1
        self.events.write(
            ts=sim_time,
            episode_id=episode_id,
            agent_id=state.agent_id,
            event="REPLAN_TRIGGERED",
            task_id=task_id,
            data={"waypoint_count": len(planned), "reason": "obstacle"},
        )
        self._transition_behavior(
            state,
            BehaviorState.EXECUTING,
            ts=sim_time,
            episode_id=episode_id,
            reason="replan_ready",
            task_id=task_id,
        )

    def _emit_command_rejected(
        self,
        sim_time: float,
        episode_id: str,
        agent_id: str,
        task_id: str,
        reason: str,
    ) -> None:
        self.metrics.command_rejection_count += 1
        self.events.write(
            ts=sim_time,
            episode_id=episode_id,
            agent_id=agent_id,
            event="COMMAND_REJECTED",
            task_id=task_id,
            data={"reason": reason},
        )

    def _fail_remaining(self, *, episode_id: str, reason: str) -> None:
        sim_time = self.clock.sim_time_sec
        for state in self.registry.list_states():
            if not state.current_task_id:
                continue
            task_id = state.current_task_id
            record = self.metrics.tasks.get(task_id)
            if record is not None:
                record.status = TaskStatus.FAILED
                record.finished_at_sec = sim_time
                record.failure_reason = reason
            state.current_task_status = TaskStatus.FAILED
            self.events.write(
                ts=sim_time,
                episode_id=episode_id,
                agent_id=state.agent_id,
                event="TASK_FAILED",
                task_id=task_id,
                data={"reason": reason},
            )
            self._transition_behavior(
                state,
                BehaviorState.FAILED,
                ts=sim_time,
                episode_id=episode_id,
                reason=reason,
                task_id=task_id,
            )
            self._release_agent_leases(state.agent_id, episode_id=episode_id)
            self.registry.clear_task(state.agent_id)
            state.current_goal = None
            self._transition_behavior(
                state,
                BehaviorState.IDLE,
                ts=sim_time,
                episode_id=episode_id,
                reason="task_complete",
                task_id="",
            )
            self._clear_navigation_state(state.agent_id)
        # Tasks that were queued but never assigned also count as failed.
        while True:
            queued = self.task_queue.pop()
            if queued is None:
                break
            record = self.metrics.tasks.get(queued.task_id)
            if record is None:
                record = _TaskRecord(
                    task_id=queued.task_id,
                    agent_id=queued.agent_id or "",
                    goal=queued.goal,
                    assigned_at_sec=sim_time,
                )
                self.metrics.tasks[queued.task_id] = record
            record.status = TaskStatus.FAILED
            record.finished_at_sec = sim_time
            record.failure_reason = reason
            self.events.write(
                ts=sim_time,
                episode_id=episode_id,
                agent_id=queued.agent_id or "",
                event="TASK_FAILED",
                task_id=queued.task_id,
                data={"reason": reason, "state": "queued"},
            )

    def tool_api(
        self,
        *,
        scenario_id: str = "",
        event_buffer: "RingBufferEventSink | None" = None,  # noqa: F821
    ):  # type: ignore[no-untyped-def]
        """Build an :class:`AgentToolApi` bound to this runtime.

        Imported lazily so :mod:`genesis_nav.core.runtime` stays free of the
        AI-agent API at import time.
        """

        from genesis_nav.agent.tools import AgentToolApi

        return AgentToolApi(self, scenario_id=scenario_id, event_buffer=event_buffer)

    def _release_agent_leases(self, agent_id: str, *, episode_id: str) -> None:
        lease_ids = self._leases_by_agent.pop(agent_id, [])
        for lease_id in lease_ids:
            self.release_resource(
                lease_id, requester_id=agent_id, episode_id=episode_id
            )


def ensure_run_layout(run_dir: Path, *, record_rosbag: bool) -> None:
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "traces").mkdir()
    if record_rosbag:
        (run_dir / "rosbag").mkdir()


__all__ = [
    "EmbodimentAdapter",
    "Runtime",
    "RuntimeMetrics",
    "SimulationMode",
    "ensure_run_layout",
]
