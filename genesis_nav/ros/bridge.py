"""ROS 2 bridge that exposes the runtime over the public ROS graph.

Responsibilities:
- publish `/clock` driven by the runtime simulation clock
- publish `/genesis_nav/events`, `/genesis_nav/scenario_state`,
  `/genesis_nav/fleet_state`, and `/genesis_nav/diagnostics`
  (`diagnostic_msgs/DiagnosticArray`, one status per agent)
- publish per-agent `<ns>/state` and `<ns>/odom`
- broadcast tf (`<ns>/odom` -> `<ns>/base_link`) and tf_static (`map` ->
  `<ns>/odom`)
- subscribe to per-agent `<ns>/cmd_vel` and forward each Twist to the
  runtime's teleop entry point, which runs it through `CommandGate` before
  it can affect actuators

The bridge runs in-process inside `gnav run --ros` and is also importable from
ROS 2 nodes via `genesis_nav.ros.RosBridge`. All ROS imports are lazy so the
core runtime stays installable without rclpy.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

from genesis_nav.core.agent import AgentRegistry, AgentState
from genesis_nav.core.clock import RuntimeClock
from genesis_nav.core.task import TaskStatus
from genesis_nav.observability.events import EventSink
from genesis_nav.ros.qos import load_qos_profiles, qos_profile_for

if TYPE_CHECKING:
    from genesis_nav.core.command_gate import CommandDecision


# A teleop ingress handler: given (agent_id, linear_x, linear_y, angular_z) it
# runs the command through the runtime's `CommandGate` and returns the gate
# `CommandDecision`. The runtime owns the gate, events, and autonomy hold; the
# bridge is pure transport. Wired to `Runtime.submit_teleop_command`.
TeleopCommandHandler = Callable[[str, float, float, float], "CommandDecision"]


@dataclass(frozen=True)
class BridgeConfig:
    node_name: str = "genesis_nav_bridge"
    qos_path: str | Path | None = None
    map_frame: str = "map"
    publish_clock: bool = True
    publish_tf: bool = True
    cmd_vel_topic: str = "cmd_vel"
    state_topic: str = "state"
    odom_topic: str = "odom"


class RosBridge:
    """Owns the rclpy node and the per-agent publisher/subscriber matrix."""

    def __init__(
        self,
        registry: AgentRegistry,
        runtime_clock: RuntimeClock,
        events: EventSink,
        *,
        config: BridgeConfig | None = None,
        teleop_command_handler: TeleopCommandHandler | None = None,
        episode_id: str = "",
    ) -> None:
        import rclpy
        from rclpy.node import Node

        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import (
            DiagnosticArray,
            DiagnosticStatus,
            KeyValue,
        )

        from genesis_nav_msgs.msg import (
            AgentState as AgentStateMsg,
            FleetState as FleetStateMsg,
            RuntimeEvent as RuntimeEventMsg,
            ScenarioState as ScenarioStateMsg,
        )

        try:
            from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        except ImportError as exc:  # pragma: no cover - exercised on minimal systems
            raise RuntimeError(
                "tf2_ros is required for the ROS bridge but is not installed"
            ) from exc

        self._rclpy = rclpy
        self._Clock = Clock
        self._Twist = Twist
        self._TransformStamped = TransformStamped
        self._Odometry = Odometry
        self._AgentStateMsg = AgentStateMsg
        self._FleetStateMsg = FleetStateMsg
        self._RuntimeEventMsg = RuntimeEventMsg
        self._ScenarioStateMsg = ScenarioStateMsg
        self._DiagnosticArray = DiagnosticArray
        self._DiagnosticStatus = DiagnosticStatus
        self._KeyValue = KeyValue
        # Optional callable returning a DiagnosticsReport (wired to
        # Runtime.diagnostics); None until set_diagnostics_provider is called.
        self._diagnostics_provider: Callable[[], Any] | None = None

        if not rclpy.ok():
            rclpy.init()
        self._owns_rclpy = True

        self.registry = registry
        self.runtime_clock = runtime_clock
        self.events = events
        self.config = config or BridgeConfig()
        self.episode_id = episode_id
        self._teleop_command_handler = teleop_command_handler
        self.external_command_count = 0
        self.external_command_reject_count = 0

        self.node: Node = rclpy.create_node(self.config.node_name)
        self._profiles = (
            load_qos_profiles(self.config.qos_path) if self.config.qos_path else {"topics": {}}
        )

        self._tf_broadcaster = TransformBroadcaster(self.node)
        self._tf_static_broadcaster = StaticTransformBroadcaster(self.node)

        self._state_pubs: dict[str, Any] = {}
        self._odom_pubs: dict[str, Any] = {}
        self._cmd_vel_subs: dict[str, Any] = {}
        self._frames_by_agent: dict[str, tuple[str, str, str]] = {}

        if self.config.publish_clock:
            self._clock_pub = self.node.create_publisher(
                Clock, "/clock", qos_profile_for("/clock", self._profiles)
            )
        else:
            self._clock_pub = None

        self._events_pub = self.node.create_publisher(
            RuntimeEventMsg,
            "/genesis_nav/events",
            qos_profile_for("/genesis_nav/events", self._profiles),
        )
        self._scenario_pub = self.node.create_publisher(
            ScenarioStateMsg,
            "/genesis_nav/scenario_state",
            qos_profile_for("/genesis_nav/scenario_state", self._profiles),
        )
        self._fleet_pub = self.node.create_publisher(
            FleetStateMsg,
            "/genesis_nav/fleet_state",
            qos_profile_for("/genesis_nav/fleet_state", self._profiles),
        )
        self._diagnostics_pub = self.node.create_publisher(
            DiagnosticArray,
            "/genesis_nav/diagnostics",
            qos_profile_for("/genesis_nav/diagnostics", self._profiles),
        )

        self._register_agents()

    # ----------------------------------------------------------- construction

    def _register_agents(self) -> None:
        for spec in self.registry.list_specs():
            namespace = spec.namespace.rstrip("/")
            state_topic = f"{namespace}/{self.config.state_topic}"
            odom_topic = f"{namespace}/{self.config.odom_topic}"
            cmd_topic = f"{namespace}/{self.config.cmd_vel_topic}"

            self._state_pubs[spec.agent_id] = self.node.create_publisher(
                self._AgentStateMsg,
                state_topic,
                qos_profile_for(state_topic, self._profiles),
            )
            self._odom_pubs[spec.agent_id] = self.node.create_publisher(
                self._Odometry,
                odom_topic,
                qos_profile_for(odom_topic, self._profiles),
            )

            agent_id = spec.agent_id

            def make_callback(target_agent: str):
                def _callback(msg) -> None:  # type: ignore[no-untyped-def]
                    self._on_cmd_vel(target_agent, msg)
                return _callback

            self._cmd_vel_subs[agent_id] = self.node.create_subscription(
                self._Twist,
                cmd_topic,
                make_callback(agent_id),
                qos_profile_for(cmd_topic, self._profiles),
            )

            self._frames_by_agent[agent_id] = (
                spec.frames.map,
                spec.frames.odom,
                spec.frames.base,
            )

            if self.config.publish_tf:
                self._publish_static_tf(agent_id, spec.frames.map, spec.frames.odom)

    # -------------------------------------------------------------- publish

    def publish_clock(self, sim_time_sec: float) -> None:
        if self._clock_pub is None:
            return
        msg = self._Clock()
        seconds = int(sim_time_sec)
        msg.clock.sec = seconds
        msg.clock.nanosec = max(0, int(round((sim_time_sec - seconds) * 1e9)))
        self._clock_pub.publish(msg)

    def publish_states(self, sim_time_sec: float) -> None:
        for state in self.registry.list_states():
            self._publish_agent_state(state, sim_time_sec)
            self._publish_agent_odom(state, sim_time_sec)
            if self.config.publish_tf:
                self._publish_dynamic_tf(state, sim_time_sec)

    def publish_scenario_state(
        self,
        sim_time_sec: float,
        *,
        scenario_id: str,
        seed: int,
        runtime_mode: str,
        paused: bool,
        recording: bool,
        extra: dict[str, Any] | None = None,
    ) -> None:
        msg = self._ScenarioStateMsg()
        msg.header.stamp = self._stamp(sim_time_sec)
        msg.header.frame_id = self.config.map_frame
        msg.scenario_id = scenario_id
        msg.episode_id = self.episode_id
        msg.seed = int(seed)
        msg.runtime_mode = runtime_mode
        msg.paused = bool(paused)
        msg.recording = bool(recording)
        msg.state_json = json.dumps(extra or {}, sort_keys=True)
        self._scenario_pub.publish(msg)

    def publish_fleet_state(
        self,
        sim_time_sec: float,
        *,
        mode: str = "active",
        pending: int = 0,
        active: int = 0,
        completed: int = 0,
        extra: dict[str, Any] | None = None,
    ) -> None:
        msg = self._FleetStateMsg()
        msg.header.stamp = self._stamp(sim_time_sec)
        msg.header.frame_id = self.config.map_frame
        msg.mode = mode
        msg.agent_count = len(self.registry.list_states())
        msg.pending_task_count = int(pending)
        msg.active_task_count = int(active)
        msg.completed_task_count = int(completed)
        msg.state_json = json.dumps(extra or {}, sort_keys=True)
        self._fleet_pub.publish(msg)

    # ----------------------------------------------------------- diagnostics

    def set_diagnostics_provider(self, provider: Callable[[], Any]) -> None:
        """Register a zero-arg callable returning a `DiagnosticsReport`.

        Wired to `Runtime.diagnostics` by `gnav run --ros`. The bridge maps the
        report onto a standard `diagnostic_msgs/DiagnosticArray` so RViz and the
        ROS diagnostic aggregator can consume the per-agent health (including the
        inter-agent proximity axes) the runtime already computes.
        """

        self._diagnostics_provider = provider

    def publish_diagnostics(self, sim_time_sec: float) -> None:
        """Publish a `DiagnosticArray` (one `DiagnosticStatus` per agent).

        No-op until a provider is set. Levels map 1:1 onto `diagnostic_msgs`
        (`OK=0 < WARN=1 < ERROR=2`), matching the runtime's `DiagnosticLevel`.
        """

        if self._diagnostics_provider is None:
            return
        report = self._diagnostics_provider()
        level_map = {
            0: self._DiagnosticStatus.OK,
            1: self._DiagnosticStatus.WARN,
            2: self._DiagnosticStatus.ERROR,
        }
        msg = self._DiagnosticArray()
        msg.header.stamp = self._stamp(sim_time_sec)
        msg.header.frame_id = self.config.map_frame
        for agent in report.agents:
            status = self._DiagnosticStatus()
            status.level = level_map.get(int(agent.level), self._DiagnosticStatus.OK)
            status.name = f"genesis_nav/{agent.agent_id}"
            status.hardware_id = agent.agent_id
            status.message = "; ".join(agent.messages) if agent.messages else "ok"
            status.values = [
                self._KeyValue(key="behavior_state", value=str(agent.behavior_state)),
                self._KeyValue(
                    key="command_age_sec", value=str(agent.command_age_sec)
                ),
            ]
            msg.status.append(status)
        self._diagnostics_pub.publish(msg)

    # ------------------------------------------------------------- EventSink

    def write(
        self,
        *,
        ts: float,
        episode_id: str,
        event: str,
        agent_id: str = "",
        task_id: str = "",
        data: dict[str, Any] | None = None,
    ) -> None:
        msg = self._RuntimeEventMsg()
        msg.stamp = self._stamp(ts)
        msg.episode_id = episode_id
        msg.agent_id = agent_id
        msg.event = event
        msg.task_id = task_id
        msg.data_json = json.dumps(data or {}, sort_keys=True)
        self._events_pub.publish(msg)

    # ----------------------------------------------------------- spin/shutdown

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
        finally:
            if self._owns_rclpy and self._rclpy.ok():
                self._rclpy.shutdown()

    def __enter__(self) -> "RosBridge":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.shutdown()

    # -------------------------------------------------------------- internals

    def _stamp(self, sim_time_sec: float):  # type: ignore[no-untyped-def]
        from builtin_interfaces.msg import Time

        stamp = Time()
        seconds = int(sim_time_sec)
        stamp.sec = seconds
        stamp.nanosec = max(0, int(round((sim_time_sec - seconds) * 1e9)))
        return stamp

    def _publish_agent_state(self, state: AgentState, sim_time_sec: float) -> None:
        msg = self._AgentStateMsg()
        map_frame, _odom_frame, base_frame = self._frames_by_agent[state.agent_id]
        msg.header.stamp = self._stamp(sim_time_sec)
        msg.header.frame_id = map_frame
        msg.agent_id = state.agent_id
        msg.embodiment_type = state.embodiment_type
        msg.lifecycle_state = state.lifecycle_state.value
        msg.authority_mode = state.authority_mode.value
        msg.pose.pose.position.x = float(state.pose[0])
        msg.pose.pose.position.y = float(state.pose[1])
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(state.pose[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(state.linear_velocity_x)
        msg.twist.twist.linear.y = float(state.linear_velocity_y)
        msg.twist.twist.angular.z = float(state.angular_velocity_z)
        msg.current_task_id = state.current_task_id
        msg.battery = float(state.battery)
        msg.emergency_stopped = bool(state.emergency_stopped)
        msg.capabilities = list(state.capabilities)
        del base_frame  # consumed by tf instead
        self._state_pubs[state.agent_id].publish(msg)

    def _publish_agent_odom(self, state: AgentState, sim_time_sec: float) -> None:
        msg = self._Odometry()
        _map_frame, odom_frame, base_frame = self._frames_by_agent[state.agent_id]
        msg.header.stamp = self._stamp(sim_time_sec)
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = float(state.pose[0])
        msg.pose.pose.position.y = float(state.pose[1])
        qx, qy, qz, qw = _yaw_to_quat(state.pose[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(state.linear_velocity_x)
        msg.twist.twist.angular.z = float(state.angular_velocity_z)
        self._odom_pubs[state.agent_id].publish(msg)

    def _publish_dynamic_tf(self, state: AgentState, sim_time_sec: float) -> None:
        _map_frame, odom_frame, base_frame = self._frames_by_agent[state.agent_id]
        tf = self._TransformStamped()
        tf.header.stamp = self._stamp(sim_time_sec)
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x = float(state.pose[0])
        tf.transform.translation.y = float(state.pose[1])
        qx, qy, qz, qw = _yaw_to_quat(state.pose[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    def _publish_static_tf(self, agent_id: str, map_frame: str, odom_frame: str) -> None:
        del agent_id
        tf = self._TransformStamped()
        tf.header.stamp = self._stamp(self.runtime_clock.sim_time_sec)
        tf.header.frame_id = map_frame
        tf.child_frame_id = odom_frame
        tf.transform.rotation.w = 1.0
        self._tf_static_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------- cmd_vel

    def _on_cmd_vel(self, agent_id: str, twist) -> None:  # type: ignore[no-untyped-def]
        # The bridge is pure transport: hand the Twist to the runtime's teleop
        # entry point, which stamps it as a TELEOP command, runs it through
        # `CommandGate`, emits COMMAND_ACCEPTED / COMMAND_REJECTED, and (on
        # accept) holds off the autonomy loop. No gating happens here.
        if self._teleop_command_handler is None:
            return
        decision = self._teleop_command_handler(
            agent_id,
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        if decision.accepted:
            self.external_command_count += 1
        else:
            self.external_command_reject_count += 1


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


__all__ = ["BridgeConfig", "RosBridge", "TeleopCommandHandler"]
