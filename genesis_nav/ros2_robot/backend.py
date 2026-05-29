"""Real-robot backend: spawns `Ros2RobotAdapter`s over an `rclpy` graph.

Mirrors `GenesisBackend` so the runtime wiring is identical regardless of
embodiment. `build_ros2_robot_backend` is the only entry point that imports
`rclpy`; if ROS 2 is missing it raises `Ros2RobotNotAvailableError` with an
actionable hint instead of failing deep inside the runtime.

Per-agent topic convention (REP-103 / REP-105 frames, identity units):
- publishes `geometry_msgs/Twist` on `/<agent_id>/cmd_vel`
- subscribes `nav_msgs/Odometry` on `/<agent_id>/odom`
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from genesis_nav.benchmarks.scenario import Scenario
from genesis_nav.core.agent import AgentSpec
from genesis_nav.core.embodiment import EmbodimentAdapter
from genesis_nav.ros2_robot.adapter import Ros2RobotAdapter


class Ros2RobotNotAvailableError(RuntimeError):
    """Raised when --backend ros2_robot is requested but rclpy is missing."""


@dataclass
class Ros2RobotBackend:
    """Owns one `rclpy` node and the per-agent adapters bound to it."""

    node: Any
    command_timeout_sec: float = 0.5
    adapters: dict[str, Ros2RobotAdapter] = field(default_factory=dict)

    def step(self, dt_sec: float) -> None:
        del dt_sec
        import rclpy

        # Drain pending odom callbacks without blocking the runtime tick.
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def reset(self) -> None:  # symmetry with GenesisBackend; nothing to reset on hardware
        return None

    def spawn(self, spec: AgentSpec) -> EmbodimentAdapter:
        transport = _RclpyRobotTransport(self.node, spec.agent_id)
        adapter = Ros2RobotAdapter(
            agent_id=spec.agent_id,
            transport=transport,
            command_timeout_sec=self.command_timeout_sec,
        )
        self.adapters[spec.agent_id] = adapter
        return adapter

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
        finally:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()


def build_ros2_robot_backend(scenario: Scenario) -> Ros2RobotBackend:
    """Build a real-robot backend for the given scenario.

    Raises `Ros2RobotNotAvailableError` if `rclpy` cannot be imported.
    """

    rclpy = _require_rclpy()
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("genesis_nav_ros2_robot")
    timeout = float(scenario.raw.get("real_robot", {}).get("command_timeout_sec", 0.5))
    return Ros2RobotBackend(node=node, command_timeout_sec=timeout)


def _require_rclpy() -> Any:
    try:
        import rclpy

        return rclpy
    except ImportError as exc:
        raise Ros2RobotNotAvailableError(
            "rclpy is not importable. Source a ROS 2 environment "
            "(e.g. `source /opt/ros/jazzy/setup.bash`) before using "
            "--backend ros2_robot."
        ) from exc


@dataclass
class _RclpyRobotTransport:
    """`RobotTransport` backed by a live `rclpy` node (publisher + subscriber).

    Kept private: it is only constructed by `Ros2RobotBackend.spawn`, which has
    already proven `rclpy` is importable.
    """

    node: Any
    agent_id: str

    def __post_init__(self) -> None:
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        self._Twist = Twist
        self._pose: tuple[float, float, float] | None = None
        self._pub = self.node.create_publisher(Twist, f"/{self.agent_id}/cmd_vel", 10)
        self._sub = self.node.create_subscription(
            Odometry, f"/{self.agent_id}/odom", self._on_odom, 10
        )

    def _on_odom(self, msg: Any) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (float(p.x), float(p.y), _yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def publish_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        twist = self._Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self._pub.publish(twist)

    def latest_pose(self) -> tuple[float, float, float] | None:
        return self._pose

    def monotonic_sec(self) -> float:
        # ROS time would be ideal, but the node clock may be sim-time; the
        # watchdog only needs a monotonic wall reference, so use the steady
        # clock the node exposes via its context.
        from time import monotonic

        return monotonic()


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


__all__ = [
    "Ros2RobotBackend",
    "Ros2RobotNotAvailableError",
    "build_ros2_robot_backend",
]
