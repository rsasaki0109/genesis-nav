"""Real-robot embodiment backend (`--backend ros2_robot`).

This package is the outbound hardware edge described in the 2026-05-29 ADR
"Real-robot adapter is an `EmbodimentAdapter`, not a new runtime path". A real
robot is just another `EmbodimentAdapter`; the only ROS 2 dependency lives
behind the `RobotTransport` boundary so the adapter logic is unit-testable
without `rclpy`.
"""

from genesis_nav.ros2_robot.adapter import (
    FakeRobotTransport,
    RobotTransport,
    Ros2RobotAdapter,
)
from genesis_nav.ros2_robot.backend import (
    Ros2RobotBackend,
    Ros2RobotNotAvailableError,
    build_ros2_robot_backend,
)

__all__ = [
    "FakeRobotTransport",
    "RobotTransport",
    "Ros2RobotAdapter",
    "Ros2RobotBackend",
    "Ros2RobotNotAvailableError",
    "build_ros2_robot_backend",
]
