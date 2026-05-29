"""Build a `Nav2Planner` backed by a live `rclpy` ComputePathToPose client.

`build_nav2_planner` is the only entry point that imports `rclpy` and
`nav2_msgs`. If they are missing it raises `Nav2NotAvailableError` with an
actionable hint instead of failing deep inside the runtime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from genesis_nav.benchmarks.scenario import Scenario
from genesis_nav.nav2.planner import Nav2Planner


class Nav2NotAvailableError(RuntimeError):
    """Raised when planner: nav2 is requested but rclpy / nav2_msgs are missing."""


def build_nav2_planner(scenario: Scenario) -> Nav2Planner:
    """Build a Nav2-delegating planner for the given scenario.

    Raises `Nav2NotAvailableError` if the ROS 2 / Nav2 action surface is not
    importable.
    """

    rclpy = _require_nav2()
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("genesis_nav_nav2_planner")
    block = scenario.raw.get("nav2", {}) if scenario.raw else {}
    action = str(block.get("compute_path_action", "compute_path_to_pose"))
    frame_id = str(block.get("frame_id", "map"))
    timeout_sec = float(block.get("timeout_sec", 5.0))
    service = _RclpyNav2PathService(
        node=node, action=action, frame_id=frame_id, timeout_sec=timeout_sec
    )
    return Nav2Planner(service)


def _require_nav2() -> Any:
    try:
        import rclpy  # noqa: F401
        from nav2_msgs.action import ComputePathToPose  # noqa: F401

        return rclpy
    except ImportError as exc:
        raise Nav2NotAvailableError(
            "rclpy / nav2_msgs are not importable. Source a ROS 2 + Nav2 "
            "environment (e.g. `source /opt/ros/jazzy/setup.bash`) before "
            "selecting runtime.navigation.planner: nav2."
        ) from exc


@dataclass
class _RclpyNav2PathService:
    """`Nav2PathService` backed by a Nav2 `ComputePathToPose` action client."""

    node: Any
    action: str
    frame_id: str
    timeout_sec: float

    def __post_init__(self) -> None:
        import rclpy  # noqa: F401
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient

        self._ComputePathToPose = ComputePathToPose
        self._client = ActionClient(self.node, ComputePathToPose, self.action)

    def compute_path(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> list[tuple[float, float, float]]:
        import rclpy
        from rclpy.task import Future

        if not self._client.wait_for_server(timeout_sec=self.timeout_sec):
            return []
        goal_msg = self._ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.start = self._pose_stamped(start)
        goal_msg.goal = self._pose_stamped(goal)

        send_future: Future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=self.timeout_sec)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return []
        result_future: Future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=self.timeout_sec)
        wrapper = result_future.result()
        if wrapper is None:
            return []
        return [self._pose_to_xyyaw(p) for p in wrapper.result.path.poses]

    def _pose_stamped(self, xyyaw: tuple[float, float, float]) -> Any:
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(xyyaw[0])
        msg.pose.position.y = float(xyyaw[1])
        msg.pose.orientation.z = math.sin(xyyaw[2] / 2.0)
        msg.pose.orientation.w = math.cos(xyyaw[2] / 2.0)
        return msg

    @staticmethod
    def _pose_to_xyyaw(pose_stamped: Any) -> tuple[float, float, float]:
        p = pose_stamped.pose.position
        q = pose_stamped.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return (float(p.x), float(p.y), math.atan2(siny_cosp, cosy_cosp))


__all__ = ["Nav2NotAvailableError", "build_nav2_planner"]
