"""Real-robot embodiment adapter and its ROS-free transport boundary.

`Ros2RobotAdapter` satisfies the same `EmbodimentAdapter` Protocol as the
deterministic fallback and the Genesis adapter (`read_pose`, `apply_command`,
`stop`). It is the *outbound* hardware edge: every `RuntimeCommand` it receives
has already passed `CommandGate`, so AI agents can no more reach a real
actuator than they can a simulated one.

All `rclpy` use lives behind the `RobotTransport` boundary. The adapter never
imports `rclpy`, which keeps it importable and unit-testable in core CI.
`FakeRobotTransport` is the in-memory transport used by tests; the real
`rclpy`-backed transport is built by `genesis_nav.ros2_robot.backend`.

Command-staleness watchdog: a real base must coast to a stop if commands stop
arriving. `seconds_since_command` / `watchdog_expired` expose that check as a
tested helper. NOTE: auto-tripping the watchdog from the runtime tick is a
v0.2 follow-up; today the helper is meant to be driven by the transport's own
node timer (where a hardware watchdog belongs) or by an explicit caller.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from genesis_nav.core.command_gate import RuntimeCommand


@runtime_checkable
class RobotTransport(Protocol):
    """Boundary between the adapter and a real robot's ROS 2 graph.

    Implementations publish velocity to the robot and surface its latest pose.
    `monotonic_sec` returns a monotonically increasing clock used by the
    staleness watchdog; it is injected so tests stay deterministic.
    """

    def publish_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None: ...

    def latest_pose(self) -> tuple[float, float, float] | None: ...

    def monotonic_sec(self) -> float: ...


@dataclass
class FakeRobotTransport:
    """In-memory `RobotTransport` for unit tests.

    Records every published velocity, lets a test set the reported pose, and
    advances a controllable monotonic clock.
    """

    pose: tuple[float, float, float] | None = None
    clock_sec: float = 0.0
    published: list[tuple[float, float, float]] = field(default_factory=list)

    def publish_velocity(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self.published.append((float(linear_x), float(linear_y), float(angular_z)))

    def latest_pose(self) -> tuple[float, float, float] | None:
        return self.pose

    def monotonic_sec(self) -> float:
        return self.clock_sec

    # --- test helpers -------------------------------------------------
    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.pose = (float(x), float(y), float(yaw))

    def advance(self, dt_sec: float) -> None:
        self.clock_sec += float(dt_sec)


@dataclass
class Ros2RobotAdapter:
    """`EmbodimentAdapter` backed by a real robot over a `RobotTransport`."""

    agent_id: str
    transport: RobotTransport
    command_timeout_sec: float = 0.5
    last_linear_x: float = 0.0
    last_linear_y: float = 0.0
    last_angular_z: float = 0.0
    _last_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    _last_command_sec: float | None = None
    _stop_reasons: list[str] = field(default_factory=list)

    def read_pose(self) -> tuple[float, float, float]:
        """Return the freshest pose the transport has seen.

        Falls back to the last cached pose (origin until the first sample) so a
        momentarily empty transport never crashes the runtime tick.
        """

        pose = self.transport.latest_pose()
        if pose is not None:
            self._last_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        return self._last_pose

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None:
        del dt_sec  # the robot integrates motion itself; we only publish setpoints
        self.last_linear_x = float(command.linear_x)
        self.last_linear_y = float(command.linear_y)
        self.last_angular_z = float(command.angular_z)
        self._last_command_sec = self.transport.monotonic_sec()
        self.transport.publish_velocity(
            self.last_linear_x, self.last_linear_y, self.last_angular_z
        )

    def stop(self, reason: str) -> None:
        self._stop_reasons.append(reason)
        self.last_linear_x = 0.0
        self.last_linear_y = 0.0
        self.last_angular_z = 0.0
        self.transport.publish_velocity(0.0, 0.0, 0.0)

    # --- command-staleness watchdog -----------------------------------
    def seconds_since_command(self, now_sec: float | None = None) -> float | None:
        """Seconds since the last `apply_command`, or None if none issued yet."""

        if self._last_command_sec is None:
            return None
        if now_sec is None:
            now_sec = self.transport.monotonic_sec()
        return now_sec - self._last_command_sec

    def watchdog_expired(self, now_sec: float | None = None) -> bool:
        """True once no command has arrived within `command_timeout_sec`.

        Returns False before the first command (nothing to time out against).
        """

        elapsed = self.seconds_since_command(now_sec)
        if elapsed is None:
            return False
        return elapsed > self.command_timeout_sec
