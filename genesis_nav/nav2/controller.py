"""Nav2-delegated local controller and its ROS-free service boundary.

Companion to `planner.py`. Where `Nav2Planner` delegates *global planning*,
`Nav2Controller` delegates *velocity generation* to a running Nav2 controller
server (its `FollowPath` / `cmd_vel` output) through the `Nav2ControllerService`
boundary. All `rclpy` use lives in `bridge.py`; this module stays importable in
core CI.

The safety contract is the point of this slice. `Nav2Controller` is a drop-in
for `SimpleLocalController`: it returns a `RuntimeCommand`, and the runtime
evaluates *that* command through `CommandGate` on the existing autonomy path
before it ever reaches an actuator. So Nav2's `cmd_vel` traverses `CommandGate`
as an ``AUTONOMY`` command by construction — a misbehaving Nav2 controller
(non-finite or over-limit velocity) is rejected and the agent stopped, exactly
as for the in-tree controller. genesis-nav never lets Nav2 drive the actuator
directly.

When the Nav2 controller has no command available yet (server not ready, no
path), the boundary returns ``None`` and `Nav2Controller` falls back to the
in-tree `SimpleLocalController` so motion degrades gracefully rather than
stalling.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import RuntimeCommand
from genesis_nav.navigation.local_controller import SimpleLocalController

Pose = tuple[float, float, float]
Velocity = tuple[float, float, float]

NAV2_CONTROLLER_SOURCE = "nav2_controller"


@runtime_checkable
class Nav2ControllerService(Protocol):
    """Boundary to a Nav2 controller server.

    Returns the latest commanded velocity ``(linear_x, linear_y, angular_z)``
    for ``agent_id`` driving from ``pose`` toward ``target``, or ``None`` when
    Nav2 has no command available (server not ready, no active path).
    """

    def compute_velocity(
        self, agent_id: str, pose: Pose, target: Pose
    ) -> Velocity | None: ...


class FakeNav2ControllerService:
    """In-memory `Nav2ControllerService` for unit tests.

    Returns a scripted sequence of velocities (the last one repeats once the
    script is exhausted) and records every request so a test can assert the
    controller delegated with the expected pose/target. A scripted ``None``
    models "Nav2 has no command yet" so the fallback path can be exercised.
    """

    def __init__(self, velocities: list[Velocity | None] | None = None) -> None:
        self.velocities: list[Velocity | None] = list(velocities or [])
        self.requests: list[tuple[str, Pose, Pose]] = []
        self._index = 0

    def compute_velocity(
        self, agent_id: str, pose: Pose, target: Pose
    ) -> Velocity | None:
        self.requests.append((agent_id, pose, target))
        if not self.velocities:
            return None
        if self._index < len(self.velocities):
            value = self.velocities[self._index]
            self._index += 1
            return value
        return self.velocities[-1]


class Nav2Controller:
    """Local controller that delegates velocity to Nav2 via a service boundary.

    Drop-in for `SimpleLocalController`: same `compute` / `at_goal` surface, so
    the runtime treats it identically and its output flows through the same
    `CommandGate`. Goal arbitration stays with genesis-nav (`at_goal` delegates
    to the fallback), so Nav2 owns velocity but not when the task is done.
    """

    def __init__(
        self,
        service: Nav2ControllerService,
        fallback: SimpleLocalController | None = None,
    ) -> None:
        self.service = service
        self.fallback = fallback or SimpleLocalController()

    def at_goal(self, pose: Pose, goal: Pose) -> bool:
        return self.fallback.at_goal(pose, goal)

    def compute(
        self,
        agent_id: str,
        pose: Pose,
        goal: Pose,
        *,
        issued_at_sec: float,
        ttl_ms: int = 200,
        authority: AuthorityMode = AuthorityMode.AUTONOMY,
        source: str = NAV2_CONTROLLER_SOURCE,
    ) -> RuntimeCommand:
        velocity = self.service.compute_velocity(agent_id, pose, goal)
        if velocity is None:
            # Nav2 has nothing to say yet — keep moving via the in-tree
            # controller rather than stalling. The fallback's command still
            # traverses CommandGate on the runtime's autonomy path.
            return self.fallback.compute(
                agent_id,
                pose,
                goal,
                issued_at_sec=issued_at_sec,
                ttl_ms=ttl_ms,
                authority=authority,
            )
        linear_x, linear_y, angular_z = velocity
        return RuntimeCommand(
            agent_id=agent_id,
            linear_x=float(linear_x),
            linear_y=float(linear_y),
            angular_z=float(angular_z),
            authority=authority,
            source=source,
            issued_at_sec=issued_at_sec,
            ttl_ms=ttl_ms,
        )


__all__ = [
    "FakeNav2ControllerService",
    "NAV2_CONTROLLER_SOURCE",
    "Nav2Controller",
    "Nav2ControllerService",
    "Pose",
    "Velocity",
]
