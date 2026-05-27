"""Navigation-intent adapter for the humanoid shell.

The v0.1 humanoid shell is a *shell*, not a locomotion stack. It exists so
that scenarios, frame conventions, fleet plumbing, and the fall-detected
safety stop can be exercised end-to-end without taking on whole-body control,
gait planning, or balance simulation.

Concretely the adapter treats incoming velocity commands as *base-frame
navigation intent* and integrates the pelvis-projected base pose with the
same planar kinematics used by `DiffDriveKinematics`. It does not pretend to
generate joint torques or footstep plans.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from genesis_nav.core.command_gate import RuntimeCommand


@dataclass
class HumanoidIntentAdapter:
    """Planar navigation-intent adapter for humanoid agents.

    Attributes mirror :class:`DiffDriveKinematics` so the runtime loop can
    treat both adapters uniformly. ``fall_detected`` is the safety signal
    polled by the runtime; ``trigger_fall`` is the test/scenario hook used
    to inject falls deterministically. Real Genesis-backed humanoids will
    set this from balance estimators.
    """

    agent_id: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0
    fall_detected: bool = False
    balance_margin: float = 1.0
    fall_reason: str = ""

    def read_pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def apply_command(self, command: RuntimeCommand, dt_sec: float) -> None:
        if dt_sec <= 0:
            return
        if self.fall_detected:
            self.linear_x = 0.0
            self.angular_z = 0.0
            return
        self.linear_x = float(command.linear_x)
        self.angular_z = float(command.angular_z)
        self.yaw = _wrap_angle(self.yaw + self.angular_z * dt_sec)
        self.x += self.linear_x * math.cos(self.yaw) * dt_sec
        self.y += self.linear_x * math.sin(self.yaw) * dt_sec

    def stop(self, reason: str) -> None:
        del reason
        self.linear_x = 0.0
        self.angular_z = 0.0

    def trigger_fall(self, reason: str = "manual") -> None:
        """Mark the humanoid as fallen and zero the intent.

        Used by tests and scenario injectors. Real adapters will derive this
        from a balance estimator and call the same setter.
        """

        self.fall_detected = True
        self.fall_reason = reason
        self.balance_margin = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0


def _wrap_angle(theta: float) -> float:
    return ((theta + math.pi) % (2.0 * math.pi)) - math.pi


__all__ = ["HumanoidIntentAdapter"]
