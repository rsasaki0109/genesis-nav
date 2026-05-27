import math

from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import CommandGate
from genesis_nav.core.embodiment import DiffDriveKinematics
from genesis_nav.navigation.local_controller import LocalControllerConfig, SimpleLocalController


def test_controller_rotates_before_moving() -> None:
    controller = SimpleLocalController()
    command = controller.compute(
        "robot_001",
        pose=(0.0, 0.0, 0.0),
        goal=(0.0, 1.0, 0.0),
        issued_at_sec=1.0,
    )

    assert command.linear_x == 0.0
    assert command.angular_z > 0.0
    assert command.authority is AuthorityMode.AUTONOMY


def test_controller_drives_forward_when_aligned() -> None:
    controller = SimpleLocalController()
    command = controller.compute(
        "robot_001",
        pose=(0.0, 0.0, 0.0),
        goal=(1.0, 0.0, 0.0),
        issued_at_sec=1.0,
    )

    assert command.linear_x > 0.0
    assert abs(command.angular_z) < 1e-6


def test_diff_drive_integrator_round_trip() -> None:
    controller = SimpleLocalController(LocalControllerConfig(max_linear_x=0.5))
    gate = CommandGate()
    adapter = DiffDriveKinematics("robot_001")

    goal = (1.0, 0.0, 0.0)
    sim_time = 0.0
    dt = 0.02
    for _ in range(500):
        if controller.at_goal(adapter.read_pose(), goal):
            break
        sim_time += dt
        command = controller.compute(
            "robot_001",
            pose=adapter.read_pose(),
            goal=goal,
            issued_at_sec=sim_time,
        )
        decision = gate.evaluate(command, now_sec=sim_time)
        assert decision.accepted and decision.command is not None
        adapter.apply_command(decision.command, dt)

    pose = adapter.read_pose()
    assert math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= 0.1
