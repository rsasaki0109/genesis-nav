"""Tests for holonomic robot kinematics and navigation."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from genesis_nav.cli.main import main
from genesis_nav.core.authority import AuthorityMode
from genesis_nav.core.command_gate import CommandGate, RuntimeCommand
from genesis_nav.navigation.local_controller import HolonomicLocalController
from genesis_nav.robots.holonomic import HolonomicKinematics


def test_holonomic_controller_strafes_without_initial_rotation() -> None:
    controller = HolonomicLocalController()
    command = controller.compute(
        "omni_001",
        pose=(0.0, 0.0, 0.0),
        goal=(0.0, 1.0, 0.0),
        issued_at_sec=1.0,
    )

    assert command.linear_x == 0.0
    assert command.linear_y > 0.0
    assert abs(command.angular_z) < 1e-6


def test_holonomic_kinematics_integrates_body_frame_velocity() -> None:
    adapter = HolonomicKinematics("omni_001")
    adapter.apply_command(
        RuntimeCommand(
            agent_id="omni_001",
            linear_y=1.0,
            authority=AuthorityMode.AUTONOMY,
            issued_at_sec=0.0,
        ),
        1.0,
    )

    pose = adapter.read_pose()
    assert pose[0] == 0.0
    assert pose[1] == pytest.approx(1.0)


def test_holonomic_smoke_reaches_lateral_goal(tmp_path: Path) -> None:
    rc = main(
        [
            "run",
            "examples/scenarios/holonomic_smoke.yaml",
            "--fast",
            "--output-dir",
            str(tmp_path),
        ]
    )
    assert rc == 0
    run_dir = next(tmp_path.iterdir())
    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    assert metrics["success_rate"] == 1.0
    assert metrics["task_succeeded_count"] == 1


def test_holonomic_reaches_goal_via_command_gate() -> None:
    controller = HolonomicLocalController()
    gate = CommandGate()
    adapter = HolonomicKinematics("omni_001")

    goal = (0.0, 1.0, 0.0)
    sim_time = 0.0
    dt = 0.02
    saw_lateral_command = False
    for _ in range(500):
        if controller.at_goal(adapter.read_pose(), goal):
            break
        sim_time += dt
        command = controller.compute(
            "omni_001",
            pose=adapter.read_pose(),
            goal=goal,
            issued_at_sec=sim_time,
        )
        if abs(command.linear_y) > 1e-6 and abs(command.linear_x) < 1e-6:
            saw_lateral_command = True
        decision = gate.evaluate(command, now_sec=sim_time)
        assert decision.accepted and decision.command is not None
        adapter.apply_command(decision.command, dt)

    pose = adapter.read_pose()
    assert saw_lateral_command
    assert math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= 0.1
