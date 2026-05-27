"""Launch the genesis-nav smoke scenario with the ROS 2 bridge enabled.

This launch wraps `gnav run examples/scenarios/smoke.yaml --fast --ros` as a
ROS-managed process so users can `ros2 launch genesis_nav_bringup
smoke.launch.py` and see runtime topics in their ROS graph.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value=str(Path("examples/scenarios/smoke.yaml")),
        description="Scenario YAML to run",
    )
    qos_arg = DeclareLaunchArgument(
        "qos_profile",
        default_value=str(Path("configs/qos/default.yaml")),
        description="QoS profile YAML",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=str(Path("runs")),
        description="Run artifact root",
    )

    gnav = ExecuteProcess(
        cmd=[
            "gnav",
            "run",
            LaunchConfiguration("scenario"),
            "--fast",
            "--ros",
            "--qos-profile",
            LaunchConfiguration("qos_profile"),
            "--output-dir",
            LaunchConfiguration("output_dir"),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([scenario_arg, qos_arg, output_dir_arg, gnav])
