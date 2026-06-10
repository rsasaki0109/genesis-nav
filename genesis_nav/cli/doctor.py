"""`gnav doctor` dependency checks."""

from __future__ import annotations

import importlib.util
import json
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class DoctorCheck:
    name: str
    ok: bool
    hint: str

    @property
    def status(self) -> str:
        return "ok" if self.ok else "missing"


def collect_doctor_checks() -> list[DoctorCheck]:
    return [
        DoctorCheck("python", True, ""),
        DoctorCheck("yaml", True, ""),
        DoctorCheck(
            "rclpy",
            importlib.util.find_spec("rclpy") is not None,
            "install ROS 2 (jazzy/humble) and "
            "`source /opt/ros/<distro>/setup.bash` before --ros",
        ),
        DoctorCheck(
            "genesis_nav_msgs",
            importlib.util.find_spec("genesis_nav_msgs") is not None,
            "colcon build --base-paths ros2_ws/src --packages-select genesis_nav_msgs",
        ),
        DoctorCheck(
            "genesis",
            importlib.util.find_spec("genesis") is not None,
            "pip install genesis-world  # required for --backend genesis",
        ),
    ]


def format_doctor_text(checks: list[DoctorCheck]) -> str:
    lines: list[str] = []
    for check in checks:
        line = f"{check.name}: {check.status}"
        if not check.ok and check.hint:
            line += f"  ({check.hint})"
        lines.append(line)
    return "\n".join(lines) + "\n"


def format_doctor_json(checks: list[DoctorCheck]) -> str:
    payload: dict[str, Any] = {
        "checks": [
            {
                "name": check.name,
                "status": check.status,
                **({"hint": check.hint} if not check.ok and check.hint else {}),
            }
            for check in checks
        ]
    }
    return json.dumps(payload, indent=2) + "\n"


def run_doctor(*, as_json: bool = False) -> int:
    checks = collect_doctor_checks()
    if as_json:
        print(format_doctor_json(checks), end="")
    else:
        print(format_doctor_text(checks), end="")
    return 0
