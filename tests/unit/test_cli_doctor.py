import json

from genesis_nav.cli.doctor import (
    DoctorCheck,
    collect_doctor_checks,
    format_doctor_json,
    format_doctor_text,
)
from genesis_nav.cli.main import main


def test_collect_doctor_checks_has_expected_names() -> None:
    names = [check.name for check in collect_doctor_checks()]
    assert names == ["python", "yaml", "rclpy", "genesis_nav_msgs", "genesis"]


def test_format_doctor_text_includes_hints_for_missing() -> None:
    text = format_doctor_text([DoctorCheck("rclpy", False, "install ROS 2")])
    assert "rclpy: missing" in text
    assert "install ROS 2" in text


def test_format_doctor_json_matches_checks() -> None:
    checks = collect_doctor_checks()
    payload = json.loads(format_doctor_json(checks))
    assert len(payload["checks"]) == len(checks)
    for entry, check in zip(payload["checks"], checks, strict=True):
        assert entry["name"] == check.name
        assert entry["status"] == check.status
        if not check.ok and check.hint:
            assert entry["hint"] == check.hint
        else:
            assert "hint" not in entry


def test_doctor_command_json_flag(capsys) -> None:
    assert main(["doctor", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert [entry["name"] for entry in payload["checks"]] == [
        "python",
        "yaml",
        "rclpy",
        "genesis_nav_msgs",
        "genesis",
    ]


def test_doctor_command_text_default(capsys) -> None:
    assert main(["doctor"]) == 0
    out = capsys.readouterr().out
    assert "python: ok" in out
    assert "yaml: ok" in out
