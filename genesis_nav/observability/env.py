"""Run-environment metadata capture.

Writes a small JSON snapshot describing the host, the git revision, the
ROS distro, and the Genesis package version at the time of a run. Kept
intentionally side-effect free so it can be unit-tested without touching
git or ROS — callers pass in whatever extras they already know.
"""

from __future__ import annotations

import importlib
import json
import os
import platform
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


def collect_env_metadata(
    *,
    scenario_id: str,
    seed: int,
    backend: str,
    mode: str,
    ros_enabled: bool,
    record_rosbag: bool,
    planner: str = "auto",
    repo_root: Path | None = None,
) -> dict[str, Any]:
    """Return a JSON-serializable run-environment snapshot."""

    return {
        "scenario_id": scenario_id,
        "seed": seed,
        "backend": backend,
        "planner": planner,
        "mode": mode,
        "ros_enabled": bool(ros_enabled),
        "record_rosbag": bool(record_rosbag),
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "hostname": platform.node(),
        "git": _git_metadata(repo_root or Path.cwd()),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "nav2_version": _nav2_version(),
        "genesis_version": _genesis_version(),
    }


def write_env_metadata(path: Path, metadata: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _git_metadata(repo_root: Path) -> dict[str, Any]:
    sha = _run_git(["rev-parse", "HEAD"], repo_root)
    if not sha:
        return {"sha": "", "dirty": False, "branch": ""}
    branch = _run_git(["rev-parse", "--abbrev-ref", "HEAD"], repo_root)
    status = _run_git(["status", "--porcelain"], repo_root)
    return {
        "sha": sha,
        "branch": branch,
        "dirty": bool(status),
    }


def _run_git(args: list[str], cwd: Path) -> str:
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=cwd,
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return ""
    if completed.returncode != 0:
        return ""
    return completed.stdout.strip()


def _genesis_version() -> str:
    try:
        module = importlib.import_module("genesis")
    except ImportError:
        return ""
    return str(getattr(module, "__version__", ""))


def _nav2_version() -> str:
    """Best-effort Nav2 stack version from the ROS 2 ament index.

    Reads ``<version>`` from the ``package.xml`` of a representative Nav2
    package. Returns ``""`` when ROS 2 / Nav2 is not on the path so pure-sim
    and core-CI runs (no `ament_index_python`) stay quiet. Never raises — the
    field is observability only and must not break a run.
    """

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return ""
    for package in ("nav2_bringup", "nav2_msgs", "nav2_core"):
        try:
            share = Path(get_package_share_directory(package))
        except Exception:  # PackageNotFoundError and anything below it
            continue
        version = _read_package_xml_version(share / "package.xml")
        if version:
            return version
    return ""


def _read_package_xml_version(path: Path) -> str:
    """Extract the ``<version>`` text from a ROS 2 ``package.xml``."""

    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return ""
    match = re.search(r"<version>\s*([^<\s]+)\s*</version>", text)
    return match.group(1) if match else ""


__all__ = ["collect_env_metadata", "write_env_metadata"]
