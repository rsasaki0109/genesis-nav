"""Re-run a stored scenario with the ROS bridge and write a rosbag2 bag.

Deterministic fallback (and loopback ros2_robot) runs can be exported after the
fact when ``gnav run`` did not pass ``--ros``. The exporter re-simulates from
``resolved_config.yaml`` and mirrors the same bridged topics that live recording
would have captured.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

from genesis_nav.benchmarks.scenario import Scenario, load_scenario
from genesis_nav.core.runtime import Runtime
from genesis_nav.ros.bag_writer import (
    RosbagNotAvailableError,
    RosbagRecorder,
    load_rosbag_profile,
)


class _NullEventSink:
    def write(
        self,
        *,
        ts: float,
        episode_id: str,
        event: str,
        agent_id: str = "",
        task_id: str = "",
        data: dict[str, Any] | None = None,
    ) -> None:
        del ts, episode_id, event, agent_id, task_id, data


def default_rosbag_profile(run_dir: Path, override: Path | None = None) -> Path:
    if override is not None:
        return override
    stored = run_dir / "rosbag_profile.yaml"
    if stored.is_file():
        return stored
    return Path("configs/rosbag/minimal.yaml")


def default_qos_profile(run_dir: Path, override: Path | None = None) -> Path:
    if override is not None:
        return override
    stored = run_dir / "qos_profile.yaml"
    if stored.is_file():
        return stored
    return Path("configs/qos/default.yaml")


def prepare_bag_directory(uri: Path) -> None:
    if uri.exists():
        shutil.rmtree(uri)


def export_run_to_rosbag(
    run_dir: Path,
    *,
    rosbag_profile: Path | None = None,
    qos_profile: Path | None = None,
) -> Path:
    """Re-simulate ``run_dir`` and write bridged topics to ``run_dir/rosbag/``."""

    scenario = load_scenario(run_dir / "resolved_config.yaml")
    env = json.loads((run_dir / "env.json").read_text(encoding="utf-8"))
    backend = str(env.get("backend", "fallback"))
    mode = str(env.get("mode", "fast"))
    if mode != "fast":
        raise ValueError("replay rosbag export currently requires mode=fast runs")

    profile_path = default_rosbag_profile(run_dir, rosbag_profile)
    qos_path = default_qos_profile(run_dir, qos_profile)
    bag_dir = run_dir / "rosbag"
    prepare_bag_directory(bag_dir)

    try:
        recorder = RosbagRecorder(
            bag_dir,
            load_rosbag_profile(profile_path),
            [spec.namespace for spec in scenario.agents],
        )
    except RosbagNotAvailableError:
        raise

    genesis_backend = None
    robot_backend = None
    if backend == "genesis":
        from genesis_nav.genesis.backend import (
            GenesisNotAvailableError,
            build_genesis_backend,
        )

        try:
            genesis_backend = build_genesis_backend(scenario)
        except GenesisNotAvailableError as exc:
            raise RuntimeError(f"backend genesis unavailable: {exc}") from exc
    elif backend == "ros2_robot":
        from genesis_nav.ros2_robot.backend import (
            Ros2RobotNotAvailableError,
            build_loopback_robot_backend,
            build_ros2_robot_backend,
            robot_transport_mode,
        )

        if robot_transport_mode(scenario) == "loopback":
            robot_backend = build_loopback_robot_backend(scenario)
        else:
            try:
                robot_backend = build_ros2_robot_backend(scenario)
            except Ros2RobotNotAvailableError as exc:
                raise RuntimeError(f"backend ros2_robot unavailable: {exc}") from exc

    adapter_factory = None
    if genesis_backend is not None:
        adapter_factory = genesis_backend.spawn
    elif robot_backend is not None:
        adapter_factory = robot_backend.spawn

    from genesis_nav.nav2.bridge import Nav2NotAvailableError

    null_sink = _NullEventSink()
    try:
        runtime = Runtime.from_scenario(scenario, null_sink, adapter_factory=adapter_factory)
    except Nav2NotAvailableError as exc:
        raise RuntimeError(f"planner nav2 unavailable: {exc}") from exc

    if genesis_backend is not None:
        genesis_backend.finalize()

    from genesis_nav.ros.bridge import BridgeConfig, RosBridge

    episode_id = f"{run_dir.name}_seed{scenario.seed}"
    max_sim_seconds = float(scenario.raw.get("max_sim_seconds", 60.0))

    def _ros_teleop(agent_id, linear_x, linear_y, angular_z):
        return runtime.submit_teleop_command(
            agent_id,
            requester_id="ros_cmd_vel",
            source="ros_cmd_vel",
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            episode_id=episode_id,
        )

    bridge = RosBridge(
        runtime.registry,
        runtime.clock,
        null_sink,
        config=BridgeConfig(qos_path=qos_path),
        teleop_command_handler=_ros_teleop,
        episode_id=episode_id,
    )
    runtime.events = bridge
    bridge.set_diagnostics_provider(runtime.diagnostics)
    bridge.set_rosbag_recorder(recorder)

    try:
        bridge.write(
            ts=0.0,
            episode_id=episode_id,
            event="SCENARIO_STARTED",
            data={
                "scenario_id": scenario.scenario_id,
                "seed": scenario.seed,
                "mode": mode,
                "agent_count": len(scenario.agents),
                "task_count": len(scenario.tasks),
                "export": "replay_to_rosbag",
            },
        )
        for task in scenario.tasks:
            runtime.submit_task(task, episode_id=episode_id)
        runtime.dispatch_pending(episode_id=episode_id)

        bridge.publish_clock(0.0)
        bridge.publish_states(0.0)
        bridge.publish_diagnostics(0.0)
        bridge.publish_scenario_state(
            0.0,
            scenario_id=scenario.scenario_id,
            seed=scenario.seed,
            runtime_mode=mode,
            paused=False,
            recording=True,
            extra={"export": "replay_to_rosbag"},
        )

        def on_step(sim_time: float) -> None:
            if genesis_backend is not None:
                genesis_backend.step(runtime.clock.step_sec)
            if robot_backend is not None:
                robot_backend.step(runtime.clock.step_sec)
            bridge.publish_clock(sim_time)
            bridge.publish_states(sim_time)
            bridge.publish_diagnostics(sim_time)
            snapshot = runtime.metrics.summary()
            bridge.publish_fleet_state(
                sim_time,
                pending=len(runtime.task_queue),
                active=sum(
                    1
                    for state in runtime.registry.list_states()
                    if state.current_task_id
                ),
                completed=int(snapshot["task_succeeded_count"]),
            )
            bridge.spin_once(timeout_sec=0.0)

        runtime.run_until_idle(
            episode_id=episode_id,
            max_sim_seconds=max_sim_seconds,
            on_step=on_step,
        )
        summary = runtime.metrics.summary()
        bridge.write(
            ts=runtime.clock.sim_time_sec,
            episode_id=episode_id,
            event="SCENARIO_FINISHED",
            data={"task_count": len(scenario.tasks), "summary": summary},
        )
        bridge.publish_fleet_state(
            runtime.clock.sim_time_sec,
            completed=int(summary["task_succeeded_count"]),
            extra={"summary": summary},
        )
    finally:
        recorder.close()
        bridge.shutdown()
        if robot_backend is not None:
            robot_backend.shutdown()

    if not (bag_dir / "metadata.yaml").is_file():
        raise RuntimeError(f"rosbag export did not produce metadata.yaml in {bag_dir}")

    skipped = bag_dir / "RECORDING_SKIPPED"
    if skipped.is_file():
        skipped.unlink()

    return bag_dir


__all__ = [
    "default_qos_profile",
    "default_rosbag_profile",
    "export_run_to_rosbag",
    "prepare_bag_directory",
]
