"""`gnav` command-line entry point."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from genesis_nav import __version__
from genesis_nav.cli.doctor import run_doctor
from genesis_nav.benchmarks.runner import run_benchmark_suite, terminal_scenario_record
from genesis_nav.benchmarks.scenario import Scenario, load_scenario
from genesis_nav.core.runtime import Runtime, ensure_run_layout
from genesis_nav.navigation.config import NavigationConfig
from genesis_nav.observability.env import collect_env_metadata, write_env_metadata
from genesis_nav.observability.events import (
    FanoutEventSink,
    JsonlEventWriter,
    RingBufferEventSink,
)
from genesis_nav.observability.metrics import MetricsSnapshot, wilson_success_rate_ci


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="gnav")
    parser.add_argument("--version", action="version", version=f"gnav {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="run a reproducible scenario")
    run.add_argument("scenario", type=Path)
    run.add_argument("--fast", action="store_true", help="run without wall-time synchronization")
    run.add_argument(
        "--record",
        action="store_true",
        help="record bridged ROS 2 topics to run_dir/rosbag (requires --ros)",
    )
    run.add_argument("--output-dir", type=Path, default=Path("runs"))
    run.add_argument(
        "--ros",
        action="store_true",
        help="bridge runtime state to a ROS 2 graph (requires rclpy and genesis_nav_msgs)",
    )
    run.add_argument(
        "--qos-profile",
        type=Path,
        default=Path("configs/qos/default.yaml"),
        help="QoS profile YAML used by --ros",
    )
    run.add_argument(
        "--backend",
        choices=("fallback", "genesis", "ros2_robot"),
        default="fallback",
        help="embodiment backend: 'fallback' uses the in-memory diff-drive, "
        "'genesis' uses the Genesis adapter via the scenario world file, "
        "'ros2_robot' drives a real robot over /<agent>/cmd_vel + /<agent>/odom "
        "(requires a sourced ROS 2 environment)",
    )
    run.add_argument(
        "--rosbag-profile",
        type=Path,
        default=Path("configs/rosbag/minimal.yaml"),
        help="topic list for --record (default: configs/rosbag/minimal.yaml)",
    )
    run.set_defaults(func=run_command)

    replay = subparsers.add_parser("replay", help="validate a run artifact for replay")
    replay.add_argument("run_dir", type=Path)
    replay.add_argument(
        "--print-events",
        action="store_true",
        help="stream task and safety events from events.jsonl in order",
    )
    replay.add_argument(
        "--to-rosbag",
        action="store_true",
        help="re-run the scenario with the ROS bridge and write run_dir/rosbag/",
    )
    replay.add_argument(
        "--rosbag-profile",
        type=Path,
        default=None,
        help="topic profile for --to-rosbag (default: run_dir/rosbag_profile.yaml "
        "or configs/rosbag/minimal.yaml)",
    )
    replay.set_defaults(func=replay_command)

    bench = subparsers.add_parser("bench", help="validate or run benchmark scenarios")
    bench.add_argument(
        "scenario",
        type=Path,
        nargs="?",
        help="benchmark scenario file (omitted with --run)",
    )
    bench.add_argument(
        "--run",
        type=Path,
        metavar="SUITE_DIR",
        help="run every *.yaml under SUITE_DIR and emit a benchmark report",
    )
    bench.add_argument(
        "--output-dir",
        type=Path,
        default=Path("benchmarks/_runs"),
        help="root directory for benchmark run artifacts (default: benchmarks/_runs)",
    )
    bench.add_argument(
        "--report",
        type=Path,
        default=None,
        help="path to write the benchmark report JSON (default: <output-dir>/<suite>_report.json)",
    )
    bench.add_argument(
        "--include-integration",
        action="store_true",
        help="also run scenarios marked benchmark.integration (need an external "
        "stack, e.g. a live Nav2 server); skipped by default",
    )
    bench.set_defaults(func=bench_command)

    doctor = subparsers.add_parser("doctor", help="check local runtime dependencies")
    doctor.add_argument(
        "--json",
        action="store_true",
        help="print checks as machine-readable JSON",
    )
    doctor.set_defaults(func=doctor_command)

    return parser


def run_command(args: argparse.Namespace) -> int:
    scenario = load_scenario(args.scenario)
    run_dir = create_run_dir(args.output_dir, scenario)
    record_rosbag = bool(args.record or scenario.record.rosbag)
    ensure_run_layout(run_dir, record_rosbag=record_rosbag)

    shutil.copyfile(scenario.source_path, run_dir / "scenario.yaml")
    write_yaml(run_dir / "resolved_config.yaml", scenario.raw)

    episode_id = f"{run_dir.name}_seed{scenario.seed}"
    max_sim_seconds = float(scenario.raw.get("max_sim_seconds", 60.0))
    mode = "fast" if args.fast else "realtime"
    planner_choice = NavigationConfig.from_scenario_raw(scenario.raw).planner

    env_metadata = collect_env_metadata(
        scenario_id=scenario.scenario_id,
        seed=scenario.seed,
        backend=args.backend,
        mode=mode,
        ros_enabled=bool(args.ros),
        record_rosbag=record_rosbag,
        planner=planner_choice,
    )
    write_env_metadata(run_dir / "env.json", env_metadata)

    if args.ros:
        qos_source = Path(args.qos_profile)
        if qos_source.exists():
            shutil.copyfile(qos_source, run_dir / "qos_profile.yaml")
    if record_rosbag:
        rosbag_source = Path(args.rosbag_profile)
        if rosbag_source.exists():
            shutil.copyfile(rosbag_source, run_dir / "rosbag_profile.yaml")

    bridge = None
    bag_recorder = None
    genesis_backend = None
    robot_backend = None
    if args.backend == "genesis":
        try:
            from genesis_nav.genesis.backend import (
                GenesisNotAvailableError,
                build_genesis_backend,
            )
            genesis_backend = build_genesis_backend(scenario)
        except GenesisNotAvailableError as exc:
            print(f"--backend genesis unavailable: {exc}", file=sys.stderr)
            return 4
    elif args.backend == "ros2_robot":
        from genesis_nav.ros2_robot.backend import (
            Ros2RobotNotAvailableError,
            build_loopback_robot_backend,
            build_ros2_robot_backend,
            robot_transport_mode,
        )

        if robot_transport_mode(scenario) == "loopback":
            # Loop-closed in process: exercises the real-robot contract end to
            # end without rclpy or hardware (see the 2026-05-29 real-robot ADR).
            robot_backend = build_loopback_robot_backend(scenario)
        else:
            try:
                robot_backend = build_ros2_robot_backend(scenario)
            except Ros2RobotNotAvailableError as exc:
                print(f"--backend ros2_robot unavailable: {exc}", file=sys.stderr)
                return 4

    with JsonlEventWriter(run_dir / "events.jsonl") as jsonl_sink:
        event_buffer = RingBufferEventSink(capacity=2048)
        event_sink: Any = FanoutEventSink([jsonl_sink, event_buffer])
        adapter_factory = None
        if genesis_backend is not None:
            adapter_factory = genesis_backend.spawn
        elif robot_backend is not None:
            adapter_factory = robot_backend.spawn
        from genesis_nav.nav2.bridge import Nav2NotAvailableError

        try:
            runtime = Runtime.from_scenario(
                scenario, event_sink, adapter_factory=adapter_factory
            )
        except Nav2NotAvailableError as exc:
            print(
                f"runtime.navigation: nav2 unavailable: {exc}",
                file=sys.stderr,
            )
            return 4

        # Genesis needs scene.build() after every agent has been spawned (which
        # happens during from_scenario) and before the first physics step.
        if genesis_backend is not None:
            genesis_backend.finalize()

        tool_api = runtime.tool_api(
            scenario_id=scenario.scenario_id, event_buffer=event_buffer
        )
        del tool_api  # exposed for downstream embedders; not used inside `gnav run`

        if args.ros:
            try:
                from genesis_nav.ros.bridge import BridgeConfig, RosBridge
            except ImportError as exc:
                print(f"--ros requires rclpy and genesis_nav_msgs: {exc}", file=sys.stderr)
                return 3
            if record_rosbag:
                from genesis_nav.ros.bag_writer import (
                    RosbagNotAvailableError,
                    RosbagRecorder,
                    load_rosbag_profile,
                )

                try:
                    bag_recorder = RosbagRecorder(
                        run_dir / "rosbag",
                        load_rosbag_profile(args.rosbag_profile),
                        [spec.namespace for spec in scenario.agents],
                    )
                except RosbagNotAvailableError as exc:
                    print(f"--record unavailable: {exc}", file=sys.stderr)
                    return 3
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
                jsonl_sink,
                config=BridgeConfig(qos_path=args.qos_profile),
                teleop_command_handler=_ros_teleop,
                episode_id=episode_id,
            )
            event_sink = FanoutEventSink([jsonl_sink, event_buffer, bridge])
            runtime.events = event_sink
            bridge.set_diagnostics_provider(runtime.diagnostics)
            if bag_recorder is not None:
                bridge.set_rosbag_recorder(bag_recorder)

        try:
            event_sink.write(
                ts=0.0,
                episode_id=episode_id,
                event="SCENARIO_STARTED",
                data={
                    "scenario_id": scenario.scenario_id,
                    "seed": scenario.seed,
                    "mode": mode,
                    "agent_count": len(scenario.agents),
                    "task_count": len(scenario.tasks),
                },
            )
            for task in scenario.tasks:
                runtime.submit_task(task, episode_id=episode_id)
            runtime.dispatch_pending(episode_id=episode_id)

            on_step = None
            if bridge is not None:
                bridge.publish_clock(0.0)
                bridge.publish_states(0.0)
                bridge.publish_diagnostics(0.0)
                bridge.publish_scenario_state(
                    0.0,
                    scenario_id=scenario.scenario_id,
                    seed=scenario.seed,
                    runtime_mode=mode,
                    paused=False,
                    recording=record_rosbag,
                )

            def on_step(sim_time: float) -> None:
                if genesis_backend is not None:
                    genesis_backend.step(runtime.clock.step_sec)
                if robot_backend is not None:
                    robot_backend.step(runtime.clock.step_sec)
                if bridge is not None:
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
            event_sink.write(
                ts=runtime.clock.sim_time_sec,
                episode_id=episode_id,
                event="SCENARIO_FINISHED",
                data={"task_count": len(scenario.tasks), "summary": summary},
            )
            if bridge is not None:
                bridge.publish_fleet_state(
                    runtime.clock.sim_time_sec,
                    completed=int(summary["task_succeeded_count"]),
                    extra={"summary": summary},
                )
        finally:
            if bag_recorder is not None:
                bag_recorder.close()
            if bridge is not None:
                bridge.shutdown()
            if robot_backend is not None:
                robot_backend.shutdown()

    if record_rosbag and bag_recorder is None:
        from genesis_nav.ros.bag_writer import write_recording_skipped_marker

        write_recording_skipped_marker(
            run_dir,
            reason="rosbag recording requires `gnav run --ros` (rclpy + genesis_nav_msgs)",
        )

    task_succeeded_count = int(summary["task_succeeded_count"])
    task_failed_count = int(summary["task_failed_count"])
    task_total = task_succeeded_count + task_failed_count
    metrics = MetricsSnapshot(
        scenario_id=scenario.scenario_id,
        seed=scenario.seed,
        agent_count=len(scenario.agents),
        task_count=len(scenario.tasks),
        success_rate=float(summary["success_rate"]),
        task_succeeded_count=task_succeeded_count,
        task_failed_count=task_failed_count,
        success_rate_ci=wilson_success_rate_ci(task_succeeded_count, task_total),
        command_accept_count=int(summary["command_accept_count"]),
        command_rejection_count=int(summary["command_rejection_count"]),
        time_to_goal_mean_sec=float(summary["time_to_goal_mean_sec"]),
        path_length_mean_m=float(summary["path_length_mean_m"]),
        sim_steps=int(summary["sim_steps"]),
        task_dispatched_count=int(summary["task_dispatched_count"]),
        task_pending_peak=int(summary["task_pending_peak"]),
        reservation_granted_count=int(summary["reservation_granted_count"]),
        reservation_conflict_count=int(summary["reservation_conflict_count"]),
        reservation_released_count=int(summary["reservation_released_count"]),
        replan_count=int(summary["replan_count"]),
        obstacle_event_count=int(summary["obstacle_event_count"]),
        watchdog_stop_count=int(summary["watchdog_stop_count"]),
        collision_count=int(summary["collision_count"]),
        near_miss_count=int(summary["near_miss_count"]),
        yield_count=int(summary["yield_count"]),
        headon_reroute_count=int(summary["headon_reroute_count"]),
        costmap_wait_count=int(summary["costmap_wait_count"]),
        dwell_count=int(summary["dwell_count"]),
        dwell_time_sec=float(summary["dwell_time_sec"]),
    )
    write_json(run_dir / "metrics.json", metrics.to_dict())
    write_report(run_dir / "report.md", scenario, metrics)

    print(run_dir)
    return 0


REPLAY_REQUIRED_ARTIFACTS = (
    "scenario.yaml",
    "resolved_config.yaml",
    "events.jsonl",
    "metrics.json",
    "report.md",
    "env.json",
)

REPLAY_REQUIRED_METRICS = (
    "scenario_id",
    "seed",
    "success_rate",
    "sim_steps",
)

REPLAY_PLAYBACK_EVENTS = frozenset(
    {
        "SCENARIO_STARTED",
        "SCENARIO_FINISHED",
        "TASK_ASSIGNED",
        "TASK_STARTED",
        "TASK_SUCCEEDED",
        "TASK_FAILED",
        "SAFETY_STOP",
        "FALL_DETECTED",
        "COLLISION",
        "NEAR_MISS",
        "AGENT_YIELDED",
        "AGENT_STUCK",
        "DWELL_STARTED",
        "DWELL_FINISHED",
    }
)


def replay_command(args: argparse.Namespace) -> int:
    run_dir = args.run_dir
    missing = [name for name in REPLAY_REQUIRED_ARTIFACTS if not (run_dir / name).exists()]
    if missing:
        print(f"missing replay artifacts: {', '.join(missing)}", file=sys.stderr)
        return 2

    events_path = run_dir / "events.jsonl"
    try:
        records = _load_events(events_path)
    except _ReplayValidationError as exc:
        print(f"invalid events.jsonl: {exc}", file=sys.stderr)
        return 2

    if not records:
        print("events.jsonl is empty", file=sys.stderr)
        return 2
    if records[0]["event"] != "SCENARIO_STARTED":
        print("first event must be SCENARIO_STARTED", file=sys.stderr)
        return 2
    terminal = terminal_scenario_record(records)
    if terminal is None or terminal["event"] != "SCENARIO_FINISHED":
        print("last scenario event must be SCENARIO_FINISHED", file=sys.stderr)
        return 2

    try:
        metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        print(f"invalid metrics.json: {exc}", file=sys.stderr)
        return 2
    metrics_missing = [key for key in REPLAY_REQUIRED_METRICS if key not in metrics]
    if metrics_missing:
        print(
            f"metrics.json missing keys: {', '.join(metrics_missing)}",
            file=sys.stderr,
        )
        return 2

    if args.print_events:
        for record in records:
            if record["event"] not in REPLAY_PLAYBACK_EVENTS:
                continue
            print(
                f"{record['ts']:>10.3f}  {record['event']:<18}  "
                f"agent={record.get('agent_id', '') or '-':<14}  "
                f"task={record.get('task_id', '') or '-'}"
            )

    if args.to_rosbag:
        from genesis_nav.ros.bag_writer import RosbagNotAvailableError
        from genesis_nav.ros.replay_export import export_run_to_rosbag

        try:
            bag_dir = export_run_to_rosbag(
                run_dir,
                rosbag_profile=args.rosbag_profile,
            )
        except RosbagNotAvailableError as exc:
            print(f"--to-rosbag unavailable: {exc}", file=sys.stderr)
            return 3
        except RuntimeError as exc:
            print(str(exc), file=sys.stderr)
            return 4
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        print(f"rosbag exported: {bag_dir}")

    print(f"replay artifacts valid: {run_dir}")
    return 0


class _ReplayValidationError(Exception):
    """Raised when events.jsonl cannot be parsed for replay validation."""


def _load_events(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, start=1):
            stripped = raw.strip()
            if not stripped:
                continue
            try:
                record = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise _ReplayValidationError(f"line {line_no}: {exc}") from exc
            for key in ("ts", "episode_id", "event"):
                if key not in record:
                    raise _ReplayValidationError(
                        f"line {line_no}: missing field '{key}'"
                    )
            records.append(record)
    return records


def bench_command(args: argparse.Namespace) -> int:
    if args.run is not None:
        return _bench_run_suite(args)
    if args.scenario is None:
        print("bench: provide a scenario path or --run <suite_dir>", file=sys.stderr)
        return 2
    scenario = load_scenario(args.scenario)
    print(
        json.dumps(
            {
                "scenario_id": scenario.scenario_id,
                "seed": scenario.seed,
                "agents": len(scenario.agents),
                "tasks": len(scenario.tasks),
                "metrics": list(scenario.metrics),
            },
            sort_keys=True,
        )
    )
    return 0


def _bench_run_suite(args: argparse.Namespace) -> int:
    return run_benchmark_suite(
        args.run,
        args.output_dir,
        args.report,
        include_integration=getattr(args, "include_integration", False),
        run_command=main,
    )


def doctor_command(args: argparse.Namespace) -> int:
    return run_doctor(as_json=bool(args.json))


def create_run_dir(root: Path, scenario: Scenario) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
    base = root / f"{stamp}_{scenario.scenario_id}_seed{scenario.seed}"
    candidate = base
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = root / f"{base.name}_{suffix}"
    return candidate


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=True)


def write_json(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_report(path: Path, scenario: Scenario, metrics: MetricsSnapshot) -> None:
    lines = [
        f"# Run Report: {scenario.scenario_id}",
        "",
        f"- seed: `{scenario.seed}`",
        f"- world: `{scenario.world}`",
        f"- agents: `{len(scenario.agents)}`",
        f"- tasks: `{len(scenario.tasks)}`",
        f"- success_rate: `{metrics.success_rate}`",
        *(
            [
                f"- success_rate_ci_95: `[{metrics.success_rate_ci['low']:.3f}, "
                f"{metrics.success_rate_ci['high']:.3f}]`"
            ]
            if metrics.success_rate_ci is not None
            else []
        ),
        f"- tasks_succeeded: `{metrics.task_succeeded_count}`",
        f"- tasks_failed: `{metrics.task_failed_count}`",
        f"- time_to_goal_mean_sec: `{metrics.time_to_goal_mean_sec:.3f}`",
        f"- path_length_mean_m: `{metrics.path_length_mean_m:.3f}`",
        f"- command_accept_count: `{metrics.command_accept_count}`",
        f"- command_rejection_count: `{metrics.command_rejection_count}`",
        f"- task_dispatched_count: `{metrics.task_dispatched_count}`",
        f"- task_pending_peak: `{metrics.task_pending_peak}`",
        f"- reservation_granted_count: `{metrics.reservation_granted_count}`",
        f"- reservation_conflict_count: `{metrics.reservation_conflict_count}`",
        f"- sim_steps: `{metrics.sim_steps}`",
        "",
        "This report is generated by the v0.1 runtime loop.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
